//! The simulated world.
//!
//! Everything the client believes about physical state lives here: the navmesh it walks on and the
//! pools of entities streamed in around it. The layer exists because the wire protocol is a *view* onto
//! a simulation rather than the simulation itself — inbound packets are applied to the world and
//! outbound packets are sampled from it, so the two directions never have to agree on an implicit
//! statement order inside one giant tick function.
//!
//! Entities are kept in per-type pools, the way both the game client (`CPlayerPool` / `CVehiclePool` /
//! `CObjectPool`) and the scripts that query them are already organised, rather than one flat entity
//! id space: the pools share a world and a clock, not a key space.

pub(crate) mod geometry;
pub(crate) mod motion;
pub(crate) mod objects;
pub(crate) mod players;
pub(crate) mod vehicles;

use std::sync::{Arc, Mutex};
use std::time::Instant;

use samp_proto::Vector3;

use geometry::bypass_point;
use objects::ObjectPool;
use players::{RemotePlayers, REMOTE_PLAYER_CLEARANCE, REMOTE_PLAYER_REACH_Z};
use vehicles::VehicleTable;

/// Ped-origin height above the navmesh floor, added to every navmesh-derived Z the walker reports.
/// GTA's engine rests a ped's origin exactly `collisionFloorZ + 1.0` (`CPed::ProcessEntityCollision`
/// snaps `pos.z = groundLineHitZ + 1.0`; the ped col model's probe line is 1.0 long), but our Recast
/// floor already sits ~0.33 above the true collision surface, so the lift that lands us on the real
/// ped height is 1.0 - 0.33. Value measured directly: median of (real player's reported Z - our nav
/// floor Z) over 5.39M inbound on-foot syncs, via `cargo run -p app --bin navcalib` — 0.674 world-wide,
/// 0.682 over the sawmill, stddev 0.11. Rounded DOWN: overshooting above a real ped's origin is the
/// hover/fly direction, undershooting only costs snap margin. Re-measure with navcalib after any
/// rebake — this constant tracks the bake, not just the engine.
/// See docs/memory/samp-client/engine.md#ped-ground-offset.
pub(crate) const PED_GROUND_OFFSET: f32 = 0.67;

/// One streamed entity, as scripts see it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EntityRef {
    pub id: u16,
    pub position: Vector3,
    /// Skin/vehicle model where the server told us (`PlayerStreamIn`); `None` for an entity we only
    /// inferred from a sync packet.
    pub model: Option<i32>,
}

/// A read-only picture of what is streamed in around the bot, republished once per send tick.
///
/// Scripts observe entities through this instead of hand-rolling pools out of raw RPCs in Luau, which
/// is what `shim.luau` and `helpers.luau` do today. It is a SNAPSHOT on purpose: scripts poll, the
/// simulation must not block on a script, and nothing here can write back — so no script can perturb
/// the world it is reading.
#[derive(Debug, Clone, Default)]
pub struct EntitySnapshot {
    pub players: Vec<EntityRef>,
    pub vehicles: Vec<EntityRef>,
}

/// Shared handle on [`EntitySnapshot`]. Cheap to clone; every read is one critical section.
#[derive(Clone, Default)]
pub struct EntityView(Arc<Mutex<EntitySnapshot>>);

impl EntityView {
    /// The latest snapshot. Returns an empty picture rather than panicking if a reader poisoned the
    /// lock — a script that cannot see entities is recoverable; a client that dies is not.
    pub fn read(&self) -> EntitySnapshot {
        self.0.lock().map(|s| s.clone()).unwrap_or_default()
    }

    pub(crate) fn publish(&self, snapshot: EntitySnapshot) {
        if let Ok(mut slot) = self.0.lock() {
            *slot = snapshot;
        }
    }
}

/// The world the local ped moves through: static navigation plus everything streamed into it.
#[derive(Default)]
pub(crate) struct World {
    /// Navmesh pathfinding view, loaded from `ClientConfig::navmesh` — enables the native walker.
    pub(crate) nav: Option<sa_nav::NavQuery>,
    /// Other players, from inbound on-foot sync.
    pub(crate) players: RemotePlayers,
    /// Streamed vehicles and the unoccupied-sync (209) ownership election.
    pub(crate) vehicles: VehicleTable,
    /// Streamed objects the static bake never saw.
    pub(crate) objects: ObjectPool,
}

impl World {
    /// Plan a route, already routed around whatever is standing in it. `seed` varies the line through
    /// the middle so repeated walks between the same two points don't retrace an identical route.
    pub(crate) fn find_path(&self, from: Vector3, to: Vector3, seed: u64) -> Option<Vec<[f32; 3]>> {
        let path = self.nav.as_ref()?.find_path_with_variety(
            [from.x, from.y, from.z],
            [to.x, to.y, to.z],
            seed,
        )?;
        Some(self.avoid(path, Instant::now()))
    }

    /// The ground under `at`, as a ped origin height: the navmesh floor lifted by
    /// [`PED_GROUND_OFFSET`], falling back to `at`'s own Z (with no navmesh, the height the bot was
    /// already standing at is the best estimate of the floor beneath it).
    pub(crate) fn ground_below(&self, at: Vector3) -> f32 {
        self.nav
            .as_ref()
            .and_then(|nav| nav.floor([at.x, at.y, at.z]))
            .map_or(at.z, |z| z + PED_GROUND_OFFSET)
    }

    /// Everything streamed in right now, for scripts to read.
    pub(crate) fn snapshot(&self, now: Instant) -> EntitySnapshot {
        EntitySnapshot {
            players: self
                .players
                .iter_fresh(now)
                .map(|p| EntityRef {
                    id: p.id,
                    position: p.position,
                    model: p.model,
                })
                .collect(),
            vehicles: self
                .vehicles
                .streamed()
                .map(|(id, position)| EntityRef {
                    id,
                    position,
                    model: None,
                })
                .collect(),
        }
    }

    /// Forget every streamed entity. Ids are per-session, so carrying a pool across a reconnect names
    /// entities that no longer exist — or worse, different ones.
    pub(crate) fn clear_entities(&mut self) {
        self.players.clear();
        self.vehicles.clear();
        self.objects.clear();
    }

    /// Nudge a path around anything standing in it — inserting a perpendicular bypass waypoint,
    /// re-verified on-mesh via `floor` and skipped if that would land off-mesh, since a residual graze
    /// beats a jump that reads as a teleport.
    fn avoid(&self, path: Vec<[f32; 3]>, now: Instant) -> Vec<[f32; 3]> {
        let Some(nav) = self.nav.as_ref() else {
            return path;
        };
        // Other players are obstacles too: GTA repels overlapping peds, so a path drawn through one has
        // the remote client shoving our rendered ped aside while our sync insists on the line.
        let live: Vec<([f32; 3], f32, f32)> = self
            .objects
            .live(now)
            .map(|p| {
                (
                    [p.x, p.y, p.z],
                    objects::CLEARANCE,
                    // WHY: a streamed object's origin is not its footprint — a tall prop legitimately
                    // sits metres above the ground it blocks — so narrowing this to a ped-sized gate
                    // stops the walker avoiding exactly the objects it was added for.
                    f32::INFINITY,
                )
            })
            .chain(self.players.iter_fresh(now).map(|p| {
                let at = p.predicted_position(now);
                (
                    [at.x, at.y, at.z],
                    REMOTE_PLAYER_CLEARANCE,
                    REMOTE_PLAYER_REACH_Z,
                )
            }))
            .collect();
        if live.is_empty() {
            return path;
        }
        let mut out = Vec::with_capacity(path.len());
        let mut prev: Option<[f32; 3]> = None;
        for wp in path {
            if let Some(p) = prev {
                for &(obstacle, clearance, reach_z) in &live {
                    if let Some(bypass) = bypass_point(p, wp, obstacle, clearance, reach_z) {
                        // Take the floor height, don't just test for it: the bypass is pushed sideways
                        // off the original segment, so its interpolated Z is wrong by the slope times
                        // that push — an unbounded Z error on the very waypoints added to avoid a
                        // collision.
                        if let Some(z) = nav.floor(bypass) {
                            out.push([bypass[0], bypass[1], z]);
                        }
                    }
                }
            }
            out.push(wp);
            prev = Some(wp);
        }
        out
    }
}
