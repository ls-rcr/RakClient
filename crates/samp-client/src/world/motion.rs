//! The local ped's body, simulated on its own clock.
//!
//! This is the one piece of the client that must keep running when nothing else is: a ped in motion
//! stays in motion whether or not the network loop got around to it. It is deliberately `Send` and
//! holds no script state, no transport and no `Rc`, so it can be stepped from a background task while
//! the engine is busy decoding a burst of packets.
//!
//! What lives here is *integration only*. The gait — how fast to go, which animation, what the analog
//! stick reads — is a wire-behaviour model calibrated per sent packet and stays with the sender, which
//! pushes its decision in through [`MotionHandle::command`]. If that model were ticked here it would
//! run at the physics rate and every one of its holds would shorten by the rate ratio.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use samp_proto::Vector3;

use super::geometry::remaining_path_xy;
use super::PED_GROUND_OFFSET;

/// The engine's simulation timestep. GTA runs the world at 50 Hz and the wire velocity unit is
/// literally "units per 1/50 s frame", so one step advances the body by exactly the magnitude a packet
/// reports.
pub(crate) const PHYSICS_DT: f32 = 0.02;

/// Max facing turn rate while walking, rad/s, slewed so corners read as a natural turn.
pub(crate) const TURN_RATE: f32 = 7.85;

/// Stop when within this many units of the final target, well under the remote's 2 m/1 m snap
/// threshold.
pub(crate) const WALK_ARRIVE: f32 = 1.0;

/// Most simulated time one catch-up may cover. A stalled stepper still has to advance the body — the
/// remote dead-reckoned straight through the gap — but an unbounded catch-up would walk the bot
/// through everything between here and there in one frame.
const MAX_CATCHUP: f32 = 0.5;

/// Which speed band the sender has asked for; the body only needs the magnitude.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Command {
    /// Wire magnitude (units per 1/50 s frame) to travel at along the current heading.
    pub(crate) mag: f32,
}

/// A navmesh walk in progress.
pub(crate) struct Walk {
    waypoints: Vec<Vector3>,
    next: usize,
    /// Current facing/travel heading (SA φ = atan2(dx, dy)), slewed toward the waypoint bearing.
    heading: Option<f32>,
    /// Where the walk was asked to end — reported back so the caller can tell a clamped path apart
    /// from a completed one.
    target: Vector3,
    /// Latched once the gait has braked to a standstill short of the target, so the sender can walk
    /// the remainder in instead of re-braking every frame.
    pub(crate) finishing: bool,
}

/// What the sender sees when it samples the body.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Sample {
    pub(crate) position: Vector3,
    pub(crate) heading: f32,
    /// Heading slew accumulated since the previous sample, reset by the sample.
    pub(crate) heading_delta: f32,
    /// Distance still to walk along the path; 0 when not walking.
    pub(crate) remaining: f32,
    pub(crate) walking: bool,
    /// Set for exactly one sample when a walk ended, carrying where the body actually stopped.
    pub(crate) arrival: Option<Vector3>,
}

#[derive(Default)]
pub(crate) struct Motion {
    position: Vector3,
    walk: Option<Walk>,
    command: Command,
    heading_delta_accum: f32,
    arrival: Option<Vector3>,
    /// Simulated time owed but not yet stepped, so a coarse or jittery caller still integrates the
    /// right total amount.
    accumulator: f32,
}

impl Motion {
    /// Advance by `elapsed` seconds in fixed [`PHYSICS_DT`] slices.
    pub(crate) fn advance(&mut self, elapsed: f32) {
        self.accumulator = (self.accumulator + elapsed).min(MAX_CATCHUP);
        while self.accumulator >= PHYSICS_DT {
            self.accumulator -= PHYSICS_DT;
            if !self.step(PHYSICS_DT) {
                break;
            }
        }
    }

    /// One fixed step. Returns false once there is nothing left to integrate.
    fn step(&mut self, dt: f32) -> bool {
        let Some(walk) = self.walk.as_mut() else {
            return false;
        };
        if remaining_path_xy(self.position, &walk.waypoints, walk.next) <= WALK_ARRIVE {
            self.arrival = Some(self.position);
            self.walk = None;
            return false;
        }

        let advance = self.command.mag * 50.0 * dt;
        // Retarget: skip waypoints within reach. Distances are HORIZONTAL (z is tracked separately).
        let (mut dx, mut dy);
        loop {
            let Some(wp) = walk.waypoints.get(walk.next) else {
                self.arrival = Some(self.position);
                self.walk = None;
                return false;
            };
            (dx, dy) = (wp.x - self.position.x, wp.y - self.position.y);
            let d = (dx * dx + dy * dy).sqrt();
            let last = walk.next + 1 == walk.waypoints.len();
            let reach = if last { WALK_ARRIVE } else { advance.max(0.5) };
            if d > reach {
                break;
            }
            walk.next += 1;
        }

        // Bearing toward the current waypoint, then SLEW the travel heading toward it at TURN_RATE so
        // position, facing and velocity stay consistent through a corner.
        let hlen = (dx * dx + dy * dy).sqrt();
        let target_bearing = if hlen > 1e-4 { dx.atan2(dy) } else { 0.0 };
        let heading = match walk.heading {
            Some(h) if hlen > 1e-4 => {
                let mut diff = target_bearing - h;
                while diff > std::f32::consts::PI {
                    diff -= std::f32::consts::TAU;
                }
                while diff < -std::f32::consts::PI {
                    diff += std::f32::consts::TAU;
                }
                let max_step = TURN_RATE * dt;
                h + diff.clamp(-max_step, max_step)
            }
            _ => target_bearing, // first step (or degenerate): snap, no spin
        };
        self.heading_delta_accum += walk.heading.map_or(0.0, |h| heading - h);
        walk.heading = Some(heading);

        // Unit travel direction from the slewed heading (φ = atan2(dx, dy) ⇒ dir = (sin φ, cos φ)).
        let (hx, hy) = (heading.sin(), heading.cos());
        let hstep = advance.min(hlen);
        // Track z toward the CURRENT waypoint's grounded z, proportional to horizontal progress, so the
        // path hugs terrain instead of diving.
        let wp_z = walk
            .waypoints
            .get(walk.next)
            .map(|w| w.z)
            .unwrap_or(walk.target.z);
        let nz = if hlen > 1e-3 {
            self.position.z + (wp_z - self.position.z) * (hstep / hlen).min(1.0)
        } else {
            wp_z
        };
        self.position = Vector3 {
            x: self.position.x + hx * hstep,
            y: self.position.y + hy * hstep,
            z: nz,
        };
        true
    }
}

/// Shared handle on the body. Every method is one critical section: the lock is never held across a
/// caller boundary, so a sampler can never observe a half-updated pose.
#[derive(Clone, Default)]
pub(crate) struct MotionHandle(Arc<Mutex<Motion>>);

impl MotionHandle {
    /// Step the body directly. Tests stand in for the background stepper this way, which keeps them
    /// deterministic instead of racing a real timer.
    #[cfg(test)]
    pub(crate) fn advance(&self, elapsed: Duration) {
        if let Ok(mut motion) = self.0.lock() {
            motion.advance(elapsed.as_secs_f32());
        }
    }

    /// Read the body and reset the per-sample accumulators.
    pub(crate) fn sample(&self) -> Sample {
        let Ok(mut m) = self.0.lock() else {
            return Sample {
                position: Vector3::default(),
                heading: 0.0,
                heading_delta: 0.0,
                remaining: 0.0,
                walking: false,
                arrival: None,
            };
        };
        let heading_delta = std::mem::take(&mut m.heading_delta_accum);
        let arrival = m.arrival.take();
        let position = m.position;
        let (heading, remaining, walking) = match m.walk.as_ref() {
            Some(w) => (
                w.heading.unwrap_or(0.0),
                remaining_path_xy(position, &w.waypoints, w.next),
                true,
            ),
            None => (0.0, 0.0, false),
        };
        Sample {
            position,
            heading,
            heading_delta,
            remaining,
            walking,
            arrival,
        }
    }

    /// Push the sender's gait decision in. Called once per sent frame, never per physics step.
    pub(crate) fn command(&self, mag: f32, finishing: bool) {
        if let Ok(mut m) = self.0.lock() {
            m.command = Command { mag };
            if let Some(w) = m.walk.as_mut() {
                w.finishing = finishing;
            }
        }
    }

    /// Teleport the body. Used for spawns, server repositions and script position writes — anything
    /// that is not the walker moving it.
    pub(crate) fn place(&self, position: Vector3) {
        if let Ok(mut m) = self.0.lock() {
            m.position = position;
        }
    }

    /// Begin a walk along `waypoints` (already navmesh-grounded), inheriting `heading` so a retarget
    /// mid-walk does not snap the facing.
    pub(crate) fn start(&self, waypoints: Vec<[f32; 3]>, target: Vector3, heading: Option<f32>) {
        if let Ok(mut m) = self.0.lock() {
            m.walk = Some(Walk {
                waypoints: waypoints
                    .into_iter()
                    .map(|p| Vector3 {
                        x: p[0],
                        y: p[1],
                        z: p[2] + PED_GROUND_OFFSET,
                    })
                    .collect(),
                next: 0,
                heading,
                target,
                finishing: false,
            });
            m.arrival = None;
            m.accumulator = 0.0;
            // INVARIANT: a walk starts from a standstill. Carrying the previous walk's magnitude lets
            // the body cruise before the sender has commanded anything, so the first frame reports a
            // displacement the gait model says it never made.
            m.command = Command::default();
            m.heading_delta_accum = 0.0;
        }
    }

    /// Abort any walk in progress, leaving the body where it stands.
    pub(crate) fn stop(&self) {
        if let Ok(mut m) = self.0.lock() {
            m.walk = None;
            m.command = Command::default();
            m.arrival = None;
        }
    }

    /// A non-owning handle for the background stepper: when the client drops, the upgrade fails and
    /// the task retires instead of stepping a body nobody reads.
    pub(crate) fn weak(&self) -> WeakMotion {
        WeakMotion(Arc::downgrade(&self.0))
    }

    /// The heading the body would keep if a new walk started now.
    pub(crate) fn heading(&self) -> Option<f32> {
        self.0.lock().ok().and_then(|m| m.walk.as_ref()?.heading)
    }
}

/// The background stepper's view of the body.
pub(crate) struct WeakMotion(std::sync::Weak<Mutex<Motion>>);

impl WeakMotion {
    /// Step the body if it is still alive; `false` once the owner is gone.
    pub(crate) fn advance(&self, elapsed: Duration) -> bool {
        let Some(motion) = self.0.upgrade() else {
            return false;
        };
        if let Ok(mut m) = motion.lock() {
            m.advance(elapsed.as_secs_f32());
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn handle_at(x: f32, y: f32) -> MotionHandle {
        let h = MotionHandle::default();
        h.place(Vector3 { x, y, z: 5.67 });
        h
    }

    /// The body is stepped in fixed slices regardless of how coarsely it is driven: one 100 ms advance
    /// and five 20 ms advances must land in the same place. That is the whole point of moving it off
    /// the network cadence.
    #[test]
    fn integration_is_independent_of_how_it_is_driven() {
        let path = vec![[0.0, 0.0, 5.0], [100.0, 0.0, 5.0]];
        let target = Vector3 {
            x: 100.0,
            y: 0.0,
            z: 5.0,
        };

        let coarse = handle_at(0.0, 0.0);
        coarse.start(path.clone(), target, None);
        coarse.command(0.1912, false);
        for _ in 0..10 {
            coarse.advance(Duration::from_millis(100));
        }

        let fine = handle_at(0.0, 0.0);
        fine.start(path, target, None);
        fine.command(0.1912, false);
        for _ in 0..50 {
            fine.advance(Duration::from_millis(20));
        }

        let (a, b) = (coarse.sample().position, fine.sample().position);
        assert!(
            (a.x - b.x).abs() < 1e-3 && (a.y - b.y).abs() < 1e-3,
            "coarse {a:?} vs fine {b:?}"
        );
        // 1 s at the sprint magnitude is ~9.56 u; the body must actually have moved.
        assert!(a.x > 9.0 && a.x < 10.0, "travelled {}", a.x);
    }

    /// Arrival is state, not an event: it is latched by the stepper and drained by exactly one sample.
    #[test]
    fn arrival_is_latched_once_and_stops_the_walk() {
        let h = handle_at(0.0, 0.0);
        h.start(
            vec![[0.0, 0.0, 5.0], [2.0, 0.0, 5.0]],
            Vector3 {
                x: 2.0,
                y: 0.0,
                z: 5.0,
            },
            None,
        );
        h.command(0.1912, false);
        for _ in 0..20 {
            h.advance(Duration::from_millis(20));
        }

        let first = h.sample();
        assert!(first.arrival.is_some(), "the walk should have arrived");
        assert!(!first.walking);
        assert!(
            h.sample().arrival.is_none(),
            "arrival must drain, not repeat"
        );
    }

    /// A stalled stepper still advances the body, but only up to the catch-up bound — otherwise a
    /// multi-second hitch walks the bot through everything in between in one frame.
    #[test]
    fn catch_up_is_bounded() {
        let h = handle_at(0.0, 0.0);
        h.start(
            vec![[0.0, 0.0, 5.0], [500.0, 0.0, 5.0]],
            Vector3 {
                x: 500.0,
                y: 0.0,
                z: 5.0,
            },
            None,
        );
        h.command(0.1912, false);
        h.advance(Duration::from_secs(30));
        let travelled = h.sample().position.x;
        assert!(
            travelled <= 0.1912 * 50.0 * MAX_CATCHUP + 1e-3,
            "a 30 s stall walked {travelled} u"
        );
        assert!(travelled > 0.0, "a stall must still advance the body");
    }
}
