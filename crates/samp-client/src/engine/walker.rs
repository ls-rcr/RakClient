//! The local ped's movement: navmesh path following, the gait/locomotion coupling, the idle facing
//! wobble, and the free fall left behind by a slap. This is the simulation half of the tick — it
//! touches no transport, so it stays synchronous and testable without a runtime.

use super::*;

impl<T: Transport> Engine<T> {
    /// Plan a navmesh path from the current position and hand it to the body. Emits `WalkFailed`
    /// when there is no navmesh or no corridor to the target.
    pub(super) fn start_walk(&mut self, target: Vector3, mode: u8) {
        let to = [target.x, target.y, target.z];
        // A fresh seed each call: repeated walks between the same two points take a slightly different
        // line through the middle (see NavQuery::find_path_with_variety) instead of the identical route.
        // A direct seek is a one-waypoint path, grounded the same way a navmesh waypoint would be, so
        // everything downstream — gait, slew, measured velocity, arrival — is identical. Nothing here
        // teleports; it just declines to ask the navmesh for a corridor it does not have.
        let path = if is_direct_mode(mode) {
            // INVARIANT: never adopt the caller's Z. On-mesh we ground on the navmesh floor;
            // off-mesh — which is the whole point of a direct seek — we hold the height we are
            // already standing at, because the caller's Z is a world object's origin and walking to
            // it lifts the ped off the ground.
            let floor = self
                .world
                .nav
                .as_ref()
                .and_then(|nav| nav.floor([target.x, target.y, target.z]))
                .unwrap_or(self.sync.position.z - PED_GROUND_OFFSET);
            Some(vec![[target.x, target.y, floor]])
        } else {
            self.world
                .find_path(self.sync.position, target, rand::random())
        };
        match path {
            Some(points) if matches!(self.state, ConnectionState::Spawned) => {
                let gait = gait_for_mode(mode);
                self.locomotion.set_target(Some(gait.kind));
                tracing::info!(
                    ?to,
                    waypoints = points.len(),
                    speed = gait.kind.nominal_wire_mag() * 50.0,
                    "walkTo: path found, walking"
                );
                // WHY: a walkTo issued mid-walk (retargeting to the next tree) must inherit the facing.
                // Restarting it from `None` snaps the heading to the new bearing, which for a target
                // behind the bot is a 180 deg flip and a full velocity reversal in one packet.
                let heading = self.motion.heading();
                self.motion.place(self.sync.position);
                self.motion.start(points, target, heading);
                self.walk = Some(WalkState {
                    gait,
                    target,
                    last_step: None,
                    last_position: self.sync.position,
                    finishing: false,
                });
                self.set_walking_flag(true);
            }
            _ => {
                tracing::warn!(
                    ?to,
                    nav = self.world.nav.is_some(),
                    "walkTo failed: no path"
                );
                // A new walkTo supersedes the old walk even when it fails to path: leaving the previous
                // one driving would walk the bot to a target the script has already moved on from,
                // while `isWalking()` reports true and the script reads that as ITS walk running. The
                // scripted fallback (walk.luau's `fly.to`) can only take over from a stopped walker.
                self.stop_walk();
                self.pending.push_back(ClientEvent::WalkFailed {
                    x: target.x,
                    y: target.y,
                    z: target.z,
                });
            }
        }
    }

    /// Abort the active walk: clear the reported forward-input/velocity/animation
    /// so the next sync shows the bot standing idle.
    pub(super) fn stop_walk(&mut self) {
        self.motion.stop();
        if self.walk.take().is_some() {
            self.set_standing();
            self.set_walking_flag(false);
            self.mirror_to_state();
        }
    }

    /// Report the idle standing state on the outbound sync (no forward analog, no
    /// movement keys, zero velocity, no animation).
    pub(super) fn set_standing(&mut self) {
        // Target alone only asks the model to ramp down; it is not ticked while idle, so its magnitude
        // would freeze at cruise and the NEXT walk's first packet would report full speed from a
        // standstill — an instantaneous 0 -> 8.6 u/s the chained ramp exists to prevent. Run it to a
        // real stop here (bounded: the ramp reaches zero well inside this many maximum-dt steps).
        self.locomotion.set_target(None);
        for _ in 0..20 {
            if self.locomotion.is_stopped() {
                break;
            }
            self.locomotion.tick(TickInput {
                dt: 0.5,
                heading_delta: 0.0,
            });
        }
        self.sync.up_down = 0;
        self.sync.left_right = 0;
        // Clear only what the GAIT owns: standing still is not a reason to drop a key a script
        // is holding (a real player stands and aims), and clearing it here would contradict the
        // preservation the walker tick guarantees.
        self.sync.keys &= !GAIT_OWNED_KEYS;
        self.sync.move_speed = Vector3::default();
        // Idle stance anim, NOT (0,0), which is a headless fingerprint; see
        // docs/memory/samp-client/engine.md#set-standing.
        self.sync.animation_id = 1189;
        self.sync.animation_flags = 0x8004;
        // Also clear the shared state so a leftover walk velocity can't leak back into idle syncs; see
        // docs/memory/samp-client/engine.md#set-standing.
        if let Some(state) = &self.bot_state {
            let mut s = state.borrow_mut();
            s.on_foot.keys &= !GAIT_OWNED_KEYS;
            s.on_foot.move_speed = Vector3::default();
            s.on_foot.animation_id = 1189;
            s.on_foot.animation_flags = 0x8004;
        }
    }

    /// Mirror the walker liveness into the shared state for `isWalking()`.
    pub(super) fn set_walking_flag(&mut self, walking: bool) {
        if let Some(state) = &self.bot_state {
            state.borrow_mut().walking = walking;
        }
    }

    /// Sample the body, decide the next gait from what it reports, and turn both into a packet.
    ///
    /// The body integrates on its own clock; this runs on the SEND cadence, which is why the gait model
    /// is ticked here and exactly once. Its holds are per-call counters fitted to a 10 Hz capture, so
    /// advancing them per physics step would shorten every one of them by the rate ratio.
    pub(super) fn step_walk(&mut self, now: Instant) {
        let Some(mut walk) = self.walk.take() else {
            return;
        };
        let wall_dt = match walk.last_step {
            Some(at) => now.duration_since(at).as_secs_f32(),
            None => 0.0,
        };
        walk.last_step = Some(now);

        let body = self.motion.sample();
        // INVARIANT: the reported velocity is the measured displacement over the interval the remote
        // dead-reckons. Deriving it from the commanded speed instead would claim motion the body did
        // not make whenever the two clocks disagree — which, with the body on its own clock, is always.
        // INVARIANT: velocity is a finite difference, so a zero-length interval has no velocity to
        // report — it must NOT be rescued with an epsilon divisor. The body keeps stepping on its own
        // clock, so on the first frame of a walk (and on the frame after a slap released the clock)
        // the numerator is a real displacement while `wall_dt` is 0; dividing there put ~10^6 u/s on
        // the wire, which is the speed-hack kick this whole model exists to avoid.
        let velocity = if wall_dt > 1e-6 {
            Vector3 {
                x: (body.position.x - walk.last_position.x) / (50.0 * wall_dt),
                y: (body.position.y - walk.last_position.y) / (50.0 * wall_dt),
                z: 0.0,
            }
        } else {
            Vector3::default()
        };
        walk.last_position = body.position;
        self.sync.position = body.position;

        if let Some(at) = body.arrival {
            self.sync.position = at;
            self.finish_walk(&walk, velocity);
            return;
        }
        if !body.walking {
            // The body stopped without arriving — someone cleared it out from under us. Report the
            // standstill rather than commanding a gait into a walk that no longer exists.
            self.finish_walk(&walk, velocity);
            return;
        }

        // Braking is a target change, not a scale factor: the model ramps down its own chain and snaps
        // to exactly zero. Stopping short is expected (see brake_distance); walking the remainder in is
        // the only honest recovery, since forcing an arrival would snap the reported position metres.
        if self.locomotion.is_stopped() {
            walk.finishing = true;
        }
        self.locomotion.set_target(if walk.finishing {
            Some(GaitKind::Walk)
        } else if body.remaining <= brake_distance(walk.gait.kind) {
            None
        } else {
            Some(walk.gait.kind)
        });
        let sample = self.locomotion.tick(TickInput {
            dt: wall_dt.min(0.5),
            heading_delta: body.heading_delta,
        });
        self.motion.command(sample.move_speed_mag, walk.finishing);

        // Report the real gait (analog + keys + 3-D velocity + anim); AC-valid since the (w,x,y,z)
        // codec fly-hack fix. See docs/memory/samp-client/engine.md#step-walk-gait.
        self.sync.up_down = sample.up_down;
        self.sync.left_right = sample.left_right;
        // Face the travel heading: quaternion about world +Z, serialized w-first.
        self.sync.quaternion = Quaternion {
            x: 0.0,
            y: 0.0,
            z: (body.heading * 0.5).sin(),
            w: (body.heading * 0.5).cos(),
        };
        self.sync.keys = walk.gait.keys | (self.sync.keys & !GAIT_OWNED_KEYS);
        // INVARIANT: vz comes from the model, never from the slope. GTA zeroes a grounded ped's
        // move_speed.z every frame and realizes descent as position corrections, so a sustained
        // grade × mag term is a vertical velocity real clients never produce.
        self.sync.move_speed = Vector3 {
            z: sample.vz,
            ..velocity
        };
        self.sync.animation_id = sample.animation_id;
        self.sync.animation_flags = sample.animation_flags;
        self.mirror_to_state();
        self.walk = Some(walk);
    }

    /// End the walk standing at the path's end: `self.walk` is already taken, so the walk finishes by
    /// simply not being put back. Stands where the PATH ended, not where the caller asked — find_path
    /// clamps a target inside a carved keep-out to the hole edge, so snapping to `target` would jump the
    /// reported position by the whole carve radius with zero velocity, past the remote's 2.0 m XY snap
    /// threshold. The caller's XY is taken only when the path actually reached it; its Z never is (for a
    /// script aiming at a world object that is the object's origin, not a ped standing height), and
    /// `WalkArrived` reports where the bot ACTUALLY stands so a script can't act on a position it is not at.
    pub(super) fn finish_walk(&mut self, walk: &WalkState, final_velocity: Vector3) {
        // The body stands exactly where it walked to. Snapping onto the caller's XY instead used to
        // put a discontinuity on the last frame — the walk decelerates into the arrival radius and then
        // jumps the remainder with zero reported velocity, which is the one thing this walker exists to
        // avoid. Nothing needs the snap: `find_path` clamps a target inside a carved keep-out to the
        // hole edge anyway, and the scripts that need to be exactly somewhere re-pin themselves after
        // checking the distance.
        let arrived_at = self.sync.position;
        let short = ((walk.target.x - arrived_at.x).powi(2)
            + (walk.target.y - arrived_at.y).powi(2))
        .sqrt()
            > WALK_ARRIVE;
        self.set_standing();
        // INVARIANT: the arrival frame still travelled — the body decelerated into the radius during
        // this very tick. Zeroing the velocity while the position moved is a one-frame teleport, the
        // exact shape this walker exists to remove; the standstill is reported by the NEXT frame, which
        // genuinely does not move.
        self.sync.move_speed = final_velocity;
        self.set_walking_flag(false);
        self.mirror_to_state();
        // Report where we ACTUALLY stand: a script told it reached a target it is a metre short of
        // acts on a position the bot is not at.
        self.pending.push_back(ClientEvent::WalkArrived {
            x: arrived_at.x,
            y: arrived_at.y,
            z: arrived_at.z,
        });
        tracing::info!(
            x = arrived_at.x,
            y = arrived_at.y,
            z = arrived_at.z,
            short,
            "walkTo: arrived"
        );
    }

    /// While the native walker isn't driving movement, nudge the reported facing with an occasional
    /// small glance instead of leaving it perfectly frozen — a real player standing still still shifts
    /// their look direction a little. Never runs while `step_walk` owns the quaternion (guarded by
    /// `self.walk`, checked fresh each call so it never fights an active walk), and only re-rolls the
    /// offset on the glance cadence, not every tick, so it doesn't defeat the on-foot sync's
    /// idle-keepalive send-rate reduction. Appearance only — never touches position/velocity/keys.
    pub(super) fn apply_idle_wobble(&mut self, now: Instant) {
        if self.walk.is_some() {
            self.idle_wobble = None; // walking again: drop the anchor, no stale offset carries over
            return;
        }
        let due = self
            .idle_wobble
            .as_ref()
            .is_none_or(|w| now >= w.next_glance_at);
        // Anchor to the SCRIPT's facing, re-read every tick. The wobble writes only `sync`, never the
        // shared state, so this can't feed back on itself — and it means a `setBotFacing` takes effect
        // on the very next packet. Anchoring once per idle stretch instead made every later
        // `setBotFacing` a permanent no-op: the bot reported the yaw it happened to stop at, so a
        // script turning to face an NPC never actually turned.
        let script_yaw = self.bot_state.as_ref().map(|state| {
            let q = state.borrow().on_foot.quaternion;
            2.0 * q.z.atan2(q.w)
        });
        if due {
            let offset = (rand::random::<f32>() * 2.0 - 1.0) * IDLE_WOBBLE_MAX;
            let next_glance_at = now
                + Duration::from_millis(
                    IDLE_GLANCE_MIN_MS
                        + (rand::random::<f32>() * IDLE_GLANCE_SPAN_MS as f32) as u64,
                );
            let base_yaw = script_yaw.unwrap_or_else(|| {
                self.idle_wobble.as_ref().map_or_else(
                    || 2.0 * self.sync.quaternion.z.atan2(self.sync.quaternion.w),
                    |w| w.base_yaw,
                )
            });
            self.idle_wobble = Some(IdleWobble {
                base_yaw,
                offset,
                next_glance_at,
            });
        } else if let (Some(yaw), Some(w)) = (script_yaw, self.idle_wobble.as_mut()) {
            w.base_yaw = yaw;
        }
        let Some(w) = self.idle_wobble.as_ref() else {
            return;
        };
        let yaw = w.base_yaw + w.offset;
        let (sin, cos) = (yaw * 0.5).sin_cos();
        self.sync.quaternion = Quaternion {
            x: 0.0,
            y: 0.0,
            z: sin,
            w: cos,
        };
    }

    /// Advance an in-progress slap fall one sync tick: accelerate `sync.position.z` down toward
    /// `ground_z` like gravity instead of snapping, reporting a matching `move_speed` so the on-foot
    /// sync never reads as a teleport. Clears `self.slap` and re-arms aim on landing, exactly as
    /// `aim_follow` would have for an ordinary reposition. See [`SlapReaction`].
    pub(super) fn step_slap_reaction(&mut self, now: Instant) {
        let Some(mut slap) = self.slap.take() else {
            return;
        };
        let dt = now
            .duration_since(slap.last_step_at)
            .as_secs_f32()
            .min(0.25);
        slap.last_step_at = now;
        slap.fall_speed = (slap.fall_speed + SLAP_GRAVITY * dt).min(SLAP_TERMINAL_SPEED);

        let pos = self.sync.position;
        let new_z = (pos.z - slap.fall_speed * dt).max(slap.ground_z);
        let landed = new_z <= slap.ground_z;
        self.sync.position = Vector3 { z: new_z, ..pos };
        self.sync.move_speed = Vector3 {
            x: 0.0,
            y: 0.0,
            // INVARIANT: wire velocity is units per 1/50 s frame, the same ×50 relation the walker
            // reports its gait speed under; dividing by anything else desynchronises the fall the
            // remote dead-reckons from the fall we actually report.
            z: -slap.fall_speed / 50.0,
        };
        self.mirror_to_state();

        if landed {
            self.sync.move_speed = Vector3::default();
            self.mirror_to_state();
            let in_vehicle = self.in_vehicle();
            self.aim
                .on_reposition(self.sync.position, self.sync.quaternion, in_vehicle);
            return;
        }
        self.slap = Some(slap);
    }

    /// Advance the local body one tick. Nothing here touches the transport, so the whole simulation
    /// step is synchronous and callable from a test without a runtime.
    ///
    /// INVARIANT: the three phases run in this order and only one body owner runs per tick. Script
    /// writes are mirrored in FIRST so the walker overrides them (a script that keeps writing position
    /// while a walk is active is expressing a stale intent); exactly one of slap / vehicle / walker
    /// then owns the body, slap pre-empting the rest because a fall is not negotiable; and the
    /// emulation's weapon and key adjustments fold in LAST so they win over both. Re-ordering any of
    /// it silently changes what reaches the wire.
    pub(super) fn advance_local(&mut self, now: Instant) {
        self.mirror_from_state();
        if self.slap.is_some() {
            // A slap owns the position this tick, so the walker's clock must not accrue the gap: its
            // next tick would advance the whole stalled interval in one step.
            if let Some(walk) = self.walk.as_mut() {
                walk.last_step = None;
            }
            self.step_slap_reaction(now);
        } else if self.in_vehicle() {
            // On-foot walker state (analog pedal, sprint key, yaw-only quaternion) has no business in a
            // vehicle body — a car reporting flat orientation at a fixed gait speed is not an engine.
            self.stop_walk();
        } else {
            self.step_walk(now);
            self.apply_idle_wobble(now);
        }
        self.adjust_on_foot_from_emulation(now);
    }
}
