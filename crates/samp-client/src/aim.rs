//! Native aim-sync emulation (ported from `aim_fix_updated.lua`): periodically sends a believable
//! aim packet so a bare bot doesn't look unnatural; see docs/memory/samp-client/aim.md#module.

use std::time::{Duration, Instant};

use samp_proto::{AimSyncData, Encode, Quaternion, Vector3};

use crate::state::AimData;

// WHY: 62, not 63 (the 6-bit max): over real captures 62 is the mode (69.2%) and 63 only 25.9%;
// "restoring" the max shifts every packet off the measured mode.
const CAM_EXT_ZOOM: u8 = 62;
const ASPECT_RATIO: u8 = 85;
// INVARIANT: a real on-foot client sends 203 at ~1 Hz, moving or not (samp.dll SendAimSync,
// ~100 ms while FIRE/AIM held); with the 100 ms on-foot tick the 203/207 ratio must stay inside
// the measured IQR 0.082–0.121 — slower, or suppressed while moving, sits below the real p05.
const AIM_MIN_MILLIS: u64 = 800;
const AIM_MAX_MILLIS: u64 = 1200;

/// Aim-sync state machine; see docs/memory/samp-client/aim.md#aimsync-usage.
pub(crate) struct AimSync {
    rng: u64,
    aim: AimData,
    next_at: Option<Instant>,
    is_regular_pos: bool,
    cam_offset: Vector3,
    cam_rel: Vector3,
}

impl AimSync {
    pub(crate) fn new(seed: u64) -> Self {
        Self {
            rng: seed | 1,
            aim: AimData::default(),
            next_at: None,
            is_regular_pos: false,
            cam_offset: Vector3::default(),
            cam_rel: Vector3::default(),
        }
    }

    /// Start the timer once the bot is spawned, generating an initial aim from the current pose.
    pub(crate) fn arm(
        &mut self,
        now: Instant,
        pos: Vector3,
        rotation: Quaternion,
        in_vehicle: bool,
    ) {
        self.generate(pos, rotation, in_vehicle, false);
        self.schedule(now);
    }

    // WHY: re-pins the camera unconditionally, not only on movement — `spoof_aim` bumps
    // `cam_pos.z` on every send, so skipping the re-pin while standing would accumulate that
    // bump into a steady camera climb at the ~1 Hz cadence.
    pub(crate) fn on_position(&mut self, pos: Vector3) {
        self.aim.cam_pos = Vector3 {
            x: pos.x + self.cam_rel.x,
            y: pos.y + self.cam_rel.y,
            z: pos.z + self.cam_rel.z,
        };
    }

    /// The server repositioned the bot: regenerate the aim and send it on the next tick.
    pub(crate) fn on_reposition(&mut self, pos: Vector3, rotation: Quaternion, in_vehicle: bool) {
        self.generate(pos, rotation, in_vehicle, self.is_regular_pos);
        self.is_regular_pos = true;
        self.next_at = Some(Instant::now() - Duration::from_secs(1)); // due now
    }

    pub(crate) fn due(&mut self, now: Instant) -> bool {
        match self.next_at {
            Some(at) if at <= now => {
                self.schedule(now);
                true
            }
            _ => false,
        }
    }

    /// The current aim (the high-level view the engine mirrors into `LocalPlayer`).
    pub(crate) fn aim(&self) -> &AimData {
        &self.aim
    }

    /// Mutable access for last-moment tweaks (client emulation adjusts camera z / weapon state here).
    pub(crate) fn aim_mut(&mut self) -> &mut AimData {
        &mut self.aim
    }

    /// Encode the current aim into the wire packet (`[203][AimSyncData]`).
    pub(crate) fn encode(&self) -> Vec<u8> {
        let wire = AimSyncData {
            cam_mode: self.aim.cam_mode,
            cam_front: self.aim.cam_front,
            cam_pos: self.aim.cam_pos,
            aim_z: 0.0,
            cam_ext_zoom_weapon_state: (self.aim.ext_zoom & 0x3F) | (self.aim.weapon_state << 6),
            aspect_ratio: ASPECT_RATIO,
        };
        wire.to_packet()
    }

    /// Reset on disconnect.
    pub(crate) fn reset(&mut self) {
        self.is_regular_pos = false;
        self.cam_offset = Vector3::default();
        self.cam_rel = Vector3::default();
        self.next_at = None;
    }

    fn schedule(&mut self, now: Instant) {
        let millis = AIM_MIN_MILLIS + self.next_u64() % (AIM_MAX_MILLIS - AIM_MIN_MILLIS + 1);
        self.next_at = Some(now + Duration::from_millis(millis));
    }

    /// `genAimSyncInfo`: camera ~2 units behind the bot per its yaw, plus a random offset, looking
    /// back at the bot with small jitter.
    fn generate(&mut self, pos: Vector3, rotation: Quaternion, in_vehicle: bool, is_static: bool) {
        if !is_static {
            self.cam_offset = self.random_vector(0.1, 1.5);
        }
        let angle = -yaw_degrees(rotation) * std::f32::consts::PI / 180.0;
        self.cam_rel = Vector3 {
            x: -2.0 * angle.sin() + self.cam_offset.x,
            y: -2.0 * angle.cos() + self.cam_offset.y,
            z: 1.0 + self.cam_offset.z,
        };
        let cam = Vector3 {
            x: pos.x + self.cam_rel.x,
            y: pos.y + self.cam_rel.y,
            z: pos.z + self.cam_rel.z,
        };
        let cam_front = if is_static {
            self.aim.cam_front
        } else {
            let jitter = self.random_vector(0.1, 0.3);
            normalize(Vector3 {
                x: pos.x - cam.x + jitter.x,
                y: pos.y - cam.y + jitter.y,
                z: pos.z - cam.z + jitter.z,
            })
        };
        self.aim = AimData {
            cam_mode: if in_vehicle { 18 } else { 4 },
            cam_pos: cam,
            cam_front,
            ext_zoom: CAM_EXT_ZOOM,
            weapon_state: 0,
        };
    }

    fn next_u64(&mut self) -> u64 {
        // SplitMix64.
        self.rng = self.rng.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.rng;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit_f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    fn random_vector(&mut self, lower: f32, upper: f32) -> Vector3 {
        Vector3 {
            x: self.random_component(lower, upper),
            y: self.random_component(lower, upper),
            z: self.random_component(lower, upper),
        }
    }

    fn random_component(&mut self, lower: f32, upper: f32) -> f32 {
        let magnitude = lower + self.unit_f32() * (upper - lower);
        if self.next_u64() & 1 == 0 {
            -magnitude
        } else {
            magnitude
        }
    }
}

/// Yaw (Z rotation) of the on-foot quaternion, in degrees.
fn yaw_degrees(q: Quaternion) -> f32 {
    (2.0 * (q.w * q.z + q.x * q.y))
        .atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        .to_degrees()
}

fn normalize(v: Vector3) -> Vector3 {
    let len = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
    if len == 0.0 {
        v
    } else {
        Vector3 {
            x: v.x / len,
            y: v.y / len,
            z: v.z / len,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use samp_proto::SyncPacketId;

    #[test]
    fn sends_when_due_then_reschedules() {
        let mut aim = AimSync::new(0xDEAD_BEEF);
        let t0 = Instant::now();
        aim.arm(t0, Vector3::default(), Quaternion::default(), false);
        assert!(!aim.due(t0));
        let later = t0 + Duration::from_secs(2);
        assert!(aim.due(later));
        let packet = aim.encode();
        assert_eq!(packet[0], SyncPacketId::AimSync as u8);
        assert_eq!(packet.len(), 1 + 31);
        assert!(!aim.due(later));
    }

    #[test]
    fn keeps_sending_while_moving_and_camera_follows() {
        let mut aim = AimSync::new(1);
        let t0 = Instant::now();
        aim.arm(t0, Vector3::default(), Quaternion::default(), false);
        let cam0 = aim.aim().cam_pos;
        aim.on_position(Vector3 {
            x: 5.0,
            y: -3.0,
            z: 1.0,
        });
        let cam1 = aim.aim().cam_pos;
        assert!((cam1.x - cam0.x - 5.0).abs() < 1e-4);
        assert!((cam1.y - cam0.y + 3.0).abs() < 1e-4);
        assert!((cam1.z - cam0.z - 1.0).abs() < 1e-4);
        assert!(aim.due(t0 + Duration::from_secs(2)), "sends while moving");
    }

    #[test]
    fn walking_minute_ratio_lands_in_real_iqr() {
        let mut aim = AimSync::new(42);
        let t0 = Instant::now();
        aim.arm(t0, Vector3::default(), Quaternion::default(), false);
        let mut sends = 0u32;
        for tick in 1u32..=600 {
            let now = t0 + Duration::from_millis(u64::from(tick) * 100);
            aim.on_position(Vector3 {
                x: tick as f32 * 0.15,
                y: 0.0,
                z: 3.0,
            });
            if aim.due(now) {
                sends += 1;
            }
        }
        let ratio = f64::from(sends) / 600.0;
        assert!(
            (0.082..=0.121).contains(&ratio),
            "203/207 ratio {ratio} outside real IQR"
        );
    }

    #[test]
    fn reposition_makes_it_due() {
        let mut aim = AimSync::new(2);
        let t0 = Instant::now();
        aim.arm(t0, Vector3::default(), Quaternion::default(), false);
        aim.on_reposition(
            Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            Quaternion::default(),
            false,
        );
        assert!(aim.due(t0), "due right after reposition");
    }

    #[test]
    fn cam_mode_reflects_vehicle_and_ext_zoom_packs() {
        let mut aim = AimSync::new(3);
        let t0 = Instant::now();
        aim.arm(t0, Vector3::default(), Quaternion::default(), true);
        assert_eq!(aim.aim().cam_mode, 18); // in vehicle
        assert_eq!(aim.aim().ext_zoom, CAM_EXT_ZOOM);
        // weapon_state in the high 2 bits, ext_zoom in the low 6. AimSyncData body layout:
        // cam_mode(1) cam_front(12) cam_pos(12) aim_z(4) cam_ext_zoom_weapon_state(1) aspect_ratio(1).
        aim.aim_mut().weapon_state = 2;
        let packet = aim.encode();
        assert_eq!(packet[1 + 29], CAM_EXT_ZOOM | (2 << 6));
    }
}
