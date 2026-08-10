use std::time::{Duration, Instant};

use samp_proto::{BitStreamReader, Quaternion, UnoccupiedSyncData, Vector3, VehicleSyncData};

use crate::world::players::RemotePlayer;

pub(crate) const UNOCCUPIED_SYNC_RANGE: f32 = 90.0;
pub(crate) const UNOCCUPIED_SYNC_MIN_INTERVAL: Duration = Duration::from_millis(100);
pub(crate) const UNOCCUPIED_SYNC_MAX_PER_TICK: usize = 3;
pub(crate) const VEHICLE_DRIVEN_STALE_AFTER: Duration = Duration::from_secs(3);
pub(crate) const VEHICLE_FOREIGN_SYNC_STALE_AFTER: Duration = Duration::from_secs(3);
pub(crate) const REQUIRE_EXACT_ROWS: bool = true;

#[derive(Clone, Copy)]
struct ObservedPose {
    roll: Vector3,
    direction: Vector3,
    health: f32,
    rows_exact: bool,
}

struct TrackedVehicle {
    id: u16,
    position: Vector3,
    pose: Option<ObservedPose>,
    driven_at: Option<Instant>,
    foreign_sync_at: Option<Instant>,
    last_sent_at: Option<Instant>,
}

#[derive(Default)]
pub(crate) struct VehicleTable {
    vehicles: Vec<TrackedVehicle>,
}

impl VehicleTable {
    pub(crate) fn on_world_vehicle_add(&mut self, payload: &[u8]) {
        let mut r = BitStreamReader::new(payload);
        let Ok(id) = r.read_u16() else { return };
        if r.read_u32().is_err() {
            return;
        }
        let Ok(position) = r.read_vector3() else {
            return;
        };
        self.vehicles.retain(|v| v.id != id);
        self.vehicles.push(TrackedVehicle {
            id,
            position,
            pose: None,
            driven_at: None,
            foreign_sync_at: None,
            last_sent_at: None,
        });
    }

    pub(crate) fn on_world_vehicle_remove(&mut self, payload: &[u8]) {
        let mut r = BitStreamReader::new(payload);
        let Ok(id) = r.read_u16() else { return };
        self.vehicles.retain(|v| v.id != id);
    }

    pub(crate) fn on_vehicle_sync(&mut self, sync: &VehicleSyncData, now: Instant) {
        let Some(v) = self.find_mut(sync.vehicle_id) else {
            return;
        };
        let (roll, direction) = quat_rows(sync.quaternion);
        v.position = sync.position;
        v.pose = Some(ObservedPose {
            roll,
            direction,
            health: f32::from(sync.vehicle_health),
            rows_exact: false,
        });
        v.driven_at = Some(now);
    }

    pub(crate) fn on_unoccupied_sync_in(&mut self, sync: &UnoccupiedSyncData, now: Instant) {
        let Some(v) = self.find_mut(sync.vehicle_id) else {
            return;
        };
        v.position = sync.position;
        v.pose = Some(ObservedPose {
            roll: sync.roll,
            direction: sync.direction,
            health: sync.vehicle_health,
            rows_exact: true,
        });
        v.foreign_sync_at = Some(now);
    }

    /// Every tracked vehicle's id and last known position, for the script-facing snapshot.
    pub(crate) fn streamed(&self) -> impl Iterator<Item = (u16, Vector3)> + '_ {
        self.vehicles.iter().map(|v| (v.id, v.position))
    }

    pub(crate) fn clear(&mut self) {
        self.vehicles.clear();
    }

    pub(crate) fn due_unoccupied_sync<'a>(
        &mut self,
        now: Instant,
        our_position: Vector3,
        peers: impl Iterator<Item = &'a RemotePlayer>,
    ) -> Vec<UnoccupiedSyncData> {
        let peer_positions: Vec<Vector3> = peers.map(|p| p.position).collect();
        let mut eligible: Vec<(usize, f32)> = self
            .vehicles
            .iter()
            .enumerate()
            .filter_map(|(i, v)| {
                let d2 = dist2(our_position, v.position);
                let ours = v.pose.is_some_and(|p| p.rows_exact || !REQUIRE_EXACT_ROWS)
                    && d2 <= UNOCCUPIED_SYNC_RANGE * UNOCCUPIED_SYNC_RANGE
                    && quiescent(v.driven_at, now, VEHICLE_DRIVEN_STALE_AFTER)
                    && quiescent(v.foreign_sync_at, now, VEHICLE_FOREIGN_SYNC_STALE_AFTER)
                    && quiescent(v.last_sent_at, now, UNOCCUPIED_SYNC_MIN_INTERVAL)
                    && peer_positions.iter().all(|p| d2 < dist2(*p, v.position));
                ours.then_some((i, d2))
            })
            .collect();
        eligible.sort_by(|a, b| {
            a.1.total_cmp(&b.1)
                .then(self.vehicles[a.0].id.cmp(&self.vehicles[b.0].id))
        });
        eligible.truncate(UNOCCUPIED_SYNC_MAX_PER_TICK);

        let mut out = Vec::new();
        for (i, _) in eligible {
            let v = &mut self.vehicles[i];
            let Some(pose) = v.pose else { continue };
            v.last_sent_at = Some(now);
            out.push(UnoccupiedSyncData {
                vehicle_id: v.id,
                seat_id: 0,
                roll: pose.roll,
                direction: pose.direction,
                position: v.position,
                move_speed: Vector3::default(),
                turn_speed: Vector3::default(),
                vehicle_health: pose.health,
            });
        }
        out
    }

    fn find_mut(&mut self, id: u16) -> Option<&mut TrackedVehicle> {
        self.vehicles.iter_mut().find(|v| v.id == id)
    }
}

fn quiescent(mark: Option<Instant>, now: Instant, hold: Duration) -> bool {
    mark.is_none_or(|at| now.duration_since(at) >= hold)
}

fn dist2(a: Vector3, b: Vector3) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    dx * dx + dy * dy + dz * dz
}

fn quat_rows(q: Quaternion) -> (Vector3, Vector3) {
    let Quaternion { x, y, z, w } = q;
    let roll = Vector3 {
        x: 1.0 - 2.0 * (y * y + z * z),
        y: 2.0 * (x * y + w * z),
        z: 2.0 * (x * z - w * y),
    };
    let direction = Vector3 {
        x: 2.0 * (x * y - w * z),
        y: 1.0 - 2.0 * (x * x + z * z),
        z: 2.0 * (y * z + w * x),
    };
    (roll, direction)
}

#[cfg(test)]
mod tests {
    use super::*;

    use samp_proto::BitStreamWriter;

    const HOLD: Duration = VEHICLE_FOREIGN_SYNC_STALE_AFTER;

    fn vec3(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x, y, z }
    }

    fn no_peers() -> std::iter::Empty<&'static RemotePlayer> {
        std::iter::empty()
    }

    fn add(table: &mut VehicleTable, id: u16, pos: Vector3) {
        let mut w = BitStreamWriter::new();
        w.write_u16(id);
        w.write_u32(457);
        w.write_vector3(pos);
        table.on_world_vehicle_add(&w.into_bytes());
    }

    fn remove(table: &mut VehicleTable, id: u16) {
        let mut w = BitStreamWriter::new();
        w.write_u16(id);
        table.on_world_vehicle_remove(&w.into_bytes());
    }

    fn foreign_209(id: u16, pos: Vector3) -> UnoccupiedSyncData {
        UnoccupiedSyncData {
            vehicle_id: id,
            seat_id: 0,
            roll: vec3(0.0, 1.0, 0.0),
            direction: vec3(-1.0, 0.0, 0.0),
            position: pos,
            move_speed: vec3(0.02, 0.0, 0.0),
            turn_speed: vec3(0.0, 0.0, 0.01),
            vehicle_health: 970.0,
        }
    }

    fn peer(id: u16, pos: Vector3, now: Instant) -> RemotePlayer {
        RemotePlayer {
            id,
            team: None,
            model: None,
            position: pos,
            move_speed: Vector3::default(),
            seen_at: now,
        }
    }

    fn taught_table(id: u16, pos: Vector3, t0: Instant) -> VehicleTable {
        let mut table = VehicleTable::default();
        add(&mut table, id, pos);
        table.on_unoccupied_sync_in(&foreign_209(id, pos), t0);
        table
    }

    #[test]
    fn add_alone_never_sends() {
        let t0 = Instant::now();
        let mut table = VehicleTable::default();
        add(&mut table, 686, vec3(-525.5, -185.3, 78.3));
        let due = table.due_unoccupied_sync(t0 + HOLD, vec3(-524.0, -185.0, 78.0), no_peers());
        assert!(due.is_empty());
    }

    #[test]
    fn foreign_209_teaches_pose_then_we_send() {
        let t0 = Instant::now();
        let spawn = vec3(-525.5, -185.3, 78.3);
        let observed = vec3(-524.0, -186.0, 78.2);
        let mut table = VehicleTable::default();
        add(&mut table, 686, spawn);
        table.on_unoccupied_sync_in(&foreign_209(686, observed), t0);

        let us = vec3(-522.0, -185.0, 78.0);
        assert!(table
            .due_unoccupied_sync(t0 + Duration::from_secs(1), us, no_peers())
            .is_empty());

        let due = table.due_unoccupied_sync(t0 + HOLD, us, no_peers());
        assert_eq!(due.len(), 1);
        let sync = due[0];
        assert_eq!(sync.vehicle_id, 686);
        assert_eq!(sync.seat_id, 0);
        assert_eq!(sync.roll, vec3(0.0, 1.0, 0.0));
        assert_eq!(sync.direction, vec3(-1.0, 0.0, 0.0));
        assert_eq!(sync.position, observed);
        assert_eq!(sync.move_speed, Vector3::default());
        assert_eq!(sync.turn_speed, Vector3::default());
        assert_eq!(sync.vehicle_health, 970.0);
    }

    #[test]
    fn closer_or_tied_peer_silences() {
        let t0 = Instant::now();
        let pos = vec3(0.0, 0.0, 0.0);
        let mut table = taught_table(9, pos, t0);
        let now = t0 + HOLD;
        let us = vec3(10.0, 0.0, 0.0);

        let closer = [peer(2, vec3(5.0, 0.0, 0.0), now)];
        assert!(table.due_unoccupied_sync(now, us, closer.iter()).is_empty());

        let tied = [peer(2, vec3(0.0, 10.0, 0.0), now)];
        assert!(table.due_unoccupied_sync(now, us, tied.iter()).is_empty());

        let farther = [peer(2, vec3(20.0, 0.0, 0.0), now)];
        assert_eq!(table.due_unoccupied_sync(now, us, farther.iter()).len(), 1);
    }

    #[test]
    fn driven_vehicle_stays_gated_until_209_reteaches() {
        let t0 = Instant::now();
        let mut table = taught_table(42, vec3(0.0, 0.0, 0.0), t0);
        let parked = vec3(3.0, 4.0, 0.0);
        let driven_at = t0 + HOLD + Duration::from_secs(2);
        table.on_vehicle_sync(
            &VehicleSyncData {
                player_id: 7,
                vehicle_id: 42,
                quaternion: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
                position: parked,
                vehicle_health: 850,
                ..Default::default()
            },
            driven_at,
        );

        let us = vec3(5.0, 5.0, 0.0);
        assert!(table
            .due_unoccupied_sync(driven_at + Duration::from_secs(1), us, no_peers())
            .is_empty());
        assert!(table
            .due_unoccupied_sync(driven_at + VEHICLE_DRIVEN_STALE_AFTER, us, no_peers())
            .is_empty());

        let retaught_at = driven_at + VEHICLE_DRIVEN_STALE_AFTER;
        table.on_unoccupied_sync_in(&foreign_209(42, parked), retaught_at);
        let due = table.due_unoccupied_sync(retaught_at + HOLD, us, no_peers());
        assert_eq!(due.len(), 1);
        assert_eq!(due[0].position, parked);
        assert_eq!(due[0].roll, vec3(0.0, 1.0, 0.0));
        assert_eq!(due[0].direction, vec3(-1.0, 0.0, 0.0));
        assert_eq!(due[0].vehicle_health, 970.0);
    }

    #[test]
    fn quat_rows_identity_and_yaw90() {
        let (roll, direction) = quat_rows(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        });
        assert_eq!(roll, vec3(1.0, 0.0, 0.0));
        assert_eq!(direction, vec3(0.0, 1.0, 0.0));

        let h = std::f32::consts::FRAC_1_SQRT_2;
        let (roll, direction) = quat_rows(Quaternion {
            x: 0.0,
            y: 0.0,
            z: h,
            w: h,
        });
        let close = |v: Vector3, e: Vector3| {
            (v.x - e.x).abs() < 1e-6 && (v.y - e.y).abs() < 1e-6 && (v.z - e.z).abs() < 1e-6
        };
        assert!(close(roll, vec3(0.0, 1.0, 0.0)), "{roll:?}");
        assert!(close(direction, vec3(-1.0, 0.0, 0.0)), "{direction:?}");
    }

    #[test]
    fn out_of_range_silences() {
        let t0 = Instant::now();
        let pos = vec3(0.0, 0.0, 0.0);
        let mut table = taught_table(9, pos, t0);
        let now = t0 + HOLD;

        assert!(table
            .due_unoccupied_sync(now, vec3(90.5, 0.0, 0.0), no_peers())
            .is_empty());
        assert_eq!(
            table
                .due_unoccupied_sync(now, vec3(90.0, 0.0, 0.0), no_peers())
                .len(),
            1
        );
    }

    #[test]
    fn per_vehicle_throttle_100ms() {
        let t0 = Instant::now();
        let mut table = taught_table(9, vec3(0.0, 0.0, 0.0), t0);
        let us = vec3(1.0, 0.0, 0.0);
        let t1 = t0 + HOLD;

        assert_eq!(table.due_unoccupied_sync(t1, us, no_peers()).len(), 1);
        assert!(table
            .due_unoccupied_sync(t1 + Duration::from_millis(50), us, no_peers())
            .is_empty());
        assert_eq!(
            table
                .due_unoccupied_sync(t1 + UNOCCUPIED_SYNC_MIN_INTERVAL, us, no_peers())
                .len(),
            1
        );
    }

    #[test]
    fn caps_at_three_nearest_first() {
        let t0 = Instant::now();
        let mut table = VehicleTable::default();
        for id in 1u16..=5 {
            let pos = vec3(f32::from(id) * 10.0, 0.0, 0.0);
            add(&mut table, id, pos);
            table.on_unoccupied_sync_in(&foreign_209(id, pos), t0);
        }
        let us = vec3(0.0, 0.0, 0.0);
        let t1 = t0 + HOLD;

        let first: Vec<u16> = table
            .due_unoccupied_sync(t1, us, no_peers())
            .iter()
            .map(|s| s.vehicle_id)
            .collect();
        assert_eq!(first, vec![1, 2, 3]);

        let second: Vec<u16> = table
            .due_unoccupied_sync(t1 + UNOCCUPIED_SYNC_MIN_INTERVAL, us, no_peers())
            .iter()
            .map(|s| s.vehicle_id)
            .collect();
        assert_eq!(second, vec![1, 2, 3]);

        let third: Vec<u16> = table
            .due_unoccupied_sync(t1 + Duration::from_millis(150), us, no_peers())
            .iter()
            .map(|s| s.vehicle_id)
            .collect();
        assert_eq!(third, vec![4, 5]);
    }

    #[test]
    fn remove_and_clear_evict() {
        let t0 = Instant::now();
        let now = t0 + HOLD;
        let us = vec3(1.0, 0.0, 0.0);

        let mut table = taught_table(9, vec3(0.0, 0.0, 0.0), t0);
        remove(&mut table, 9);
        assert!(table.due_unoccupied_sync(now, us, no_peers()).is_empty());
        assert!(table.vehicles.is_empty());

        let mut table = taught_table(9, vec3(0.0, 0.0, 0.0), t0);
        table.clear();
        assert!(table.due_unoccupied_sync(now, us, no_peers()).is_empty());
        assert!(table.vehicles.is_empty());
    }

    #[test]
    fn re_add_drops_stale_pose() {
        let t0 = Instant::now();
        let mut table = taught_table(9, vec3(0.0, 0.0, 0.0), t0);
        let moved = vec3(50.0, 0.0, 0.0);
        add(&mut table, 9, moved);
        assert_eq!(table.vehicles.len(), 1);
        assert_eq!(table.vehicles[0].position, moved);
        assert!(table
            .due_unoccupied_sync(t0 + HOLD, vec3(51.0, 0.0, 0.0), no_peers())
            .is_empty());
    }

    #[test]
    fn untracked_and_malformed_inputs_are_ignored() {
        let t0 = Instant::now();
        let mut table = VehicleTable::default();
        table.on_unoccupied_sync_in(&foreign_209(9, Vector3::default()), t0);
        table.on_vehicle_sync(
            &VehicleSyncData {
                vehicle_id: 9,
                ..Default::default()
            },
            t0,
        );
        table.on_world_vehicle_add(&[0x01]);
        table.on_world_vehicle_remove(&[]);
        assert!(table.vehicles.is_empty());
    }
}
