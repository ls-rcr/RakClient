use std::time::{Duration, Instant};

use samp_proto::{PlayerSyncData, Vector3};

pub(crate) const REMOTE_PLAYER_STALE_AFTER: Duration = Duration::from_secs(3);

pub(crate) const REMOTE_PLAYER_CLEARANCE: f32 = 1.2;

/// Vertical half-reach of a ped, for deciding whether a peer shares our floor at all.
pub(crate) const REMOTE_PLAYER_REACH_Z: f32 = 2.0;

/// How far ahead a peer's last reported velocity is still worth believing.
// WHY: `move_speed` is a snapshot of one 1/50 s frame, so extrapolating it over the full staleness
// window (3 s, ~17 u at jog) invents a position rather than predicting one; past this horizon the last
// seen position is the better guess.
const PREDICT_HORIZON: Duration = Duration::from_millis(500);

pub(crate) struct RemotePlayer {
    pub(crate) id: u16,
    /// Team and skin, known only once the server streams the player in (RPC 32). `None` for a player
    /// we inferred from a sync packet alone.
    pub(crate) team: Option<u8>,
    pub(crate) model: Option<i32>,
    pub(crate) position: Vector3,
    pub(crate) move_speed: Vector3,
    pub(crate) seen_at: Instant,
}

impl RemotePlayer {
    /// Where this peer is *now*, dead-reckoned from the last sync: wire `move_speed` is units per
    /// 1/50 s frame, so world velocity is `move_speed * 50`.
    pub(crate) fn predicted_position(&self, now: Instant) -> Vector3 {
        let ahead = now
            .duration_since(self.seen_at)
            .min(PREDICT_HORIZON)
            .as_secs_f32()
            * 50.0;
        Vector3 {
            x: self.position.x + self.move_speed.x * ahead,
            y: self.position.y + self.move_speed.y * ahead,
            z: self.position.z + self.move_speed.z * ahead,
        }
    }
}

#[derive(Default)]
pub(crate) struct RemotePlayers {
    players: Vec<RemotePlayer>,
}

impl RemotePlayers {
    pub(crate) fn update(&mut self, sync: &PlayerSyncData, now: Instant) {
        self.players
            .retain(|p| now.duration_since(p.seen_at) < REMOTE_PLAYER_STALE_AFTER);
        match self.players.iter_mut().find(|p| p.id == sync.player_id) {
            Some(p) => {
                p.position = sync.position;
                p.move_speed = sync.move_speed;
                p.seen_at = now;
            }
            None => self.players.push(RemotePlayer {
                id: sync.player_id,
                team: None,
                model: None,
                position: sync.position,
                move_speed: sync.move_speed,
                seen_at: now,
            }),
        }
    }

    /// The server streamed a player in: identity and an explicit lifetime, rather than existence
    /// inferred from a sync packet that may simply be the first one we happened to catch.
    pub(crate) fn stream_in(
        &mut self,
        id: u16,
        team: u8,
        model: i32,
        position: Vector3,
        now: Instant,
    ) {
        match self.players.iter_mut().find(|p| p.id == id) {
            Some(p) => {
                p.team = Some(team);
                p.model = Some(model);
                p.position = position;
                p.seen_at = now;
            }
            None => self.players.push(RemotePlayer {
                id,
                team: Some(team),
                model: Some(model),
                position,
                move_speed: Vector3::default(),
                seen_at: now,
            }),
        }
    }

    /// The server streamed a player out. They are gone NOW — waiting for the staleness window to
    /// expire would keep routing the walker around someone who is no longer there.
    pub(crate) fn stream_out(&mut self, id: u16) {
        self.players.retain(|p| p.id != id);
    }

    /// Forget everyone. A reconnect lands us in a fresh streaming set, and player ids are reassigned
    /// per session — carrying the old table over routes the walker around players who are not there.
    pub(crate) fn clear(&mut self) {
        self.players.clear();
    }

    pub(crate) fn iter_fresh(&self, now: Instant) -> impl Iterator<Item = &RemotePlayer> {
        self.players
            .iter()
            .filter(move |p| now.duration_since(p.seen_at) < REMOTE_PLAYER_STALE_AFTER)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vec3(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x, y, z }
    }

    fn sync(id: u16, position: Vector3) -> PlayerSyncData {
        PlayerSyncData {
            player_id: id,
            position,
            ..Default::default()
        }
    }

    /// A peer's reported velocity is worth believing for a moment, not for the whole staleness window:
    /// at jog, extrapolating a 3 s old sample puts the ghost 17 u from the player.
    #[test]
    fn predicted_position_dead_reckons_then_stops_believing_the_velocity() {
        let t0 = Instant::now();
        let peer = RemotePlayer {
            id: 7,
            team: None,
            model: None,
            position: vec3(0.0, 0.0, 3.0),
            move_speed: vec3(0.1, 0.0, 0.0),
            seen_at: t0,
        };

        assert_eq!(peer.predicted_position(t0).x, 0.0);
        let quarter = peer.predicted_position(t0 + Duration::from_millis(250));
        assert!((quarter.x - 1.25).abs() < 1e-4, "got {}", quarter.x);

        let capped = peer.predicted_position(t0 + Duration::from_secs(3)).x;
        let horizon = peer.predicted_position(t0 + PREDICT_HORIZON).x;
        assert_eq!(capped, horizon, "extrapolation must stop at the horizon");
    }

    /// Stream-out is authoritative and immediate; the staleness window is only a backstop for players
    /// we never saw streamed in.
    #[test]
    fn stream_in_gives_identity_and_stream_out_removes_at_once() {
        let t0 = Instant::now();
        let mut table = RemotePlayers::default();
        table.stream_in(7, 2, 270, vec3(5.0, 5.0, 3.0), t0);

        let p = table.iter_fresh(t0).next().expect("streamed in");
        assert_eq!((p.id, p.team, p.model), (7, Some(2), Some(270)));

        // A sync for the same player updates the pose without dropping the identity.
        table.update(&sync(7, vec3(6.0, 5.0, 3.0)), t0);
        let p = table.iter_fresh(t0).next().expect("still there");
        assert_eq!(p.position.x, 6.0);
        assert_eq!(p.team, Some(2));

        table.stream_out(7);
        assert_eq!(table.iter_fresh(t0).count(), 0);
    }

    #[test]
    fn insert_refresh_evict() {
        let t0 = Instant::now();
        let mut table = RemotePlayers::default();
        table.update(&sync(1, vec3(10.0, 0.0, 3.0)), t0);
        table.update(&sync(2, vec3(20.0, 0.0, 3.0)), t0);
        assert_eq!(table.iter_fresh(t0).count(), 2);

        let t1 = t0 + Duration::from_secs(2);
        table.update(&sync(1, vec3(11.0, 0.0, 3.0)), t1);
        assert_eq!(table.iter_fresh(t1).count(), 2);

        let t2 = t0 + Duration::from_secs(4);
        let fresh: Vec<_> = table.iter_fresh(t2).collect();
        assert_eq!(fresh.len(), 1);
        assert_eq!(fresh[0].id, 1);
        assert_eq!(fresh[0].position.x, 11.0);

        table.update(&sync(3, vec3(0.0, 0.0, 0.0)), t2);
        assert_eq!(table.players.len(), 2);
        assert!(table.players.iter().all(|p| p.id != 2));
    }
}
