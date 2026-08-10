//! Streamed objects the static navmesh bake never saw.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use samp_proto::Vector3;

/// A streamed `CreateObject` the static navmesh bake has no idea about (it was baked once from a mined
/// pcap snapshot) — another player's dropped cart, a parked vehicle, anything placed after that
/// snapshot or that moves.
struct TrackedObject {
    position: Vector3,
    seen_at: Instant,
}

/// Drop a tracked object if we haven't seen a Create/Move update for this long — defends against a
/// missed DestroyObject (e.g. a disconnect edge case) leaving a stale entry forever.
const STALE_AFTER: Duration = Duration::from_secs(180);

/// How close a path segment may pass to a tracked object before we route around it.
pub(crate) const CLEARANCE: f32 = 1.5;

#[derive(Default)]
pub(crate) struct ObjectPool {
    objects: HashMap<u16, TrackedObject>,
}

impl ObjectPool {
    pub(crate) fn track(&mut self, id: u16, position: Vector3, now: Instant) {
        self.objects.insert(
            id,
            TrackedObject {
                position,
                seen_at: now,
            },
        );
    }

    pub(crate) fn forget(&mut self, id: u16) {
        self.objects.remove(&id);
    }

    pub(crate) fn clear(&mut self) {
        self.objects.clear();
    }

    /// Positions still worth avoiding. Staleness is filtered on read rather than pruned here: removal
    /// stays event-driven (`DestroyObject`), and this is the defensive backstop for a missed one.
    pub(crate) fn live(&self, now: Instant) -> impl Iterator<Item = Vector3> + '_ {
        self.objects
            .values()
            .filter(move |o| now.duration_since(o.seen_at) < STALE_AFTER)
            .map(|o| o.position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_updates_and_forgets() {
        let now = Instant::now();
        let mut pool = ObjectPool::default();
        let at = |x: f32| Vector3 { x, y: 0.0, z: 5.0 };

        pool.track(1, at(10.0), now);
        pool.track(2, at(20.0), now);
        assert_eq!(pool.live(now).count(), 2);

        // A move is an update of the same id, not a second entry.
        pool.track(1, at(11.0), now);
        assert_eq!(pool.live(now).count(), 2);
        assert!(pool.live(now).any(|p| p.x == 11.0));

        pool.forget(1);
        assert_eq!(pool.live(now).count(), 1);

        // A missed DestroyObject must not leave the entry blocking paths forever.
        assert_eq!(pool.live(now + STALE_AFTER).count(), 0);

        pool.clear();
        assert_eq!(pool.live(now).count(), 0);
    }
}
