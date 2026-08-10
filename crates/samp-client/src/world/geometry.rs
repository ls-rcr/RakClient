//! Pure path geometry: no state, no clock, no I/O. Everything here is a function of its arguments, so
//! the walker's routing decisions can be exercised without a body, a navmesh or an engine.

use samp_proto::Vector3;

/// The closest point on segment `a -> b` (XY only) to `p`, and the XY distance to it.
pub(crate) fn closest_point_on_segment(a: [f32; 3], b: [f32; 3], p: [f32; 3]) -> ([f32; 3], f32) {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 1e-6 {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let c = [a[0] + dx * t, a[1] + dy * t, a[2] + (b[2] - a[2]) * t];
    let (ex, ey) = (p[0] - c[0], p[1] - c[1]);
    (c, (ex * ex + ey * ey).sqrt())
}

/// If `obstacle` sits within `clearance` of segment `a -> b` and within `reach_z` of it vertically, a
/// bypass point at the segment's closest approach, pushed perpendicular AWAY from the obstacle by just
/// enough (+0.5 m margin) to clear it — `None` if the segment is already clear, if the obstacle is on
/// another floor, or if it sits exactly on the line (a degenerate case with no defined push direction,
/// left to normal path handling rather than an arbitrary sideways guess).
pub(crate) fn bypass_point(
    a: [f32; 3],
    b: [f32; 3],
    obstacle: [f32; 3],
    clearance: f32,
    reach_z: f32,
) -> Option<[f32; 3]> {
    let (closest, dist) = closest_point_on_segment(a, b, obstacle);
    if dist >= clearance || dist < 1e-3 || (closest[2] - obstacle[2]).abs() > reach_z {
        return None;
    }
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len = (dx * dx + dy * dy).sqrt().max(1e-3);
    let (px, py) = (-dy / len, dx / len); // unit perpendicular to the segment
    let side = (closest[0] - obstacle[0]) * px + (closest[1] - obstacle[1]) * py;
    let sign = if side >= 0.0 { 1.0 } else { -1.0 };
    let push = (clearance - dist) + 0.5;
    Some([
        closest[0] + px * push * sign,
        closest[1] + py * push * sign,
        closest[2],
    ])
}

/// XY distance still to travel ALONG THE PATH from `pos` through the remaining waypoints.
// WHY: straight-line distance to the target is a different number, and braking on it hangs the walker —
// a corridor that detours around a carve or an obstacle passes within a metre of the target with tens of
// metres left, which brakes the gait to a standstill that no later step can undo.
pub(crate) fn remaining_path_xy(pos: Vector3, waypoints: &[Vector3], next: usize) -> f32 {
    let mut from = pos;
    let mut total = 0.0;
    for wp in waypoints.iter().skip(next) {
        total += ((wp.x - from.x).powi(2) + (wp.y - from.y).powi(2)).sqrt();
        from = *wp;
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vec3(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x, y, z }
    }

    /// A straight walk that passes exactly over an obstacle gets pushed sideways by enough to clear it;
    /// a walk far from the obstacle is left untouched.
    #[test]
    fn bypass_point_routes_around_a_blocking_obstacle() {
        let a = [0.0, 0.0, 5.0];
        let b = [10.0, 0.0, 5.0];
        let obstacle = [5.0, 0.5, 5.0]; // 0.5 m off the line: within clearance, not dead-centre
        let bypass =
            bypass_point(a, b, obstacle, 1.5, f32::INFINITY).expect("obstacle blocks the segment");
        let (_, cleared_dist) = closest_point_on_segment(a, b, obstacle);
        let (dx, dy) = (bypass[0] - obstacle[0], bypass[1] - obstacle[1]);
        let dist_from_obstacle = (dx * dx + dy * dy).sqrt();
        assert!(
            dist_from_obstacle >= 1.5,
            "bypass point should clear the obstacle, got {dist_from_obstacle}"
        );
        assert!(
            cleared_dist < 1.5,
            "sanity: obstacle was actually near the original segment"
        );

        let far_obstacle = [5.0, 50.0, 5.0];
        assert!(bypass_point(a, b, far_obstacle, 1.5, f32::INFINITY).is_none());
    }

    /// An obstacle sitting exactly on the line is a degenerate case (the push direction is undefined at
    /// zero distance) — left to normal path handling rather than an arbitrary sideways guess.
    #[test]
    fn bypass_point_skips_a_dead_centre_obstacle() {
        let a = [0.0, 0.0, 5.0];
        let b = [10.0, 0.0, 5.0];
        assert!(bypass_point(a, b, [5.0, 0.0, 5.0], 1.5, f32::INFINITY).is_none());
    }

    /// Clearance is measured in XY, so without a vertical gate a player on a roof or a floor above
    /// pushes a bypass waypoint into a path they are nowhere near.
    #[test]
    fn bypass_point_ignores_an_obstacle_on_another_floor() {
        let a = [0.0, 0.0, 5.0];
        let b = [10.0, 0.0, 5.0];
        let overhead = [5.0, 0.5, 12.0];
        assert!(bypass_point(a, b, overhead, 1.5, 2.0).is_none());
        assert!(bypass_point(a, b, overhead, 1.5, f32::INFINITY).is_some());
    }

    /// What matters for braking is the walk still ahead, which a detour makes far longer than the
    /// straight line to the same target — the distinction the walker used to get wrong and hang on.
    #[test]
    fn remaining_path_follows_the_corridor_not_the_crow_line() {
        let pos = vec3(18.0, 17.5, 5.0);
        let detour = [vec3(2.0, 18.0, 5.0), vec3(18.0, 18.0, 5.0)];

        let remaining = remaining_path_xy(pos, &detour, 0);
        assert!(
            remaining > 30.0,
            "detour measured {remaining} u, barely more than the 0.5 u crow line"
        );
        assert_eq!(remaining_path_xy(pos, &detour, detour.len()), 0.0);
    }
}
