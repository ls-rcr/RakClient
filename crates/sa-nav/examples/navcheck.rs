//! Ad-hoc navmesh probe: load a `.nav` and, for each probe arg, report whether a point is on
//! walkable mesh (`locate`/`floor`) or whether a straight A→B segment is forced to detour.
//!
//!   cargo run --example navcheck -p sa-nav -- <file.nav> <-x,-y,-z> ... seg:ax,ay,az,bx,by,bz
//!
//! Used to verify tree keep-outs: a carved trunk centre should read `off-mesh`, and a segment
//! driven straight through it should bow out (path length > straight-line distance).

use sa_nav::{NavMesh, NavQuery};

fn parse(s: &str) -> Vec<f32> {
    s.split(',').filter_map(|v| v.trim().parse().ok()).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(nav_path) = args.first() else {
        eprintln!("usage: navcheck <file.nav> <x,y,z|seg:ax,ay,az,bx,by,bz> ...");
        std::process::exit(2);
    };
    let mut f = std::io::BufReader::new(std::fs::File::open(nav_path).expect("open nav"));
    let q = NavQuery::new(NavMesh::load(&mut f).expect("load nav"));

    for a in &args[1..] {
        if let Some(rest) = a.strip_prefix("seg:") {
            let c = parse(rest);
            if c.len() < 6 {
                continue;
            }
            let (from, to) = ([c[0], c[1], c[2]], [c[3], c[4], c[5]]);
            let straight = {
                let d = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            match q.find_path(from, to) {
                Some(path) => {
                    let mut len = 0.0f32;
                    let mut prev = from;
                    for p in &path {
                        let d = [p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]];
                        len += (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                        prev = *p;
                    }
                    println!(
                        "seg {from:?}->{to:?}: {} wp, path_len={len:.2} straight={straight:.2} detour=+{:.2}m",
                        path.len(),
                        len - straight
                    );
                    let zmin = path.iter().fold(f32::INFINITY, |m, p| m.min(p[2]));
                    let zmax = path.iter().fold(f32::NEG_INFINITY, |m, p| m.max(p[2]));
                    println!("    z range {zmin:.2}..{zmax:.2}");
                    for (i, p) in path.iter().enumerate() {
                        println!("    wp{i:<3} {:9.2} {:9.2} {:8.2}", p[0], p[1], p[2]);
                    }
                }
                None => println!("seg {from:?}->{to:?}: NO PATH"),
            }
        } else if let Some(rest) = a.strip_prefix("vseg:") {
            // Like seg:, but runs find_path_with_variety across several seeds and confirms every waypoint
            // stays strictly on-mesh (contained, not just locate's off-mesh-tolerant snap) and that the
            // routes actually differ between seeds.
            let c = parse(rest);
            if c.len() < 6 {
                continue;
            }
            let (from, to) = ([c[0], c[1], c[2]], [c[3], c[4], c[5]]);
            let m = q.mesh();
            let contains_xy = |pt: [f32; 3]| -> bool {
                (0..m.polys.len()).any(|pi| {
                    let poly = &m.polys[pi];
                    let n = poly.verts.len();
                    if n < 3 {
                        return false;
                    }
                    let mut sign = 0.0f32;
                    for j in 0..n {
                        let va = m.verts[poly.verts[j] as usize];
                        let vb = m.verts[poly.verts[(j + 1) % n] as usize];
                        let cr =
                            (vb[0] - va[0]) * (pt[1] - va[1]) - (vb[1] - va[1]) * (pt[0] - va[0]);
                        if cr.abs() < 1e-6 {
                            continue;
                        }
                        if sign == 0.0 {
                            sign = cr.signum();
                        } else if cr.signum() != sign {
                            return false;
                        }
                    }
                    true
                })
            };
            let mut seen_lens = Vec::new();
            for seed in 0..5u64 {
                match q.find_path_with_variety(from, to, seed * 7919 + 17) {
                    Some(path) => {
                        let off_mesh = path.iter().filter(|p| !contains_xy(**p)).count();
                        let len: f32 = {
                            let mut prev = from;
                            let mut total = 0.0;
                            for p in &path {
                                let d = [p[0] - prev[0], p[1] - prev[1], p[2] - prev[2]];
                                total += (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                                prev = *p;
                            }
                            total
                        };
                        seen_lens.push(len);
                        println!(
                            "vseg seed={seed}: {} wp, len={len:.2}, off-mesh waypoints={off_mesh} (0 expected)",
                            path.len()
                        );
                    }
                    None => println!("vseg seed={seed}: NO PATH"),
                }
            }
            let all_same = seen_lens.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-3);
            println!(
                "vseg summary: {} seeds, lengths {}",
                seen_lens.len(),
                if all_same {
                    "IDENTICAL (no variety!)"
                } else {
                    "vary (expected)"
                }
            );
        } else if let Some(rest) = a.strip_prefix("reach:") {
            // Path to the target from 12 m due east and report how far short of it the path stops
            // (a carved trunk clamps the endpoint to the hole edge).
            let c = parse(rest);
            if c.len() < 3 {
                continue;
            }
            let to = [c[0], c[1], c[2]];
            let from = [c[0] + 12.0, c[1], c[2]];
            match q.find_path(from, to) {
                Some(path) => {
                    let end = *path.last().unwrap();
                    let d = ((end[0] - to[0]).powi(2) + (end[1] - to[1]).powi(2)).sqrt();
                    println!("reach {to:?}: path stops {d:.2} m from target (end {end:?})");
                }
                None => println!("reach {to:?}: NO PATH"),
            }
        } else {
            let c = parse(a);
            if c.len() < 3 {
                continue;
            }
            let pt = [c[0], c[1], c[2]];
            // True containment: scan every poly for one that strictly contains (x, y) in XY. A carved
            // trunk hole shows `contained=false` even though `locate` snaps to the ring via its fallback.
            let m = q.mesh();
            let contained = (0..m.polys.len()).any(|pi| {
                let poly = &m.polys[pi];
                let n = poly.verts.len();
                if n < 3 {
                    return false;
                }
                let mut sign = 0.0f32;
                for j in 0..n {
                    let va = m.verts[poly.verts[j] as usize];
                    let vb = m.verts[poly.verts[(j + 1) % n] as usize];
                    let cr = (vb[0] - va[0]) * (pt[1] - va[1]) - (vb[1] - va[1]) * (pt[0] - va[0]);
                    if cr.abs() < 1e-6 {
                        continue;
                    }
                    if sign == 0.0 {
                        sign = cr.signum();
                    } else if cr.signum() != sign {
                        return false;
                    }
                }
                true
            });
            println!(
                "point {pt:?}: contained={contained} locate={:?} floor={:?}",
                q.locate(pt).is_some(),
                q.floor(pt)
            );
        }
    }
}
