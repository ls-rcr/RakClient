//! `navgen` — build an on-foot navmesh for a SA-world region and write it as a `.nav` file; see docs/memory/sa-nav/navgen.md#module-overview

use std::collections::HashMap;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 3 {
        eprintln!("usage: navgen <gta3.img> <data-dir> <out.nav> [objects.csv] [cx cy half]");
        std::process::exit(2);
    }
    let (img, data_dir, out_path) = (&args[0], &args[1], &args[2]);
    let csv = args.get(3).filter(|s| s.ends_with(".csv"));
    let tail: Vec<f32> = args
        .iter()
        .skip(if csv.is_some() { 4 } else { 3 })
        .filter_map(|s| s.parse().ok())
        .collect();
    let (cx, cy, half) = match tail.as_slice() {
        [a, b, c] => (*a, *b, *c),
        _ => (-520.0, -190.0, 400.0), // default: the sawmill region
    };

    // NAV_VISUAL_UPGRADE=1 is a one-off diagnostic escape hatch (see docs/memory/sa-nav/navgen.md):
    // swaps sparse/missing custom-prop collision for its dense render mesh, INCLUDING vegetation
    // canopies. Do not ship a .nav baked this way without re-verifying the NoGoVolume tree carving
    // still leaves the ~2.7 m chop spots walkable — canopies are much wider than a trunk.
    let visual_upgrade = std::env::var("NAV_VISUAL_UPGRADE").is_ok_and(|v| v == "1");
    eprintln!(
        "assembling world (visual render-mesh upgrade: {})…",
        visual_upgrade
    );
    let (models, instances) = match sa_map::load::assemble_world_opts(img, data_dir, visual_upgrade)
    {
        Ok(v) => v,
        Err(e) => {
            eprintln!("world load failed: {e}");
            std::process::exit(1);
        }
    };
    let mut mesh = sa_map::world::build(&models, &instances, Some(0));

    // Streamed-objects overlay (server CreateObjects mined from a pcap) — part of
    // the world the bot collides with, so part of the navmesh input.
    if let Some(csv) = csv {
        let ide_map = sa_map::load::load_ide_map(data_dir);
        let objects = load_objects_csv(csv);
        let overlay = sa_map::world::place_objects(&models, &ide_map, &objects);
        let base = mesh.positions.len() as u32;
        mesh.positions.extend_from_slice(&overlay.positions);
        mesh.indices
            .extend(overlay.indices.iter().map(|i| i + base));
    }

    // Cull to the region, drop unreferenced verts, convert SA -> Y-up.
    let (verts, tris) = cull_region(&mesh, cx, cy, half);
    eprintln!(
        "region [{:.0}±{half:.0}, {:.0}±{half:.0}]: {} tris ({} verts) of {} total",
        cx,
        cy,
        tris.len(),
        verts.len(),
        mesh.indices.len() / 3,
    );
    if tris.is_empty() {
        eprintln!("no geometry in region");
        std::process::exit(1);
    }

    let mut cfg = sa_nav::onfoot_config();
    // Bake tunables overridable from the environment for experiments (obstacle carving
    // vs terrain fragmentation) without recompiling: e.g. NAV_CLIMB=0.4 carves a low
    // коряга/stump the default 0.9 m climb steps over.
    let envf = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<f32>().ok());
    if let Some(v) = envf("NAV_CLIMB") {
        cfg.agent_max_climb = v;
    }
    if let Some(v) = envf("NAV_RADIUS") {
        cfg.agent_radius = v;
    }
    if let Some(v) = envf("NAV_CELL") {
        cfg.cell_size = v;
    }
    if let Some(v) = envf("NAV_SLOPE") {
        cfg.walkable_slope_angle = v;
    }
    eprintln!(
        "building navmesh (cell {} m, agent r={} h={} climb={} slope={}°, tiled 64 m)…",
        cfg.cell_size,
        cfg.agent_radius,
        cfg.agent_height,
        cfg.agent_max_climb,
        cfg.walkable_slope_angle,
    );
    // Tree keep-outs: the choppable Arizona firs (fir_dark_*) and most vegetation carry empty or
    // sphere-only collision, so their trunks never rasterize into the bake and the walker routes a
    // straight line through them (the "walking into trees" bug). Stamp a tight NOT_WALKABLE prism
    // around each trunk so paths detour and the walker's off-mesh clamp parks the bot at the trunk
    // edge. Radius stays trunk-sized so the ~2.7 m chop spots stay on walkable mesh and reachable.
    // NAV_TREE_CARVE=0 disables; NAV_TREE_R scales every radius (tune the trunk/clearance tradeoff).
    let mut nogo: Vec<navmesh_recast::NoGoVolume> = Vec::new();
    let by_name: HashMap<String, &sa_map::ColModel> = models
        .iter()
        .map(|m| (m.name.to_ascii_lowercase(), m))
        .collect();
    let in_region = |p: sa_map::Vec3, r: f32| {
        (p.x - cx).abs() <= half + r + 8.0 && (p.y - cy).abs() <= half + r + 8.0
    };
    if std::env::var("NAV_TREE_CARVE")
        .map(|v| v != "0")
        .unwrap_or(true)
    {
        let tree_scale = envf("NAV_TREE_R").unwrap_or(1.0);
        // Base-world IPL vegetation (outdoor interior only).
        for inst in &instances {
            if inst.interior != 0 {
                continue;
            }
            let name = inst.model_name.to_ascii_lowercase();
            let r = veg_keepout_radius(&name).or_else(|| {
                // Sphere-only collision — exactly the case `world::build` silently drops. Carve its
                // sphere so the obstacle exists at all, clamped so a fat bounding sphere can't seal a path.
                by_name.get(&name).and_then(|m| {
                    (m.faces.is_empty() && m.boxes.is_empty() && !m.spheres.is_empty()).then(|| {
                        m.spheres
                            .iter()
                            .map(|s| s.radius)
                            .fold(0.0f32, f32::max)
                            .clamp(0.4, 1.5)
                    })
                })
            });
            if let Some(r) = r {
                // Ceiling 2.0 m so radius + ~0.5 m erosion stays under the ~2.7 m chop-spot offset even
                // with an aggressive NAV_TREE_R — a bigger keep-out would swallow the spots.
                let r = (r * tree_scale).clamp(0.2, 2.0);
                if in_region(inst.position, r) {
                    nogo.push(veg_prism(inst.position, r));
                }
            }
        }
        // Streamed CreateObjects (objects.csv) — the sawmill's choppable firs live here, not in the IPLs.
        if let Some(csv) = csv {
            let ide_map = sa_map::load::load_ide_map(data_dir);
            for (id, pos, _euler) in load_objects_csv(csv) {
                if let Some(r) = ide_map.get(&id).and_then(|name| veg_keepout_radius(name)) {
                    // Ceiling 2.0 m so radius + ~0.5 m erosion stays under the ~2.7 m chop-spot offset even
                    // with an aggressive NAV_TREE_R — a bigger keep-out would swallow the spots.
                    let r = (r * tree_scale).clamp(0.2, 2.0);
                    if in_region(pos, r) {
                        nogo.push(veg_prism(pos, r));
                    }
                }
            }
        }
    }
    let trunks = nogo.len();
    // Structures whose ROOF bakes as the only walkable level at their footprint while the surface
    // players actually stand on does not: at the sawmill the walkable mesh lands on saw_mill's upper
    // deck (~82.5) though real players stand ~3 m lower on the fir_dark_logs2 piles, whose cylindrical
    // crowns fail the slope filter and whose log-to-log gaps exceed agent_max_climb. Paths to chop
    // spots inside the footprint then ground on that phantom roof and the remote hard-snaps every
    // frame (measured: ~16 of 512 real legs). Null the whole column so nothing can be grounded there;
    // targets inside clamp to the footprint edge like a carved trunk. NAV_STRUCTURE_CARVE=0 disables.
    if std::env::var("NAV_STRUCTURE_CARVE")
        .map(|v| v != "0")
        .unwrap_or(true)
    {
        for inst in &instances {
            if inst.interior != 0
                || !PHANTOM_TOP_STRUCTURES.contains(&inst.model_name.to_ascii_lowercase().as_str())
            {
                continue;
            }
            if let Some(model) = by_name.get(&inst.model_name.to_ascii_lowercase()).copied() {
                if in_region(inst.position, model.bound_radius) {
                    nogo.push(structure_prism(inst, model));
                }
            }
        }
    }
    eprintln!(
        "keep-outs: {trunks} trunk prisms + {} structure footprints stamped",
        nogo.len() - trunks
    );

    let t0 = std::time::Instant::now();
    // SA_NAV_SOLO=1 forces the single-tile pipeline — the A/B knob for isolating
    // cross-tile seam-linking problems from genuine geometry fragmentation.
    let solo = std::env::var("SA_NAV_SOLO").is_ok_and(|v| v == "1");
    let built = if solo {
        navmesh_recast::build_rerecast(&verts, &tris, &[], &nogo, &cfg)
    } else {
        navmesh_recast::build_rerecast_tiled(&verts, &tris, &[], &nogo, &cfg, 64.0)
    };
    let mut nav = match built {
        Ok(n) => n,
        Err(e) => {
            eprintln!("navmesh build failed: {e}");
            std::process::exit(1);
        }
    };
    let built = t0.elapsed();
    let (kept, dropped, comps) = nav.retain_largest_component();
    eprintln!(
        "built in {:.1?}: {} polys kept, {} dropped ({} unreachable components: roofs/pockets), {} detail tris",
        built,
        kept,
        dropped,
        comps,
        nav.detail_tris.len(),
    );

    let sa = sa_nav::NavMesh::from_recast(&nav);
    let mut f = match std::fs::File::create(out_path) {
        Ok(f) => std::io::BufWriter::new(f),
        Err(e) => {
            eprintln!("cannot create {out_path}: {e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = sa.save(&mut f) {
        eprintln!("write failed: {e}");
        std::process::exit(1);
    }
    eprintln!("wrote {out_path}");
}

/// Trunk keep-out radius (m) for a vegetation model by name, or `None` for non-vegetation.
/// The Arizona choppable firs (`fir_dark_*`) and most trees carry empty or sphere-only collision,
/// so their trunks never rasterize into the bake — carve a small NOT_WALKABLE prism instead.
/// Bushes/plants/grass have no in-game trunk collision (peds walk through them) — never carved.
fn veg_keepout_radius(name: &str) -> Option<f32> {
    let n = name.to_ascii_lowercase();
    // Exclusions first (they win). Two guards: (a) real ground cover with no trunk collision, and
    // (b) substrings that would falsely trip the tree arms below — "street" contains "tree",
    // "fire"/"firehydrant" contains "fir" (note "fir_dark" does NOT contain "fire"), "lamp" for
    // lampposts. Without these, streetlights and hydrants get carved as trees.
    if [
        "bush", "plant", "flower", "grass", "fern", "weed", "shrub", "leaf", "street", "fire",
        "hydrant", "lamp",
    ]
    .iter()
    .any(|s| n.contains(s))
    {
        return None;
    }
    // Radius = visual trunk + a walking-clearance margin. The runtime funnel hugs the eroded hole
    // boundary (carve radius + ~0.5 m agent erosion), so this is what sets how far the path bows around
    // a trunk. Kept under ~1.5 m so the ~2.7 m-offset chop spots stay on walkable mesh.
    if n.contains("fir_dark_small") {
        return Some(0.7);
    }
    if n.contains("fir_dark_big") {
        return Some(1.3);
    }
    if ["fir", "pine", "redwood", "cedar", "spruce"]
        .iter()
        .any(|s| n.contains(s))
    {
        return Some(1.1);
    }
    if n.contains("palm") {
        return Some(0.9);
    }
    if ["tree", "oak", "birch", "veg_"]
        .iter()
        .any(|s| n.contains(s))
    {
        return Some(1.0);
    }
    None
}

/// Collision-model names (lowercase) whose baked walkable top is a surface no player stands on; see
/// the `NAV_STRUCTURE_CARVE` block for why the sawmill qualifies.
const PHANTOM_TOP_STRUCTURES: &[&str] = &["saw_mill"];

/// How far above a carved structure's base its real ground floor reaches. The keep-out starts here, so
/// the walkable ground inside the footprint survives while every phantom level above it (upper deck,
/// roof, and the log piles stacked against it) is nulled. Carving the whole column instead cost 110 of
/// 810 real legs their path; starting the band above the ground floor keeps them reachable.
const STRUCTURE_GROUND_BAND: f32 = 2.0;

/// A no-go prism (recast space) over an instance's whole collision footprint: the model's bounding box
/// corners taken to world space, the XY hull kept as a rectangle and the vertical span bracketed so the
/// column is nulled from below the base to above the roof. `sa_to_recast` maps `(x, y, z) -> (x, z, -y)`,
/// so the footprint is recast `(x, -y)` and the span is SA `z`; the winding matches [`veg_prism`].
fn structure_prism(
    inst: &sa_map::Instance,
    model: &sa_map::ColModel,
) -> navmesh_recast::NoGoVolume {
    let (lo, hi) = (model.bound_min, model.bound_max);
    let (mut min_x, mut max_x) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut min_y, mut max_y) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut min_z, mut max_z) = (f32::INFINITY, f32::NEG_INFINITY);
    for &(x, y, z) in &[
        (lo.x, lo.y, lo.z),
        (hi.x, lo.y, lo.z),
        (hi.x, hi.y, lo.z),
        (lo.x, hi.y, lo.z),
        (lo.x, lo.y, hi.z),
        (hi.x, lo.y, hi.z),
        (hi.x, hi.y, hi.z),
        (lo.x, hi.y, hi.z),
    ] {
        let w = inst.to_world(sa_map::Vec3::new(x, y, z));
        min_x = min_x.min(w.x);
        max_x = max_x.max(w.x);
        min_y = min_y.min(w.y);
        max_y = max_y.max(w.y);
        min_z = min_z.min(w.z);
        max_z = max_z.max(w.z);
    }
    navmesh_recast::NoGoVolume {
        xz: vec![
            [min_x, -max_y],
            [max_x, -max_y],
            [max_x, -min_y],
            [min_x, -min_y],
        ],
        min_y: min_z + STRUCTURE_GROUND_BAND,
        max_y: max_z + 2.0,
    }
}

/// A vertical octagonal no-go prism (recast space) around a tree trunk at SA `pos`.
/// `sa_to_recast` maps `(x, y, z) -> (x, z, -y)`, so the horizontal footprint is recast `(x, -y)`
/// and the vertical span is SA `z`. The span brackets the trunk generously so the ground under it
/// is nulled regardless of where the model origin sits relative to the terrain.
fn veg_prism(pos: sa_map::Vec3, radius: f32) -> navmesh_recast::NoGoVolume {
    let (cx, cz) = (pos.x, -pos.y);
    let xz = (0..8)
        .map(|k| {
            let a = k as f32 * std::f32::consts::FRAC_PI_4;
            [cx + radius * a.cos(), cz + radius * a.sin()]
        })
        .collect();
    navmesh_recast::NoGoVolume {
        xz,
        min_y: pos.z - 2.0,
        max_y: pos.z + 6.0,
    }
}

/// Read a streamed-objects CSV (`model_id,x,y,z,rx,ry,rz`).
fn load_objects_csv(path: &str) -> Vec<(i32, sa_map::Vec3, sa_map::Vec3)> {
    let Ok(text) = std::fs::read_to_string(path) else {
        eprintln!("could not read objects csv {path}");
        return Vec::new();
    };
    let mut out = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 7 {
            continue;
        }
        let p: Vec<f32> = f[1..7]
            .iter()
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        if let (Ok(id), 6) = (f[0].trim().parse::<i32>(), p.len()) {
            out.push((
                id,
                sa_map::Vec3::new(p[0], p[1], p[2]),
                sa_map::Vec3::new(p[3], p[4], p[5]),
            ));
        }
    }
    out
}

/// Keep triangles whose SA-xy bbox intersects the region square, compact verts, convert to recast Y-up, and normalise winding up; see docs/memory/sa-nav/navgen.md#cull-region
fn cull_region(mesh: &sa_map::Mesh, cx: f32, cy: f32, half: f32) -> (Vec<[f32; 3]>, Vec<[u32; 3]>) {
    let (x0, x1, y0, y1) = (cx - half, cx + half, cy - half, cy + half);
    let mut remap: HashMap<u32, u32> = HashMap::new();
    let mut verts: Vec<[f32; 3]> = Vec::new();
    let mut tris: Vec<[u32; 3]> = Vec::new();
    for t in mesh.indices.chunks_exact(3) {
        let p: Vec<[f32; 3]> = t.iter().map(|&i| mesh.positions[i as usize]).collect();
        if p.iter().all(|v| v[0] < x0)
            || p.iter().all(|v| v[0] > x1)
            || p.iter().all(|v| v[1] < y0)
            || p.iter().all(|v| v[1] > y1)
        {
            continue;
        }
        let mut lt = [0u32; 3];
        for (k, &gi) in t.iter().enumerate() {
            lt[k] = *remap.entry(gi).or_insert_with(|| {
                let idx = verts.len() as u32;
                verts.push(sa_nav::sa_to_recast(mesh.positions[gi as usize]));
                idx
            });
        }
        // Orient up in recast space: flip if the normal's Y is negative.
        let (a, b, c) = (
            verts[lt[0] as usize],
            verts[lt[1] as usize],
            verts[lt[2] as usize],
        );
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let ny = u[2] * v[0] - u[0] * v[2];
        if ny < 0.0 {
            lt.swap(1, 2);
        }
        tris.push(lt);
    }
    (verts, tris)
}
