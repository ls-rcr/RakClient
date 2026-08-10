use std::collections::{HashMap, HashSet};
use std::net::Ipv4Addr;
use std::path::PathBuf;

use anyhow::{bail, Context};
use samp_proto::{
    AimSyncData, BitStreamReader, PlayerSyncData, UnoccupiedSyncData, VehicleSyncData,
};

const CLIENT_IP: Ipv4Addr = Ipv4Addr::new(10, 13, 37, 1);
const ONFOOT_SYNC_ID: u8 = 207;
const AIM_SYNC_ID: u8 = 203;
const VEHICLE_SYNC_ID: u8 = 200;
const UNOCCUPIED_SYNC_ID: u8 = 209;
const RPC_WORLD_VEHICLE_ADD: u8 = 164;
const SPEED_ZERO: f32 = 1e-6;
const KEY_BIT_NAMES: [(u16, &str); 15] = [
    (0x0001, "ACTION"),
    (0x0002, "CROUCH"),
    (0x0004, "FIRE"),
    (0x0008, "SPRINT"),
    (0x0010, "SECONDARY_ATTACK"),
    (0x0020, "JUMP"),
    (0x0040, "LOOK_RIGHT"),
    (0x0080, "AIM"),
    (0x0100, "LOOK_LEFT"),
    (0x0200, "LOOK_BEHIND"),
    (0x0400, "WALK"),
    (0x0800, "ANALOG_UP"),
    (0x1000, "ANALOG_DOWN"),
    (0x2000, "ANALOG_LEFT"),
    (0x4000, "ANALOG_RIGHT"),
];
const VERTICAL_SPEED_CUTOFF: f32 = 0.02;
const FLOOR_CACHE_QUANT_XY: f32 = 0.25;
const FLOOR_CACHE_QUANT_Z: f32 = 2.0;
const BBOX_MARGIN: f32 = 3.0;
const CELL_SIZE: f32 = 2.0;
const MIN_CELL_SAMPLES: usize = 10;
const WORST_CELL_COUNT: usize = 30;
const SAWMILL_MIN: [f32; 2] = [-535.0, -205.0];
const SAWMILL_MAX: [f32; 2] = [-460.0, -140.0];
const LIFT_STEP: f64 = 0.05;
const LIFT_MAX: f64 = 1.0;
const OFFENDER_RESIDUAL: f64 = 1.0;
const TRIM_MIN: f32 = 0.0;
const TRIM_MAX: f32 = 1.5;
const SURFACE_MATCH: f64 = 1.0;
const LEG_STEP: f32 = 0.5;
const LEG_Z_MIN: f32 = 73.0;
const LEG_Z_MAX: f32 = 82.0;
const PHANTOM_Z: f32 = 82.0;

type CellKey = (i64, i64);

struct Sample {
    x: f32,
    y: f32,
    z: f32,
    dz: f32,
}

#[derive(Default)]
struct Counters {
    datagrams: u64,
    inbound: u64,
    onfoot_seen: u64,
    decode_failed: u64,
    nonfinite: u64,
    surfing: u64,
    vertical: u64,
    off_mesh: u64,
    kept: u64,
}

impl Counters {
    fn add(&mut self, other: &Counters) {
        self.datagrams += other.datagrams;
        self.inbound += other.inbound;
        self.onfoot_seen += other.onfoot_seen;
        self.decode_failed += other.decode_failed;
        self.nonfinite += other.nonfinite;
        self.surfing += other.surfing;
        self.vertical += other.vertical;
        self.off_mesh += other.off_mesh;
        self.kept += other.kept;
    }
}

struct FloorSampler {
    query: sa_nav::NavQuery,
    min: [f32; 2],
    max: [f32; 2],
    cache: HashMap<(i64, i64, i64), Option<f32>>,
}

impl FloorSampler {
    fn new(mesh: sa_nav::NavMesh) -> Self {
        let mut min = [f32::INFINITY; 2];
        let mut max = [f32::NEG_INFINITY; 2];
        for v in &mesh.verts {
            for k in 0..2 {
                min[k] = min[k].min(v[k]);
                max[k] = max[k].max(v[k]);
            }
        }
        FloorSampler {
            query: sa_nav::NavQuery::new(mesh),
            min: min.map(|v| v - BBOX_MARGIN),
            max: max.map(|v| v + BBOX_MARGIN),
            cache: HashMap::new(),
        }
    }

    fn floor(&mut self, p: [f32; 3]) -> Option<f32> {
        if p[0] < self.min[0] || p[0] > self.max[0] || p[1] < self.min[1] || p[1] > self.max[1] {
            return None;
        }
        let key = (
            (p[0] / FLOOR_CACHE_QUANT_XY).floor() as i64,
            (p[1] / FLOOR_CACHE_QUANT_XY).floor() as i64,
            (p[2] / FLOOR_CACHE_QUANT_Z).floor() as i64,
        );
        if let Some(&v) = self.cache.get(&key) {
            return v;
        }
        let v = self.query.floor(p);
        self.cache.insert(key, v);
        v
    }
}

fn main() -> anyhow::Result<()> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut targets_path: Option<String> = None;
    if let Some(i) = args.iter().position(|a| a == "--targets") {
        if i + 1 >= args.len() {
            eprintln!("--targets needs a file argument");
            std::process::exit(2);
        }
        targets_path = Some(args.remove(i + 1));
        args.remove(i);
    }
    if args.len() < 2 {
        eprintln!(
            "usage: navcalib <file.nav> [--targets <walkto-targets.txt>] <capture.pcap>...\n       navcalib profile <capture.pcap>..."
        );
        std::process::exit(2);
    }
    if args[0] == "profile" {
        return profile_main(&args[1..]);
    }
    let nav_path = &args[0];
    let pcap_paths = expand_inputs(&args[1..]);
    if pcap_paths.is_empty() {
        bail!("no pcap files matched the given arguments");
    }

    let mut nav_file =
        std::fs::File::open(nav_path).with_context(|| format!("cannot open {nav_path}"))?;
    let mesh =
        sa_nav::NavMesh::load(&mut nav_file).with_context(|| format!("cannot load {nav_path}"))?;
    println!(
        "navmesh {nav_path}: {} polys, {} verts",
        mesh.polys.len(),
        mesh.verts.len()
    );
    let mut sampler = FloorSampler::new(mesh);

    let mut samples: Vec<Sample> = Vec::new();
    let mut players: HashSet<u16> = HashSet::new();
    let mut totals = Counters::default();

    for path in &pcap_paths {
        let bytes = match std::fs::read(path) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("skipping {}: {e}", path.display());
                continue;
            }
        };
        let mut c = Counters::default();
        if let Err(e) = process_pcap(&bytes, &mut sampler, &mut samples, &mut players, &mut c) {
            eprintln!("skipping {}: {e}", path.display());
            continue;
        }
        println!(
            "{}: {} datagrams ({} inbound), {} x207, kept {}",
            path.display(),
            c.datagrams,
            c.inbound,
            c.onfoot_seen,
            c.kept
        );
        if c.onfoot_seen == 0 {
            println!(
                "  WARNING: no inbound 207s in {} — verify with dissect",
                path.display()
            );
        }
        totals.add(&c);
    }

    report(
        &totals,
        &samples,
        &players,
        &mut sampler,
        targets_path.as_deref(),
    )
}

fn expand_inputs(args: &[String]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for a in args {
        let p = PathBuf::from(a);
        if p.is_dir() {
            let mut files: Vec<(u64, PathBuf)> = std::fs::read_dir(&p)
                .into_iter()
                .flatten()
                .flatten()
                .filter(|e| {
                    e.path()
                        .extension()
                        .is_some_and(|x| x.eq_ignore_ascii_case("pcap"))
                })
                .map(|e| (e.metadata().map(|m| m.len()).unwrap_or(0), e.path()))
                .collect();
            files.sort_by_key(|f| std::cmp::Reverse(f.0));
            out.extend(files.into_iter().map(|(_, p)| p));
        } else if a.contains('*') || a.contains('?') {
            let dir = p
                .parent()
                .filter(|d| !d.as_os_str().is_empty())
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from("."));
            let pat = p
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            let mut matched: Vec<PathBuf> = std::fs::read_dir(&dir)
                .into_iter()
                .flatten()
                .flatten()
                .filter(|e| {
                    wildcard_match(pat.as_bytes(), e.file_name().to_string_lossy().as_bytes())
                })
                .map(|e| e.path())
                .collect();
            matched.sort();
            out.extend(matched);
        } else {
            out.push(p);
        }
    }
    out
}

fn wildcard_match(pat: &[u8], s: &[u8]) -> bool {
    let (mut pi, mut si) = (0usize, 0usize);
    let mut star: Option<usize> = None;
    let mut mark = 0usize;
    while si < s.len() {
        if pi < pat.len() && (pat[pi] == b'?' || pat[pi].eq_ignore_ascii_case(&s[si])) {
            pi += 1;
            si += 1;
        } else if pi < pat.len() && pat[pi] == b'*' {
            star = Some(pi);
            mark = si;
            pi += 1;
        } else if let Some(sp) = star {
            pi = sp + 1;
            mark += 1;
            si = mark;
        } else {
            return false;
        }
    }
    while pi < pat.len() && pat[pi] == b'*' {
        pi += 1;
    }
    pi == pat.len()
}

fn process_pcap(
    bytes: &[u8],
    sampler: &mut FloorSampler,
    samples: &mut Vec<Sample>,
    players: &mut HashSet<u16>,
    c: &mut Counters,
) -> anyhow::Result<()> {
    if bytes.len() < 24 || bytes[0..4] != [0xd4, 0xc3, 0xb2, 0xa1] {
        bail!("not a little-endian libpcap file (bad magic)");
    }
    let mut off = 24;
    while off + 16 <= bytes.len() {
        let incl = u32::from_le_bytes(bytes[off + 8..off + 12].try_into()?) as usize;
        off += 16;
        if off + incl > bytes.len() {
            break;
        }
        let pkt = &bytes[off..off + incl];
        off += incl;
        c.datagrams += 1;
        process_record(pkt, sampler, samples, players, c);
    }
    Ok(())
}

fn process_record(
    pkt: &[u8],
    sampler: &mut FloorSampler,
    samples: &mut Vec<Sample>,
    players: &mut HashSet<u16>,
    c: &mut Counters,
) {
    if pkt.len() < 28 || (pkt[0] >> 4) != 4 {
        return;
    }
    let ihl = (pkt[0] & 0x0f) as usize * 4;
    if pkt.len() < ihl + 8 {
        return;
    }
    let src_ip = Ipv4Addr::new(pkt[12], pkt[13], pkt[14], pkt[15]);
    if src_ip == CLIENT_IP {
        return;
    }
    c.inbound += 1;
    let payload = &pkt[ihl + 8..];
    if payload.starts_with(b"SAMP") {
        return;
    }
    let (_acks, msgs) = raknet::dissect_datagram(payload);
    for m in &msgs {
        if m.split.is_some() || m.payload.first() != Some(&ONFOOT_SYNC_ID) {
            continue;
        }
        c.onfoot_seen += 1;
        let mut r = BitStreamReader::new(&m.payload[1..]);
        let sync = match PlayerSyncData::decode(&mut r) {
            Ok(s) => s,
            Err(_) => {
                c.decode_failed += 1;
                continue;
            }
        };
        let pos = sync.position;
        if !(pos.x.is_finite()
            && pos.y.is_finite()
            && pos.z.is_finite()
            && sync.move_speed.z.is_finite())
        {
            c.nonfinite += 1;
            continue;
        }
        if sync.surfing_vehicle_id.is_some() {
            c.surfing += 1;
            continue;
        }
        if sync.move_speed.z.abs() > VERTICAL_SPEED_CUTOFF {
            c.vertical += 1;
            continue;
        }
        let Some(floor_z) = sampler.floor([pos.x, pos.y, pos.z]) else {
            c.off_mesh += 1;
            continue;
        };
        c.kept += 1;
        players.insert(sync.player_id);
        samples.push(Sample {
            x: pos.x,
            y: pos.y,
            z: pos.z,
            dz: pos.z - floor_z,
        });
    }
}

struct Dist {
    n: usize,
    mean: f64,
    median: f64,
    p05: f64,
    p25: f64,
    p75: f64,
    p95: f64,
    min: f64,
    max: f64,
    stddev: f64,
}

fn median(mut v: Vec<f32>) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n == 0 {
        return f64::NAN;
    }
    if n.is_multiple_of(2) {
        (v[n / 2 - 1] as f64 + v[n / 2] as f64) / 2.0
    } else {
        v[n / 2] as f64
    }
}

fn count_le(sorted: &[f32], v: f64) -> usize {
    sorted.partition_point(|&x| (x as f64) <= v)
}

fn count_lt(sorted: &[f32], v: f64) -> usize {
    sorted.partition_point(|&x| (x as f64) < v)
}

fn abs_dev_count_le(sorted: &[f32], l: f64, t: f64) -> usize {
    count_le(sorted, l + t) - count_lt(sorted, l - t)
}

fn abs_dev_percentile(sorted: &[f32], l: f64, p: f64) -> f64 {
    let target = (p / 100.0 * sorted.len() as f64).ceil().max(1.0) as usize;
    let (mut lo, mut hi) = (0.0f64, 32.0f64);
    for _ in 0..64 {
        let mid = (lo + hi) / 2.0;
        if abs_dev_count_le(sorted, l, mid) >= target {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

fn lift_sweep(label: &str, dz: &[f32]) -> Option<f64> {
    if dz.is_empty() {
        return None;
    }
    let mut sorted = dz.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let n = sorted.len();
    println!();
    println!("=== lift sweep: residual r = dz - L ({label}, n={n}) ===");
    println!(
        "  {:>5} {:>9} {:>9} {:>8} {:>8} {:>8} {:>9}",
        "L", "|r|>1.0%", "|r|>0.8%", "p50|r|", "p95|r|", "p99|r|", "above%"
    );
    let mut best = (f64::INFINITY, 0.0f64);
    let mut step = 0u32;
    loop {
        let l = step as f64 * LIFT_STEP;
        if l > LIFT_MAX + 1e-9 {
            break;
        }
        let over10 = (n - abs_dev_count_le(&sorted, l, 1.0)) as f64 / n as f64 * 100.0;
        let over08 = (n - abs_dev_count_le(&sorted, l, 0.8)) as f64 / n as f64 * 100.0;
        let p50 = abs_dev_percentile(&sorted, l, 50.0);
        let p95 = abs_dev_percentile(&sorted, l, 95.0);
        let p99 = abs_dev_percentile(&sorted, l, 99.0);
        let above = count_lt(&sorted, l) as f64 / n as f64 * 100.0;
        println!(
            "  {l:>5.2} {over10:>9.3} {over08:>9.3} {p50:>8.3} {p95:>8.3} {p99:>8.3} {above:>9.3}"
        );
        if over10 < best.0 {
            best = (over10, l);
        }
        step += 1;
    }
    println!(
        "  best L by min |r|>1.0%: L={:.2} ({:.3}%); above% = fraction with dz < L (we would report Z above the real ped)",
        best.1, best.0
    );
    Some(best.1)
}

fn poly_contains_xy(mesh: &sa_nav::NavMesh, pi: usize, x: f32, y: f32) -> bool {
    let poly = &mesh.polys[pi];
    let m = poly.verts.len();
    let mut sign = 0.0f32;
    for j in 0..m {
        let a = mesh.verts[poly.verts[j] as usize];
        let b = mesh.verts[poly.verts[(j + 1) % m] as usize];
        let cross = (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]);
        if cross.abs() < 1e-6 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return false;
        }
    }
    true
}

fn tri_height(a: [f32; 3], b: [f32; 3], c: [f32; 3], x: f32, y: f32) -> Option<f32> {
    let (v0x, v0y) = (b[0] - a[0], b[1] - a[1]);
    let (v1x, v1y) = (c[0] - a[0], c[1] - a[1]);
    let (v2x, v2y) = (x - a[0], y - a[1]);
    let den = v0x * v1y - v1x * v0y;
    if den.abs() < 1e-9 {
        return None;
    }
    let inv = 1.0 / den;
    let u = (v2x * v1y - v1x * v2y) * inv;
    let v = (v0x * v2y - v2x * v0y) * inv;
    let eps = 1e-3;
    if u >= -eps && v >= -eps && u + v <= 1.0 + eps {
        Some(a[2] + u * (b[2] - a[2]) + v * (c[2] - a[2]))
    } else {
        None
    }
}

fn poly_ground(mesh: &sa_nav::NavMesh, pi: usize, x: f32, y: f32) -> Option<f32> {
    let &[first, count] = mesh.detail_meshes.get(pi)?;
    let tris = mesh
        .detail_tris
        .get(first as usize..(first + count) as usize)?;
    let mut nearest: Option<(f32, f32)> = None;
    for t in tris {
        let a = mesh.detail_verts[t[0] as usize];
        let b = mesh.detail_verts[t[1] as usize];
        let c = mesh.detail_verts[t[2] as usize];
        if let Some(z) = tri_height(a, b, c, x, y) {
            return Some(z);
        }
        for v in [a, b, c] {
            let d2 = (v[0] - x) * (v[0] - x) + (v[1] - y) * (v[1] - y);
            if nearest.is_none_or(|(bd, _)| d2 < bd) {
                nearest = Some((d2, v[2]));
            }
        }
    }
    nearest.map(|(_, z)| z)
}

fn surfaces_at(mesh: &sa_nav::NavMesh, x: f32, y: f32) -> Vec<f32> {
    let mut out: Vec<f32> = mesh
        .polys
        .iter()
        .enumerate()
        .filter(|(pi, poly)| poly.verts.len() >= 3 && poly_contains_xy(mesh, *pi, x, y))
        .filter_map(|(pi, _)| poly_ground(mesh, pi, x, y))
        .collect();
    out.sort_by(|a, b| a.total_cmp(b));
    out.dedup_by(|a, b| (*a - *b).abs() < 0.05);
    out
}

fn print_trimmed(label: &str, dz: &[f32]) {
    let trimmed: Vec<f32> = dz
        .iter()
        .copied()
        .filter(|&d| (TRIM_MIN..=TRIM_MAX).contains(&d))
        .collect();
    let Some(d) = distribution(&trimmed) else {
        return;
    };
    println!(
        "{label}: n={} of {} ({:.3}% retained by dz in [{TRIM_MIN}, {TRIM_MAX}])",
        d.n,
        dz.len(),
        d.n as f64 / dz.len() as f64 * 100.0
    );
    println!(
        "  mean={:+.3}  median={:+.3}  stddev={:.3}",
        d.mean, d.median, d.stddev
    );
    println!(
        "  p05={:+.3}  p25={:+.3}  p75={:+.3}  p95={:+.3}",
        d.p05, d.p25, d.p75, d.p95
    );
}

struct OffenderCell {
    cx: f32,
    cy: f32,
    n: usize,
    med_dz: f64,
    resid: f64,
    med_z: f64,
    frac_normal: f64,
    surfaces: Vec<f32>,
    real: bool,
}

fn offender_report(
    samples: &[Sample],
    l: f64,
    sampler: &FloorSampler,
) -> (HashSet<CellKey>, HashSet<CellKey>) {
    let mesh = sampler.query.mesh();
    let mut cells: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, s) in samples.iter().enumerate() {
        let key = (
            (s.x / CELL_SIZE).floor() as i64,
            (s.y / CELL_SIZE).floor() as i64,
        );
        cells.entry(key).or_default().push(i);
    }
    let mut offenders: Vec<OffenderCell> = Vec::new();
    let mut real_cells: HashSet<(i64, i64)> = HashSet::new();
    let mut confusion_cells: HashSet<(i64, i64)> = HashSet::new();
    for (k, idx) in &cells {
        if idx.len() < MIN_CELL_SAMPLES {
            continue;
        }
        let med_dz = median(idx.iter().map(|&i| samples[i].dz).collect());
        let resid = med_dz - l;
        if resid.abs() <= OFFENDER_RESIDUAL {
            continue;
        }
        let mx = median(idx.iter().map(|&i| samples[i].x).collect()) as f32;
        let my = median(idx.iter().map(|&i| samples[i].y).collect()) as f32;
        let med_z = median(idx.iter().map(|&i| samples[i].z).collect());
        let normal = idx
            .iter()
            .filter(|&&i| (TRIM_MIN..=TRIM_MAX).contains(&samples[i].dz))
            .count();
        let surfaces = surfaces_at(mesh, mx, my);
        let real = !surfaces
            .iter()
            .any(|&s| (s as f64 - med_z).abs() <= SURFACE_MATCH);
        if real {
            real_cells.insert(*k);
        } else {
            confusion_cells.insert(*k);
        }
        offenders.push(OffenderCell {
            cx: (k.0 as f32 + 0.5) * CELL_SIZE,
            cy: (k.1 as f32 + 0.5) * CELL_SIZE,
            n: idx.len(),
            med_dz,
            resid,
            med_z,
            frac_normal: normal as f64 / idx.len() as f64,
            surfaces,
            real,
        });
    }
    offenders.sort_by_key(|o| std::cmp::Reverse(o.n));
    println!();
    println!(
        "=== post-lift offenders at L={l:.2}: |cell_median_dz - L| > {OFFENDER_RESIDUAL}, >= {MIN_CELL_SAMPLES} samples ===",
    );
    println!(
        "  {} cells: {} REAL defects (no walkable surface within {SURFACE_MATCH} m of players' median Z), {} LEVEL-CONFUSION (mesh has a matching surface)",
        offenders.len(),
        offenders.iter().filter(|o| o.real).count(),
        offenders.iter().filter(|o| !o.real).count()
    );
    for (title, want_real) in [
        ("REAL defects (sorted by n)", true),
        ("LEVEL-CONFUSION (sorted by n)", false),
    ] {
        println!("  --- {title} ---");
        println!(
            "  {:>8} {:>8} {:>7} {:>8} {:>8} {:>9} {:>7}  surfaces_z",
            "cell_x", "cell_y", "n", "med_dz", "resid", "med_z", "norm%"
        );
        for o in offenders.iter().filter(|o| o.real == want_real) {
            let surf = if o.surfaces.is_empty() {
                "(none)".to_string()
            } else {
                o.surfaces
                    .iter()
                    .take(6)
                    .map(|s| format!("{s:.2}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            println!(
                "  {:>8.1} {:>8.1} {:>7} {:>+8.3} {:>+8.3} {:>9.2} {:>7.1}  {surf}",
                o.cx,
                o.cy,
                o.n,
                o.med_dz,
                o.resid,
                o.med_z,
                o.frac_normal * 100.0,
            );
        }
    }
    (real_cells, confusion_cells)
}

fn parse_target_line(line: &str) -> Option<[f32; 3]> {
    let cleaned = line.replace(['[', ']'], "");
    let parts: Vec<f32> = cleaned
        .split([',', ' '])
        .filter(|t| !t.is_empty())
        .map(|t| t.trim().parse::<f32>())
        .collect::<Result<_, _>>()
        .ok()?;
    if parts.len() == 3 {
        Some([parts[0], parts[1], parts[2]])
    } else {
        None
    }
}

fn in_sawmill_leg_box(p: [f32; 3]) -> bool {
    p[0] >= SAWMILL_MIN[0]
        && p[0] <= SAWMILL_MAX[0]
        && p[1] >= SAWMILL_MIN[1]
        && p[1] <= SAWMILL_MAX[1]
        && p[2] >= LEG_Z_MIN
        && p[2] <= LEG_Z_MAX
}

fn route_report(
    targets_path: &str,
    sampler: &FloorSampler,
    real: &HashSet<CellKey>,
    confusion: &HashSet<CellKey>,
) -> anyhow::Result<()> {
    let text = std::fs::read_to_string(targets_path)
        .with_context(|| format!("cannot read {targets_path}"))?;
    let mut legs: Vec<([f32; 3], [f32; 3])> = Vec::new();
    let mut seen: HashSet<(i64, i64, i64, i64)> = HashSet::new();
    let mut prev: Option<[f32; 3]> = None;
    for line in text.lines() {
        let line = line.trim();
        if line == "---" || line.is_empty() {
            prev = None;
            continue;
        }
        let Some(t) = parse_target_line(line) else {
            continue;
        };
        if let Some(p) = prev {
            let same = (p[0] - t[0]).abs() < 0.01 && (p[1] - t[1]).abs() < 0.01;
            if !same && in_sawmill_leg_box(p) && in_sawmill_leg_box(t) {
                let key = (
                    (p[0] * 10.0) as i64,
                    (p[1] * 10.0) as i64,
                    (t[0] * 10.0) as i64,
                    (t[1] * 10.0) as i64,
                );
                if seen.insert(key) {
                    legs.push((p, t));
                }
            }
        }
        prev = Some(t);
    }
    println!();
    println!(
        "=== route check: {} unique sawmill legs from {targets_path} (consecutive walkTo targets, box x[{}, {}] y[{}, {}] z[{LEG_Z_MIN}, {LEG_Z_MAX}]) ===",
        legs.len(),
        SAWMILL_MIN[0],
        SAWMILL_MAX[0],
        SAWMILL_MIN[1],
        SAWMILL_MAX[1]
    );
    let mut no_path = 0usize;
    let mut real_hit_legs = 0usize;
    let mut conf_hit_legs = 0usize;
    let mut phantom_legs = 0usize;
    let mut real_cells_hit: HashSet<(i64, i64)> = HashSet::new();
    let mut wps: Vec<f32> = Vec::new();
    let mut lens: Vec<f32> = Vec::new();
    let mut z_lo = f32::INFINITY;
    let mut z_hi = f32::NEG_INFINITY;
    for (from, to) in &legs {
        let Some(path) = sampler.query.find_path(*from, *to) else {
            no_path += 1;
            println!(
                "  NO PATH: [{:.2}, {:.2}, {:.2}] -> [{:.2}, {:.2}, {:.2}]",
                from[0], from[1], from[2], to[0], to[1], to[2]
            );
            continue;
        };
        let mut cells_on_path: HashSet<(i64, i64)> = HashSet::new();
        let mut leg_lo = f32::INFINITY;
        let mut leg_hi = f32::NEG_INFINITY;
        let mut leg_len = 0.0f32;
        for w in &path {
            leg_lo = leg_lo.min(w[2]);
            leg_hi = leg_hi.max(w[2]);
        }
        for seg in path.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
            leg_len += len;
            let steps = (len / LEG_STEP).ceil().max(1.0) as usize;
            for s in 0..=steps {
                let t = s as f32 / steps as f32;
                let x = a[0] + (b[0] - a[0]) * t;
                let y = a[1] + (b[1] - a[1]) * t;
                cells_on_path.insert((
                    (x / CELL_SIZE).floor() as i64,
                    (y / CELL_SIZE).floor() as i64,
                ));
            }
        }
        z_lo = z_lo.min(leg_lo);
        z_hi = z_hi.max(leg_hi);
        wps.push(path.len() as f32);
        lens.push(leg_len);
        if leg_hi > PHANTOM_Z {
            phantom_legs += 1;
        }
        let real_hits: Vec<_> = cells_on_path.iter().filter(|c| real.contains(c)).collect();
        if cells_on_path.iter().any(|c| confusion.contains(c)) {
            conf_hit_legs += 1;
        }
        println!(
            "  LEG [{:.2}, {:.2}]->[{:.2}, {:.2}] wp={} len={leg_len:.2} z[{leg_lo:.2}, {leg_hi:.2}] real={}",
            from[0],
            from[1],
            to[0],
            to[1],
            path.len(),
            if real_hits.is_empty() { 0 } else { 1 }
        );
        if !real_hits.is_empty() {
            real_hit_legs += 1;
            real_cells_hit.extend(real_hits.iter().copied());
            let cells: Vec<String> = real_hits
                .iter()
                .map(|c| {
                    format!(
                        "({:.0},{:.0})",
                        (c.0 as f32 + 0.5) * CELL_SIZE,
                        (c.1 as f32 + 0.5) * CELL_SIZE
                    )
                })
                .collect();
            println!(
                "  REAL-cell hit: [{:.1}, {:.1}] -> [{:.1}, {:.1}]  wp={} z[{leg_lo:.2}, {leg_hi:.2}]  cells: {}",
                from[0],
                from[1],
                to[0],
                to[1],
                path.len(),
                cells.join(" ")
            );
        }
    }
    println!(
        "  legs={} paths_found={} no_path={no_path} legs_z_above_{PHANTOM_Z}={phantom_legs} legs_through_REAL_cells={real_hit_legs} legs_through_confusion_cells={conf_hit_legs}",
        legs.len(),
        legs.len() - no_path
    );
    if !wps.is_empty() {
        println!(
            "  waypoints per found leg: mean={:.2} median={:.1}; XY length: mean={:.2} median={:.2}",
            wps.iter().sum::<f32>() / wps.len() as f32,
            median(wps.clone()),
            lens.iter().sum::<f32>() / lens.len() as f32,
            median(lens.clone())
        );
        println!("  waypoint z range over all found paths: [{z_lo:.2}, {z_hi:.2}]");
    }
    if real_hit_legs == 0 {
        println!("  no leg touches any REAL-defect cell");
    } else {
        let cells: Vec<String> = real_cells_hit
            .iter()
            .map(|c| {
                format!(
                    "({:.0},{:.0})",
                    (c.0 as f32 + 0.5) * CELL_SIZE,
                    (c.1 as f32 + 0.5) * CELL_SIZE
                )
            })
            .collect();
        println!("  distinct REAL cells touched: {}", cells.join(" "));
    }
    Ok(())
}

#[derive(Default)]
struct Profile {
    n207: u64,
    n203: u64,
    bad207: u64,
    bad203: u64,
    anim_none: u64,
    anim_pairs: HashMap<(u16, u16), u64>,
    keys: HashMap<u16, u64>,
    lr_none: u64,
    lr: HashMap<u16, u64>,
    ud_none: u64,
    ud: HashMap<u16, u64>,
    special: HashMap<u8, u64>,
    health: HashMap<u8, u64>,
    armor: HashMap<u8, u64>,
    speed_zero: u64,
    speeds: Vec<f32>,
    speed_exact: HashMap<u32, u64>,
    vz_zero: u64,
    vz: Vec<f32>,
    vz_pos: u64,
    vz_neg: u64,
    prev_pos: HashMap<(u32, u16), [f32; 3]>,
    still_n: u64,
    still_dz_zero: u64,
    still_dz: Vec<f32>,
    aspect: HashMap<u8, u64>,
    cam_mode: HashMap<u8, u64>,
    zoom: HashMap<u8, u64>,
    wstate: HashMap<u8, u64>,
    per_player: HashMap<(u32, u16), (u64, u64)>,
    series: HashMap<u16, Vec<f32>>,
    n200: u64,
    bad200: u64,
    n209: u64,
    bad209: u64,
    veh200: HashMap<u16, u64>,
    veh209: HashMap<u16, u64>,
    inbox200: u64,
    inbox209: u64,
    bbox200: Option<[f32; 6]>,
    bbox209: Option<[f32; 6]>,
    inbox200_veh: HashMap<u16, (u64, [f32; 3])>,
    inbox209_veh: HashMap<u16, (u64, [f32; 3])>,
    pcaps_with_209: HashSet<u32>,
    pcaps_with_200: HashSet<u32>,
    rpc: HashMap<u8, u64>,
    rpc164_parsed: u64,
    rpc164_short: u64,
    rpc164_models: HashMap<u32, u64>,
    rpc164_inbox_models: HashMap<u32, (u64, [f32; 3])>,
}

fn le_f32_at(b: &[u8], o: usize) -> Option<f32> {
    Some(f32::from_le_bytes(b.get(o..o + 4)?.try_into().ok()?))
}

fn le_u32_at(b: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_le_bytes(b.get(o..o + 4)?.try_into().ok()?))
}

fn bbox_add(b: &mut Option<[f32; 6]>, x: f32, y: f32, z: f32) {
    match b {
        Some(v) => {
            v[0] = v[0].min(x);
            v[1] = v[1].max(x);
            v[2] = v[2].min(y);
            v[3] = v[3].max(y);
            v[4] = v[4].min(z);
            v[5] = v[5].max(z);
        }
        None => *b = Some([x, x, y, y, z, z]),
    }
}

fn in_sawmill_xy(x: f32, y: f32) -> bool {
    (SAWMILL_MIN[0]..=SAWMILL_MAX[0]).contains(&x) && (SAWMILL_MIN[1]..=SAWMILL_MAX[1]).contains(&y)
}

fn each_inbound_message(bytes: &[u8], f: &mut impl FnMut(&[u8])) -> anyhow::Result<(u64, u64)> {
    if bytes.len() < 24 || bytes[0..4] != [0xd4, 0xc3, 0xb2, 0xa1] {
        bail!("not a little-endian libpcap file (bad magic)");
    }
    let mut datagrams = 0u64;
    let mut inbound = 0u64;
    let mut off = 24;
    while off + 16 <= bytes.len() {
        let incl = u32::from_le_bytes(bytes[off + 8..off + 12].try_into()?) as usize;
        off += 16;
        if off + incl > bytes.len() {
            break;
        }
        let pkt = &bytes[off..off + incl];
        off += incl;
        datagrams += 1;
        if pkt.len() < 28 || (pkt[0] >> 4) != 4 {
            continue;
        }
        let ihl = (pkt[0] & 0x0f) as usize * 4;
        if pkt.len() < ihl + 8 {
            continue;
        }
        if Ipv4Addr::new(pkt[12], pkt[13], pkt[14], pkt[15]) == CLIENT_IP {
            continue;
        }
        inbound += 1;
        let payload = &pkt[ihl + 8..];
        if payload.starts_with(b"SAMP") {
            continue;
        }
        let (_acks, msgs) = raknet::dissect_datagram(payload);
        for m in &msgs {
            if m.split.is_none() {
                f(&m.payload);
            }
        }
    }
    Ok((datagrams, inbound))
}

fn profile_message(payload: &[u8], prof: &mut Profile, file_idx: u32, collect_series: bool) {
    match payload.first() {
        Some(&ONFOOT_SYNC_ID) => {
            let mut r = BitStreamReader::new(&payload[1..]);
            let Ok(s) = PlayerSyncData::decode(&mut r) else {
                prof.bad207 += 1;
                return;
            };
            prof.n207 += 1;
            prof.per_player
                .entry((file_idx, s.player_id))
                .or_default()
                .0 += 1;
            match s.animation_id {
                Some(id) => *prof.anim_pairs.entry((id, s.animation_flags)).or_default() += 1,
                None => prof.anim_none += 1,
            }
            *prof.keys.entry(s.keys_data).or_default() += 1;
            match s.left_right_keys {
                Some(v) => *prof.lr.entry(v).or_default() += 1,
                None => prof.lr_none += 1,
            }
            match s.up_down_keys {
                Some(v) => *prof.ud.entry(v).or_default() += 1,
                None => prof.ud_none += 1,
            }
            *prof.special.entry(s.special_action).or_default() += 1;
            *prof.health.entry(s.health).or_default() += 1;
            *prof.armor.entry(s.armor).or_default() += 1;
            let v = s.move_speed;
            let mag = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
            if mag.is_finite() {
                if mag <= SPEED_ZERO {
                    prof.speed_zero += 1;
                } else {
                    prof.speeds.push(mag);
                    *prof
                        .speed_exact
                        .entry((mag as f64 * 10000.0).round() as u32)
                        .or_default() += 1;
                }
                if v.z.abs() <= SPEED_ZERO {
                    prof.vz_zero += 1;
                } else {
                    prof.vz.push(v.z.abs());
                    if v.z > 0.0 {
                        prof.vz_pos += 1;
                    } else {
                        prof.vz_neg += 1;
                    }
                }
                if collect_series {
                    prof.series.entry(s.player_id).or_default().push(mag);
                }
            }
            let p = s.position;
            if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                if let Some(prev) = prof
                    .prev_pos
                    .insert((file_idx, s.player_id), [p.x, p.y, p.z])
                {
                    let dxy = ((p.x - prev[0]).powi(2) + (p.y - prev[1]).powi(2)).sqrt();
                    if dxy <= 0.001 {
                        prof.still_n += 1;
                        let dz = (p.z - prev[2]).abs();
                        if dz == 0.0 {
                            prof.still_dz_zero += 1;
                        } else {
                            prof.still_dz.push(dz);
                        }
                    }
                }
            }
        }
        Some(&AIM_SYNC_ID) => {
            let mut r = BitStreamReader::new(&payload[1..]);
            let decoded = r.read_u16().ok().zip(AimSyncData::decode(&mut r).ok());
            let Some((pid, aim)) = decoded else {
                prof.bad203 += 1;
                return;
            };
            prof.n203 += 1;
            prof.per_player.entry((file_idx, pid)).or_default().1 += 1;
            *prof.aspect.entry(aim.aspect_ratio).or_default() += 1;
            *prof.cam_mode.entry(aim.cam_mode).or_default() += 1;
            *prof
                .zoom
                .entry(aim.cam_ext_zoom_weapon_state & 0x3F)
                .or_default() += 1;
            *prof
                .wstate
                .entry(aim.cam_ext_zoom_weapon_state >> 6)
                .or_default() += 1;
        }
        Some(&VEHICLE_SYNC_ID) => {
            let mut r = BitStreamReader::new(&payload[1..]);
            let Ok(s) = VehicleSyncData::decode(&mut r) else {
                prof.bad200 += 1;
                return;
            };
            prof.n200 += 1;
            prof.pcaps_with_200.insert(file_idx);
            *prof.veh200.entry(s.vehicle_id).or_default() += 1;
            let p = s.position;
            if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                bbox_add(&mut prof.bbox200, p.x, p.y, p.z);
                if in_sawmill_xy(p.x, p.y) {
                    prof.inbox200 += 1;
                    let e = prof
                        .inbox200_veh
                        .entry(s.vehicle_id)
                        .or_insert((0, [p.x, p.y, p.z]));
                    e.0 += 1;
                    e.1 = [p.x, p.y, p.z];
                }
            }
        }
        Some(&UNOCCUPIED_SYNC_ID) => {
            let mut r = BitStreamReader::new(&payload[1..]);
            let decoded = r
                .read_u16()
                .ok()
                .zip(UnoccupiedSyncData::decode(&mut r).ok());
            let Some((_prefix, s)) = decoded else {
                prof.bad209 += 1;
                return;
            };
            prof.n209 += 1;
            prof.pcaps_with_209.insert(file_idx);
            *prof.veh209.entry(s.vehicle_id).or_default() += 1;
            let p = s.position;
            if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                bbox_add(&mut prof.bbox209, p.x, p.y, p.z);
                if in_sawmill_xy(p.x, p.y) {
                    prof.inbox209 += 1;
                    let e = prof
                        .inbox209_veh
                        .entry(s.vehicle_id)
                        .or_insert((0, [p.x, p.y, p.z]));
                    e.0 += 1;
                    e.1 = [p.x, p.y, p.z];
                }
            }
        }
        _ => {
            let Some((rpc, body)) = raknet::wire::parse_rpc(payload) else {
                return;
            };
            *prof.rpc.entry(rpc).or_default() += 1;
            if rpc == RPC_WORLD_VEHICLE_ADD {
                let parsed = le_u32_at(&body, 2)
                    .zip(le_f32_at(&body, 6))
                    .and_then(|(model, x)| {
                        Some((model, x, le_f32_at(&body, 10)?, le_f32_at(&body, 14)?))
                    });
                let Some((model, x, y, z)) = parsed else {
                    prof.rpc164_short += 1;
                    return;
                };
                prof.rpc164_parsed += 1;
                *prof.rpc164_models.entry(model).or_default() += 1;
                if x.is_finite() && y.is_finite() && in_sawmill_xy(x, y) {
                    let e = prof
                        .rpc164_inbox_models
                        .entry(model)
                        .or_insert((0, [x, y, z]));
                    e.0 += 1;
                    e.1 = [x, y, z];
                }
            }
        }
    }
}

fn top_by_count<K: Copy + Ord>(map: &HashMap<K, u64>, n: usize) -> Vec<(K, u64)> {
    let mut v: Vec<(K, u64)> = map.iter().map(|(&k, &c)| (k, c)).collect();
    v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    v.truncate(n);
    v
}

fn pct_of(count: u64, total: u64) -> f64 {
    if total == 0 {
        0.0
    } else {
        count as f64 / total as f64 * 100.0
    }
}

fn profile_main(args: &[String]) -> anyhow::Result<()> {
    let paths = expand_inputs(args);
    if paths.is_empty() {
        bail!("no pcap files matched the given arguments");
    }
    let mut prof = Profile::default();
    for (i, path) in paths.iter().enumerate() {
        let bytes = match std::fs::read(path) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("skipping {}: {e}", path.display());
                continue;
            }
        };
        let before = (prof.n207, prof.n203);
        let collect_series = i == 0;
        match each_inbound_message(&bytes, &mut |p| {
            profile_message(p, &mut prof, i as u32, collect_series)
        }) {
            Ok((datagrams, inbound)) => println!(
                "{}: {} datagrams ({} inbound), {} x207, {} x203",
                path.display(),
                datagrams,
                inbound,
                prof.n207 - before.0,
                prof.n203 - before.1
            ),
            Err(e) => eprintln!("skipping {}: {e}", path.display()),
        }
        if collect_series {
            println!("  (per-player speed series collected from this file)");
        }
    }
    print_profile(&prof);
    Ok(())
}

fn print_profile(prof: &Profile) {
    println!();
    println!(
        "=== profile totals: {} x207 ({} bad), {} x203 ({} bad) ===",
        prof.n207, prof.bad207, prof.n203, prof.bad203
    );

    println!();
    println!(
        "=== animation (id, flags) pairs, top 30 of {} distinct ===",
        prof.anim_pairs.len()
    );
    println!(
        "  no-anim-group: {} ({:.3}%)",
        prof.anim_none,
        pct_of(prof.anim_none, prof.n207)
    );
    for ((id, flags), c) in top_by_count(&prof.anim_pairs, 30) {
        println!(
            "  ({id:>5}, 0x{flags:04X})  {c:>9}  {:>7.3}%",
            pct_of(c, prof.n207)
        );
    }

    println!();
    println!("=== keys_data, top 25 of {} distinct ===", prof.keys.len());
    for (k, c) in top_by_count(&prof.keys, 25) {
        let names: Vec<&str> = KEY_BIT_NAMES
            .iter()
            .filter(|(bit, _)| k & bit != 0)
            .map(|(_, n)| *n)
            .collect();
        println!(
            "  0x{k:04X}  {c:>9}  {:>7.3}%  {}",
            pct_of(c, prof.n207),
            names.join("+")
        );
    }
    println!("  per-bit set rates:");
    for (bit, name) in KEY_BIT_NAMES {
        let c: u64 = prof
            .keys
            .iter()
            .filter(|(k, _)| *k & bit != 0)
            .map(|(_, c)| c)
            .sum();
        if c > 0 {
            println!(
                "    0x{bit:04X} {name:<17} {c:>9}  {:>7.3}%",
                pct_of(c, prof.n207)
            );
        }
    }

    println!();
    println!(
        "=== left_right analog: present {:.3}% (absent {}), top 12 of {} distinct ===",
        pct_of(prof.n207 - prof.lr_none, prof.n207),
        prof.lr_none,
        prof.lr.len()
    );
    for (v, c) in top_by_count(&prof.lr, 12) {
        println!(
            "  0x{v:04X} ({:>6})  {c:>9}  {:>7.3}%",
            v as i16,
            pct_of(c, prof.n207)
        );
    }
    println!();
    println!(
        "=== up_down analog: present {:.3}% (absent {}), top 12 of {} distinct ===",
        pct_of(prof.n207 - prof.ud_none, prof.n207),
        prof.ud_none,
        prof.ud.len()
    );
    for (v, c) in top_by_count(&prof.ud, 12) {
        println!(
            "  0x{v:04X} ({:>6})  {c:>9}  {:>7.3}%",
            v as i16,
            pct_of(c, prof.n207)
        );
    }

    println!();
    println!("=== special_action ({} distinct) ===", prof.special.len());
    for (v, c) in top_by_count(&prof.special, 20) {
        println!("  {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n207));
    }

    println!();
    println!("=== health (top 12) / armor (top 8) ===");
    for (v, c) in top_by_count(&prof.health, 12) {
        println!("  hp {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n207));
    }
    let armor_nonzero: u64 = prof
        .armor
        .iter()
        .filter(|(v, _)| **v > 0)
        .map(|(_, c)| c)
        .sum();
    println!(
        "  armor nonzero: {armor_nonzero} ({:.3}%)",
        pct_of(armor_nonzero, prof.n207)
    );
    for (v, c) in top_by_count(&prof.armor, 8) {
        println!("  ap {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n207));
    }

    println!();
    let moving = prof.speeds.len() as u64;
    println!(
        "=== |move_speed| (3D): zero {} ({:.3}%), moving {} ({:.3}%) ===",
        prof.speed_zero,
        pct_of(prof.speed_zero, prof.n207),
        moving,
        pct_of(moving, prof.n207)
    );
    if let Some(d) = distribution(&prof.speeds) {
        println!(
            "  p01={:.4} p05={:.4} p10={:.4} p25={:.4} p50={:.4} p75={:.4} p90={:.4} p95={:.4} p99={:.4} max={:.4}",
            pct_sorted(&prof.speeds, 1.0),
            d.p05,
            pct_sorted(&prof.speeds, 10.0),
            d.p25,
            d.median,
            d.p75,
            pct_sorted(&prof.speeds, 90.0),
            d.p95,
            pct_sorted(&prof.speeds, 99.0),
            d.max
        );
    }
    println!("  top 20 exact values (rounded to 4dp) among moving samples:");
    for (k, c) in top_by_count(&prof.speed_exact, 20) {
        println!(
            "    {:.4}  {c:>9}  {:>7.3}% of moving",
            k as f64 / 10000.0,
            pct_of(c, moving)
        );
    }

    println!();
    let vz_nonzero = prof.vz.len() as u64;
    println!(
        "=== move_speed.z: zero {} ({:.3}%), nonzero {} ({:.3}%) ===",
        prof.vz_zero,
        pct_of(prof.vz_zero, prof.vz_zero + vz_nonzero),
        vz_nonzero,
        pct_of(vz_nonzero, prof.vz_zero + vz_nonzero)
    );
    println!(
        "  sign of nonzero vz: up {} ({:.3}%), down {} ({:.3}%)",
        prof.vz_pos,
        pct_of(prof.vz_pos, vz_nonzero),
        prof.vz_neg,
        pct_of(prof.vz_neg, vz_nonzero)
    );
    println!(
        "=== stationary (|dXY| <= 0.001) consecutive dz: n={} exactly-zero {} ({:.3}%) ===",
        prof.still_n,
        prof.still_dz_zero,
        pct_of(prof.still_dz_zero, prof.still_n)
    );
    if let Some(d) = distribution(&prof.still_dz) {
        println!(
            "  |dz| when nonzero: p50={:.4} p95={:.4} p99={:.4} max={:.4}",
            d.median,
            d.p95,
            pct_sorted(&prof.still_dz, 99.0),
            d.max
        );
    }
    if let Some(d) = distribution(&prof.vz) {
        println!(
            "  |vz| nonzero: p50={:.4} p95={:.4} p99={:.4} max={:.4}",
            d.median,
            d.p95,
            pct_sorted(&prof.vz, 99.0),
            d.max
        );
    }

    println!();
    println!("=== 203 aim sync ===");
    println!("  aspect_ratio (top 12 of {} distinct):", prof.aspect.len());
    for (v, c) in top_by_count(&prof.aspect, 12) {
        println!("    {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n203));
    }
    println!("  cam_mode (top 12 of {} distinct):", prof.cam_mode.len());
    for (v, c) in top_by_count(&prof.cam_mode, 12) {
        println!("    {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n203));
    }
    println!(
        "  cam_ext_zoom low6 (top 8 of {} distinct):",
        prof.zoom.len()
    );
    for (v, c) in top_by_count(&prof.zoom, 8) {
        println!("    {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n203));
    }
    println!("  weapon_state high2:");
    for (v, c) in top_by_count(&prof.wstate, 4) {
        println!("    {v:>3}  {c:>9}  {:>7.3}%", pct_of(c, prof.n203));
    }

    println!();
    let ratios: Vec<f32> = prof
        .per_player
        .values()
        .filter(|(a, _)| *a >= 500)
        .map(|(a, b)| *b as f32 / *a as f32)
        .collect();
    println!(
        "=== 203/207 rate ratio per player-session (n207 >= 500; {} sessions) ===",
        ratios.len()
    );
    if let Some(d) = distribution(&ratios) {
        println!(
            "  mean={:.3} p05={:.3} p25={:.3} p50={:.3} p75={:.3} p95={:.3}",
            d.mean, d.p05, d.p25, d.median, d.p75, d.p95
        );
    }

    println!();
    println!("=== vehicle traffic: 209 UnoccupiedSync / 200 VehicleSync / creation RPCs ===");
    println!(
        "  209: total={} (bad {}), distinct vehicle ids={}, pcaps containing any={}, positions in sawmill box={}",
        prof.n209,
        prof.bad209,
        prof.veh209.len(),
        prof.pcaps_with_209.len(),
        prof.inbox209
    );
    if let Some(b) = prof.bbox209 {
        println!(
            "    209 position bbox: x[{:.1}, {:.1}] y[{:.1}, {:.1}] z[{:.1}, {:.1}]",
            b[0], b[1], b[2], b[3], b[4], b[5]
        );
        println!("    209 top vehicle ids (all-map):");
        for (v, c) in top_by_count(&prof.veh209, 10) {
            println!("      id {v:>5}  {c:>7}");
        }
        println!(
            "    209 IN-BOX vehicles ({} distinct), top 15 by count with last position:",
            prof.inbox209_veh.len()
        );
        let mut v: Vec<(&u16, &(u64, [f32; 3]))> = prof.inbox209_veh.iter().collect();
        v.sort_by_key(|e| std::cmp::Reverse(e.1 .0));
        for (id, (c, p)) in v.into_iter().take(15) {
            println!(
                "      id {id:>5}  {c:>7}  [{:.1}, {:.1}, {:.1}]",
                p[0], p[1], p[2]
            );
        }
    }
    println!(
        "  200: total={} (bad {}), distinct vehicle ids={}, pcaps containing any={}, positions in sawmill box={}",
        prof.n200,
        prof.bad200,
        prof.veh200.len(),
        prof.pcaps_with_200.len(),
        prof.inbox200
    );
    if let Some(b) = prof.bbox200 {
        println!(
            "    200 position bbox: x[{:.1}, {:.1}] y[{:.1}, {:.1}] z[{:.1}, {:.1}]",
            b[0], b[1], b[2], b[3], b[4], b[5]
        );
        println!(
            "    200 IN-BOX vehicles ({} distinct), top 10 by count with last position:",
            prof.inbox200_veh.len()
        );
        let mut v: Vec<(&u16, &(u64, [f32; 3]))> = prof.inbox200_veh.iter().collect();
        v.sort_by_key(|e| std::cmp::Reverse(e.1 .0));
        for (id, (c, p)) in v.into_iter().take(10) {
            println!(
                "      id {id:>5}  {c:>7}  [{:.1}, {:.1}, {:.1}]",
                p[0], p[1], p[2]
            );
        }
    }
    let named_rpc = [
        (44u8, "CreateObject"),
        (164, "WorldVehicleAdd"),
        (165, "WorldVehicleRemove"),
    ];
    for (id, name) in named_rpc {
        println!(
            "  RPC {id:>3} {name:<18} {}",
            prof.rpc.get(&id).copied().unwrap_or(0)
        );
    }
    println!(
        "  inbound RPC histogram (top 20 of {} distinct):",
        prof.rpc.len()
    );
    for (id, c) in top_by_count(&prof.rpc, 20) {
        println!("    rpc {id:>3}  {c:>9}");
    }
    println!(
        "  RPC 164 bodies parsed {} (too short {}); model histogram (top 15 of {} distinct):",
        prof.rpc164_parsed,
        prof.rpc164_short,
        prof.rpc164_models.len()
    );
    for (m, c) in top_by_count(&prof.rpc164_models, 15) {
        println!("    model {m:>6}  {c:>7}");
    }
    println!(
        "  RPC 164 adds with IN-BOX position ({} distinct models), by count with last pos:",
        prof.rpc164_inbox_models.len()
    );
    let mut v: Vec<(&u32, &(u64, [f32; 3]))> = prof.rpc164_inbox_models.iter().collect();
    v.sort_by_key(|e| std::cmp::Reverse(e.1 .0));
    for (m, (c, p)) in v.into_iter().take(15) {
        println!(
            "    model {m:>6}  {c:>7}  [{:.1}, {:.1}, {:.1}]",
            p[0], p[1], p[2]
        );
    }

    println!();
    println!("=== per-player |move_speed| time series (largest pcap) ===");
    let mut by_n: Vec<(&u16, &Vec<f32>)> = prof.series.iter().collect();
    by_n.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.0.cmp(b.0)));
    for (pid, series) in by_n.iter().take(4) {
        println!("  player {pid} ({} samples):", series.len());
        let mut deltas: Vec<f32> = Vec::new();
        for w in series.windows(2) {
            if w[0] > SPEED_ZERO && w[1] > SPEED_ZERO {
                deltas.push((w[1] - w[0]).abs());
            }
        }
        if let Some(d) = distribution(&deltas) {
            println!(
                "    |dv| between consecutive moving samples: n={} p50={:.4} p95={:.4} max={:.4}",
                d.n, d.median, d.p95, d.max
            );
        }
        let moving: Vec<f32> = series.iter().copied().filter(|&v| v > SPEED_ZERO).collect();
        if !moving.is_empty() {
            let mut freq: HashMap<u32, u64> = HashMap::new();
            for &v in &moving {
                *freq.entry((v as f64 * 1000.0).round() as u32).or_default() += 1;
            }
            let top: Vec<String> = top_by_count(&freq, 5)
                .into_iter()
                .map(|(k, c)| {
                    format!(
                        "{:.3}x{:.1}%",
                        k as f64 / 1000.0,
                        pct_of(c, moving.len() as u64)
                    )
                })
                .collect();
            println!("    moving plateaus (3dp, top5): {}", top.join(" "));
        }
        let mut shown = 0;
        let mut i = 1usize;
        while i < series.len() && shown < 2 {
            if series[i - 1] <= SPEED_ZERO && series[i] > SPEED_ZERO {
                let run: Vec<f32> = series[i..]
                    .iter()
                    .copied()
                    .take_while(|&v| v > SPEED_ZERO)
                    .take(36)
                    .collect();
                if run.len() >= 25 {
                    println!("    episode @{i}:");
                    for chunk in run.chunks(12) {
                        let vals: Vec<String> = chunk.iter().map(|v| format!("{v:.3}")).collect();
                        println!("      {}", vals.join(" "));
                    }
                    shown += 1;
                    i += run.len();
                    continue;
                }
            }
            i += 1;
        }
    }
}

fn pct_sorted(values: &[f32], p: f64) -> f64 {
    let mut v = values.to_vec();
    v.sort_by(|a, b| a.total_cmp(b));
    if v.is_empty() {
        return f64::NAN;
    }
    let idx = p / 100.0 * (v.len() - 1) as f64;
    let lo = idx.floor() as usize;
    let hi = idx.ceil() as usize;
    let frac = idx - lo as f64;
    v[lo] as f64 * (1.0 - frac) + v[hi] as f64 * frac
}

fn distribution(values: &[f32]) -> Option<Dist> {
    if values.is_empty() {
        return None;
    }
    let mut v: Vec<f32> = values.to_vec();
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    let mean = v.iter().map(|&x| x as f64).sum::<f64>() / n as f64;
    let var = v.iter().map(|&x| (x as f64 - mean).powi(2)).sum::<f64>() / n as f64;
    let pct = |p: f64| -> f64 {
        let idx = p / 100.0 * (n - 1) as f64;
        let lo = idx.floor() as usize;
        let hi = idx.ceil() as usize;
        let frac = idx - lo as f64;
        v[lo] as f64 * (1.0 - frac) + v[hi] as f64 * frac
    };
    Some(Dist {
        n,
        mean,
        median: pct(50.0),
        p05: pct(5.0),
        p25: pct(25.0),
        p75: pct(75.0),
        p95: pct(95.0),
        min: v[0] as f64,
        max: v[n - 1] as f64,
        stddev: var.sqrt(),
    })
}

fn print_dist(label: &str, d: &Dist) {
    println!("{label}: n={}", d.n);
    println!(
        "  mean={:+.3}  median={:+.3}  stddev={:.3}",
        d.mean, d.median, d.stddev
    );
    println!(
        "  min={:+.3}  p05={:+.3}  p25={:+.3}  p75={:+.3}  p95={:+.3}  max={:+.3}",
        d.min, d.p05, d.p25, d.p75, d.p95, d.max
    );
}

fn print_fractions(samples: &[&Sample], median: f64) {
    let n = samples.len();
    if n == 0 {
        return;
    }
    let pct = |count: usize| count as f64 / n as f64 * 100.0;
    let over_10 = samples.iter().filter(|s| s.dz > 1.0).count();
    let over_08 = samples.iter().filter(|s| s.dz > 0.8).count();
    let under_neg10 = samples.iter().filter(|s| s.dz < -1.0).count();
    let band_015 = samples
        .iter()
        .filter(|s| (s.dz as f64 - median).abs() <= 0.15)
        .count();
    let band_025 = samples
        .iter()
        .filter(|s| (s.dz as f64 - median).abs() <= 0.25)
        .count();
    println!(
        "  dz > 1.0 m: {over_10}/{n} = {:.3}%   dz > 0.8 m: {over_08}/{n} = {:.3}%   dz < -1.0 m: {under_neg10}/{n} = {:.3}%",
        pct(over_10),
        pct(over_08),
        pct(under_neg10)
    );
    println!(
        "  |dz - median| <= 0.15 m: {:.3}%   <= 0.25 m: {:.3}%",
        pct(band_015),
        pct(band_025)
    );
}

fn print_grid(samples: &[&Sample], global_median: f64) {
    let mut cells: HashMap<(i64, i64), Vec<f32>> = HashMap::new();
    for s in samples {
        let key = (
            (s.x / CELL_SIZE).floor() as i64,
            (s.y / CELL_SIZE).floor() as i64,
        );
        cells.entry(key).or_default().push(s.dz);
    }
    let mut ranked: Vec<((i64, i64), usize, f64)> = cells
        .iter()
        .filter(|(_, v)| v.len() >= MIN_CELL_SAMPLES)
        .map(|(&k, v)| (k, v.len(), median(v.clone())))
        .collect();
    println!(
        "  {} cells with >= {MIN_CELL_SAMPLES} samples (2 m grid); worst {} by |cell_median - global_median|:",
        ranked.len(),
        WORST_CELL_COUNT.min(ranked.len())
    );
    ranked.sort_by(|a, b| {
        (b.2 - global_median)
            .abs()
            .total_cmp(&(a.2 - global_median).abs())
    });
    println!(
        "  {:>9} {:>9} {:>7} {:>10} {:>8}",
        "cell_x", "cell_y", "n", "median_dz", "delta"
    );
    for (k, n, m) in ranked.iter().take(WORST_CELL_COUNT) {
        let cx = (k.0 as f32 + 0.5) * CELL_SIZE;
        let cy = (k.1 as f32 + 0.5) * CELL_SIZE;
        println!(
            "  {cx:>9.1} {cy:>9.1} {n:>7} {m:>+10.3} {:>+8.3}",
            m - global_median
        );
    }
}

fn report(
    totals: &Counters,
    samples: &[Sample],
    players: &HashSet<u16>,
    sampler: &mut FloorSampler,
    targets: Option<&str>,
) -> anyhow::Result<()> {
    println!();
    println!("=== totals ===");
    println!(
        "datagrams={} inbound={} onfoot_207_seen={} distinct_players={}",
        totals.datagrams,
        totals.inbound,
        totals.onfoot_seen,
        players.len()
    );
    println!(
        "dropped: decode_failed={} nonfinite={} surfing={} vertical_motion={} off_mesh={}  kept={}",
        totals.decode_failed,
        totals.nonfinite,
        totals.surfing,
        totals.vertical,
        totals.off_mesh,
        totals.kept
    );
    println!(
        "vertical filter: |move_speed.z| <= {VERTICAL_SPEED_CUTOFF} units/frame (= 1.0 m/s at the \
         50 fps sync convention moveSpeed = speed/50): standing/walking peds report ~0 vertical \
         velocity, jumps and falls exceed it"
    );

    let all: Vec<&Sample> = samples.iter().collect();
    let Some(dist) = distribution(&samples.iter().map(|s| s.dz).collect::<Vec<_>>()) else {
        println!("no samples kept — nothing to report");
        return Ok(());
    };
    println!();
    println!("=== dz = reported_z - nav_floor_z (all kept samples) ===");
    print_dist("dz", &dist);
    print_fractions(&all, dist.median);
    println!();
    println!("=== spatial grid, all kept samples ===");
    println!("  global median dz = {:+.3}", dist.median);
    print_grid(&all, dist.median);

    let boxed: Vec<&Sample> = samples
        .iter()
        .filter(|s| {
            s.x >= SAWMILL_MIN[0]
                && s.x <= SAWMILL_MAX[0]
                && s.y >= SAWMILL_MIN[1]
                && s.y <= SAWMILL_MAX[1]
        })
        .collect();
    println!();
    println!(
        "=== sawmill work-area box x[{}, {}] y[{}, {}] ===",
        SAWMILL_MIN[0], SAWMILL_MAX[0], SAWMILL_MIN[1], SAWMILL_MAX[1]
    );
    match distribution(&boxed.iter().map(|s| s.dz).collect::<Vec<_>>()) {
        Some(bd) => {
            print_dist("dz (box)", &bd);
            print_fractions(&boxed, bd.median);
            println!("  box median dz = {:+.3}", bd.median);
            print_grid(&boxed, bd.median);
        }
        None => println!("no kept samples inside the box"),
    }

    let all_dz: Vec<f32> = samples.iter().map(|s| s.dz).collect();
    let box_dz: Vec<f32> = boxed.iter().map(|s| s.dz).collect();
    println!();
    println!("=== trimmed ped-origin estimate: dz in [{TRIM_MIN}, {TRIM_MAX}] only ===");
    print_trimmed("trimmed dz (all)", &all_dz);
    print_trimmed("trimmed dz (box)", &box_dz);
    let best_all = lift_sweep("all kept samples", &all_dz);
    let best_box = lift_sweep("sawmill box", &box_dz);
    if let Some(l) = best_box.or(best_all) {
        let (real, confusion) = offender_report(samples, l, sampler);
        if let Some(path) = targets {
            route_report(path, sampler, &real, &confusion)?;
        }
    }
    Ok(())
}
