#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum GaitKind {
    Walk,
    Jog,
    Sprint,
}

const CHAIN: [f32; 4] = [0.0, 0.0311, 0.114, 0.1912];
const MOVING_EPS: f32 = 0.004;
const SLEW_EPS: f32 = 0.02;
const SEG_TIME_LO: f32 = 0.29;
const SEG_TIME_HI: f32 = 0.31;
// WHY: sprint is a separate moveState engaged by the sprint key, not mbr climbed past run, so this leg is
// deliberately faster than the mbr-capped SEG_TIME_* — unifying them reinstates the over-long ramp.
const SPRINT_ENGAGE_TIME: f32 = 0.13;

impl GaitKind {
    pub(crate) fn nominal_wire_mag(self) -> f32 {
        CHAIN[self.chain_index()]
    }

    fn chain_index(self) -> usize {
        match self {
            GaitKind::Walk => 1,
            GaitKind::Jog => 2,
            GaitKind::Sprint => 3,
        }
    }

    fn band(self) -> (f32, f32) {
        match self {
            GaitKind::Walk => (0.0270, 0.0355),
            GaitKind::Jog => (0.1070, 0.1260),
            GaitKind::Sprint => (0.1500, 0.2400),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TickInput {
    pub dt: f32,
    pub heading_delta: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LocomotionSample {
    /// INVARIANT: the caller must advance the reported position by exactly
    /// `move_speed_mag * 50.0` u/s along its heading and write the same magnitude into the wire
    /// `move_speed` XY — scaling either side independently re-creates the drift-then-snap teleport
    /// fingerprint this model exists to remove.
    pub move_speed_mag: f32,
    /// Ground-contact vertical noise, always <= 0. This model only describes a ped in contact with
    /// the floor; an airborne ped's vertical velocity is an integrated fall, which belongs to
    /// whatever owns that fall, never here.
    /// INVARIANT: the caller must not add slope-grade terms on top — sustained `grade * mag`
    /// vertical velocity is the p95-level fingerprint the real engine (hard zero every grounded
    /// frame) never produces.
    pub vz: f32,
    pub up_down: i16,
    pub left_right: i16,
    pub animation_id: u16,
    pub animation_flags: u16,
}

type AnimPair = (u16, u16);

const ANIM_IDLE: [(AnimPair, u32); 3] = [
    ((1189, 0x8004), 78),
    ((1189, 0x80E8), 12),
    ((0xFFFF, 0xFFFF), 10),
];
const ANIM_WALK: [(AnimPair, u32); 2] = [((1231, 0x8004), 85), ((1231, 0x8002), 15)];
const ANIM_JOG: [(AnimPair, u32); 4] = [
    ((1231, 0x8002), 62),
    ((1231, 0x8004), 18),
    ((1231, 0x80E8), 12),
    ((1266, 0x8002), 8),
];
const ANIM_SPRINT: [(AnimPair, u32); 4] = [
    ((1231, 0x8002), 55),
    ((1266, 0x8002), 25),
    ((1231, 0x80E8), 12),
    ((1231, 0x8004), 8),
];

struct Pcg32 {
    state: u64,
}

const PCG_MUL: u64 = 6364136223846793005;
const PCG_INC: u64 = 1442695040888963407;

impl Pcg32 {
    fn new(seed: u64) -> Self {
        let mut rng = Self {
            state: seed.wrapping_add(PCG_INC),
        };
        rng.next_u32();
        rng
    }

    fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old.wrapping_mul(PCG_MUL).wrapping_add(PCG_INC);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    fn unit(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.unit()
    }

    fn range_u32(&mut self, lo: u32, hi: u32) -> u32 {
        lo + self.next_u32() % (hi - lo + 1)
    }

    fn normal(&mut self) -> f32 {
        let mut sum = 0.0f32;
        for _ in 0..12 {
            sum += self.unit();
        }
        sum - 6.0
    }
}

fn fold(mut x: f32, lo: f32, hi: f32) -> f32 {
    for _ in 0..4 {
        if x > hi {
            x = hi + hi - x;
        } else if x < lo {
            x = lo + lo - x;
        } else {
            return x;
        }
    }
    x.clamp(lo, hi)
}

fn wire_to_mbr(wire: f32) -> f32 {
    if wire <= CHAIN[1] {
        wire / CHAIN[1]
    } else {
        1.0 + (wire - CHAIN[1]) / (CHAIN[2] - CHAIN[1])
    }
}

fn mbr_to_wire(mbr: f32) -> f32 {
    if mbr <= 1.0 {
        mbr * CHAIN[1]
    } else {
        CHAIN[1] + (mbr - 1.0) * (CHAIN[2] - CHAIN[1])
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum AnimState {
    Idle,
    Moving(GaitKind),
}

pub(crate) struct LocomotionModel {
    rng: Pcg32,
    target: Option<GaitKind>,
    mag: f32,
    cruising: bool,
    seg_time: f32,
    cruise_center: f32,
    center_hold: u32,
    sprint_goal: f32,
    anim: AnimPair,
    anim_state: AnimState,
    anim_hold: u32,
    pedal: i16,
    pedal_hold: u32,
}

impl LocomotionModel {
    pub(crate) fn new(seed: u64) -> Self {
        let mut rng = Pcg32::new(seed);
        let seg_time = rng.range(SEG_TIME_LO, SEG_TIME_HI);
        Self {
            rng,
            target: None,
            mag: 0.0,
            cruising: false,
            seg_time,
            cruise_center: 0.0,
            center_hold: 0,
            sprint_goal: 0.0,
            anim: (1189, 0x8004),
            anim_state: AnimState::Idle,
            anim_hold: 0,
            pedal: -128,
            pedal_hold: 0,
        }
    }

    pub(crate) fn set_target(&mut self, target: Option<GaitKind>) {
        if target == self.target {
            return;
        }
        self.target = target;
        self.cruising = false;
        self.seg_time = self.rng.range(SEG_TIME_LO, SEG_TIME_HI);
    }

    pub(crate) fn is_stopped(&self) -> bool {
        self.target.is_none() && self.mag == 0.0
    }

    pub(crate) fn tick(&mut self, input: TickInput) -> LocomotionSample {
        let dt = input.dt.clamp(0.01, 0.5);
        self.step_mag(dt);
        let moving = self.mag > MOVING_EPS;
        let (up_down, left_right) = self.step_analog(moving, input.heading_delta);
        let (animation_id, animation_flags) = self.step_anim(moving);
        let vz = self.step_vz(moving);
        LocomotionSample {
            move_speed_mag: self.mag,
            vz,
            up_down,
            left_right,
            animation_id,
            animation_flags,
        }
    }

    fn step_mag(&mut self, dt: f32) {
        let Some(target) = self.target else {
            self.ramp_down(dt, 0.0);
            if self.mag <= MOVING_EPS {
                self.mag = 0.0;
            }
            return;
        };
        let (lo, hi) = target.band();
        if self.cruising {
            if self.mag < lo || self.mag > hi {
                self.cruising = false;
            } else {
                self.cruise(target, lo, hi);
                return;
            }
        }
        if self.mag < lo {
            self.ramp_up(dt, target, lo, hi);
        } else if self.mag > hi {
            self.ramp_down(dt, hi);
            if self.mag <= hi {
                self.mag = hi - self.rng.range(0.0, 0.2 * (hi - lo));
                self.enter_band(target, lo, hi);
            }
        } else {
            self.enter_band(target, lo, hi);
        }
    }

    fn ramp_up(&mut self, dt: f32, target: GaitKind, lo: f32, hi: f32) {
        let next = if self.mag < CHAIN[2] {
            let mbr = (wire_to_mbr(self.mag) + dt / self.seg_time).min(2.0);
            mbr_to_wire(mbr)
        } else {
            let rate = (CHAIN[3] - CHAIN[2]) / SPRINT_ENGAGE_TIME;
            self.mag + rate * dt
        };
        if next >= lo {
            self.mag = fold(next, lo, hi);
            self.enter_band(target, lo, hi);
        } else {
            self.mag = next.max(0.0);
        }
    }

    fn ramp_down(&mut self, dt: f32, floor: f32) {
        if self.mag <= floor {
            return;
        }
        let next = if self.mag > CHAIN[2] {
            let rate = (CHAIN[3] - CHAIN[2]) / SPRINT_ENGAGE_TIME;
            self.mag - rate * dt
        } else {
            let mbr = (wire_to_mbr(self.mag) - dt / self.seg_time).max(0.0);
            mbr_to_wire(mbr)
        };
        self.mag = next.max(floor);
    }

    fn enter_band(&mut self, target: GaitKind, lo: f32, hi: f32) {
        self.cruising = true;
        self.mag = fold(self.mag, lo, hi);
        match target {
            GaitKind::Sprint => self.sprint_goal = self.rng.range(lo, hi),
            GaitKind::Jog | GaitKind::Walk => {
                self.reroll_cruise_center(target);
            }
        }
    }

    fn reroll_cruise_center(&mut self, target: GaitKind) {
        let (center_lo, center_hi) = if target == GaitKind::Jog {
            (0.1110, 0.1190)
        } else {
            (0.0295, 0.0325)
        };
        self.cruise_center = self.rng.range(center_lo, center_hi);
        self.center_hold = self.rng.range_u32(30, 80);
    }

    fn cruise(&mut self, target: GaitKind, lo: f32, hi: f32) {
        match target {
            GaitKind::Sprint => {
                let gap = self.sprint_goal - self.mag;
                let step = (self.rng.range(0.018, 0.062) + self.rng.normal() * 0.003).max(0.004);
                self.mag += gap.signum() * step;
                if (self.sprint_goal - self.mag) * gap <= 0.0 {
                    self.sprint_goal = self.rng.range(lo, hi);
                }
            }
            GaitKind::Jog | GaitKind::Walk => {
                if self.center_hold == 0 {
                    self.reroll_cruise_center(target);
                } else {
                    self.center_hold -= 1;
                }
                let sigma = if target == GaitKind::Jog {
                    0.0060
                } else {
                    0.0016
                };
                self.mag += 0.25 * (self.cruise_center - self.mag) + self.rng.normal() * sigma;
            }
        }
        self.mag = fold(self.mag, lo, hi);
    }

    fn step_analog(&mut self, moving: bool, heading_delta: f32) -> (i16, i16) {
        if !moving {
            self.pedal = -128;
            self.pedal_hold = 0;
            return (0, 0);
        }
        if self.pedal_hold > 0 {
            self.pedal_hold -= 1;
        } else if self.rng.unit() < 0.035 {
            self.pedal = -(self.rng.range_u32(120, 127) as i16);
            self.pedal_hold = self.rng.range_u32(2, 3);
        } else {
            self.pedal = -128;
        }
        let steer = if heading_delta > SLEW_EPS {
            128
        } else if heading_delta < -SLEW_EPS {
            -128
        } else {
            0
        };
        (self.pedal, steer)
    }

    fn step_anim(&mut self, moving: bool) -> AnimPair {
        let state = if moving {
            match self.target {
                Some(gait) => AnimState::Moving(gait),
                None => self.anim_state,
            }
        } else {
            AnimState::Idle
        };
        if state != self.anim_state || self.anim_hold == 0 {
            self.anim_state = state;
            self.anim = self.pick_anim(state);
            self.anim_hold = self.rng.range_u32(20, 60);
        } else {
            self.anim_hold -= 1;
        }
        self.anim
    }

    fn pick_anim(&mut self, state: AnimState) -> AnimPair {
        let table: &[(AnimPair, u32)] = match state {
            AnimState::Idle => &ANIM_IDLE,
            AnimState::Moving(GaitKind::Walk) => &ANIM_WALK,
            AnimState::Moving(GaitKind::Jog) => &ANIM_JOG,
            AnimState::Moving(GaitKind::Sprint) => &ANIM_SPRINT,
        };
        let total: u32 = table.iter().map(|&(_, weight)| weight).sum();
        let mut roll = self.rng.range_u32(0, total - 1);
        for &(pair, weight) in table {
            if roll < weight {
                return pair;
            }
            roll -= weight;
        }
        table[table.len() - 1].0
    }

    fn step_vz(&mut self, moving: bool) -> f32 {
        let (p_zero, p_small) = if moving {
            (0.910, 0.075)
        } else {
            (0.995, 0.004)
        };
        let roll = self.rng.unit();
        if roll < p_zero {
            0.0
        } else if roll < p_zero + p_small || !moving {
            -(0.008 + self.rng.normal() * 0.0035).clamp(0.001, 0.016)
        } else {
            let u = self.rng.unit();
            -(0.02 + u * u * 0.09)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::HashMap;

    const TICK: TickInput = TickInput {
        dt: 0.1,
        heading_delta: 0.0,
    };

    fn pct(sorted: &[f32], p: f64) -> f32 {
        sorted[((sorted.len() - 1) as f64 * p).round() as usize]
    }

    fn warmed(seed: u64, gait: GaitKind) -> LocomotionModel {
        let mut model = LocomotionModel::new(seed);
        model.set_target(Some(gait));
        for _ in 0..30 {
            model.tick(TICK);
        }
        model
    }

    fn run_mags(seed: u64, gait: GaitKind, ticks: usize) -> Vec<f32> {
        let mut model = LocomotionModel::new(seed);
        model.set_target(Some(gait));
        (0..ticks)
            .map(|_| model.tick(TICK).move_speed_mag)
            .collect()
    }

    #[test]
    fn same_seed_reproduces_identical_stream() {
        let mut a = LocomotionModel::new(0xA11CE);
        let mut b = LocomotionModel::new(0xA11CE);
        a.set_target(Some(GaitKind::Sprint));
        b.set_target(Some(GaitKind::Sprint));
        for i in 0..600 {
            if i == 200 {
                a.set_target(Some(GaitKind::Jog));
                b.set_target(Some(GaitKind::Jog));
            }
            if i == 400 {
                a.set_target(None);
                b.set_target(None);
            }
            if i == 420 {
                a.set_target(Some(GaitKind::Walk));
                b.set_target(Some(GaitKind::Walk));
            }
            assert_eq!(a.tick(TICK), b.tick(TICK));
        }

        let first = run_mags(1, GaitKind::Jog, 50);
        let second = run_mags(2, GaitKind::Jog, 50);
        assert_ne!(first, second);
    }

    #[test]
    fn no_bit_exact_plateau_and_band_envelope() {
        for (seed, gait) in [
            (11, GaitKind::Jog),
            (12, GaitKind::Sprint),
            (13, GaitKind::Walk),
        ] {
            let mags = run_mags(seed, gait, 3000);
            assert!(mags.iter().all(|&m| (0.0..=0.26).contains(&m)));
            let (lo, hi) = gait.band();
            assert!(
                mags[20..].iter().all(|&m| m >= lo && m <= hi),
                "{gait:?}: cruise left the band"
            );

            let rounded: Vec<i64> = mags[20..]
                .iter()
                .map(|&m| (f64::from(m) * 10000.0).round() as i64)
                .collect();
            let mut run = 1usize;
            let mut max_run = 1usize;
            for pair in rounded.windows(2) {
                if pair[0] == pair[1] {
                    run += 1;
                    max_run = max_run.max(run);
                } else {
                    run = 1;
                }
            }
            assert!(max_run <= 3, "{gait:?}: {max_run}-tick bit-exact plateau");

            let mut counts: HashMap<i64, u32> = HashMap::new();
            for &value in &rounded {
                *counts.entry(value).or_insert(0) += 1;
            }
            let modal = counts.values().copied().max().unwrap_or(0);
            let modal_frac = f64::from(modal) / rounded.len() as f64;
            assert!(
                modal_frac < 0.03,
                "{gait:?}: modal 4dp value at {modal_frac:.4}"
            );
        }
    }

    #[test]
    fn delta_percentiles_track_real_capture() {
        let mut model = LocomotionModel::new(21);
        let mut mags = Vec::new();
        let schedule = [
            (GaitKind::Jog, 1500),
            (GaitKind::Sprint, 1500),
            (GaitKind::Jog, 500),
            (GaitKind::Sprint, 500),
        ];
        for (gait, ticks) in schedule {
            model.set_target(Some(gait));
            for _ in 0..ticks {
                mags.push(model.tick(TICK).move_speed_mag);
            }
        }
        let mut deltas: Vec<f32> = mags[15..]
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).abs())
            .collect();
        deltas.sort_by(f32::total_cmp);
        let p50 = pct(&deltas, 0.50);
        let p95 = pct(&deltas, 0.95);
        assert!(
            (0.008..=0.028).contains(&p50),
            "|dv| p50 {p50:.4} outside the real 0.012-0.017 neighbourhood"
        );
        assert!(
            (0.045..=0.085).contains(&p95),
            "|dv| p95 {p95:.4} outside the real 0.065-0.069 neighbourhood"
        );
    }

    #[test]
    fn ramp_chains_through_adjacent_gaits() {
        for seed in [31u64, 32, 33, 34, 35] {
            let mut model = LocomotionModel::new(seed);
            model.set_target(Some(GaitKind::Sprint));
            let mags: Vec<f32> = (0..20).map(|_| model.tick(TICK).move_speed_mag).collect();
            let entry = mags
                .iter()
                .position(|&m| m >= 0.150)
                .expect("never reached the sprint band");
            assert!(
                (6..=9).contains(&(entry + 1)),
                "seed {seed}: sprint band entry at tick {}",
                entry + 1
            );

            let mut deltas = vec![mags[0]];
            for pair in mags[..=entry].windows(2) {
                deltas.push(pair[1] - pair[0]);
            }
            let max_delta = deltas.iter().copied().fold(f32::MIN, f32::max);
            let min_delta = deltas.iter().copied().fold(f32::MAX, f32::min);
            assert!(min_delta > 0.0, "seed {seed}: ramp not monotone");
            assert!(
                max_delta > min_delta * 1.6,
                "seed {seed}: ramp slopes uniform ({min_delta:.4}..{max_delta:.4}), not chained"
            );

            let mbr = |mag: f32| {
                if mag <= CHAIN[1] {
                    mag / CHAIN[1]
                } else {
                    1.0 + (mag - CHAIN[1]) / (CHAIN[2] - CHAIN[1])
                }
            };
            let mut prev = 0.0f32;
            for &mag in mags[..=entry].iter().take_while(|&&m| m <= CHAIN[2]) {
                let mbr_step = mbr(mag) - mbr(prev);
                assert!(
                    mbr_step <= 0.35,
                    "seed {seed}: capped-region mbr step {mbr_step:.3} above the 3.5/s blend cap"
                );
                prev = mag;
            }

            let sprint_leg = mags[..=entry].iter().filter(|&&m| m > CHAIN[2]).count();
            assert!(
                sprint_leg <= 2,
                "seed {seed}: jog→sprint leg took {sprint_leg} ticks — not the fast anim engage"
            );
        }

        for seed in [36u64, 37, 38] {
            let mut model = LocomotionModel::new(seed);
            model.set_target(Some(GaitKind::Jog));
            let mags: Vec<f32> = (0..20).map(|_| model.tick(TICK).move_speed_mag).collect();
            let entry = mags
                .iter()
                .position(|&m| m >= 0.107)
                .expect("never reached the jog band");
            assert!(
                (5..=7).contains(&(entry + 1)),
                "seed {seed}: jog band entry at tick {}",
                entry + 1
            );
        }
    }

    #[test]
    fn analog_follows_turn_and_forward_model() {
        let mut model = warmed(41, GaitKind::Jog);
        let mut inputs: Vec<f32> = Vec::new();
        for corner in 0..400 {
            let sign = if corner % 2 == 0 { 1.0f32 } else { -1.0 };
            inputs.extend(std::iter::repeat_n(0.0f32, 14));
            inputs.extend(std::iter::repeat_n(0.6 * sign, 3));
            inputs.push(0.01 * sign);
        }

        let total = inputs.len();
        let (mut steering, mut forward, mut tail) = (0usize, 0usize, 0usize);
        for &heading_delta in &inputs {
            let sample = model.tick(TickInput {
                dt: 0.1,
                heading_delta,
            });
            if heading_delta > SLEW_EPS {
                assert_eq!(sample.left_right, 128);
            } else if heading_delta < -SLEW_EPS {
                assert_eq!(sample.left_right, -128);
            } else {
                assert_eq!(sample.left_right, 0);
            }
            if sample.left_right != 0 {
                steering += 1;
            }
            assert!(
                sample.up_down == -128 || (-127..=-120).contains(&sample.up_down),
                "moving up_down {} outside forward range",
                sample.up_down
            );
            if sample.up_down == -128 {
                forward += 1;
            } else {
                tail += 1;
            }
        }
        let steer_frac = steering as f64 / total as f64;
        assert!(
            (steer_frac - 3.0 / 18.0).abs() < 1e-9,
            "steer duty {steer_frac:.4} != turning duty"
        );
        let forward_frac = forward as f64 / total as f64;
        let tail_frac = tail as f64 / total as f64;
        assert!(
            (0.80..=0.95).contains(&forward_frac),
            "fwd {forward_frac:.3}"
        );
        assert!((0.05..=0.20).contains(&tail_frac), "tail {tail_frac:.3}");

        model.set_target(None);
        for _ in 0..20 {
            model.tick(TICK);
        }
        let idle = model.tick(TickInput {
            dt: 0.1,
            heading_delta: 0.5,
        });
        assert_eq!(idle.left_right, 0);
        assert_eq!(idle.up_down, 0);
    }

    #[test]
    fn ground_contact_vz_is_hard_zero_with_a_tiny_downward_tail() {
        let mut model = warmed(51, GaitKind::Jog);
        let total = 20000usize;
        let vzs: Vec<f32> = (0..total).map(|_| model.tick(TICK).vz).collect();

        assert!(vzs.iter().all(|&v| (-0.16..=0.0).contains(&v)));
        let zero_frac = vzs.iter().filter(|&&v| v == 0.0).count() as f64 / total as f64;
        assert!((0.85..=0.97).contains(&zero_frac), "zeros {zero_frac:.3}");

        let mut magnitudes: Vec<f32> = vzs.iter().map(|v| v.abs()).collect();
        magnitudes.sort_by(f32::total_cmp);
        assert_eq!(pct(&magnitudes, 0.50), 0.0);
        let p95 = pct(&magnitudes, 0.95);
        let p99 = pct(&magnitudes, 0.99);
        assert!((0.004..=0.018).contains(&p95), "|vz| p95 {p95:.4}");
        assert!((0.015..=0.12).contains(&p99), "|vz| p99 {p99:.4}");
    }

    #[test]
    fn animation_pairs_per_gait() {
        let in_table = |table: &[(AnimPair, u32)], pair: AnimPair| {
            table.iter().any(|&(entry, _)| entry == pair)
        };

        let cases = [
            (
                61u64,
                Some(GaitKind::Jog),
                &ANIM_JOG[..],
                (1231, 0x8002),
                0.45,
            ),
            (
                62,
                Some(GaitKind::Walk),
                &ANIM_WALK[..],
                (1231, 0x8004),
                0.60,
            ),
            (
                63,
                Some(GaitKind::Sprint),
                &ANIM_SPRINT[..],
                (1231, 0x8002),
                0.35,
            ),
            (64, None, &ANIM_IDLE[..], (1189, 0x8004), 0.60),
        ];
        for (seed, target, table, dominant, dominant_min) in cases {
            let mut model = LocomotionModel::new(seed);
            model.set_target(target);
            for _ in 0..30 {
                model.tick(TICK);
            }
            let total = 6000usize;
            let pairs: Vec<AnimPair> = (0..total)
                .map(|_| {
                    let sample = model.tick(TICK);
                    (sample.animation_id, sample.animation_flags)
                })
                .collect();
            assert!(pairs.iter().all(|&p| in_table(table, p)));
            let dominant_frac =
                pairs.iter().filter(|&&p| p == dominant).count() as f64 / total as f64;
            assert!(
                dominant_frac >= dominant_min,
                "target {target:?}: dominant pair at {dominant_frac:.3}"
            );
            let changes = pairs.windows(2).filter(|w| w[0] != w[1]).count();
            assert!(
                changes <= total / 10,
                "target {target:?}: anim flips {changes} times"
            );
        }
    }

    #[test]
    fn stop_ramps_down_to_exact_zero() {
        let mut model = warmed(71, GaitKind::Sprint);
        model.set_target(None);
        let mags: Vec<f32> = (0..20).map(|_| model.tick(TICK).move_speed_mag).collect();
        assert!(model.is_stopped());
        assert_eq!(*mags.last().unwrap(), 0.0);
        let first_zero = mags.iter().position(|&m| m == 0.0).unwrap();
        assert!(
            (5..=15).contains(&first_zero),
            "stopped at tick {first_zero}"
        );
        for pair in mags[..first_zero].windows(2) {
            assert!(pair[1] < pair[0] + 0.004, "decel not monotone");
        }

        let idle = model.tick(TICK);
        assert_eq!(idle.move_speed_mag, 0.0);
        assert_eq!(idle.up_down, 0);
        assert_eq!(idle.left_right, 0);
    }
}
