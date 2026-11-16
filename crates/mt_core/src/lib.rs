//! motor-therm core: lumped two-node thermal model of a robot joint motor.
//!
//! Conventions (textbook, see README):
//! - `kt` is the peak-amplitude (dq, amplitude-invariant) torque constant: τ_motor = Kt·I_q.
//! - Copper loss P = 1.5·I_q²·R_ph(T), with R_ph = R_pp / 2 and R(T) = R25·(1 + α(T − 25)).
//! - Winding node C1 = τ1/Rth1 couples to the housing through Rth1; housing node C2 = τ2/Rth2
//!   couples to ambient through Rth2·cooling. The cooling factor changes resistance only.
//! - Only copper loss is modelled (no iron, eddy or drive losses); ambient is constant.

/// Gravity used for payload torque (m/s²).
pub const G: f64 = 9.81;
/// Temperatures are capped here so a runaway case stays finite for plotting.
pub const T_CAP: f64 = 1000.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motor {
    /// Torque constant, N·m/A (peak-amplitude convention).
    pub kt: f64,
    /// Phase-to-phase resistance at 25 °C, Ω.
    pub r_pp25: f64,
    /// Copper temperature coefficient, 1/K.
    pub alpha: f64,
    pub ratio: f64,
    /// Gearbox efficiency 0..1.
    pub eta: f64,
    /// Winding → housing, K/W.
    pub rth1: f64,
    /// Housing → ambient (free air), K/W.
    pub rth2: f64,
    /// Winding / housing thermal time constants, s.
    pub tau1: f64,
    pub tau2: f64,
    pub t_amb: f64,
    pub t_limit: f64,
    /// Multiplies Rth2 (1.0 free air, 0.5 fan / heatsink).
    pub cooling: f64,
}

impl Motor {
    /// Copy with every parameter clamped to a physically usable range (no NaN / zero divisions).
    pub fn sanitized(&self) -> Motor {
        let f = |v: f64, lo: f64, hi: f64, d: f64| if v.is_finite() { v.clamp(lo, hi) } else { d };
        Motor {
            kt: f(self.kt, 1e-4, 100.0, 0.1),
            r_pp25: f(self.r_pp25, 1e-4, 1e3, 1.0),
            alpha: f(self.alpha, 0.0, 0.02, 0.00393),
            ratio: f(self.ratio, 0.01, 1e4, 1.0),
            eta: f(self.eta, 0.05, 1.0, 0.9),
            rth1: f(self.rth1, 1e-3, 1e3, 1.0),
            rth2: f(self.rth2, 1e-3, 1e3, 1.0),
            tau1: f(self.tau1, 1e-3, 1e5, 30.0),
            tau2: f(self.tau2, 1e-3, 1e6, 600.0),
            t_amb: f(self.t_amb, -50.0, 150.0, 25.0),
            t_limit: f(self.t_limit, -40.0, 400.0, 120.0),
            cooling: f(self.cooling, 0.05, 10.0, 1.0),
        }
    }
    pub fn r_ph(&self, t: f64) -> f64 {
        0.5 * self.r_pp25 * (1.0 + self.alpha * (t - 25.0))
    }
    pub fn c1(&self) -> f64 {
        self.tau1 / self.rth1
    }
    pub fn c2(&self) -> f64 {
        self.tau2 / self.rth2
    }
    pub fn rth2_eff(&self) -> f64 {
        self.rth2 * self.cooling
    }
    /// Phase current amplitude for an output (joint) torque.
    pub fn current(&self, out_torque: f64) -> f64 {
        out_torque / (self.ratio * self.eta) / self.kt
    }
    /// Copper loss (W) at current amplitude `i` and winding temperature `t`.
    pub fn loss(&self, i: f64, t: f64) -> f64 {
        1.5 * i * i * self.r_ph(t).max(0.0)
    }
    /// Steady winding temperature for a constant mean of I², or `None` on thermal runaway
    /// (α·P25·Rtot ≥ 1 means the loss grows faster than the network can shed it).
    pub fn steady_winding(&self, mean_i2: f64) -> Option<f64> {
        let rt = self.rth1 + self.rth2_eff();
        let k = 1.5 * mean_i2 * 0.5 * self.r_pp25 * rt; // K of rise per unit (1 + α(T−25))
        let den = 1.0 - self.alpha * k;
        (den > 1e-9).then(|| (self.t_amb + k * (1.0 - 25.0 * self.alpha)) / den)
    }
    /// Continuous output torque that settles exactly at the winding limit (closed form with R(T)).
    pub fn continuous_torque(&self) -> f64 {
        let rise = self.t_limit - self.t_amb;
        if rise <= 0.0 {
            return 0.0;
        }
        let i = (rise / (1.5 * self.r_ph(self.t_limit) * (self.rth1 + self.rth2_eff()))).sqrt();
        i * self.kt * self.ratio * self.eta
    }
}

/// One constant-torque piece of a repeating load cycle (joint output side).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub dur: f64,
    pub torque: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Cycle {
    pub segs: Vec<Segment>,
}

impl Cycle {
    /// Keeps only usable segments (positive finite duration); an empty cycle becomes 1 s at 0 N·m.
    pub fn new(segs: &[Segment]) -> Cycle {
        let mut s: Vec<Segment> =
            segs.iter().filter(|s| s.dur.is_finite() && s.dur > 1e-3 && s.torque.is_finite()).copied().collect();
        if s.is_empty() {
            s.push(Segment { dur: 1.0, torque: 0.0 });
        }
        Cycle { segs: s }
    }
    pub fn period(&self) -> f64 {
        self.segs.iter().map(|s| s.dur).sum()
    }
    /// Segment index active at time `t` (periodic).
    pub fn index_at(&self, t: f64) -> usize {
        let mut u = t.rem_euclid(self.period());
        for (i, s) in self.segs.iter().enumerate() {
            if u < s.dur {
                return i;
            }
            u -= s.dur;
        }
        self.segs.len() - 1
    }
    pub fn torque_at(&self, t: f64) -> f64 {
        self.segs[self.index_at(t)].torque
    }
    pub fn rms(&self) -> f64 {
        (self.segs.iter().map(|s| s.torque * s.torque * s.dur).sum::<f64>() / self.period()).sqrt()
    }
    /// Gravity torque of a payload at the arm tip added to every loaded segment (same sign).
    pub fn with_payload(&self, kg: f64, arm: f64) -> Cycle {
        let extra = kg.max(0.0) * G * arm.max(0.0);
        let segs = self.segs.iter().map(|s| Segment { torque: s.torque + extra * sign(s.torque), ..*s }).collect();
        Cycle { segs }
    }
}

fn sign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Winding / housing temperatures.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    pub tw: f64,
    pub th: f64,
}

impl Motor {
    pub fn cold(&self) -> State {
        State { tw: self.t_amb, th: self.t_amb }
    }
    /// Time resolution for traces and limit detection: a fraction of the fastest network time scale.
    pub fn res_dt(&self) -> f64 {
        let fast = (self.c2() * self.rth1 * self.rth2_eff() / (self.rth1 + self.rth2_eff())).min(self.tau1);
        fast / 10.0
    }
    /// Advance `dt` at constant I² with the exact solution of the linear 2-node system.
    /// The loss is affine in winding temperature (P = p0 + b·Tw), so x' = A·x + u holds exactly and
    /// x(dt) = x* + e^{A·dt}(x0 − x*). Unconditionally stable; a positive eigenvalue means runaway.
    pub fn advance(&self, s: State, i2: f64, dt: f64) -> State {
        let k = 0.75 * i2 * self.r_pp25;
        let (p0, b) = (k * (1.0 - 25.0 * self.alpha), k * self.alpha);
        let (c1, c2, r1, r2) = (self.c1(), self.c2(), self.rth1, self.rth2_eff());
        let (mut a11, a12, a21, a22) = ((b - 1.0 / r1) / c1, 1.0 / (r1 * c1), 1.0 / (r1 * c2), -(1.0 / r1 + 1.0 / r2) / c2);
        let u = [p0 / c1, self.t_amb / (r2 * c2)];
        if (a11 * a22 - a12 * a21).abs() < 1e-12 * a12 * a21 {
            a11 -= 1e-6 * a11.abs().max(1e-12); // exactly on the runaway boundary: nudge off it
        }
        let det = a11 * a22 - a12 * a21;
        let xs = [-(a22 * u[0] - a12 * u[1]) / det, -(-a21 * u[0] + a11 * u[1]) / det];
        let (m, d) = (0.5 * (a11 + a22), (0.25 * (a11 - a22).powi(2) + a12 * a21).sqrt());
        let (l1, l2) = (m + d, m - d);
        let (e1, e2) = ((l1 * dt).min(50.0).exp(), (l2 * dt).exp());
        // e^{At} = (e1·(A − λ2 I) − e2·(A − λ1 I)) / (λ1 − λ2)
        let e = |aij: f64, diag: bool| (e1 * (aij - if diag { l2 } else { 0.0 }) - e2 * (aij - if diag { l1 } else { 0.0 })) / (l1 - l2);
        let (y0, y1) = (s.tw - xs[0], s.th - xs[1]);
        let fin = |v: f64| if v.is_finite() { v.min(T_CAP) } else { T_CAP };
        State {
            tw: fin(xs[0] + e(a11, true) * y0 + e(a12, false) * y1),
            th: fin(xs[1] + e(a21, false) * y0 + e(a22, true) * y1),
        }
    }
}

/// Simulation output: downsampled traces plus metrics from the full-resolution steps.
#[derive(Clone, Debug, Default)]
pub struct Run {
    pub t: Vec<f64>,
    pub tw: Vec<f64>,
    pub th: Vec<f64>,
    pub peak_tw: f64,
    pub peak_th: f64,
    /// First time the winding reaches the limit (None = not within the simulated time).
    pub t_limit: Option<f64>,
    pub end: State,
}

/// Simulate `total` seconds of the repeating cycle from `start`, keeping about `samples` points.
pub fn simulate(m: &Motor, c: &Cycle, start: State, total: f64, samples: usize) -> Run {
    let i2: Vec<f64> = c.segs.iter().map(|s| m.current(s.torque).powi(2)).collect();
    // Exact steps are stable at any size; only resolution is bounded (≤ ~100k steps per run).
    let h = m.res_dt().max(total / 100_000.0);
    let every = (total / samples.max(1) as f64).max(1e-9);
    let mut r = Run { peak_tw: start.tw, peak_th: start.th, end: start, ..Run::default() };
    let (mut s, mut t, mut k, mut next) = (start, 0.0, 0usize, 0.0);
    let push = |r: &mut Run, t: f64, s: State| {
        r.t.push(t);
        r.tw.push(s.tw);
        r.th.push(s.th);
    };
    while t < total - 1e-12 {
        let seg_end = t + c.segs[k].dur;
        // Step to the next sample time or segment boundary, whichever comes first.
        while t < seg_end - 1e-12 && t < total - 1e-12 {
            if t >= next - 1e-12 {
                push(&mut r, t, s);
                next += every;
            }
            let t1 = seg_end.min(total).min(next.max(t + 1e-9)).min(t + h);
            let n = s;
            s = m.advance(s, i2[k], t1 - t);
            if r.t_limit.is_none() && s.tw >= m.t_limit {
                let f = ((m.t_limit - n.tw) / (s.tw - n.tw)).clamp(0.0, 1.0);
                r.t_limit = Some(t + f * (t1 - t));
            }
            r.peak_tw = r.peak_tw.max(s.tw);
            r.peak_th = r.peak_th.max(s.th);
            t = t1;
        }
        k = (k + 1) % c.segs.len();
    }
    push(&mut r, t, s);
    r.end = s;
    r
}

/// Periodic steady state (min / max winding temperature over one cycle), or `None` on runaway.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Periodic {
    pub tw_min: f64,
    pub tw_max: f64,
    pub th_mean: f64,
}

pub fn mean_i2(m: &Motor, c: &Cycle) -> f64 {
    c.segs.iter().map(|s| m.current(s.torque).powi(2) * s.dur).sum::<f64>() / c.period()
}

pub fn periodic(m: &Motor, c: &Cycle) -> Option<Periodic> {
    let mi2 = mean_i2(m, c);
    let tw = m.steady_winding(mi2)?;
    let p = m.loss(mi2.sqrt(), tw);
    // Warm start at the mean steady state, then let the ripple settle for a few fast time constants.
    let mut s = State { tw, th: m.t_amb + p * m.rth2_eff() };
    let per = c.period();
    let n = ((5.0 * m.tau1 / per).ceil() as usize).clamp(3, 2000);
    for _ in 0..n {
        s = simulate(m, c, s, per, 1).end;
    }
    let r = simulate(m, c, s, per, 64);
    let lo = r.tw.iter().cloned().fold(f64::INFINITY, f64::min);
    Some(Periodic { tw_min: lo.min(r.peak_tw), tw_max: r.peak_tw, th_mean: r.th.iter().sum::<f64>() / r.th.len() as f64 })
}

/// Largest fraction of time the peak-|torque| segment can take while the rest of the cycle keeps its
/// own mean I², so that the cycle mean I² stays at the continuous limit (mean-based, ignores ripple).
pub fn max_peak_duty(m: &Motor, c: &Cycle) -> f64 {
    let pk = c.segs.iter().enumerate().max_by(|a, b| a.1.torque.abs().total_cmp(&b.1.torque.abs())).map(|x| x.0).unwrap_or(0);
    let i2 = |tq: f64| m.current(tq).powi(2);
    let ip2 = i2(c.segs[pk].torque);
    let rest: f64 = c.segs.iter().enumerate().filter(|x| x.0 != pk).map(|x| i2(x.1.torque) * x.1.dur).sum();
    let rest_dur = c.period() - c.segs[pk].dur;
    let ir2 = if rest_dur > 0.0 { rest / rest_dur } else { 0.0 };
    let ic2 = i2(m.continuous_torque());
    if ip2 <= ic2 {
        1.0
    } else if ir2 >= ic2 {
        0.0
    } else {
        ((ic2 - ir2) / (ip2 - ir2)).clamp(0.0, 1.0)
    }
}

/// Max constant output torque that keeps the winding at or below the limit after `on` seconds from
/// cold (bisection; the winding rises monotonically under constant torque).
pub fn safe_torque(m: &Motor, on: f64) -> f64 {
    let ok = |tq: f64| simulate(m, &Cycle::new(&[Segment { dur: on, torque: tq }]), m.cold(), on, 1).end.tw <= m.t_limit;
    let mut lo = m.continuous_torque();
    if !ok(lo) {
        return lo;
    }
    let mut hi = lo.max(1e-3) * 2.0;
    let mut n = 0;
    while ok(hi) && n < 40 {
        lo = hi;
        hi *= 2.0;
        n += 1;
    }
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if ok(mid) {
            lo = mid
        } else {
            hi = mid
        }
    }
    lo
}

/// What-if sweep parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sweep {
    Cooling,
    Ratio,
}

/// Peak winding temperature over `total` seconds for `n` values of the swept parameter.
/// Cooling 0.3…1.5 (linear); gear ratio 0.4…2.5 × the current ratio (logarithmic).
pub fn sweep(m: &Motor, c: &Cycle, total: f64, which: Sweep, n: usize) -> Vec<(f64, f64)> {
    (0..n)
        .map(|k| {
            let u = k as f64 / (n - 1).max(1) as f64;
            let mut mm = *m;
            let x = match which {
                Sweep::Cooling => {
                    mm.cooling = 0.3 + 1.2 * u;
                    mm.cooling
                }
                Sweep::Ratio => {
                    mm.ratio = m.ratio * 0.4 * (2.5f64 / 0.4).powf(u);
                    mm.ratio
                }
            };
            (x, simulate(&mm, c, mm.cold(), total, 1).peak_tw)
        })
        .collect()
}

// ---------- presets (generic, "typical" orders of magnitude; not from any datasheet) ----------

pub const MOTOR_PRESETS: [(&str, Motor); 3] = [
    ("small", Motor { kt: 0.012, r_pp25: 2.0, alpha: 0.00393, ratio: 200.0, eta: 0.6, rth1: 3.0, rth2: 12.0, tau1: 20.0, tau2: 400.0, t_amb: 25.0, t_limit: 120.0, cooling: 1.0 }),
    ("qdd", Motor { kt: 0.09, r_pp25: 0.25, alpha: 0.00393, ratio: 9.0, eta: 0.95, rth1: 1.2, rth2: 2.5, tau1: 60.0, tau2: 900.0, t_amb: 25.0, t_limit: 120.0, cooling: 1.0 }),
    ("large", Motor { kt: 0.25, r_pp25: 0.4, alpha: 0.00393, ratio: 50.0, eta: 0.75, rth1: 0.6, rth2: 1.0, tau1: 120.0, tau2: 1800.0, t_amb: 25.0, t_limit: 120.0, cooling: 1.0 }),
];

/// Load-cycle presets as (duration s, torque as a fraction of the motor's continuous output torque),
/// so each preset is meaningful for every motor size.
pub const CYCLE_PRESETS: [(&str, &[(f64, f64)]); 4] = [
    ("hold", &[(1.0, 0.8)]),
    ("gait", &[(0.30, 1.4), (0.15, 2.2), (0.40, 0.25), (0.15, 1.8)]),
    ("pick", &[(0.4, 2.0), (1.5, 0.9), (0.4, -1.6), (1.7, 0.0)]),
    ("stairs", &[(0.35, 2.4), (0.25, 3.2), (0.40, 0.3), (0.20, 2.6)]),
];

pub fn cycle_preset(idx: usize, m: &Motor) -> Vec<Segment> {
    let tc = m.sanitized().continuous_torque();
    CYCLE_PRESETS[idx.min(CYCLE_PRESETS.len() - 1)].1.iter().map(|&(d, f)| Segment { dur: d, torque: (f * tc * 100.0).round() / 100.0 }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base() -> Motor {
        Motor { kt: 0.1, r_pp25: 0.4, alpha: 0.0, ratio: 10.0, eta: 1.0, rth1: 1.0, rth2: 2.0, tau1: 30.0, tau2: 300.0, t_amb: 25.0, t_limit: 120.0, cooling: 1.0 }
    }
    fn hold(tq: f64) -> Cycle {
        Cycle::new(&[Segment { dur: 1.0, torque: tq }])
    }
    /// Slow mode ≈ τ2 + τ1·Rth2/Rth1; ten of them settles any state.
    fn settle(m: &Motor) -> f64 {
        10.0 * (m.tau2 + m.tau1 * m.rth2_eff() / m.rth1)
    }

    #[test]
    fn zero_torque_stays_at_ambient() {
        let m = base();
        let r = simulate(&m, &hold(0.0), m.cold(), 600.0, 50);
        assert!(r.tw.iter().chain(&r.th).all(|t| (t - 25.0).abs() < 1e-9));
        assert_eq!(r.t_limit, None);
    }

    #[test]
    fn constant_loss_steady_state() {
        let m = base();
        // I = 10/(10·1)/0.1 = 10 A, P = 1.5·100·0.2 = 30 W → ΔT = 30·3 = 90 K.
        let r = simulate(&m, &hold(10.0), m.cold(), settle(&m), 10);
        let want = 25.0 + 30.0 * 3.0;
        assert!((r.end.tw - want).abs() / want < 0.005, "{} vs {want}", r.end.tw);
        assert!((m.steady_winding(100.0).unwrap() - want).abs() < 1e-9);
    }

    #[test]
    fn single_node_step_63_percent_at_tau2() {
        // Tiny τ1: the winding follows the housing; the housing rise is first order with τ2 = Rth2·C2.
        let m = Motor { tau1: 0.01, ..base() };
        let r = simulate(&m, &hold(10.0), m.cold(), m.tau2, 1);
        let rise = (r.end.th - 25.0) / (30.0 * 2.0);
        assert!((rise - 0.632).abs() / 0.632 < 0.02, "{rise}");
    }

    #[test]
    fn rms_of_square_wave() {
        let c = Cycle::new(&[Segment { dur: 0.5, torque: 8.0 }, Segment { dur: 0.5, torque: 0.0 }]);
        assert!((c.rms() - 8.0 / 2f64.sqrt()).abs() < 1e-12);
        assert_eq!(c.torque_at(1.2), 8.0);
        assert_eq!(c.torque_at(1.7), 0.0);
    }

    #[test]
    fn continuous_torque_settles_at_limit() {
        let m = Motor { alpha: 0.00393, ..base() };
        let tc = m.continuous_torque();
        let r = simulate(&m, &hold(tc), m.cold(), settle(&m), 10);
        assert!((r.end.tw - m.t_limit).abs() < 1.0, "{}", r.end.tw);
        let p = periodic(&m, &hold(tc)).unwrap();
        assert!((p.tw_max - m.t_limit).abs() < 1.0);
    }

    #[test]
    fn alpha_raises_steady_temperature() {
        let (a0, a1) = (base(), Motor { alpha: 0.00393, ..base() });
        let t0 = simulate(&a0, &hold(10.0), a0.cold(), settle(&a0), 1).end.tw;
        let t1 = simulate(&a1, &hold(10.0), a1.cold(), settle(&a1), 1).end.tw;
        assert!(t1 > t0 + 5.0, "{t1} vs {t0}");
        assert!((a1.steady_winding(100.0).unwrap() - t1).abs() < 0.5);
    }

    #[test]
    fn halving_cooling_halves_housing_rise() {
        let (m1, m2) = (base(), Motor { cooling: 0.5, ..base() });
        let h1 = simulate(&m1, &hold(10.0), m1.cold(), settle(&m1), 1).end.th - 25.0;
        let h2 = simulate(&m2, &hold(10.0), m2.cold(), settle(&m2), 1).end.th - 25.0;
        assert!((h2 / h1 - 0.5).abs() < 0.005, "{h1} {h2}");
    }

    #[test]
    fn runaway_detected() {
        // α·P25·Rtot = 0.00393·(1.5·400·0.2)·3·... ≫ 1.
        let m = Motor { alpha: 0.00393, ..base() };
        assert!(m.steady_winding(400.0 * 100.0).is_none());
        assert!(periodic(&m, &hold(200.0)).is_none());
        let r = simulate(&m, &hold(200.0), m.cold(), 600.0, 10);
        assert!(r.t_limit.is_some() && r.peak_tw <= T_CAP && r.peak_tw.is_finite());
    }

    #[test]
    fn duty_envelope_and_sweep_are_consistent() {
        let m = Motor { alpha: 0.00393, ..base() };
        let tc = m.continuous_torque();
        let e: Vec<f64> = [1.0, 30.0, 300.0].iter().map(|&t| safe_torque(&m, t)).collect();
        assert!(e[0] > e[1] && e[1] > e[2] && e[2] > tc);
        let c = Cycle::new(&[Segment { dur: 0.25, torque: 2.0 * tc }, Segment { dur: 0.75, torque: 0.0 }]);
        assert!((max_peak_duty(&m, &c) - 0.25).abs() < 1e-9);
        let s = sweep(&m, &c, 600.0, Sweep::Cooling, 5);
        assert!(s.windows(2).all(|w| w[1].1 > w[0].1), "more Rth2 → hotter");
        assert!(Motor { kt: 0.0, ..m }.sanitized().continuous_torque().is_finite());
    }
}
