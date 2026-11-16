//! 3D scene: cutaway joint actuator (housing, stator + coils, rotor, shaft, link arm), thermometer,
//! heat-flow arrows, orbit / pan / zoom camera with inertia and arm-tip payload dragging.
use crate::gl::{self, Geo, Mesh, Paint, Renderer};
use crate::math::{self, Orbit, OrbitCam, M4, V};
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use web_sys::HtmlCanvasElement;

/// Actuator axis (along z) height, housing half length, arm plane z, arm length (world units).
const AXY: f32 = 0.78;
const HALF: f32 = 0.25;
const ARM_Z: f32 = 0.56;
const ARM_LEN: f32 = 0.9;
const THERMO_X: f32 = -0.95;
/// The cutaway removes the +x / +y quadrant, so the drawn sector runs from 90° to 360°.
const CUT_A0: f32 = FRAC_PI_2;

/// Per-frame state the scene shows.
pub struct Frame {
    pub tw: f32,
    pub th: f32,
    pub t_amb: f32,
    pub limit: f32,
    /// Heat flow winding → housing and housing → air (W), and a reference for arrow scaling.
    pub q12: f32,
    pub q2a: f32,
    pub q_ref: f32,
    pub arm: f32,
    pub rotor: f32,
    pub payload: f32,
}

#[derive(Clone, Copy)]
enum Drag {
    None,
    Orbit,
    Pan,
    Payload { y0: f32, p0: f32 },
    Pinch { d: f32, mid: [f32; 2] },
}

pub struct View {
    r: Renderer,
    canvas: HtmlCanvasElement,
    housing: [Mesh; 2],
    edges: [Mesh; 2],
    stator: [Mesh; 2],
    rotor: Mesh,
    shaft: Mesh,
    hub: Mesh,
    cube: Mesh,
    cyl: Mesh,
    sphere: Mesh,
    head: Mesh,
    pub cam: OrbitCam,
    pub preset: usize,
    /// cutaway, heat flow, thermometer, link arm
    pub show: [bool; 4],
    pub hover: bool,
    inv_vp: M4,
    tip: V,
    weight: Option<V>,
    ptrs: Vec<(i32, f32, f32)>,
    drag: Drag,
}

/// Blue → yellow → red ramp for u = 0…1 (ambient … limit); beyond the limit it heats toward white.
pub fn heat(u: f32) -> V {
    let (b, y, r) = ([0.15, 0.38, 0.95], [1.0, 0.86, 0.2], [0.95, 0.12, 0.08]);
    let mix = |a: V, c: V, t: f32| math::add(math::scale(a, 1.0 - t), math::scale(c, t));
    match u {
        u if u < 0.5 => mix(b, y, u.max(0.0) * 2.0),
        u if u <= 1.0 => mix(y, r, (u - 0.5) * 2.0),
        u => mix(r, [1.0, 0.9, 0.9], ((u - 1.0) * 2.0).min(1.0)),
    }
}

/// Hollow tube sector along z between radii r0 < r1, z0…z1, angles a0…a1 (rad), with end faces
/// and, for a partial sector, the two cut faces.
fn tube(r0: f32, r1: f32, z0: f32, z1: f32, a0: f32, a1: f32, n: usize) -> Geo {
    let mut g = Geo::default();
    let p = |r: f32, a: f32, z: f32| [r * a.cos(), r * a.sin(), z];
    let quad = |g: &mut Geo, v: [V; 4], nr: [V; 4]| {
        let b = (g.pos.len() / 3) as u16;
        for k in 0..4 {
            g.pos.extend(v[k]);
            g.nrm.extend(nr[k]);
        }
        g.idx.extend([b, b + 1, b + 2, b, b + 2, b + 3]);
    };
    let (zp, zm) = ([0.0, 0.0, 1.0], [0.0, 0.0, -1.0]);
    for i in 0..n {
        let a = a0 + (a1 - a0) * i as f32 / n as f32;
        let b = a0 + (a1 - a0) * (i + 1) as f32 / n as f32;
        let (na, nb) = ([a.cos(), a.sin(), 0.0], [b.cos(), b.sin(), 0.0]);
        quad(&mut g, [p(r1, a, z0), p(r1, b, z0), p(r1, b, z1), p(r1, a, z1)], [na, nb, nb, na]);
        if r0 > 0.0 {
            quad(&mut g, [p(r0, a, z1), p(r0, b, z1), p(r0, b, z0), p(r0, a, z0)], [na, nb, nb, na].map(|v| math::scale(v, -1.0)));
        }
        quad(&mut g, [p(r0, a, z1), p(r0, b, z1), p(r1, b, z1), p(r1, a, z1)], [zp; 4]);
        quad(&mut g, [p(r0, a, z0), p(r1, a, z0), p(r1, b, z0), p(r0, b, z0)], [zm; 4]);
    }
    if a1 - a0 < TAU - 1e-4 {
        for a in [a0, a1] {
            let nn = [-a.sin(), a.cos(), 0.0];
            quad(&mut g, [p(r0, a, z0), p(r1, a, z0), p(r1, a, z1), p(r0, a, z1)], [nn; 4]);
        }
    }
    g
}

/// Bright outline of a housing sector: end circles plus axial lines.
fn outline(r: f32, a0: f32, a1: f32, n: usize) -> Geo {
    let p = |a: f32, z: f32| [r * a.cos(), r * a.sin(), z];
    let mut pts = vec![];
    for i in 0..n {
        let (a, b) = (a0 + (a1 - a0) * i as f32 / n as f32, a0 + (a1 - a0) * (i + 1) as f32 / n as f32);
        for z in [-HALF, HALF] {
            pts.extend([p(a, z), p(b, z)]);
        }
    }
    for k in 0..=4 {
        let a = a0 + (a1 - a0) * k as f32 / 4.0;
        pts.extend([p(a, -HALF), p(a, HALF)]);
    }
    gl::lines(&pts)
}

fn rot_z(a: f32, t: V) -> M4 {
    let (s, c) = a.sin_cos();
    math::basis([c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0], [1.0; 3], t)
}

fn preset_orbit(i: usize) -> Orbit {
    let target = [-0.05, 0.62, 0.15];
    let (yaw, pitch, dist) = match i {
        1 => (0.0, 1.5, 3.6),
        2 => (0.0, 0.04, 3.2),
        3 => (1.25, 0.62, 2.3),
        _ => (0.65, 0.32, 3.4),
    };
    Orbit { target, yaw, pitch, dist }
}

impl View {
    pub fn new(canvas: &HtmlCanvasElement, dark: bool) -> Option<View> {
        let mut r = Renderer::new(canvas)?;
        r.theme = if dark { gl::DARK } else { gl::LIGHT };
        r.fog = [6.0, 28.0];
        r.cell = 0.25;
        let full = |r0, r1, z0, z1| tube(r0, r1, z0, z1, 0.0, TAU, 48);
        let cut = |r0, r1, z0, z1| tube(r0, r1, z0, z1, CUT_A0, TAU, 36);
        Some(View {
            housing: [r.mesh(&full(0.47, 0.5, -HALF, HALF))?, r.mesh(&cut(0.47, 0.5, -HALF, HALF))?],
            edges: [r.mesh(&outline(0.505, 0.0, TAU, 48))?, r.mesh(&outline(0.505, CUT_A0, TAU, 36))?],
            stator: [r.mesh(&full(0.36, 0.46, -0.2, 0.2))?, r.mesh(&cut(0.36, 0.46, -0.2, 0.2))?],
            rotor: r.mesh(&full(0.05, 0.24, -0.19, 0.19))?,
            shaft: r.mesh(&full(0.0, 0.06, 0.0, ARM_Z + 0.05))?,
            hub: r.mesh(&full(0.0, 0.1, ARM_Z - 0.045, ARM_Z + 0.045))?,
            cube: r.mesh(&gl::cube())?,
            cyl: r.mesh(&gl::cylinder(24))?,
            sphere: r.mesh(&gl::sphere(16))?,
            head: r.mesh(&gl::frustum([0.0, 1.0, 0.0], &[[-0.5, 0.0, -0.5], [0.5, 0.0, -0.5], [0.5, 0.0, 0.5], [-0.5, 0.0, 0.5]]))?,
            r,
            canvas: canvas.clone(),
            cam: OrbitCam::new(preset_orbit(0)),
            preset: 0,
            show: [true; 4],
            hover: false,
            inv_vp: math::IDENTITY,
            tip: [0.0; 3],
            weight: None,
            ptrs: vec![],
            drag: Drag::None,
        })
    }

    pub fn set_preset(&mut self, i: usize) {
        self.preset = i;
        self.cam.set_goal(preset_orbit(i));
    }

    fn arrow(&self, from: V, to: V, w: f32, col: [f32; 4]) {
        let d = math::sub(to, from);
        let len = math::dot(d, d).sqrt();
        if len < 1e-4 {
            return;
        }
        let y = math::scale(d, 1.0 / len);
        let x = math::norm(math::cross(y, if y[1].abs() < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] }));
        let z = math::cross(x, y);
        let at = |f: f32| math::add(from, math::scale(y, len * f));
        let p = Paint { glow: 0.15, ..Paint::lit(col) };
        self.r.draw(&self.cyl, &math::basis(x, y, z, [w, len * 0.68, w], at(0.34)), p);
        self.r.draw(&self.head, &math::basis(x, y, z, [w * 2.3, len * 0.32, w * 2.3], at(0.68)), p);
    }

    pub fn render(&mut self, f: &Frame, dt: f32) {
        let dpr = web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0);
        let (cw, ch) = (self.canvas.client_width().max(1), self.canvas.client_height().max(1));
        let (pw, ph) = ((cw as f64 * dpr) as u32, (ch as f64 * dpr) as u32);
        if self.canvas.width() != pw || self.canvas.height() != ph {
            self.canvas.set_width(pw);
            self.canvas.set_height(ph);
        }
        self.cam.step(dt);
        let eye = self.cam.cur.eye();
        let view = math::look_at(eye, self.cam.cur.target, [0.0, 1.0, 0.0]);
        let proj = math::perspective(0.8, pw as f32 / ph as f32, 0.05, 120.0);
        self.inv_vp = math::invert(&math::mul(&proj, &view)).unwrap_or(math::IDENTITY);
        let r = &self.r;
        r.begin(&view, &proj, eye, [0, 0, pw as i32, ph as i32]);

        let [cut, flow, thermo, arm_on] = self.show;
        let ci = cut as usize;
        let span = (f.limit - f.t_amb).max(1.0);
        let uw = (f.tw - f.t_amb) / span;
        let uh = (f.th - f.t_amb) / span;
        let c = [0.0, AXY, 0.0];
        let at = math::translate(c);
        let rgba = |v: V, a: f32| [v[0], v[1], v[2], a];
        let steel = Paint::lit([0.62, 0.65, 0.7, 1.0]);

        // Arm pose and pick targets.
        let (sa, ca) = f.arm.sin_cos();
        let (d, perp) = ([ca, sa, 0.0], [-sa, ca, 0.0]);
        let pivot = [0.0, AXY, ARM_Z];
        self.tip = math::add(pivot, math::scale(d, ARM_LEN));
        let ws = 0.06 + 0.08 * (f.payload / 10.0).cbrt();
        self.weight = (f.payload > 0.01).then(|| math::sub(self.tip, [0.0, 0.1 + ws * 0.5, 0.0]));
        let arm_m = math::basis(d, perp, [0.0, 0.0, 1.0], [ARM_LEN, 0.09, 0.06], math::add(pivot, math::scale(d, ARM_LEN * 0.5)));
        let tip_m = math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.075; 3], self.tip);
        let base_m = math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.72, 0.34, 0.46], [0.0, 0.17, 0.0]);
        let tb = 0.2; // thermometer scale bottom height
        let t_top = (f.limit + 0.25 * span).max(f.tw);
        let ty = |t: f32| tb + ((t - f.t_amb) / (t_top - f.t_amb).max(1.0)).clamp(0.0, 1.0) * 1.2;
        let glass_m = math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.12, 1.3, 0.12], [THERMO_X, 0.17 + 0.65, 0.0]);
        let bulb_m = math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.1; 3], [THERMO_X, 0.12, 0.0]);
        let glow = if self.hover || matches!(self.drag, Drag::Payload { .. }) { 0.4 } else { 0.0 };
        let wm = self.weight.map(|w| math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [ws; 3], w));

        // Shadows first (on the ground), then opaque, lines, translucent.
        r.shadow(&self.cube, &base_m);
        r.shadow(&self.housing[0], &at);
        if arm_on {
            r.shadow(&self.cube, &arm_m);
            r.shadow(&self.sphere, &tip_m);
            if let Some(m) = &wm {
                r.shadow(&self.cube, m);
            }
        }
        if thermo {
            r.shadow(&self.cyl, &glass_m);
            r.shadow(&self.sphere, &bulb_m);
        }

        r.draw(&self.cube, &base_m, Paint::lit([0.36, 0.38, 0.42, 1.0]));
        r.draw(&self.stator[ci], &at, Paint::lit([0.42, 0.45, 0.5, 1.0]));
        let coil = Paint { glow: (uw - 0.9).clamp(0.0, 0.4), ..Paint::lit(rgba(heat(uw), 1.0)) };
        for k in 0..12 {
            let a = (k as f32 + 0.5) / 12.0 * TAU;
            if cut && a < CUT_A0 {
                continue;
            }
            let (s, co) = a.sin_cos();
            let m = math::basis([co, s, 0.0], [-s, co, 0.0], [0.0, 0.0, 1.0], [0.08, 0.13, 0.44], math::add(c, [0.31 * co, 0.31 * s, 0.0]));
            r.draw(&self.cube, &m, coil);
        }
        r.draw(&self.rotor, &rot_z(f.rotor, c), steel);
        for k in 0..8 {
            let a = f.rotor + k as f32 / 8.0 * TAU;
            let (s, co) = a.sin_cos();
            let col = if k % 2 == 0 { [0.8, 0.22, 0.2, 1.0] } else { [0.22, 0.4, 0.85, 1.0] };
            let m = math::basis([co, s, 0.0], [-s, co, 0.0], [0.0, 0.0, 1.0], [0.03, 0.15, 0.36], math::add(c, [0.25 * co, 0.25 * s, 0.0]));
            r.draw(&self.cube, &m, Paint::lit(col));
        }
        // Rear cover (the front stays open in the cutaway).
        r.draw(&self.cyl, &math::basis([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.03, 1.0], math::add(c, [0.0, 0.0, -HALF])), Paint::lit([0.3, 0.32, 0.36, 1.0]));
        r.draw(&self.shaft, &rot_z(f.arm, c), steel);
        if arm_on {
            r.draw(&self.hub, &rot_z(f.arm, c), Paint::lit([0.5, 0.52, 0.56, 1.0]));
            r.draw(&self.cube, &arm_m, Paint::lit([0.92, 0.55, 0.15, 1.0]));
            r.draw(&self.sphere, &tip_m, Paint { glow, ..Paint::lit([0.95, 0.95, 0.98, 1.0]) });
            if let (Some(m), Some(w)) = (&wm, self.weight) {
                r.draw(&self.cube, m, Paint { glow, ..Paint::lit([0.3, 0.3, 0.34, 1.0]) });
                r.draw(&self.cube, &math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.01, 0.1, 0.01], math::scale(math::add(w, self.tip), 0.5)), Paint::lit([0.2, 0.2, 0.2, 1.0]));
            }
        }
        if thermo {
            let col = rgba(heat(uw), 1.0);
            let y1 = ty(f.tw);
            r.draw(&self.sphere, &math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.085; 3], [THERMO_X, 0.12, 0.0]), Paint::lit(col));
            r.draw(&self.cyl, &math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.07, y1 - 0.12, 0.07], [THERMO_X, 0.5 * (y1 + 0.12), 0.0]), Paint { glow: 0.1, ..Paint::lit(col) });
            let ly = ty(f.limit);
            r.draw(&self.cyl, &math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.24, 0.018, 0.24], [THERMO_X, ly, 0.0]), Paint::flat([1.0, 0.2, 0.15, 1.0]));
            // Ticks every 20 % of the scale.
            for k in 0..=5 {
                let y = tb + k as f32 * 0.24;
                r.draw(&self.cube, &math::basis([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.05, 0.008, 0.01], [THERMO_X + 0.085, y, 0.06]), Paint::flat([0.85, 0.88, 0.92, 1.0]));
            }
        }
        if flow {
            let q = |w: f32| (w.max(0.0) / f.q_ref.max(1e-3)).sqrt().min(1.5);
            let (k1, k2) = (q(f.q12), q(f.q2a));
            if k1 > 0.02 {
                for a in [PI * 0.25, PI * 0.9, PI * 1.5] {
                    let (s, co) = a.sin_cos();
                    let p0 = math::add(c, [0.3 * co, 0.3 * s, 0.05]);
                    let p1 = math::add(c, [(0.36 + 0.12 * k1) * co, (0.36 + 0.12 * k1) * s, 0.05]);
                    self.arrow(p0, p1, 0.012 + 0.014 * k1, [1.0, 0.35, 0.1, 1.0]);
                }
            }
            if k2 > 0.02 {
                for (a, z) in [(PI * 0.5, 0.1), (PI * 0.25, -0.1), (PI * 0.75, -0.1), (PI * 0.08, 0.12)] {
                    let (s, co) = a.sin_cos();
                    let p0 = math::add(c, [0.52 * co, 0.52 * s, z]);
                    let p1 = math::add(c, [(0.6 + 0.3 * k2) * co, (0.6 + 0.3 * k2) * s + 0.08 * k2, z]);
                    self.arrow(p0, p1, 0.014 + 0.014 * k2, [1.0, 0.72, 0.2, 1.0]);
                }
            }
        }
        r.draw(&self.edges[ci], &at, Paint::flat([0.55, 0.9, 1.0, 1.0]));
        let hc = math::add(math::scale([0.55, 0.62, 0.72], 0.55), math::scale(heat(uh), 0.45));
        r.draw(&self.housing[ci], &at, Paint::lit(rgba(hc, 0.24)));
        if thermo {
            r.draw(&self.cyl, &glass_m, Paint::lit([0.85, 0.9, 1.0, 0.2]));
        }
    }

    fn ray(&self, x: f32, y: f32) -> (V, V) {
        let (w, h) = (self.canvas.client_width().max(1) as f32, self.canvas.client_height().max(1) as f32);
        math::ndc_ray(&self.inv_vp, 2.0 * x / w - 1.0, 1.0 - 2.0 * y / h)
    }

    /// True if the pointer (CSS px in the canvas) is over the arm tip or its payload.
    pub fn pick(&self, x: f32, y: f32) -> bool {
        if !self.show[3] {
            return false;
        }
        let (o, d) = self.ray(x, y);
        math::ray_sphere(o, d, self.tip, 0.13).is_some() || self.weight.is_some_and(|w| math::ray_sphere(o, d, w, 0.16).is_some())
    }

    fn pan_px(&mut self, dx: f32, dy: f32) {
        let o = self.cam.cur;
        let fwd = math::norm(math::sub(o.target, o.eye()));
        let right = math::norm(math::cross(fwd, [0.0, 1.0, 0.0]));
        let up = math::cross(right, fwd);
        let k = o.dist * 1.1 / self.canvas.client_height().max(1) as f32;
        self.cam.pan(math::add(math::scale(right, -dx * k), math::scale(up, dy * k)));
    }

    fn pinch(&self) -> (f32, [f32; 2]) {
        let (a, b) = (self.ptrs[0], self.ptrs[1]);
        (((a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt().max(1.0), [(a.1 + b.1) * 0.5, (a.2 + b.2) * 0.5])
    }

    pub fn down(&mut self, id: i32, x: f32, y: f32, button: i16, payload: f32) {
        self.ptrs.retain(|p| p.0 != id);
        self.ptrs.push((id, x, y));
        self.cam.held = true;
        self.drag = if self.ptrs.len() >= 2 {
            let (d, mid) = self.pinch();
            Drag::Pinch { d, mid }
        } else if button == 0 && self.pick(x, y) {
            Drag::Payload { y0: y, p0: payload }
        } else if button == 2 || button == 1 {
            Drag::Pan
        } else {
            Drag::Orbit
        };
    }

    /// Pointer moved; returns a new payload (kg) while the arm tip is being dragged.
    pub fn moved(&mut self, id: i32, x: f32, y: f32) -> Option<f32> {
        let Some(i) = self.ptrs.iter().position(|p| p.0 == id) else {
            self.hover = self.pick(x, y);
            return None;
        };
        let (dx, dy) = (x - self.ptrs[i].1, y - self.ptrs[i].2);
        self.ptrs[i] = (id, x, y);
        match self.drag {
            Drag::Orbit => self.cam.rotate(-dx * 0.008, dy * 0.008),
            Drag::Pan => self.pan_px(dx, dy),
            Drag::Pinch { d, mid } if self.ptrs.len() >= 2 => {
                let (d1, m1) = self.pinch();
                self.pan_px(m1[0] - mid[0], m1[1] - mid[1]);
                self.cam.zoom(d / d1);
                self.drag = Drag::Pinch { d: d1, mid: m1 };
            }
            Drag::Payload { y0, p0 } => {
                let h = self.canvas.client_height().max(1) as f32;
                return Some((p0 + (y - y0) / h * 25.0).clamp(0.0, 10.0));
            }
            _ => {}
        }
        None
    }

    pub fn up(&mut self, id: i32) {
        self.ptrs.retain(|p| p.0 != id);
        self.cam.held = !self.ptrs.is_empty();
        // Lifting one finger of a pinch continues as an orbit with the remaining one.
        self.drag = if self.ptrs.is_empty() { Drag::None } else { Drag::Orbit };
    }

    pub fn dragging_payload(&self) -> bool {
        matches!(self.drag, Drag::Payload { .. })
    }

    pub fn wheel(&mut self, dy: f32) {
        self.cam.zoom((dy * 0.0015).exp());
    }

    pub fn reset(&mut self) {
        self.set_preset(self.preset);
    }
}
