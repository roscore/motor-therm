//! motor-therm web UI: plain web-sys wiring of static markup (index.html) to the core model,
//! 2D plots and the 3D view. Everything runs locally; nothing is sent anywhere.
pub mod gl;
mod i18n;
pub mod math;
mod plot;
mod view3d;

use i18n::t;
use mt_core::{self as core, Cycle, Motor, Periodic, Run, Segment, Sweep};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, Event, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement, PointerEvent, WheelEvent};

const MOTOR_IDS: [&str; 12] = ["kt", "rpp", "alpha", "ratio", "eta", "rth1", "rth2", "tau1", "tau2", "tamb", "tlim", "cool"];
const ENVELOPE: [f64; 4] = [1.0, 5.0, 30.0, 300.0];
const MAX_SEGS: usize = 6;

struct App {
    doc: Document,
    ko: bool,
    dark: bool,
    motor: Motor,
    cycle: Cycle,
    total: f64,
    payload: f64,
    run: Run,
    per: Option<Periodic>,
    sweep_kind: Sweep,
    sweep: Vec<(f64, f64)>,
    t_play: f64,
    playing: bool,
    speed: f64,
    /// Wall-clock phase of the illustrative arm motion (s).
    anim: f64,
    rotor: f64,
    view: Option<view3d::View>,
    need_sim: bool,
    last_ts: f64,
}

thread_local! {
    static APP: RefCell<Option<App>> = const { RefCell::new(None) };
}

fn with<R>(f: impl FnOnce(&mut App) -> R) -> Option<R> {
    APP.with(|a| a.borrow_mut().as_mut().map(f))
}

fn el(doc: &Document, id: &str) -> Element {
    doc.get_element_by_id(id).unwrap_or_else(|| doc.create_element("div").expect("element"))
}
fn input(doc: &Document, id: &str) -> HtmlInputElement {
    el(doc, id).unchecked_into()
}
fn num(doc: &Document, id: &str) -> f64 {
    input(doc, id).value().trim().parse().unwrap_or(f64::NAN)
}
fn set_num(doc: &Document, id: &str, v: f64) {
    input(doc, id).set_value(&format!("{}", (v * 1e6).round() / 1e6));
}
fn on(target: &web_sys::EventTarget, ev: &str, f: impl FnMut(Event) + 'static) {
    let c = Closure::<dyn FnMut(Event)>::new(f);
    let _ = target.add_event_listener_with_callback(ev, c.as_ref().unchecked_ref());
    c.forget();
}
fn on_id(doc: &Document, id: &str, ev: &str, f: impl FnMut(Event) + 'static) {
    on(&el(doc, id), ev, f);
}
fn set_class(doc: &Document, id: &str, cls: &str, on: bool) {
    let _ = el(doc, id).class_list().toggle_with_force(cls, on);
}

fn fmt_time(s: f64) -> String {
    if s < 120.0 {
        format!("{s:.0} s")
    } else {
        format!("{}:{:02} min", (s / 60.0) as i64, (s % 60.0) as i64)
    }
}

impl App {
    fn tr(&self, k: &str) -> &'static str {
        t(self.ko, k)
    }

    fn load_motor_preset(&self, i: usize) {
        let m = core::MOTOR_PRESETS[i.min(2)].1;
        let v = [m.kt, m.r_pp25, m.alpha, m.ratio, m.eta, m.rth1, m.rth2, m.tau1, m.tau2, m.t_amb, m.t_limit, m.cooling];
        for (id, x) in MOTOR_IDS.iter().zip(v) {
            set_num(&self.doc, id, x);
        }
    }

    fn read_motor(&self) -> Motor {
        let v: Vec<f64> = MOTOR_IDS.iter().map(|id| num(&self.doc, id)).collect();
        Motor { kt: v[0], r_pp25: v[1], alpha: v[2], ratio: v[3], eta: v[4], rth1: v[5], rth2: v[6], tau1: v[7], tau2: v[8], t_amb: v[9], t_limit: v[10], cooling: v[11] }
            .sanitized()
    }

    fn load_cycle_preset(&self, i: usize) {
        let segs = core::cycle_preset(i, &self.read_motor());
        for k in 0..MAX_SEGS {
            let (d, q) = (input(&self.doc, &format!("d{k}")), input(&self.doc, &format!("q{k}")));
            match segs.get(k) {
                Some(s) => {
                    d.set_value(&format!("{}", s.dur));
                    q.set_value(&format!("{}", s.torque));
                }
                None => {
                    d.set_value("");
                    q.set_value("");
                }
            }
        }
    }

    fn read_cycle(&self) -> Cycle {
        let segs: Vec<Segment> = (0..MAX_SEGS)
            .map(|k| Segment { dur: num(&self.doc, &format!("d{k}")), torque: num(&self.doc, &format!("q{k}")) })
            .filter(|s| s.dur > 0.0)
            .map(|s| Segment { torque: if s.torque.is_finite() { s.torque } else { 0.0 }, ..s })
            .collect();
        Cycle::new(&segs)
    }

    /// Re-read every input and re-run the model, sweep and envelope.
    fn simulate(&mut self) {
        let d = &self.doc;
        self.motor = self.read_motor();
        self.total = (num(d, "simmin") * 60.0).clamp(60.0, 7200.0);
        if !self.total.is_finite() {
            self.total = 1200.0;
        }
        let p = num(d, "payload");
        self.payload = if p.is_finite() { p.clamp(0.0, 10.0) } else { 0.0 };
        let arm = num(d, "arm");
        self.cycle = self.read_cycle().with_payload(self.payload, if arm.is_finite() { arm.clamp(0.0, 5.0) } else { 0.0 });
        self.run = core::simulate(&self.motor, &self.cycle, self.motor.cold(), self.total, 1200);
        self.per = core::periodic(&self.motor, &self.cycle);
        self.sweep = core::sweep(&self.motor, &self.cycle, self.total, self.sweep_kind, 25);
        self.t_play = self.t_play.min(self.total);
        self.need_sim = false;
        self.results();
    }

    fn results(&self) {
        let (d, m, r) = (&self.doc, &self.motor, &self.run);
        let card = |label: &str, value: String| format!("<div><span>{label}</span><b>{value}</b></div>");
        let tq = |x: f64| format!("{x:.2} N·m");
        let per = match self.per {
            Some(p) => format!("{:.1}–{:.1} °C", p.tw_min, p.tw_max),
            None => self.tr("runaway").to_string(),
        };
        let html = [
            card(self.tr("peak"), if r.peak_tw >= core::T_CAP - 0.5 { format!("≥ {:.0} °C", core::T_CAP) } else { format!("{:.1} °C", r.peak_tw) }),
            card(self.tr("tolimit"), r.t_limit.map(fmt_time).unwrap_or_else(|| self.tr("never").into())),
            card(self.tr("steady"), per),
            card(self.tr("rms"), tq(self.cycle.rms())),
            card(self.tr("cont"), tq(m.continuous_torque())),
            card(self.tr("duty"), format!("{:.0} %", 100.0 * core::max_peak_duty(m, &self.cycle))),
        ]
        .concat();
        el(d, "res").set_inner_html(&html);
        let warn = match self.per {
            None => self.tr("w_runaway"),
            Some(p) if p.tw_max > m.t_limit => self.tr("w_over"),
            _ => "",
        };
        el(d, "warn").set_text_content(Some(warn));
        let mut rows = String::new();
        let row = |on: String, tqv: f64| format!("<tr><td>{on}</td><td>{tqv:.2} N·m</td><td>{:.2} A</td></tr>", m.current(tqv));
        for on in ENVELOPE {
            rows += &row(fmt_time(on), core::safe_torque(m, on));
        }
        rows += &row(self.tr("continuous").into(), m.continuous_torque());
        el(d, "envt").set_inner_html(&rows);
    }

    fn apply_lang(&self) {
        let nodes = self.doc.query_selector_all("[data-i18n]");
        if let Ok(nodes) = nodes {
            for i in 0..nodes.length() {
                if let Some(e) = nodes.item(i).and_then(|n| n.dyn_into::<Element>().ok()) {
                    let k = e.get_attribute("data-i18n").unwrap_or_default();
                    e.set_text_content(Some(self.tr(&k)));
                }
            }
        }
        el(&self.doc, "lang").set_text_content(Some(if self.ko { "English" } else { "한국어" }));
        if let Some(h) = self.doc.document_element() {
            let _ = h.set_attribute("lang", if self.ko { "ko" } else { "en" });
        }
        el(&self.doc, "play").set_text_content(Some(self.tr(if self.playing { "pause" } else { "play" })));
        self.results();
    }

    /// Linear interpolation of the downsampled run at time `t`.
    fn temps_at(&self, t: f64) -> (f64, f64) {
        let r = &self.run;
        let i = r.t.partition_point(|&x| x <= t).clamp(1, r.t.len().max(2) - 1);
        if r.t.len() < 2 {
            return (self.motor.t_amb, self.motor.t_amb);
        }
        let f = ((t - r.t[i - 1]) / (r.t[i] - r.t[i - 1]).max(1e-12)).clamp(0.0, 1.0);
        (r.tw[i - 1] + f * (r.tw[i] - r.tw[i - 1]), r.th[i - 1] + f * (r.th[i] - r.th[i - 1]))
    }

    fn frame(&mut self, dt: f64) {
        if self.need_sim {
            self.simulate();
        }
        if self.playing {
            self.t_play += dt * self.speed;
            self.anim += dt;
            if self.t_play >= self.total {
                self.t_play = self.total;
                self.playing = false;
                el(&self.doc, "play").set_text_content(Some(self.tr("play")));
            }
            input(&self.doc, "time").set_value(&format!("{}", (self.t_play / self.total * 1000.0).round()));
        }
        let m = self.motor;
        let tq = self.cycle.torque_at(self.t_play);
        let cur = m.current(tq);
        let (tw, th) = self.temps_at(self.t_play);
        let loss = m.loss(cur, tw);
        let k = |key| self.tr(key);
        let hud = format!(
            "{:<8}{:>10}\n{:<8}{:>8.2} N·m\n{:<8}{:>8.2} A\n{:<8}{:>8.1} W\n{:<8}{:>7.1} °C\n{:<8}{:>7.1} °C\n{:<8}{:>7.1} K",
            k("h_time"), fmt_time(self.t_play), k("h_out"), tq, k("h_cur"), cur, k("h_loss"), loss,
            k("h_wind"), tw, k("h_house"), th, k("h_margin"), m.t_limit - tw
        );
        el(&self.doc, "hud").set_text_content(Some(&hud));

        // Illustrative arm motion: swings with the cycle phase at real-time speed (thermal playback
        // is usually 60× faster); a single-segment cycle is a static pose.
        let per = self.cycle.period();
        let phase = (self.anim % per) / per;
        let multi = self.cycle.segs.len() > 1;
        let arm = if multi { -0.35 + 0.45 * (phase * std::f64::consts::TAU).sin() } else { -0.25 };
        if self.playing {
            self.rotor += dt * if multi { 3.0 * (phase * std::f64::consts::TAU).cos() } else { 0.0 };
        }
        let rf = m.rth1;
        let qref = m.loss(m.current(m.continuous_torque()), m.t_limit);
        let f = view3d::Frame {
            tw: tw as f32,
            th: th as f32,
            t_amb: m.t_amb as f32,
            limit: m.t_limit as f32,
            q12: ((tw - th) / rf) as f32,
            q2a: ((th - m.t_amb) / m.rth2_eff()) as f32,
            q_ref: qref as f32,
            arm: arm as f32,
            rotor: self.rotor as f32,
            payload: self.payload as f32,
        };
        if let Some(v) = self.view.as_mut() {
            v.render(&f, dt as f32);
        }
        self.plots();
    }

    fn plots(&self) {
        let (d, r, m) = (&self.doc, &self.run, &self.motor);
        let canvas = |id: &str| el(d, id).unchecked_into::<HtmlCanvasElement>();
        let tmin: Vec<f64> = r.t.iter().map(|t| t / 60.0).collect();
        let (cw, chs, lim) = if self.dark { ("#ff6b4a", "#5aa9ff", "#ff4d6d") } else { ("#d9480f", "#1c64d8", "#c2185b") };
        plot::draw(
            &canvas("ptemp"),
            &plot::Plot {
                series: vec![
                    plot::Series { x: &tmin, y: &r.tw, color: cw, label: self.tr("l_wind"), step: false },
                    plot::Series { x: &tmin, y: &r.th, color: chs, label: self.tr("l_house"), step: false },
                ],
                hlines: vec![(m.t_limit, lim, self.tr("l_limit"))],
                cursor: Some(self.t_play / 60.0),
                xlabel: self.tr("x_min"),
                ylabel: "°C",
                ..Default::default()
            },
            self.dark,
        );
        // Two periods of the load cycle as a staircase, cursor at the current phase.
        let per = self.cycle.period();
        let (mut xs, mut ys, mut acc) = (vec![], vec![], 0.0);
        for _ in 0..2 {
            for s in &self.cycle.segs {
                xs.push(acc);
                ys.push(s.torque);
                acc += s.dur;
            }
        }
        xs.push(acc);
        ys.push(*ys.last().unwrap_or(&0.0));
        let rms = self.cycle.rms();
        plot::draw(
            &canvas("ptorque"),
            &plot::Plot {
                series: vec![plot::Series { x: &xs, y: &ys, color: chs, label: "", step: true }],
                hlines: vec![(rms, cw, "RMS"), (m.continuous_torque(), lim, self.tr("continuous"))],
                cursor: Some(self.t_play % per),
                xlabel: self.tr("x_s"),
                ylabel: "N·m",
                ..Default::default()
            },
            self.dark,
        );
        let (sx, sy): (Vec<f64>, Vec<f64>) = self.sweep.iter().copied().unzip();
        let cur_x = match self.sweep_kind {
            Sweep::Cooling => m.cooling,
            Sweep::Ratio => m.ratio,
        };
        plot::draw(
            &canvas("psweep"),
            &plot::Plot {
                series: vec![plot::Series { x: &sx, y: &sy, color: cw, label: "", step: false }],
                hlines: vec![(m.t_limit, lim, self.tr("l_limit"))],
                marker: (sx.first().is_some_and(|&a| cur_x >= a) && sx.last().is_some_and(|&b| cur_x <= b)).then_some((cur_x, r.peak_tw)),
                xlabel: self.tr(if self.sweep_kind == Sweep::Cooling { "x_cool" } else { "x_ratio" }),
                ylabel: "°C",
                ..Default::default()
            },
            self.dark,
        );
    }
}

fn wire(doc: &Document) {
    // Segment table rows.
    let rows: String = (0..MAX_SEGS)
        .map(|k| format!("<tr><td>{}</td><td><input id=\"d{k}\" type=\"number\" step=\"0.05\" min=\"0\"></td><td><input id=\"q{k}\" type=\"number\" step=\"0.1\"></td></tr>", k + 1))
        .collect();
    el(doc, "segs").set_inner_html(&rows);
    let resim = |_: Event| {
        with(|a| a.need_sim = true);
    };
    for id in MOTOR_IDS.iter().chain(&["arm", "payload", "simmin"]) {
        on_id(doc, id, "input", resim);
    }
    for k in 0..MAX_SEGS {
        on_id(doc, &format!("d{k}"), "input", resim);
        on_id(doc, &format!("q{k}"), "input", resim);
    }
    let sel = |doc: &Document, id: &str| el(doc, id).unchecked_into::<HtmlSelectElement>().selected_index().max(0) as usize;
    on_id(doc, "mpreset", "change", move |_| {
        with(|a| {
            a.load_motor_preset(sel(&a.doc, "mpreset"));
            a.load_cycle_preset(sel(&a.doc, "cpreset"));
            a.simulate();
        });
    });
    on_id(doc, "cpreset", "change", move |_| {
        with(|a| {
            a.load_cycle_preset(sel(&a.doc, "cpreset"));
            a.simulate();
        });
    });
    on_id(doc, "sweep", "change", move |_| {
        with(|a| {
            a.sweep_kind = if sel(&a.doc, "sweep") == 1 { Sweep::Ratio } else { Sweep::Cooling };
            a.need_sim = true;
        });
    });
    on_id(doc, "lang", "click", |_| {
        with(|a| {
            a.ko = !a.ko;
            a.apply_lang();
        });
    });
    on_id(doc, "play", "click", |_| {
        with(|a| {
            if a.t_play >= a.total {
                a.t_play = 0.0;
            }
            a.playing = !a.playing;
            el(&a.doc, "play").set_text_content(Some(a.tr(if a.playing { "pause" } else { "play" })));
        });
    });
    for (id, s) in [("s1", 1.0), ("s10", 10.0), ("s60", 60.0)] {
        on_id(doc, id, "click", move |_| {
            with(|a| {
                a.speed = s;
                for b in ["s1", "s10", "s60"] {
                    set_class(&a.doc, b, "on", b == id);
                }
            });
        });
    }
    on_id(doc, "time", "input", |_| {
        with(|a| a.t_play = num(&a.doc, "time") / 1000.0 * a.total);
    });
    for (i, id) in ["t_cut", "t_flow", "t_thermo", "t_arm"].into_iter().enumerate() {
        on_id(doc, id, "click", move |_| {
            with(|a| {
                if let Some(v) = a.view.as_mut() {
                    v.show[i] = !v.show[i];
                    set_class(&a.doc, id, "on", v.show[i]);
                }
            });
        });
    }
    for i in 0..4 {
        on_id(doc, &format!("v{i}"), "click", move |_| {
            with(|a| {
                if let Some(v) = a.view.as_mut() {
                    v.set_preset(i);
                }
                for k in 0..4 {
                    set_class(&a.doc, &format!("v{k}"), "on", k == i);
                }
            });
        });
    }

    // 3D canvas pointer input.
    let canvas = el(doc, "gl");
    let pos = |e: &PointerEvent| (e.offset_x() as f32, e.offset_y() as f32);
    on(&canvas, "pointerdown", move |e| {
        let e: PointerEvent = e.unchecked_into();
        let (x, y) = pos(&e);
        if let Some(t) = e.target().and_then(|t| t.dyn_into::<Element>().ok()) {
            let _ = t.set_pointer_capture(e.pointer_id());
        }
        with(|a| {
            if let Some(v) = a.view.as_mut() {
                v.down(e.pointer_id(), x, y, e.button(), a.payload as f32);
            }
        });
        e.prevent_default();
    });
    on(&canvas, "pointermove", move |e| {
        let e: PointerEvent = e.unchecked_into();
        let (x, y) = pos(&e);
        with(|a| {
            let Some(v) = a.view.as_mut() else { return };
            if let Some(p) = v.moved(e.pointer_id(), x, y) {
                set_num(&a.doc, "payload", (p as f64 * 10.0).round() / 10.0);
                a.need_sim = true;
            }
            let cur = if v.dragging_payload() { "ns-resize" } else if v.hover { "grab" } else { "default" };
            if let Some(c) = a.doc.get_element_by_id("gl").and_then(|c| c.dyn_into::<HtmlElement>().ok()) {
                let _ = c.style().set_property("cursor", cur);
            }
        });
    });
    for ev in ["pointerup", "pointercancel"] {
        on(&canvas, ev, |e| {
            let e: PointerEvent = e.unchecked_into();
            with(|a| a.view.as_mut().map(|v| v.up(e.pointer_id())));
        });
    }
    on(&canvas, "wheel", |e| {
        let e: WheelEvent = e.unchecked_into();
        e.prevent_default();
        with(|a| a.view.as_mut().map(|v| v.wheel(e.delta_y() as f32)));
    });
    on(&canvas, "dblclick", |_| {
        with(|a| a.view.as_mut().map(|v| v.reset()));
    });
    on(&canvas, "contextmenu", |e| e.prevent_default());
}

fn raf(f: &Closure<dyn FnMut(f64)>) {
    if let Some(w) = web_sys::window() {
        let _ = w.request_animation_frame(f.as_ref().unchecked_ref());
    }
}

#[wasm_bindgen(start)]
pub fn start() {
    let Some(win) = web_sys::window() else { return };
    let Some(doc) = win.document() else { return };
    let dark = win.match_media("(prefers-color-scheme: dark)").ok().flatten().is_some_and(|m| m.matches());
    let ko = win.navigator().language().is_some_and(|l| l.to_lowercase().starts_with("ko"));
    let canvas: HtmlCanvasElement = el(&doc, "gl").unchecked_into();
    let view = view3d::View::new(&canvas, dark);
    if view.is_none() {
        let _ = el(&doc, "nogl").unchecked_into::<HtmlElement>().style().set_property("display", "flex");
        for id in ["tog", "views", "hint"] {
            let _ = el(&doc, id).unchecked_into::<HtmlElement>().style().set_property("display", "none");
        }
    }
    wire(&doc);
    let app = App {
        doc: doc.clone(),
        ko,
        dark,
        motor: core::MOTOR_PRESETS[1].1,
        cycle: Cycle::new(&[]),
        total: 1200.0,
        payload: 0.0,
        run: Run::default(),
        per: None,
        sweep_kind: Sweep::Cooling,
        sweep: vec![],
        t_play: 0.0,
        playing: true,
        speed: 60.0,
        anim: 0.0,
        rotor: 0.0,
        view,
        need_sim: false,
        last_ts: -1.0,
    };
    el(&doc, "mpreset").unchecked_into::<HtmlSelectElement>().set_selected_index(1);
    el(&doc, "cpreset").unchecked_into::<HtmlSelectElement>().set_selected_index(1);
    app.load_motor_preset(1);
    app.load_cycle_preset(1);
    APP.with(|a| *a.borrow_mut() = Some(app));
    with(|a| {
        a.simulate();
        a.apply_lang();
    });

    let cb: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let cb2 = cb.clone();
    *cb.borrow_mut() = Some(Closure::new(move |ts: f64| {
        with(|a| {
            let dt = if a.last_ts < 0.0 { 1.0 / 60.0 } else { ((ts - a.last_ts) / 1000.0).clamp(0.0, 0.1) };
            a.last_ts = ts;
            a.frame(dt);
        });
        if let Some(f) = cb2.borrow().as_ref() {
            raf(f);
        }
    }));
    let first = cb.borrow();
    if let Some(f) = first.as_ref() {
        raf(f);
    }
}
