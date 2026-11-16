//! Tiny canvas-2D line plot: auto range, nice ticks, step or line series, horizontal reference
//! lines, a vertical playback cursor and a point marker.
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d as Ctx, HtmlCanvasElement};

pub struct Series<'a> {
    pub x: &'a [f64],
    pub y: &'a [f64],
    pub color: &'a str,
    pub label: &'a str,
    /// Draw as a staircase (value holds until the next x).
    pub step: bool,
}

#[derive(Default)]
pub struct Plot<'a> {
    pub series: Vec<Series<'a>>,
    /// Horizontal lines: (y, colour, label).
    pub hlines: Vec<(f64, &'a str, &'a str)>,
    pub cursor: Option<f64>,
    pub marker: Option<(f64, f64)>,
    pub xlabel: &'a str,
    pub ylabel: &'a str,
}

fn nice_step(span: f64, n: f64) -> f64 {
    let raw = (span / n).max(1e-12);
    let p = 10f64.powf(raw.log10().floor());
    let f = raw / p;
    p * if f < 1.5 { 1.0 } else if f < 3.5 { 2.0 } else if f < 7.5 { 5.0 } else { 10.0 }
}

fn fmt(v: f64, step: f64) -> String {
    let v = if v.abs() < step * 1e-6 { 0.0 } else { v };
    let d = if step >= 1.0 { 0 } else { (-step.log10()).ceil() as usize };
    format!("{v:.d$}")
}

pub fn draw(canvas: &HtmlCanvasElement, p: &Plot, dark: bool) {
    let dpr = web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0);
    let (cw, ch) = (canvas.client_width() as f64, canvas.client_height() as f64);
    if cw < 10.0 || ch < 10.0 {
        return;
    }
    let (pw, ph) = ((cw * dpr) as u32, (ch * dpr) as u32);
    if canvas.width() != pw || canvas.height() != ph {
        canvas.set_width(pw);
        canvas.set_height(ph);
    }
    let Some(ctx) = canvas.get_context("2d").ok().flatten().and_then(|c| c.dyn_into::<Ctx>().ok()) else { return };
    let _ = ctx.set_transform(dpr, 0.0, 0.0, dpr, 0.0, 0.0);
    let (fg, mute, grid) = if dark { ("#e4e7ec", "#9aa2ae", "#2e3440") } else { ("#1b1e24", "#5d6470", "#e1e4ea") };
    ctx.clear_rect(0.0, 0.0, cw, ch);

    let mut xr = (f64::INFINITY, f64::NEG_INFINITY);
    let mut yr = xr;
    for s in &p.series {
        for (&x, &y) in s.x.iter().zip(s.y) {
            xr = (xr.0.min(x), xr.1.max(x));
            yr = (yr.0.min(y), yr.1.max(y));
        }
    }
    for h in &p.hlines {
        yr = (yr.0.min(h.0), yr.1.max(h.0));
    }
    if !xr.0.is_finite() {
        return;
    }
    if xr.1 - xr.0 < 1e-9 {
        xr.1 = xr.0 + 1.0;
    }
    if yr.1 - yr.0 < 1e-6 {
        yr = (yr.0 - 1.0, yr.1 + 1.0);
    }
    let pad = 0.06 * (yr.1 - yr.0);
    yr = (yr.0 - pad, yr.1 + pad);
    let (l, r, t, b) = (46.0, 10.0, 16.0, 30.0);
    let (w, h) = (cw - l - r, ch - t - b);
    let sx = |x: f64| l + (x - xr.0) / (xr.1 - xr.0) * w;
    let sy = |y: f64| t + (1.0 - (y - yr.0) / (yr.1 - yr.0)) * h;

    ctx.set_font("11px system-ui, sans-serif");
    ctx.set_line_width(1.0);
    let ys = nice_step(yr.1 - yr.0, (h / 32.0).max(2.0));
    let mut v = (yr.0 / ys).ceil() * ys;
    while v <= yr.1 {
        ctx.set_stroke_style_str(grid);
        line(&ctx, l, sy(v), l + w, sy(v));
        ctx.set_fill_style_str(mute);
        ctx.set_text_align("right");
        let _ = ctx.fill_text(&fmt(v, ys), l - 4.0, sy(v) + 4.0);
        v += ys;
    }
    let xs = nice_step(xr.1 - xr.0, (w / 70.0).max(2.0));
    let mut v = (xr.0 / xs).ceil() * xs;
    ctx.set_text_align("center");
    while v <= xr.1 + 1e-9 {
        ctx.set_stroke_style_str(grid);
        line(&ctx, sx(v), t, sx(v), t + h);
        ctx.set_fill_style_str(mute);
        let _ = ctx.fill_text(&fmt(v, xs), sx(v), t + h + 13.0);
        v += xs;
    }
    let _ = ctx.fill_text(p.xlabel, l + w / 2.0, ch - 3.0);
    ctx.set_text_align("left");
    let _ = ctx.fill_text(p.ylabel, 4.0, 11.0);

    for (i, (y, col, lab)) in p.hlines.iter().enumerate() {
        ctx.set_stroke_style_str(col);
        let _ = ctx.set_line_dash(&js_sys::Array::of2(&5.0.into(), &4.0.into()));
        line(&ctx, l, sy(*y), l + w, sy(*y));
        let _ = ctx.set_line_dash(&js_sys::Array::new());
        ctx.set_fill_style_str(col);
        // Alternate label sides so close reference lines stay readable.
        ctx.set_text_align(if i % 2 == 0 { "right" } else { "left" });
        let _ = ctx.fill_text(lab, if i % 2 == 0 { l + w - 2.0 } else { l + 4.0 }, sy(*y) - 3.0);
    }
    ctx.set_line_width(2.0);
    let mut lx = l + 90.0;
    for s in &p.series {
        ctx.set_stroke_style_str(s.color);
        ctx.begin_path();
        for (i, (&x, &y)) in s.x.iter().zip(s.y).enumerate() {
            if i == 0 {
                ctx.move_to(sx(x), sy(y));
            } else {
                if s.step {
                    ctx.line_to(sx(x), sy(s.y[i - 1]));
                }
                ctx.line_to(sx(x), sy(y));
            }
        }
        ctx.stroke();
        if !s.label.is_empty() {
            ctx.set_fill_style_str(s.color);
            ctx.set_text_align("left");
            let _ = ctx.fill_text(s.label, lx, 11.0);
            lx += ctx.measure_text(s.label).map(|m| m.width()).unwrap_or(40.0) + 14.0;
        }
    }
    if let Some(c) = p.cursor {
        ctx.set_line_width(1.0);
        ctx.set_stroke_style_str(fg);
        line(&ctx, sx(c), t, sx(c), t + h);
    }
    if let Some((x, y)) = p.marker {
        ctx.set_fill_style_str(fg);
        ctx.begin_path();
        let _ = ctx.arc(sx(x), sy(y), 4.5, 0.0, std::f64::consts::TAU);
        ctx.fill();
    }
}

fn line(ctx: &Ctx, x0: f64, y0: f64, x1: f64, y1: f64) {
    ctx.begin_path();
    ctx.move_to(x0, y0);
    ctx.line_to(x1, y1);
    ctx.stroke();
}
