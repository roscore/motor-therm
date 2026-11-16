# Task: robot joint motor thermal lab (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook methods only (I²R copper loss with temperature coefficient, lumped two-node thermal
RC network, RMS torque). Motor presets are generic and clearly labelled "typical" (no vendor names,
no copied datasheet tables).

## Product (SMALL scope — core + UI about 800-1200 lines of new Rust; renderer/math are reused)
A single-page web app (GitHub Pages, sub-path `/motor-therm/`): "My humanoid's joint motor runs this
torque cycle — how hot does the winding get, when does it hit the limit, and what continuous torque /
duty can it sustain?"

1. **Inputs**: motor constants — torque constant Kt (N·m/A), phase-to-phase resistance at 25 °C (Ω),
   copper temperature coefficient (default 0.00393 /K), gear ratio and gearbox efficiency,
   thermal resistance winding→housing Rth1 (K/W), housing→ambient Rth2 (K/W), thermal time constants
   (winding τ1, housing τ2, seconds; capacitances derived as C = τ/R), ambient temperature,
   winding limit (default 120 °C), optional cooling factor for Rth2 (1.0 = free air, 0.5 = fan/heatsink).
   Presets (editable): "small servo (typical)", "mid quasi-direct-drive (typical)", "large frameless
   torque motor (typical)".
   Load cycle at the joint output (N·m vs time, repeating): presets "hold (static pose)", "walking gait
   knee" (periodic stance/swing pattern), "pick-and-place" (accelerate / hold / return / rest),
   "stair climb" (higher peaks). Plus a simple editable segment table (up to 6 rows: duration s,
   torque N·m) and a total simulation time (60 s - 2 h, default 20 min).
2. **Model (core)**: motor torque = output torque / (ratio·η); current I = τ/Kt; copper loss
   P = 1.5·I_q²·R_ph(T) with R_ph = R_pp/2 and R(T) = R25·(1 + α(T−25)) (peak-amplitude Kt
   convention; document it in README); two-node RC network
   integrated with a stable explicit scheme (sub-stepped, dt ≤ τmin/20) or exact exponential steps.
   Outputs: winding and housing temperature vs time, peak winding temperature, time to reach the
   limit (or "never"), steady-state periodic temperature, RMS torque of the cycle, continuous torque at
   the limit (closed-form steady state including R(T)), max sustainable duty for the peak segment.
   Thermal runaway check: if α·P makes steady state diverge, report "runaway".
3. **Extras (small)**: (a) "what-if" sweep: peak winding temperature vs cooling factor (0.3-1.5) or vs
   gear ratio (selectable), plotted as a curve with the current value marked; (b) a "safe torque
   envelope" readout: torque vs allowed on-time from cold (e.g. 1, 5, 30, 300 s, continuous).
4. **3D view (WebGL2, game-grade — see below)**: a cutaway joint actuator: outer housing cylinder
   (translucent volume with bright edges), stator ring with coil segments, rotor, output shaft and a
   simple link arm attached to the output that swings with the load cycle phase (illustrative motion).
   Coil colour = winding temperature on a blue→yellow→red heat ramp, housing tinted by its temperature;
   a vertical "thermometer" bar next to the actuator with the limit line; heat-flow arrows from coil
   to housing to air scaled by power. Checker ground with shadow.
5. **2D plots** (canvas 2D, own small module): winding + housing temperature vs time with limit line
   and playback cursor; load torque vs time (cycle) below; the sweep curve from extra (a).
6. Korean + English toggle (browser default). Mobile friendly. CSS prefers-color-scheme only.
Out of scope (do NOT build): iron loss / eddy current modelling, FEM, drive (inverter) losses,
URL share, JSON/CSV import/export, real data import.

## 3D view requirements (game-like, reuse the existing renderer)
- `crates/mt_web/src/gl.rs` and `crates/mt_web/src/math.rs` already exist: a tool-independent WebGL2
  renderer (lit meshes, checker/grid ground with fog, sky gradient, stencil planar shadows, translucent
  volumes, lines) and pure math (mat4, damped orbit camera with inertia, ray-plane/ray-sphere picking)
  with unit tests. Reuse them; extend only if needed and keep them tool-independent and tested.
- Scene code in `view3d.rs`. Orbit (left drag) / pan (right drag, two fingers) / zoom (wheel, pinch)
  with inertia, double-click reset, `touch-action:none`. View presets: Perspective / Top / Side /
  Section (camera looks into the cutaway).
- Direct manipulation: dragging the link arm tip vertically sets a "payload at arm tip" value (kg,
  0-10) that scales the hold torque of the current cycle and re-runs the simulation live; hover
  highlight + cursor change.
- Toggle buttons over the view: cutaway, heat-flow arrows, thermometer, link arm.
- Playback: play/pause, speed 1× / 10× / 60× (default 60× because thermal is slow), time slider;
  the plot cursor follows. HUD over the canvas: time, output torque, current, copper loss W,
  winding °C, housing °C, margin to limit.
- If WebGL2 is unavailable, show a message and keep the numeric readouts and 2D plots working.
- Release wasm ≲ 350 KB. No crates besides wasm-bindgen, web-sys, js-sys.

## Core crate (pure Rust, no web deps, unit tested) — `crates/mt_core`
- Motor + thermal params, load cycle (segments, periodic lookup, RMS), simulator, metrics, sweeps.
- Tests (hand-checkable, 6-10): zero torque → temperatures stay at ambient; constant loss P with α=0
  → steady winding temperature = T_amb + P·(Rth1+Rth2) within 0.5 %; single-node limit (tiny τ1) step
  response reaches 63.2 % of the rise at t = τ2 within 2 %; RMS of a 50 % duty square wave of torque
  T is T/√2; continuous torque at the limit, plugged back into the simulator, settles at the limit
  within 1 °C; α > 0 gives a higher steady temperature than α = 0 for the same current; halving the
  cooling factor lowers steady housing rise accordingly; runaway detected for an extreme case.
- `crates/mt_web`: UI, plain web-sys. The existing math.rs tests must keep passing.

## Repo hygiene
- README (Korean first, then English): what it is, model and assumptions (lumped 2-node, copper loss
  only, constant ambient, the current/resistance convention), controls, local build
  (`cargo test`, `trunk build --release`), MIT license, "educational estimates — validate on real
  hardware with the motor maker's data".
- LICENSE: MIT, copyright "motor-therm contributors" (exists). `.gitignore` for target/ dist/ (exists).
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  `Trunk.toml` exists (`filehash = false`, `public_url = "/motor-therm/"`, external `web/boot.js`
  copied via `<link data-trunk rel="copy-file" href="web/boot.js">`, which imports `./mt_web.js`).
- `.github/workflows/pages.yml` exists (trunk 0.22.1, privacy check, cargo test, deploy-pages). Keep it.
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test` green (workspace); `trunk build --release` succeeds with no warnings;
  `sh tools/privacy_check.sh` clean.
- Commit author `motor-therm contributors <59640857+roscore@users.noreply.github.com>`.
- Commit everything with clear commits; open a PR. Do NOT create new GitHub repos or change repo settings.
