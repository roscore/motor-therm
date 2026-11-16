# motor-therm

**한국어** · [English](#english)

로봇 관절 모터 열 실험실. "내 휴머노이드 관절 모터가 이 토크 사이클로 돌면 권선이 얼마나 뜨거워지고,
언제 한계에 닿고, 연속으로 얼마의 토크/듀티를 버틸 수 있을까?"에 답하는 정적 웹 앱입니다.
Rust → WebAssembly로 만들었고 브라우저 안에서만 동작합니다 (서버, 분석 도구, 외부 요청 없음).

> **교육용 추정치입니다. 실제 하드웨어와 모터 제조사 데이터로 반드시 검증하세요.**

## 기능
- 모터 상수(Kt, 선간 저항, 구리 온도계수, 감속비·효율), 열 저항 Rth1/Rth2, 시정수 τ1/τ2, 주위 온도,
  권선 한계, Rth2 냉각 계수 입력. 프리셋 3종(소형 서보 / 중형 QDD / 대형 프레임리스, 모두 "일반값").
- 관절 출력축 부하 사이클: 정지 자세 유지 / 보행 무릎 / 픽 앤 플레이스 / 계단 오르기 프리셋 +
  최대 6행 구간 표(시간 s, 토크 N·m), 시뮬레이션 시간 1–120분(기본 20분).
  사이클 프리셋은 선택한 모터의 연속 토크에 대한 비율로 정의되어 어떤 모터 크기에도 의미가 있습니다.
- 결과: 권선·하우징 온도 곡선, 최고 권선 온도, 한계 도달 시간(또는 "도달 안 함"), 주기 정상상태 온도,
  사이클 RMS 토크, 한계 온도에서의 연속 토크, 피크 구간 최대 듀티, 열폭주 경고.
- 가정 분석: 냉각 계수(0.3–1.5) 또는 감속비(현재값의 0.4–2.5배)에 따른 최고 권선 온도 곡선.
- 안전 토크 범위: 냉간 시작에서 1 s / 5 s / 30 s / 5 min / 연속으로 허용되는 최대 출력 토크.
- 3D 뷰(WebGL2): 단면 액추에이터(반투명 하우징, 스테이터와 코일, 로터, 출력축, 링크 암),
  코일 색 = 권선 온도(파랑→노랑→빨강), 하우징 색 = 하우징 온도, 한계선이 있는 온도계, 열 흐름 화살표.
  WebGL2가 없으면 수치와 2D 그래프만 표시합니다.
- 한국어/영어 전환(브라우저 언어 기본), 모바일 대응, 다크 모드는 `prefers-color-scheme`을 따릅니다.

## 모델과 가정
- **2노드 집중 열 회로**: 권선 노드(C1 = τ1/Rth1)와 하우징 노드(C2 = τ2/Rth2).
  권선 →(Rth1)→ 하우징 →(Rth2·냉각 계수)→ 주위. 냉각 계수는 저항만 바꾸고 열용량은 그대로입니다.
- **동손만 고려**: 철손, 와전류, 인버터 손실, 마찰 발열은 모델링하지 않습니다. 주위 온도는 일정합니다.
- **전류/저항 규약 (피크 진폭 Kt)**: 모터 토크 = 출력 토크 / (감속비·η), I_q = τ_motor / Kt
  (I_q는 상전류 진폭, dq 진폭 불변 변환). 동손 P = 1.5·I_q²·R_ph(T), R_ph = R_pp / 2,
  R(T) = R25·(1 + α(T − 25 °C)). 데이터시트 Kt가 RMS 상전류 기준이라면 먼저 Kt_peak = Kt_rms / √2로 변환해 입력하세요
  (그대로 넣으면 손실을 2배 과소평가합니다).
  감속기 효율은 역구동 방향에도 같은 방식으로 적용합니다(보수적).
- **적분**: 각 일정 토크 구간에서 손실은 권선 온도에 대해 아핀(P = p0 + b·Tw)이므로 2노드 시스템은
  선형입니다. 구간 경계까지 정확한 지수 해(2×2 행렬 지수)로 진행하므로 시간 간격과 무관하게 안정적입니다.
- **연속 토크(닫힌 해)**: ΔT = T_lim − T_amb = 1.5·I²·R_ph(T_lim)·(Rth1 + Rth2·k)에서 I를 구합니다.
- **열폭주**: α·P25·(Rth1 + Rth2·k) ≥ 1이면 정상상태가 없습니다("열폭주"로 표시). 그래프는 1000 °C에서 자릅니다.
- **주기 정상상태**: 평균 I²에 대한 정상상태에서 시작해 사이클을 반복해 리플을 수렴시킵니다.
- **피크 구간 최대 듀티**: 나머지 구간의 평균 I²을 유지한 채 피크 구간 비율을 늘릴 때
  사이클 평균 I²이 연속 한계와 같아지는 비율(평균 기반, 리플 무시).
- **암 끝 하중**: 부하가 있는 모든 구간의 토크에 같은 부호로 m·g·L을 더합니다(예시용 단순 모델).
- 3D 링크 암의 움직임은 사이클 위상을 실제 시간 속도로 보여주는 예시 동작입니다.

## 조작
- 3D 뷰: 왼쪽 드래그 = 회전, 오른쪽 드래그/두 손가락 = 이동, 휠/핀치 = 확대, 더블클릭 = 초기화(관성 있음).
- 뷰 프리셋: 원근 / 위 / 측면 / 단면 보기. 토글: 단면, 열 흐름, 온도계, 링크 암.
- 암 끝(흰 구)을 위아래로 끌면 하중(0–10 kg)이 바뀌고 즉시 다시 계산합니다.
- 재생: 재생/일시정지, 속도 1× / 10× / 60×(기본 60×), 시간 슬라이더. 그래프 커서가 따라갑니다.

## 로컬 빌드
```sh
rustup target add wasm32-unknown-unknown
cargo test                 # 코어 모델 + 수학 테스트
trunk build --release      # dist/ 에 정적 사이트 생성 (/motor-therm/ 하위 경로용)
trunk serve                # 로컬 개발 서버
sh tools/privacy_check.sh  # 공개 저장소 개인정보 검사
```
구성: `crates/mt_core`(순수 Rust 모델, 웹 의존성 없음), `crates/mt_web`(web-sys UI, WebGL2 렌더러, 2D 그래프).

## 라이선스
MIT, © motor-therm contributors. [LICENSE](LICENSE) 참고.

---

<a id="english"></a>
## English

A robot joint motor thermal lab: "My humanoid's joint motor runs this torque cycle — how hot does the
winding get, when does it hit the limit, and what continuous torque / duty can it sustain?"
A static Rust → WebAssembly web app that runs entirely in the browser (no server, no analytics, no
network requests).

> **Educational estimates — validate on real hardware with the motor maker's data.**

### Features
- Motor constants (Kt, phase-to-phase resistance, copper temperature coefficient, gear ratio and
  efficiency), thermal resistances Rth1/Rth2, time constants τ1/τ2, ambient, winding limit and a
  cooling factor on Rth2. Three editable presets (small servo / mid QDD / large frameless, all "typical").
- Load cycle at the joint output: hold / walking gait knee / pick-and-place / stair climb presets plus a
  6-row segment table (duration s, torque N·m) and a simulated time of 1–120 min (default 20).
  Cycle presets are defined as fractions of the selected motor's continuous torque.
- Results: winding and housing temperature traces, peak winding temperature, time to limit (or
  "never"), periodic steady-state temperature, RMS torque, continuous torque at the limit, max duty of
  the peak segment, thermal runaway warning.
- What-if sweep: peak winding temperature vs cooling factor (0.3–1.5) or gear ratio (0.4–2.5× current).
- Safe torque envelope from cold: 1 s / 5 s / 30 s / 5 min / continuous.
- WebGL2 3D view of a cutaway actuator with heat-coloured coils, housing tint, thermometer with limit
  line and heat-flow arrows. Without WebGL2 the numbers and 2D plots still work.
- Korean / English toggle (browser default), mobile friendly, dark mode via `prefers-color-scheme`.

### Model and assumptions
- **Lumped two-node network**: winding node C1 = τ1/Rth1, housing node C2 = τ2/Rth2;
  winding →(Rth1)→ housing →(Rth2·cooling)→ ambient. The cooling factor scales resistance only.
- **Copper loss only**: no iron / eddy-current, drive (inverter) or friction losses; constant ambient.
- **Current / resistance convention (peak-amplitude Kt)**: motor torque = output torque / (ratio·η),
  I_q = τ_motor / Kt where I_q is the phase-current amplitude (amplitude-invariant dq).
  Copper loss P = 1.5·I_q²·R_ph(T) with R_ph = R_pp / 2 and R(T) = R25·(1 + α(T − 25 °C)).
  If a datasheet quotes Kt per RMS phase current, enter Kt_peak = Kt_rms / √2 (entering it as is
  would understate the loss by 2×). Gear efficiency is applied the same way
  when back-driven (conservative).
- **Integration**: within a constant-torque segment the loss is affine in winding temperature, so the
  network is linear; it is stepped with the exact 2×2 matrix-exponential solution up to each segment
  boundary (stable for any step).
- **Continuous torque (closed form)**: solve T_lim − T_amb = 1.5·I²·R_ph(T_lim)·(Rth1 + Rth2·k) for I.
- **Runaway**: if α·P25·(Rth1 + Rth2·k) ≥ 1 there is no steady state ("runaway"); traces are capped at 1000 °C.
- **Periodic steady state**: warm-started at the mean-I² steady state, then cycled until the ripple settles.
- **Max duty of the peak segment**: the peak-segment fraction at which the cycle mean I² equals the
  continuous limit while the other segments keep their own mean I² (mean based, ripple ignored).
- **Payload**: adds m·g·L (same sign) to every loaded segment — an illustrative simplification.
- The link-arm swing in 3D is illustrative motion following the cycle phase in real time.

### Controls
- 3D: left drag orbit, right drag / two fingers pan, wheel / pinch zoom, double-click reset (with inertia).
- View presets Perspective / Top / Side / Section; toggles cutaway, heat flow, thermometer, link arm.
- Drag the arm tip (white sphere) up/down to set the payload (0–10 kg); the model re-runs live.
- Playback: play/pause, 1× / 10× / 60× (default 60×), time slider; plot cursors follow.

### Local build
```sh
rustup target add wasm32-unknown-unknown
cargo test                 # core model + math tests
trunk build --release      # static site in dist/ (served under /motor-therm/)
trunk serve                # local dev server
sh tools/privacy_check.sh  # privacy check for this public repository
```
Layout: `crates/mt_core` (pure Rust model, no web deps), `crates/mt_web` (web-sys UI, WebGL2 renderer,
canvas-2D plots).

### License
MIT, © motor-therm contributors. See [LICENSE](LICENSE).
