# D435i Calibration Field Guide

**A complete, field-tested calibration of one Intel RealSense D435i — where every
parameter ships with an external-reference verdict, and every pitfall is written down.**

[中文版 README](README_zh.md) · full field notes in [CALIBRATION.md](CALIBRATION.md) (Chinese) ·
error-propagation analysis in [IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md) (Chinese)

![verdict card](docs/img/verdict_card.png)

Calibration's worst failure mode is **looking beautiful while being consistently wrong**.
Reprojection error tells you the model fits the data it was fitted to — nothing more.
So every result here is checked against something *outside* the optimization:
factory parameters, local gravity, repeated independent captures, a ruler.
And when a parameter fails its check, the tool says so — the red row above is this rig's
camera–IMU translation, shipped with the verdict *"do not freeze this number,
let your VIO estimate it online."*

## What this is (and is not)

**This is** a working example: one camera, calibrated end to end (RGB intrinsics →
stereo IR → RGB↔IR extrinsics → depth noise model → camera-IMU → accelerometer
intrinsics → thermal drift model), with the tools, the sanity-check layer, the GUI,
and ~15 documented pitfalls that cost real hours.

**This is not** (yet) a universal calibration suite. Scripts are written for the
D435i and tested on exactly one unit. If they help you calibrate something else,
that's a bonus — the *method* (verdict layer + external references) transfers even
where the code doesn't.

## Results at a glance

| Stage | Key numbers | External check | Verdict |
|---|---|---|---|
| RGB intrinsics | fx/fy 884.8/883.9, reproj σ 0.56 px | principal point vs factory: 0.13 px | ✅ freeze |
| Stereo IR + baseline | baseline 50.148 mm | vs factory 50.228 mm (0.16%); 5 independent captures within 0.14 mm | ✅ freeze |
| RGB↔IR extrinsics | \|t\| diff vs factory 0.023 mm, rot 0.287° | factory calibration | ✅ freeze |
| Depth noise model | σ² = (a·z²)² + flatness², σ = 4.1/16.3/65 mm @ 1/2/4 m | two rounds, target flatness cross-checked | ✅ use as weights |
| Camera–IMU | timeshift 4.04 ms; rotation ~1° class | solved gravity 9.8070 vs local 9.80665; RGB/IR timeshift agree to 21 µs | ⚠️ freeze rotation + timeshift only |
| Camera–IMU **translation** | three independent solutions spread 24.6–31.7 mm | the quantity itself is ~26 mm | ❌ **do not freeze** — 2–3 cm lever arm is structurally unobservable here |
| Accelerometer intrinsics | largest non-orthogonality 2.83°; static ‖a‖ residual 3.76 mm/s² over 12 poses | local gravity from latitude/altitude | ✅ use; see negative result below |
| Thermal drift model | depth scale −372 ppm/°C, focal +203 ppm/°C, gyro bias 0.0046 °/s/°C | R² per channel; principal-point drift **falsified** (R²=0.07) | ✅ compensate depth scale |

Machine-readable outputs live in [`data/*.yaml`](data) and [`results/*.json`](results);
plots in [`results/*.png`](results).

## Negative results (published on purpose)

Things that *didn't* work are half the value of a calibration log:

- **The camera–IMU translation is not calibratable on this rig.** Three independent
  solves scatter across 24.6–31.7 mm for a ~26 mm quantity. The lever arm (IMU sits
  2–3 cm from the optical center) is too short for handheld excitation. Ship the
  rotation, ship the timeshift, hand the translation to your VIO as an initial guess.
- **Six-position accelerometer calibration does not fix dynamic residuals.** A strict
  controlled experiment (same bag, only the accel preprocessing changed) left Kalibr's
  normalized residuals essentially unchanged — the static-pose intrinsics were real,
  but the dynamic residual is dominated by vibration, blur and sync jitter, not by
  scale/non-orthogonality. Full table in [CALIBRATION.md](CALIBRATION.md).
- **IMU-intrinsic correction couples into the extrinsic rotation** (~0.9° shift):
  whatever axis-correction you calibrate with, you must also deploy with.
- **Principal-point thermal drift is a myth on this unit** (R²=0.07 over a 13 °C
  sweep) — measured, falsified, crossed off the worry list.
- **A single static pose cannot separate bias, scale and non-orthogonality.** I tried.
  It produced a confident, wrong scale-factor claim; 12 poses later the real culprit
  was non-orthogonality. Documented as a worked example of the identifiability trap.

## Pitfalls that cost real hours

Fifteen+ entries with full stories in [CALIBRATION.md](CALIBRATION.md) (Chinese, but the
code snippets and numbers read universally). Highlights:

| # | Pitfall | One-line takeaway |
|---|---|---|
| 1 | `cv2.aruco` default adaptive-threshold windows (3…23 step 10) | at ~6 px/tag-cell in IR you detect 2/36 tags; use 3…15 step 2 → 29/36 |
| 2 | Both RealSense sensors report `supports(exposure)` | a loop "find the sensor" sets exposure on **RGB**, your IR stays untouched — *if a knob changes nothing, the knob isn't connected* |
| 3 | Overexposure kills tag detection at any distance | lock short exposure, watch the histogram, not the pretty image |
| 4 | Working-distance window = f(board size, focal, resolution) | at 848×480 the "board fits" and "tags big enough" windows barely overlap — switch resolution, don't fight distance |
| 5 | RealSense options are device-persistent | yesterday's experiment silently poisons today's capture |
| 6 | Kalibr's silent-NaN focal init | non-interactive runs read an empty focal answer, set fx≈2.6e-315, then print "Calibration complete" — feed initials via stdin (see `calibrate_cam.sh`) |
| 7 | Kalibr's pose spline is hard-coded at 100 knots/s | jerky handheld motion crashes the solver — move slower, not smoother |
| 8 | RGB and IR have no hardware sync | during motion a ~44 ms offset gets absorbed into the *extrinsics* (mm-level bias); capture stills with a settle detector — offset drops to ~0.1 ms |
| 9 | Emitter dilemma: depth wants speckle, tags/VIO hate it | `emitter_on_off` alternation gives both; depth rate is *not* halved (only IR splits), verified two independent ways |
| 10 | Allan variance vs thermal modeling need **opposite** captures | Allan wants constant temperature, thermal wants a sweep — one dataset cannot serve both |
| 11 | USB2 negotiation was the *cable* | hub and port were innocent all along; USB-C cables are visually indistinguishable — check `bcdUSB`, then swap the cable first |
| 12 | Stream-pairing must use a pending queue | "drop if out of range" silently discards valid samples (gyro/accel interleave) |
| 13 | Range-along-ray vs z-depth confusion | stored ray length as depth once — a flat wall renders as a sphere (101 mm residual, right angles read 81.8°) |
| 14 | WebGL pages never finish "loading" | `requestAnimationFrame` keeps headless screenshots timing out — ship a `?static=1` mode; verify your GUI with your own eyes |

## The GUI

A WebGL2 point-cloud viewer with three tabs: **live cloud** (16-bit depth over WebSocket,
in-shader deprojection), **calibration results** (the verdict cards above, generated
from the actual yaml/json outputs), and **pending experiments** (placeholders with
why/how/cost). Thermal compensation can be applied to the live cloud with one checkbox.

```bash
cd viewer
python server.py --source d435i --alt-emitter     # live, with emitter alternation
python server.py --source synthetic               # no camera needed
# open http://localhost:8080         (add ?static=1 for a screenshot-friendly page)
```

![calibration page](docs/img/calib_page.png)

## Repository layout

```
CALIBRATION.md          the field notes: all results + verdicts + pitfalls (Chinese)
IMPACT_ANALYSIS.md      which errors matter for mapping/localization, and why (Chinese)
kalibr.sh               dockerized Kalibr wrapper (X11, host UID, repo mounted at /data)
calibrate_cam.sh        camera / stereo calibration with explicit focal initials
calibrate_imu_cam.sh    camera-IMU calibration, optional --bag-from-to trimming
aprilgrid_6x6_35.2mm.yaml   the target (tag6-320; spacing verified 4 independent ways)
tools/
  capture.py            AprilGrid capture: settle detection, resume, exposure lock,
                        color|ir|stereo|trio streams
  record.py             bag recording (cam / cam+imu / imu), gyro-clock IMU pairing
  bagio.py              ROS1 bag writing via rosbags (no ROS installation needed)
  check_depth.py        plane-fit depth noise model, two-round protocol
  allan.py              Allan deviation from a 3 h static bag
  record_thermal.py     cold-start thermal sweep recorder (IMU + depth + tags)
  analyze_thermal.py    per-channel linear thermal model + R²
  record_imu_poses.py   12-pose static capture for accelerometer intrinsics
  imu_intrinsic.py      T·K·(a−b) least-squares solve against local gravity
  apply_imu_intrinsic.py  rewrite a bag with corrected accel (for A/B experiments)
viewer/                 WebGL2 viewer + calib summary server (stdlib http + websockets)
setup/                  udev rules, hardware setup
data/                   calibration outputs (yaml/txt tracked; bags are not)
results/                allan/thermal/depth models (json/yaml) + plots
```

## Reproducing on your own D435i

1. `pip install -r requirements.txt` (Python ≥3.10; no ROS required)
2. Build the Kalibr docker image once: `docker build -t kalibr:noetic -f kalibr/Dockerfile_ros1_20_04 kalibr/`
   (clone [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) into `kalibr/` first)
3. Print an AprilGrid, **measure** tag size and spacing with a ruler, edit the yaml
4. Capture → calibrate → *check the verdict rows before you trust anything*:
   the exact commands for every stage are at the end of [CALIBRATION.md](CALIBRATION.md)

Hardware note: results in this repo are from one D435i (fw 5.12.7.100) on USB 3.2.
Your unit's numbers **will** differ — that's the point of calibrating.

## Why publish error *impact*, not just error

[IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md) propagates every measured error into
mapping and localization terms (with a LiDAR/LiDAR-camera comparison column for a
Mid-360 rig), under one organizing principle: classify each error as zero-mean random /
constant systematic / **state-dependent systematic** — the third class is what breaks
maps (double walls from thermal depth-scale drift) and the second silently caps
map-based localization. It also re-ranks which pending calibrations are worth doing
at all. If you only calibrate what your application actually feels, you save days.

## License

MIT — see [LICENSE](LICENSE).
