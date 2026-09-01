# LiDAR-Camera-IMU Calibration

**LiDAR-Camera-IMU Calibration is an open, one-stop multi-sensor calibration
and deployment workbench for cameras, IMUs, LiDARs and their cross-sensor
transforms. The documented D435i/MID-360S rig includes camera calibration,
MID-360S IMU calibration, LiDAR–camera and LiDAR–IMU geometry, cross-device time
alignment, scan deskew, native MID-360S viewing, one-canvas point-cloud fusion,
and a ROS 2 calibrated-IMU runtime.**

[中文版 README](README_zh.md) · field notes [CALIBRATION.md](CALIBRATION.md) (Chinese) ·
error propagation [IMPACT_ANALYSIS.md](IMPACT_ANALYSIS.md) (Chinese)

Calibration's worst failure mode is **looking beautiful while being consistently wrong**.
This workbench pairs fitted metrics with physical references, repeated captures
and device/mount identity, then publishes the applicable evidence tier beside
the result. The same facts feed the CLI, report, GUI and live fused viewer.

![Live D435i and MID-360S fused point cloud in one camera-frame canvas](results/lidar_rgbd_fused_viewer.gif)

## Delivered scope

**D435i calibration**: RGB intrinsics → stereo IR → RGB↔IR extrinsics →
depth noise model → camera-IMU → accelerometer intrinsics → thermal drift
model, together with capture/calibration tools, the GUI, and field-tested
engineering notes.

**Declarative verdict layer**: 26 rules feed the CLI, `REPORT.md`, and GUI cards
through one engine, so supporting another device means adding rules rather than
forking verdict code.

**D435i/MID-360S rig**: the viewer renders arbitrary ROS 2
`sensor_msgs/msg/PointCloud2` streams and Livox `CustomMsg`; the direct
LiDAR–camera workflow covers capture through import; and live D435i/MID-360S
clouds share one camera-frame canvas. The documented mount has four bound
operational MID-360S results: its IMU calibration, five-scene LiDAR–camera
extrinsic, LiDAR–IMU geometry, and LiDAR–D435i constant time offset. Per-point
`offset_time` and the 200 Hz built-in gyro drive scan-end rotational deskew
with the calibrated IMU lever arm.

## The documented rig

<p align="center">
  <img src="docs/img/rig_mid360s_d435i_ready.jpg" alt="Rigid MID-360S and D435i sensor rig ready for use" width="47%">
  <img src="docs/img/rig_mid360s_d435i_wiring.jpg" alt="MID-360S and D435i rig powered for data capture" width="47%">
</p>
<p align="center"><sub>Rigid MID-360S + D435i assembly: clean ready state (left), powered capture state (right)</sub></p>

The rig uses an Intel RealSense D435i (fw 5.12.7.100, USB 3.2) and a DFOPTIX
`Tag6-320-35.2mm` rigid AprilGrid. The manufactured target rules out print
scaling as an error source. `tagSpacing = 0.3` was confirmed four independent
ways: ruler-measured 10.6 mm gap, part number, Kalibr default, and a
reprojection-error sweep with a clear minimum at 0.30.

## End-to-end MID-360S IMU pipeline and ROS 2 deployment

The fail-closed entry point covers identity preflight, automatic stable-pose
capture (or existing rosbag2/NPZ inputs), explicit `g × 9.80665` conversion,
calibration, independent holdout/observability gates, formal-result promotion,
and the viewer-registry check:

```bash
# Live: automatically collect 12 fit + 3 holdout orientations from /livox/imu.
./calibrate_mid360s_imu.sh --project-root /path/to/your_rig_workspace

# Deterministically replay this documented rig's accepted result.
./calibrate_mid360s_imu.sh --verify-existing --inputs \
  data/mid360s_imu_intrinsic_20260901_run1 \
  data/mid360s_imu_intrinsic_20260901_run2 \
  data/mid360s_imu_intrinsic_20260901_run3
```

The repository is also an installable ROS 2 package. With ROS 2 Jazzy sourced:

```bash
colcon build --symlink-install --packages-select rgbd_fullcalib
source install/setup.bash
ros2 launch rgbd_fullcalib mid360s_imu_calibration.launch.py \
  project_root:=/absolute/path/to/artifact-root \
  config_file:=/absolute/path/to/mid360s_imu_calibration.yaml
ros2 launch rgbd_fullcalib mid360s_imu_runtime.launch.py \
  project_root:=/absolute/path/to/artifact-root \
  config_file:=/absolute/path/to/mid360s_imu_calibration.yaml
```

The runtime node subscribes to raw `/livox/imu`, applies the accepted
accelerometer bias/scale/non-orthogonality and gyro static bias, and publishes
calibrated SI measurements on `/livox/imu_calibrated`. Topics, frames, identity,
paths, and outputs are ROS parameters in
[`config/mid360s_imu_calibration.yaml`](config/mid360s_imu_calibration.yaml).

The exact printable bracket used by the documented rig is included as a
two-part, millimetre-unit [3MF package](hardware/MID360S_D435i_RK3588S_BATTERY_REV6_A1_PLA_4W15.3mf)
with [geometry/orientation notes](hardware/README.md). ROS installation places
it under `share/rgbd_fullcalib/hardware`.

## Results at a glance

| Stage | Key numbers | External check | Verdict |
|---|---|---|---|
| RGB intrinsics | fx/fy 884.8/883.9, reproj σ 0.56 px | principal point vs factory: 0.13 px | ✅ freeze |
| Stereo IR + baseline | baseline 50.148 mm | vs factory 50.228 mm (0.16%); two independent calibrations differ by 0.005 mm | ✅ freeze |
| Depth noise model | σ² = (a·z²)² + flatness², σ = 4.1/16.3/65 mm @ 1/2/4 m | two rounds, target flatness cross-checked | ✅ use as weights |
| Camera–IMU | timeshift 4.0494 ms; rotation ~1° class | solved gravity 9.8070; RGB/IR timeshift agree to 21 µs | ✅ freeze rotation + timeshift |
| RGB↔IR extrinsics | \|t\| diff vs factory 0.023 mm, rot 0.287° | factory calibration | ✅ freeze |
| Accelerometer intrinsics | largest non-orthogonality 2.83°; static ‖a‖ residual 3.76 mm/s² over 12 poses | local gravity from latitude/altitude | ✅ use |
| Thermal drift model | depth scale −372 ppm/°C, focal +203 ppm/°C, gyro bias 0.0046 °/s/°C | R² per channel; principal-point drift falsified (R²=0.07) | ✅ compensate depth scale |
| MID-360S IMU | accel bias `[−0.01745,+0.02770,−0.01560]` m/s²; scale `[0.999665,1.006801,1.007030]`; misalignment `[−0.002235,+0.001770,−0.003168]` rad | 14 fit + 3 holdout poses; RMS 0.00962/0.01194 m/s²; rank 9/9 | ✅ current rig operational |
| MID-360S→D435i extrinsic | 5-scene dense alignment; inliers median/P90 10.0/16.9 mm | multi-seed convergence 0.0072° / 0.12 mm | ✅ current rig operational |
| MID-360S LiDAR–IMU + deskew | axes identical; IMU at `[+11.00,+23.29,−44.12]` mm; high-rotation plane P95 70.04→20.39 mm | Livox manual; 70/70 scans improve, low-rotation control 17.255→17.262 mm | ✅ current rig operational |
| MID-360S→D435i time | dual-gyro offset −1.940 ms; final `t_depth = t_livox − 5.989 ms` | 12,910 matched samples; fit RMSE 0.01836 rad/s | ✅ current rig operational constant offset |

The GUI and result API expose **11 current-rig results** and a separate
**7-item calibration reference** catalog for other devices and deployment
conditions.

Machine-readable outputs live in [`data/*.yaml`](data) and [`results/*.json`](results),
including [`results/mid360s_d435i_timesync.json`](results/mid360s_d435i_timesync.json),
[`results/mid360s_imu.json`](results/mid360s_imu.json),
[`results/mid360s_lidar_imu.json`](results/mid360s_lidar_imu.json), and
[`results/mid360s_deskew_validation.json`](results/mid360s_deskew_validation.json);
plots live in [`results/*.png`](results).

The real-scan deskew validation had 100% valid point offsets. A same-point
ablation reduced P95 from 20.21 to 19.48 mm when the calibrated lever-arm
component was enabled (about 3.6% by group median; paired median about 2.8%).

The MID-360S IMU result also records gyro bias
`[−0.006402,+0.000822,−0.008318] rad/s` and per-axis 0.496 s short-window
white-noise density at 196.156 Hz: accel
`[0.004974,0.005405,0.006853] m/s²/√Hz`, gyro
`[0.001421,0.001216,0.001749] rad/s/√Hz`. The deployed correction uses the
measured accelerometer model and gyro static bias.

The LiDAR–camera transform belongs to the documented rigid installation;
another rig or a remount gets its own transform through the same workflow.

## Validated applicability

Every published result is tied to its sensor and rigid-mount identity. The
runtime applies only parameters that passed observability, repeatability, and
holdout checks; detailed controlled studies and deployment decisions are
preserved in [CALIBRATION.md](CALIBRATION.md).

## Engineering notes

Fifteen+ field-tested implementation notes are preserved in
[CALIBRATION.md](CALIBRATION.md) (Chinese, with universal code snippets and
numbers). Every entry below is resolved in the shipped tooling. Highlights:

| # | Engineering case | Applied solution |
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
| 14 | Headless WebGL capture with live animation | `requestAnimationFrame` keeps headless screenshots active — use the shipped `?static=1` mode for deterministic capture |

## Declarative verdict engine

![Depth-model verdict with online-calibration and SLAM-impact badges](docs/img/verdict_depth_badges.png)

```bash
python -m verdicts                      # terminal verdicts
python -m verdicts --md REPORT.md       # markdown report (committed in repo)
```

The 26 published checks live in
[`verdicts/rules_d435i.yaml`](verdicts/rules_d435i.yaml), each one
`{value expression, external reference, tolerance, action}` — executed by a
~200-line engine ([`verdicts/engine.py`](verdicts/engine.py)) that feeds the CLI
report, [`REPORT.md`](REPORT.md), and the GUI cards from one source of truth.
The same engine enforces completeness of the published artifact set. Adding a
device's verdict checks means writing rules, not forking verdict code.

The rules distinguish two independently captured baseline calibrations from
camera-chain values inherited into camera–IMU outputs; the independent pair
differs by 0.005 mm.

## The GUI

A WebGL2 point-cloud viewer with three tabs: **live cloud** (16-bit depth with
in-shader deprojection, or native xyz/intensity/RGB through VBOs), **calibration
results** (the verdict cards above, generated
from the actual yaml/json outputs — each card also carries two SLAM badges:
*can this quantity be calibrated online?* (extrinsic rotation / time offset / IMU bias
are standard online states in VINS-class systems; intrinsics and the depth chain are not)
and *how hard does it hit SLAM?*, tiered with the measured propagation number behind
each tier), and **calibration reference** (applicability/method/output for seven
reusable checks: four D435i and three LiDAR/rig references). The third tab keeps
this reusable method catalog separate from the 11 rig-specific results. Thermal
compensation can be applied to the live cloud with one checkbox.

![Calibration results with online-calibration and SLAM-impact badges](results/gui_slam_badges.png)

```bash
./view_pointcloud.sh fused                      # D435i + MID-360S, one frame/canvas
./view_pointcloud.sh mid360
./view_pointcloud.sh ros2 /your/pointcloud/topic
./view_pointcloud.sh ros2 auto                    # only when one candidate exists
./view_pointcloud.sh d435i --alt-emitter
./view_pointcloud.sh synthetic-points             # hardware-free native-stream test
```

`fused` opens one `camera_color_optical_frame` canvas and transforms both live
inputs into it. The documented rig's
[LiDAR–camera extrinsic](results/mid360s_d435i_extrinsic.local.json),
[time alignment](results/mid360s_d435i_timesync.json), and
[LiDAR–IMU geometry](results/mid360s_lidar_imu.json) load automatically;
`--extrinsic PATH` selects another rig's
camera transform. Livox `CustomMsg` scans use every point's `offset_time`, the
built-in IMU, and the known IMU lever arm for validated scan-end rotational
deskew. The
[generic LiDAR–camera workflow](docs/LIDAR_CAMERA_EXTRINSIC.md) covers capture,
solving, result conventions, import, and fused viewing.

The launcher opens `http://localhost:8080`, isolates ROS Jazzy from Conda, and
shuts both backends down when either live stream disconnects. Native ROS 2
clouds are rendered in their message frame as current-frame data.

## Repository layout

```
CALIBRATION.md          the field notes: all results + verdicts + pitfalls (Chinese)
IMPACT_ANALYSIS.md      which errors matter for mapping/localization, and why (Chinese)
package.xml / CMakeLists.txt   installable ROS 2 package: rgbd_fullcalib
launch/ + config/       calibration and calibrated-IMU runtime launch/config
kalibr.sh               dockerized Kalibr wrapper (X11, host UID, repo mounted at /data)
calibrate_cam.sh        camera / stereo calibration with explicit focal initials
calibrate_imu_cam.sh    camera-IMU calibration, optional --bag-from-to trimming
calibrate_mid360s_imu.sh  MID-360S IMU capture→solve→holdout→promotion pipeline
aprilgrid_6x6_35.2mm.yaml   the target (tag6-320; spacing verified 4 independent ways)
tools/                  [generic] = no RealSense dependency, reusable as-is
  capture.py            [D435i]   AprilGrid capture: settle detection, resume,
                        exposure lock, color|ir|stereo|trio streams
  record.py             [D435i]   bag recording (cam / cam+imu / imu), gyro-clock pairing
  bagio.py              [generic] ROS1 bag writing via rosbags (no ROS needed)
  check_depth.py        [D435i]   plane-fit depth noise model, two-round protocol
  allan.py              [generic] Allan deviation from any static IMU bag
  record_thermal.py     [D435i]   cold-start thermal sweep recorder (IMU + depth + tags)
  analyze_thermal.py    [generic] per-channel linear thermal model + R² (reads npz)
  record_imu_poses.py   [D435i]   12-pose static capture for accelerometer intrinsics
  imu_intrinsic.py      [generic] T·K·(a−b) least-squares solve against local gravity
  apply_imu_intrinsic.py [generic] rewrite any bag with corrected accel (A/B experiments)
  record_mid360s_imu_poses.py      automatic ROS 2 stable-pose capture
  calibrate_mid360s_imu_intrinsics.py  bag/NPZ solve + holdout/coverage gates
  promote_mid360s_imu.py           strict current-rig formal-result boundary
  mid360s_imu_runtime.py           raw-g ROS 2 IMU → calibrated SI topic
  calibrate_lidar_camera_timesync.py [generic] dual-gyro constant-offset fit
  imgs2bag.py           [generic] image folders → ROS1 bag
verdicts/               the verdict engine + rules (see above); python -m verdicts
view_pointcloud.sh      one entry for fused D435i+MID-360S, individual sensors,
                        arbitrary ROS 2 topics and hardware-free sources;
                        isolates Jazzy from Conda
viewer/                 WebGL2 viewer + calib summary server (stdlib http + websockets)
  lidar_calib.py        MID-360S rig-bound result and evidence lifecycle
  livox_deskew.py       per-point scan-end rotational deskew + IMU lever arm
  sources/base.py       [generic] source abstraction: dense depth maps and native
                        point streams are different render paths, split here on purpose
  sources/d435i.py      [D435i]   depth/color/IR via pyrealsense2
  sources/synthetic.py  [generic] camera-free test source
  sources/synthetic_points.py [generic] native-stream renderer regression source
  sources/ros2_points.py [generic] PointCloud2 discovery/decoding + optional Livox input
  protocol.py           [generic] depth/image/native xyz-intensity-RGB wire format
setup/                  udev rules, hardware setup
hardware/               exact printable two-part rig 3MF + orientation/hash notes
data/                   calibration outputs (yaml/txt tracked; bags are not)
results/                camera models + rig-bound LiDAR–camera, LiDAR–IMU,
                        time-alignment and deskew validation outputs + plots
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
map-based localization. It also ranks calibration priorities by measured impact.
If you only calibrate what your application actually feels, you save days.

## License

MIT — see [LICENSE](LICENSE).
