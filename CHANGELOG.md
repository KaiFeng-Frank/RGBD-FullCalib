# Changelog

## [Unreleased]

- Added a frozen, reproducible acceptance test for firmware/SDK-delivered D435i
  IR1/IR2 epipolar alignment. It uses an independent 16-pair holdout with no
  remap or fitted parameters, preserves insufficient/negative outcomes, and
  reports the non-blocking verdict inside the existing IR calibration card.
- Generalized the WebGL2 viewer from depth images to native xyz point streams,
  with range/intensity/RGB/height rendering and source-aware controls.
- Added ROS 2 point-topic discovery and generic `PointCloud2` decoding (QoS,
  padded organized clouds, endianness, FLOAT32/FLOAT64 coordinates, optional
  intensity/RGB), plus direct compatibility with an existing Livox CustomMsg
  publisher.
- Added `view_pointcloud.sh` for MID-360S, arbitrary ROS 2 topics, D435i and
  hardware-free synthetic point streams. ROS Jazzy runs in an isolated
  system-Python 3.12 venv rather than the calibration Conda environment.
- Added a same-HTML fused view: live D435i and MID-360S inputs are transformed
  into one `camera_color_optical_frame` canvas. The UI no longer opens two
  point-cloud windows and binds startup to the selected rig extrinsic.
- Implemented the direct LiDAR-camera capture/solve/import workflow and
  published a five-scene `LOCAL operational` transform for the documented
  mount and fused viewer.
- Added a post-start stream-freshness watchdog: five seconds without frames is
  a hard disconnect, the backend exits, D435i always releases its pipeline,
  and the fused launcher cascades cleanup after an unplug.
- Added portable, rig-bound result handling with explicit provenance tiers for
  the direct MID-360S–D435i 6DoF extrinsic.

## [0.2.0] — 2026-08-28

- **The verdict layer is data.** 24 checks moved from prose into
  `verdicts/rules_d435i.yaml`; one engine feeds CLI / REPORT.md / GUI
  from one source. Factory reference params dumped to machine-readable JSON.
  First catch: the "five independent baseline measurements" claim was fake
  independence (Kalibr inherits the camera chain) — now honestly "two".
- Depth nonlinearity closed: the repeatable pixel-locking sawtooth was
  attributed to the device (cross-capture r=0.97), bounded below the vendor
  specification, and shipped with a validated correction model
- Corner multipath quantified at 25.2 mm on the floor side; the result is now a
  verdict rule that recommends rejecting depth near concave intersections
- Added a hardware-free integration test that keeps the 24-rule schema, CLI
  JSON, committed `REPORT.md`, and GUI projection in lockstep
- Renamed `d435i-calibration-field-guide` → `RGBD-FullCalib`
- Rig photos added

At the 0.2.0 release, the next planned stage was Mid-360 + LiDAR-camera
extrinsics; see the current [ROADMAP.md](ROADMAP.md) for its implemented status.

## [0.1.0] — 2026-08-28

First public release: one Intel RealSense D435i calibrated end to end.

- Seven calibration stages, each with an external-reference check and an
  explicit verdict (freeze / weight / do-not-freeze)
- Negative results published on purpose (unobservable cam-IMU translation,
  six-position accel calibration vs dynamic residuals, falsified
  principal-point thermal drift, single-pose identifiability trap)
- Tools: capture with settle detection, ROS1 bag writing without ROS,
  Allan analysis, thermal sweep + per-channel model, accelerometer
  intrinsics from 12 static poses, A/B bag rewriting
- WebGL2 viewer: live point cloud, verdict cards, pending-experiment
  placeholders; `?static=1` screenshot mode
- Field notes: 15+ documented pitfalls (CALIBRATION.md), error-impact
  analysis with LiDAR comparison columns (IMPACT_ANALYSIS.md)
