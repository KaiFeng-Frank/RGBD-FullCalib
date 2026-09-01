# Changelog

## [0.3.0] — 2026-09-01

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
- Added the documented rig's dual-gyroscope constant time alignment:
  `t_d435i_imu = t_livox_imu − 1.940 ms`, composed with the frozen D435i
  camera–IMU shift as `t_depth = t_livox − 5.989 ms` for fused viewing.
- Added the MID-360S LiDAR–IMU operational transform from the official Livox
  coordinate definition: identity axes with IMU position
  `[+11.00,+23.29,−44.12] mm` in `livox_frame`.
- Added a 17-pose MID-360S IMU operational calibration: 14 fit poses and 3
  independent holdouts, fit/holdout RMS 0.00962/0.01194 m/s², with
  accelerometer bias/scale/misalignment, gyro bias, and per-axis 0.496 s
  short-window white-noise density. The result labels this as short-window
  white noise rather than long-duration Allan bias instability/random walk.
- Added one fail-closed MID-360S IMU pipeline from identity preflight and
  automatic pose capture (or rosbag2/NPZ reuse) through solve, independent
  holdout/observability gates, exclusive formal promotion, and result-catalog
  verification.
- Added an installable `rgbd_fullcalib` ROS 2 package with calibration and
  runtime launch files. The runtime node converts the Livox raw-g stream,
  applies the promoted accelerometer model and gyro static bias, and publishes
  calibrated SI `sensor_msgs/Imu` without claiming gyro scale.
- Added the exact two-part printable 3MF used by the documented rig, including
  its millimetre-unit geometry, print settings, orientation note, and SHA-256.
- Added per-point Livox `offset_time` scan-end rotational deskew using the
  built-in IMU and calibrated lever arm. Real-scan A/B reduced the
  high-rotation dominant-plane P95 from 70.04 to 20.39 mm on 70/70 accepted
  scans, while the low-rotation control remained 17.255→17.262 mm. This scope
  is rotational, not platform-translation or full-6DoF compensation.
- Split GUI calibration content into 11 current-rig result cards and a
  7-item reference catalog for other devices/deployments. Reference entries
  do not become current-rig pending/rework and do not define a release plan.
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

The Mid-360 + LiDAR-camera stage named in v0.2.0 was completed in v0.3.0.

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
- WebGL2 viewer: live point cloud, verdict cards, calibration-stage
  placeholders; `?static=1` screenshot mode
- Field notes: 15+ documented pitfalls (CALIBRATION.md), error-impact
  analysis with LiDAR comparison columns (IMPACT_ANALYSIS.md)
