# Changelog

## [Unreleased]

- **v0.2 core landed: the verdict layer is data.** 22 checks moved from prose
  into `verdicts/rules_d435i.yaml`; a small engine feeds CLI / REPORT.md / GUI
  from one source. Factory reference params dumped to machine-readable JSON.
  First catch: the "five independent baseline measurements" claim was fake
  independence (Kalibr inherits the camera chain) — now honestly "two".
- Depth nonlinearity fully closed: pixel-locking sawtooth attributed to the
  device (cross-capture r=0.97), correction model shipped and validated
- Renamed `d435i-calibration-field-guide` → `RGBD-FullCalib`
- Depth nonlinearity: three independent methods attempted, verdict is
  rig-limited (see CALIBRATION.md) — published as a negative result
- Rig photos added

See [ROADMAP.md](ROADMAP.md) — next up: declarative verdict schema (v0.2),
Mid-360 + LiDAR-camera extrinsics (v0.3, hardware ordered).

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
