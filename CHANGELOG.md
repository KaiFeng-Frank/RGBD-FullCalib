# Changelog

## [Unreleased]

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
