# Completed scope and release history

> Renamed to **RGBD-FullCalib** on 2026-08-28 — the D435i is the first
> complete instance, not the scope.

## Completed scope

RGBD-FullCalib is an open, one-stop multi-sensor calibration workbench with
visualized, machine-readable results and explicit evidence and deployment
scope. The completed repository includes:

- a full D435i chain covering RGB intrinsics, stereo IR, RGB↔IR extrinsics,
  depth noise, camera–IMU, accelerometer intrinsics, and thermal behavior;
- a declarative verdict layer shared by the CLI, `REPORT.md`, and GUI;
- native xyz/intensity/RGB point-stream rendering, generic ROS 2
  `PointCloud2` input, and direct Livox compatibility;
- a direct MID-360S↔D435i capture, solve, import, and identity-bound result
  workflow;
- one live fused D435i/MID-360S canvas using the documented mount's
  five-scene operational extrinsic;
- a dual-gyroscope constant-offset fit for the documented rig
  (`t_d435i_imu = t_livox_imu − 1.940 ms`) and the composed viewer equation
  `t_depth = t_livox − 5.989 ms`;
- a 17-pose MID-360S IMU operational calibration (14 fit + 3 holdout) covering
  accelerometer bias/scale/misalignment, gyro bias, and explicitly labeled
  0.496 s short-window white-noise density;
- a fail-closed end-to-end MID-360S IMU pipeline for live pose capture or
  recorded-bag replay, automatic solving, holdout/observability gates, formal
  result promotion, and safe verification of the documented rig;
- an installable ROS 2 package (`rgbd_fullcalib`) with launch files for both
  calibration and runtime correction from `/livox/imu` to
  `/livox/imu_calibrated`;
- the MID-360S manufacturer-defined LiDAR–IMU transform: identity axes and
  IMU position `[+11.00,+23.29,−44.12] mm` in `livox_frame`;
- per-point `offset_time` scan-end rotational deskew, including the calibrated
  IMU lever-arm component, with real-scan A/B validation (high-rotation
  dominant-plane P95 70.04→20.39 mm, 70/70 scans improved; low-rotation
  control 17.255→17.262 mm);
- 11 current-rig result cards and a separate 7-item method-reference catalog
  for other devices and deployment conditions; references are not current-rig
  pending/rework and are not a release plan;
- the exact two-part printable 3MF for the documented integrated
  MID-360S/D435i mount, retained with its print settings and installation
  metadata;
- measured error-impact analysis and a published pitfall knowledge base.

## Release history

### v0.1 — one camera, end to end (shipped 2026-08-28)

The D435i established the complete calibration path: seven stages, external
checks, explicit trust decisions, a GUI, and 15+ documented pitfalls.

### v0.2.0 — the verdict layer becomes data (shipped 2026-08-28)

Twenty-four checks moved into declarative yaml rules — `{parameter, source,
external reference, tolerance, verdict}` — with one engine feeding the CLI,
`REPORT.md`, and GUI. Factory references became machine-readable, depth
nonlinearity was bounded with a validated correction, and corner multipath was
quantified with a deployment action.

### v0.3.0 — LiDAR and cross-sensor fusion (shipped 2026-09-01)

MID-360S support added native point transport, generic ROS 2 point-cloud input,
the complete direct LiDAR–camera workflow, a rig-bound five-scene operational
extrinsic, MID-360S IMU operational calibration, LiDAR–IMU geometry, constant
LiDAR–D435i time alignment, and scan-end rotational deskew with the known IMU
lever arm. The release also ships the fail-closed end-to-end IMU calibration
pipeline, ROS 2 runtime correction node, and the exact two-part integrated-mount
3MF. Live D435i/MID-360S fusion runs in one camera-frame canvas; stream
freshness checks and cascaded shutdown complete the live hardware lifecycle.
