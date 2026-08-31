# Roadmap

> Renamed to **RGBD-FullCalib** on 2026-08-28 — the D435i is the first instance,
> not the scope.

## The destination

**A one-stop, multi-sensor calibration workbench.** Describe your rig — cameras,
IMUs, LiDARs, wheels — then capture with guidance, calibrate intrinsics and
cross-sensor extrinsics, and receive visualized, machine-readable results with
an explicit evidence and deployment scope.
Plus the two things parameter files never carry: an **impact analyzer** that tells
you which errors your application will actually feel (so you stop calibrating
what doesn't matter), and a growing **pitfall knowledge base** so nobody pays for
the same failure twice.

RGBD-FullCalib now provides that open foundation: a complete D435i chain, a
declarative verdict layer, native LiDAR transport, direct LiDAR–camera
calibration and live fused viewing. Each version extends the same workbench.

## The road

### v0.1 — one camera, end to end (shipped)

The D435i as proof of method: seven calibration stages, each with an external
check and a trust decision. Negative results published. GUI. 15+ pitfalls.

### v0.2 — the verdict layer becomes data (shipped)

Twenty-four checks now live as declarative yaml rules — `{parameter, source,
external reference, tolerance, verdict}` — with one engine feeding the CLI,
`REPORT.md`, and the GUI. Factory references are machine-readable, and the
CLI, report, and GUI verdict rows no longer carry separate copies of the
decisions; GUI stage metadata remains device-specific. v0.2 established the
shared engine, and v0.3 exercised its device boundary with a real second sensor.

### v0.3 — LiDAR and cross-sensor fusion (implemented)

Livox Mid-360 hardware, native viewer transport and the direct LiDAR-camera
workflow are integrated. The viewer implements the native point-stream channel
(`T_POINTS`), generic PointCloud2 input, optional direct Livox compatibility,
and one fused D435i/MID-360S canvas. The author's unchanged mount has a
five-scene operational extrinsic used by that canvas. The direct chain also
establishes the rig-graph conventions used by the LiDAR–IMU and time-sync
extensions. The Mid-360 integration supplies the concrete generic/device
boundary that additional device families will continue to harden in v0.4.

### v0.4 — more devices, rig as a description file

Second and third RGB-D families (D455 / OAK / Orbbec class), ported from real
user breakpoints (there's a standing issue collecting them). A `rig.yaml`
describes sensors and links; the workbench derives the capture plan and the
calibration graph from it and expands the established generic/device boundary.

### v0.5 — the impact analyzer becomes a tool

What IMPACT_ANALYSIS.md does by hand today: input your motion profile and map
type, output which error terms are first-order for *you*, with the propagation
math shown. Calibration effort ranked by consequence, not by tradition.

### v1.0 — the workbench

Guided capture (the tool tells you which poses are missing, not just whether
corners were found), calibration regression (re-calibrate, diff against history,
alarm on drift), the knowledge base searchable by symptom —
full-stack RGB-D calibration, one command, every number with a verdict.

## Research extensions (ranked by measured impact)

Completed in v0.2: depth nonlinearity was bounded and its repeatable
pixel-locking component gained a validated correction; corner multipath was
quantified and recorded as a verdict action recommending a 20 cm
depth-membership exclusion.

Ranked next: controlled reflectivity/per-material validity → time-offset
thermal drift → rolling-shutter characterization → gyro scale factor.
