# Roadmap

> Renamed to **RGBD-FullCalib** on 2026-08-28 — the D435i is the first instance,
> not the scope.

## The destination

**A one-stop, multi-sensor calibration workbench.** Describe your rig — cameras,
IMUs, LiDARs, wheels — then: capture with guidance, calibrate every intrinsic and
every cross-sensor extrinsic, and get every parameter back **with a verdict**:
checked against references from outside the optimization, error surfaces
visualized, unusable numbers flagged before they poison your SLAM stack.
Plus the two things parameter files never carry: an **impact analyzer** that tells
you which errors your application will actually feel (so you stop calibrating
what doesn't matter), and a growing **pitfall knowledge base** so nobody pays for
the same failure twice.

That tool does not exist. Kalibr stops at vision+IMU, vendor tools are black
boxes, the rest is scattered scripts. This repo is building it in public,
one honest version at a time.

## The road

### v0.1 — one camera, end to end (shipped)

The D435i as proof of method: seven calibration stages, each with an external
check and a trust decision. Negative results published. GUI. 15+ pitfalls.

### v0.2 — the verdict layer becomes data (shipped)

Twenty-four checks now live as declarative yaml rules — `{parameter, source,
external reference, tolerance, verdict}` — with one engine feeding the CLI,
`REPORT.md`, and the GUI. Factory references are machine-readable, and the
CLI, report, and GUI verdict rows no longer carry separate copies of the
decisions; GUI stage metadata remains device-specific. v0.2 deliberately stops
at this shared verdict engine; device plug-ins and dependency boundaries need a
real second device to expose the right seams.

### v0.3 — LiDAR enters, cross-sensor begins

Livox Mid-360 (hardware ordered): LiDAR-IMU rotation, LiDAR-camera extrinsics
and time sync — the full "camera-and-LiDAR-on-one-rig" chain under the same
verdict discipline. The viewer already reserves the native point-stream channel
(`T_POINTS`) and the source abstraction was designed for this split. This is
also the first real test of the generic/device package boundary: the Mid-360's
actual integration breakpoints, rather than a speculative abstraction, decide
what moves into the generic package and where `pyrealsense2` becomes optional.
If one second device is not enough evidence, that split completes in v0.4.

### v0.4 — more devices, rig as a description file

Second and third RGB-D families (D455 / OAK / Orbbec class), ported from real
user breakpoints (there's a standing issue collecting them). A `rig.yaml`
describes sensors and links; the workbench derives the capture plan and the
calibration graph from it. Complete the generic/device package boundary here if
the Mid-360 integration did not establish it cleanly in v0.3.

### v0.5 — the impact analyzer becomes a tool

What IMPACT_ANALYSIS.md does by hand today: input your motion profile and map
type, output which error terms are first-order for *you*, with the propagation
math shown. Calibration effort ranked by consequence, not by tradition.

### v1.0 — the workbench

Guided capture (the tool tells you which poses are missing, not just whether
corners were found), calibration regression (re-calibrate, diff against history,
alarm on drift), the knowledge base searchable by symptom —
full-stack RGB-D calibration, one command, every number with a verdict.

## Standing experiment queue (ranked by measured impact)

Completed in v0.2: depth nonlinearity was bounded and its repeatable
pixel-locking component gained a validated correction; corner multipath was
quantified and recorded as a verdict action recommending a 20 cm
depth-membership exclusion.

Remaining, in order: controlled reflectivity/per-material validity → time-offset
thermal drift → rolling-shutter characterization → gyro scale factor.
