# Roadmap

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

### v0.2 — the verdict layer becomes data

Checks turn into declarative yaml rules — `{parameter, source, external
reference, tolerance, verdict}` — and the renderer just executes them. Adding a
device becomes writing rules, not forking code. The generic/device split already
marked in the README becomes a real package boundary (`pip install` the generic
half without pyrealsense2).

### v0.3 — LiDAR enters, cross-sensor begins

Livox Mid-360 (hardware ordered): LiDAR-IMU rotation, LiDAR-camera extrinsics
and time sync — the full "camera-and-LiDAR-on-one-rig" chain under the same
verdict discipline. The viewer already reserves the native point-stream channel
(`T_POINTS`) and the source abstraction was designed for this split.

### v0.4 — more devices, rig as a description file

Second and third RGB-D families (D455 / OAK / Orbbec class), ported from real
user breakpoints (there's a standing issue collecting them). A `rig.yaml`
describes sensors and links; the workbench derives the capture plan and the
calibration graph from it.

### v0.5 — the impact analyzer becomes a tool

What IMPACT_ANALYSIS.md does by hand today: input your motion profile and map
type, output which error terms are first-order for *you*, with the propagation
math shown. Calibration effort ranked by consequence, not by tradition.

### v1.0 — the workbench

Guided capture (the tool tells you which poses are missing, not just whether
corners were found), calibration regression (re-calibrate, diff against history,
alarm on drift), the knowledge base searchable by symptom. At this point the
repo outgrows its D435i name and gets renamed — GitHub redirects.

## Standing experiment queue (ranked by measured impact)

Depth nonlinearity beyond 0.92 m (the one unbounded row in the error budget) →
multipath at corners → per-material validity. ~30 min each with existing tools;
placeholder cards already in the GUI.
