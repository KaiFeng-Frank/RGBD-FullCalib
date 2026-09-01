# MID-360S + D435i rig model

[`MID360S_D435i_RK3588S_BATTERY_REV6_A1_PLA_4W15.3mf`](MID360S_D435i_RK3588S_BATTERY_REV6_A1_PLA_4W15.3mf)
is the printable model used by the documented rigid sensor assembly. The 3MF
uses millimetres and preserves the two printable parts and the Bambu Studio
plate settings:

- `MID360S_D435i_RK3588S_BATTERY_REV6_main_A1_PRINT.stl`
- `MID360S_D435i_RK3588S_BATTERY_REV6_backpack_A1_PRINT.stl`

The embedded meshes are healthy according to the saved slicer metadata
(no repaired edges, removed facets, reversed facets, or backward edges). Their
source-coordinate bounding sizes are 92 × 187 × 118 mm and 77 × 79 × 60.2 mm.

The package passed a complete ZIP/3MF integrity check. Its SHA-256 is:

```text
9304ddc0b86f38d048308ebc4c0d892858fe2a0d73f266eabbd33514c36028d8
```

Physical orientation of the calibrated assembly: the MID-360S power cable
faces the bracket side carrying the D435i. The package contains the bracket
geometry, not vendor CAD models of the sensors. Calibration transforms remain
defined by the frame conventions in `results/`; print-bed coordinates are not
sensor coordinate frames.

The model is distributed under the repository [LICENSE](../LICENSE).
