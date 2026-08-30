# VURP CAD component architecture

The CAD model is composed from typed Python objects rather than passing raw
Rhino geometry between Grasshopper components. Rhino conversion happens only
at the output boundary.

## Core concepts

- `Cutout` describes a reusable cutting operation. The same type represents a
  through-cut or a depth-controlled recess.
- `LowerPlate` owns the host plate, walls, shared fillet, and top reveal.
- `BreadboardMount`, `MotorDriverMount`, and `DisplayMount` own the dimensions
  and behavior needed to integrate their devices into the host plate.
- `UpperMountPattern` and `ChassisMountPattern` are reusable interface
  contracts: they expose hole diameters and centers independently of the part
  that consumes them.
- `PlateAssembly` composes the host and all mounts/patterns in a deterministic
  boolean order.
- `DifferentialMechanismPlatform` consumes `UpperMountPattern` and derives its
  four mating pegs from that shared contract.

## Grasshopper data flow

```text
Lower Plate Definition ─┐
Breadboard Mount ───────┤
Motor Driver Mount ─────┤
Display Mount ──────────┼─> Lower Plate Assembly ─> body + display holder
Upper Mount Pattern ────┤
Chassis Mount Pattern ──┘
          │
          └────────────────> Differential Mechanism Platform
```

Feature components output configuration objects, not Breps. This keeps them
CAD-agnostic, makes validation explicit, and allows downstream parts to reuse
interfaces such as `UpperMountPattern` without copying coordinates.

`build_vurp_lower_plate(PlateParameters)` remains available as a compatibility
wrapper for scripts written against the original flat parameter API.
