# VURP — Parametric Lower Electronics Plate (COMPAS handoff)

## Goal

Write a small Python script using **COMPAS** that generates the first printable structural part for VURP: a top plate that sits on the Black Gladiator's aluminium deck, with front/back skirts descending below the deck, an integrated tilted display cradle, a longitudinal half-breadboard opening, and four mounting holes for a future upper mechanism layer.

Keep the model **fully parametric** and organized as reusable geometry-building functions. Do **not** model the differential mechanism yet.

## Coordinate convention

- `X` = chassis length, **front = +X**
- `Y` = chassis width, left/right
- `Z` = up
- Top surface of the Gladiator metal deck = `Z = 0`
- Main printed plate extends upward from `Z = 0`
- Front/rear walls extend downward into `-Z`

## Important chassis dimension note

DFRobot publishes **193 × 163 × 60 mm** for the *assembled rover*, including tracks, not the footprint of the central aluminium deck. Do **not** use 193 × 163 mm as the plate size.

Make the actual metal-deck dimensions explicit parameters:
```python
plate_length = ...  # measured metal deck length
plate_width  = ...  # measured metal deck width
```

## Main parameters

```python
plate_length          # measured Gladiator metal deck length
plate_width           # measured Gladiator metal deck width
plate_thickness = 8.0

front_wall_height     # downward from deck
rear_wall_height      # downward from deck
wall_thickness        # independent parameter

breadboard_length
breadboard_width
breadboard_clearance  # XY fit clearance
breadboard_offset_x = 0.0
breadboard_offset_y = 0.0

display_width
display_height
display_depth
display_clearance
display_tilt_deg
display_offset_y
display_recess_depth

upper_hole_diameter
upper_hole_x_positions  # two longitudinal positions, symmetric by default
upper_hole_y_offset     # one row on each side of breadboard
```

All dimensions in **mm**.

## Geometry

### 1. Main plate
Rectangular solid matching the central metal deck footprint.

- thickness default: **8 mm**
- lower face at `Z = 0`
- top face at `Z = +plate_thickness`

### 2. Front + rear downward walls
Continuous walls attached to the **front and rear edges only**.

- extend from the plate down into `-Z`
- independent parametric heights
- intended to visually/protectively cover the motor/battery hardware below the aluminium deck
- keep wall thickness parametrizable
- no side walls in this first iteration

Prefer small fillets/chamfers only if they remain robust and easy to print; geometry should work without them.

### 3. Half-breadboard opening
Make a rectangular **through-opening** in the plate, long axis parallel to `X`.

The half-breadboard is **10 mm tall**. Because the plate is 8 mm thick and the breadboard rests on the original metal deck at `Z=0`, it should protrude approximately **2 mm above the printed plate**.

- opening dimensions = breadboard XY dimensions + clearance
- centered by default, but `offset_x/y` must be exposed
- simple rounded corners are optional/parametric

### 4. Four upper-layer mounting holes
Four vertical through-holes in the plate for a future upper electronics/mechanism layer.

- two holes on each side of the breadboard opening
- symmetric by default
- hole diameter parametrizable
- `Y` spacing controlled by `upper_hole_y_offset`
- the two `X` positions controlled explicitly/parametrically
- future layer will plug/screw/standoff into these holes; do not design that layer yet

### 5. Integrated tilted display cradle
At the **front** of the plate, integrate a tilted display support similar to the current VURP concept render.

The display should sit on an inclined planar surface; do **not** rely on the PCB's mounting holes.

Requirements:
- tilt angle parametrizable
- shallow rectangular recess/depression matching the display body/PCB envelope + clearance
- recess prevents lateral sliding
- leave display face exposed
- cradle must blend structurally into the plate/front area rather than being a loose bracket

Also generate an **optional separate bezel/frame** that sits over the perimeter of the display and mechanically traps it in the recess. The bezel can use small screws/snap tabs later; for this iteration, just make its geometry and fit parameters clear. Do not cover the active screen area.

## Script architecture

Prefer simple functions, e.g.:

```python
def make_base_plate(params): ...
def add_front_rear_walls(body, params): ...
def cut_breadboard_opening(body, params): ...
def cut_upper_mount_holes(body, params): ...
def add_display_cradle(body, params): ...
def make_display_bezel(params): ...
def build_vurp_lower_plate(params): ...
```

Return at least:
```python
body
display_bezel  # optional second printable object
```

Use COMPAS geometry/Brep/mesh operations appropriate to the installed version, but keep boolean operations isolated so the backend can be swapped later.

## Print/design assumptions

- Intended for FDM printing, initially on a Bambu A1 Mini.
- Avoid unnecessary decorative geometry.
- Prefer function-driven geometry, sensible clearances, printable overhangs, and serviceability.
- The plate should remain visually clean enough that a later iteration can add a smooth curved outer cover over the exposed electronics.
- Do not add the differential mechanism, upper platform, battery mounts, MCU mounts, or cable routing yet.

## First deliverable

Produce:
1. one self-contained Python script;
2. a clear parameter block at the top;
3. generated COMPAS geometry for the lower plate + optional display bezel;
4. a simple preview/export path (STL preferred, STEP if straightforward);
5. comments marking dimensions that must be measured from the physical Gladiator/display/breadboard before final printing.
