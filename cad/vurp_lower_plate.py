"""Parametric lower electronics plate for VURP.

The geometry layer deliberately imports only COMPAS.  It does not import Rhino,
Grasshopper, or a CAD-specific boolean kernel.  A COMPAS Brep backend must be
available at runtime (Rhino is used by ``rhino_preview.py``).

Coordinate system: front = +X, left/right = Y, up = +Z, metal deck top = Z 0.
All dimensions are millimetres.
"""

from dataclasses import dataclass, field
from math import cos, pi, radians, sin, tan
from typing import Dict, Iterable, List, Optional, Tuple

from compas.datastructures import Mesh
from compas.files import STL
from compas.geometry import Box, Brep, Cylinder, Frame, NurbsCurve, Point, Vector


# Concrete Brep implementation. CAD adapters may replace this with (for
# example) compas_rhino.geometry.RhinoBrep without leaking Rhino imports here.
BREP_BACKEND = Brep


def set_brep_backend(backend) -> None:
    """Inject a COMPAS-compatible Brep implementation."""
    global BREP_BACKEND
    BREP_BACKEND = backend


@dataclass
class PlateParameters:
    # ---------------------------------------------------------------------
    # MEASURE BEFORE PRINTING.  These are conservative preview placeholders,
    # NOT DFRobot's published 193 x 163 mm assembled-rover envelope.
    plate_length: float = 145.0
    plate_width: float = 105.0
    # ---------------------------------------------------------------------
    plate_thickness: float = 8.0
    front_wall_height: float = 35.0
    rear_wall_height: float = 28.0
    wall_thickness: float = 4.0
    fillet_radius: float = 2.0

    # MEASURE the actual half breadboard (including clips/lips).
    breadboard_length: float = 82.5
    breadboard_width: float = 47.0
    breadboard_clearance: float = 0.6  # total XY clearance, not per side
    breadboard_offset_x: float = 0.0
    breadboard_offset_y: float = 0.0

    # Adafruit Motor FeatherWing PCB outline (product 2927). Only the X
    # placement is intended as a Grasshopper control in this iteration.
    motor_driver_offset_x: float = -55.0
    motor_driver_length: float = 50.8  # long axis, oriented along global Y
    motor_driver_width: float = 22.9
    motor_driver_clearance: float = 0.6  # total XY clearance
    motor_driver_corner_radius: float = 2.5
    # None tracks half the current plate thickness; a supplied value overrides it.
    motor_driver_recess_depth: Optional[float] = None
    motor_driver_mount_hole_diameter: float = 2.54
    motor_driver_mount_hole_spacing_x: float = 17.78
    motor_driver_mount_hole_spacing_y: float = 45.72
    motor_driver_draft_deg: float = 1.5
    motor_driver_counterbore_diameter: float = 5.0
    motor_driver_counterbore_depth: float = 2.0
    motor_driver_cable_hole_diameter: float = 6.0
    motor_driver_cable_channel_width: float = 4.0
    motor_driver_cable_channel_depth: float = 1.2
    top_reveal_width: float = 0.8
    top_reveal_depth: float = 0.5

    # MEASURE the complete display PCB/body and active screen opening.
    display_width: float = 38.0
    display_height: float = 28.0
    display_depth: float = 4.0
    display_active_width: float = 25.0
    display_active_height: float = 14.0
    display_window_offset_y: float = 0.0
    display_clearance: float = 0.5  # total width/height clearance
    display_tilt_deg: float = 55.0  # degrees above horizontal
    display_offset_y: float = 0.0
    display_recess_depth: float = 2.0
    display_support_margin: float = 4.0
    display_support_thickness: float = 5.0
    display_front_overlap: float = 5.0
    display_joint_width: float = 22.0
    display_joint_depth_x: float = 7.0
    display_joint_insertion_depth: float = 6.0
    display_joint_clearance: float = 0.35  # total clearance, not per side
    display_retainer_tab_size: float = 5.0
    display_retainer_overlap: float = 1.0
    display_retainer_thickness: float = 1.0

    upper_hole_diameter: float = 4.2
    upper_hole_x_positions: Tuple[float, float] = (-48.0, 48.0)
    upper_hole_y_offset: float = 36.0

    # Fasteners into the Gladiator metal deck's longitudinal slits. Each
    # front/rear coordinate defines a symmetric left/right pair.
    chassis_hole_diameter: float = 3.5
    chassis_counterbore_diameter: float = 6.5
    chassis_counterbore_depth: float = 2.0
    chassis_rear_hole_x: float = -55.0
    chassis_rear_hole_y_offset: float = 32.0
    chassis_front_hole_x: float = 55.0
    chassis_front_hole_y_offset: float = 32.0

    bezel_border: float = 3.0
    bezel_thickness: float = 2.0
    boolean_epsilon: float = 0.1

    def validate(self) -> None:
        positive = {
            name: value
            for name, value in vars(self).items()
            if name not in {
                "breadboard_offset_x",
                "breadboard_offset_y",
                "motor_driver_offset_x",
                "display_offset_y",
                "display_window_offset_y",
                "chassis_rear_hole_x",
                "chassis_front_hole_x",
                "fillet_radius",
            }
            and isinstance(value, (int, float))
            and name != "display_tilt_deg"
        }
        invalid = [name for name, value in positive.items() if value <= 0]
        if invalid:
            raise ValueError("Parameters must be positive: {}".format(", ".join(invalid)))
        if not 10.0 <= self.display_tilt_deg <= 85.0:
            raise ValueError("display_tilt_deg must be between 10 and 85 degrees")
        recess_depth = _motor_driver_recess_depth(self)
        if recess_depth <= 0.0 or recess_depth >= self.plate_thickness:
            raise ValueError("motor_driver_recess_depth must be greater than zero and less than plate_thickness")
        if self.motor_driver_counterbore_depth >= self.plate_thickness - recess_depth:
            raise ValueError("Motor-driver counterbore must leave material below the pocket")
        if self.chassis_counterbore_depth >= self.plate_thickness:
            raise ValueError("chassis_counterbore_depth must be less than plate_thickness")
        if self.chassis_counterbore_diameter <= self.chassis_hole_diameter:
            raise ValueError("chassis_counterbore_diameter must exceed chassis_hole_diameter")
        opening_width = self.breadboard_width + self.breadboard_clearance
        if 2.0 * self.upper_hole_y_offset <= opening_width + self.upper_hole_diameter:
            raise ValueError("Upper mounting-hole rows collide with the breadboard opening")
        if self.display_recess_depth >= self.display_support_thickness:
            raise ValueError("Display recess must be shallower than its supporting slab")
        aperture_h = self.display_active_height + self.display_clearance
        cavity_h = self.display_height + self.display_clearance
        if abs(self.display_window_offset_y) + 0.5 * aperture_h > 0.5 * cavity_h:
            raise ValueError("display_window_offset_y moves the screen opening outside the display body")
        required_holder_depth = (
            self.bezel_thickness
            + self.display_depth
            + self.display_clearance
            + 0.5 * self.display_retainer_thickness
        )
        if self.display_support_thickness < required_holder_depth:
            raise ValueError(
                "display_support_thickness must be at least {:.2f} mm for the rear-loading frame"
                .format(required_holder_depth)
            )
        if self.fillet_radius < 0:
            raise ValueError("fillet_radius cannot be negative")


@dataclass
class PlateResult:
    body: Brep
    display_holder: Brep
    parameters: PlateParameters
    features: Dict[str, object] = field(default_factory=dict)


@dataclass
class Cutout:
    """Reusable cut operation whose depth can vary without changing its type."""

    length: float
    width: float
    clearance: float = 0.0
    offset_x: float = 0.0
    offset_y: float = 0.0
    depth: Optional[float] = None
    corner_radius: float = 0.0
    through: bool = False

    @property
    def is_through(self) -> bool:
        return self.through

    def effective_depth(self, host_thickness: float) -> float:
        if self.through:
            return host_thickness
        return 0.5 * host_thickness if self.depth is None else self.depth


@dataclass
class LowerPlate:
    length: float = 145.0
    width: float = 105.0
    thickness: float = 8.0
    front_wall_height: float = 35.0
    rear_wall_height: float = 28.0
    wall_thickness: float = 4.0
    fillet_radius: float = 2.0
    top_reveal_width: float = 0.8
    top_reveal_depth: float = 0.5

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "LowerPlate":
        return cls(
            params.plate_length, params.plate_width, params.plate_thickness,
            params.front_wall_height, params.rear_wall_height,
            params.wall_thickness, params.fillet_radius,
            params.top_reveal_width, params.top_reveal_depth,
        )

    def configure(self, params: PlateParameters) -> None:
        params.plate_length = self.length
        params.plate_width = self.width
        params.plate_thickness = self.thickness
        params.front_wall_height = self.front_wall_height
        params.rear_wall_height = self.rear_wall_height
        params.wall_thickness = self.wall_thickness
        params.fillet_radius = self.fillet_radius
        params.top_reveal_width = self.top_reveal_width
        params.top_reveal_depth = self.top_reveal_depth


@dataclass
class BreadboardMount:
    cutout: Cutout = field(default_factory=lambda: Cutout(
        82.5, 47.0, 0.6, through=True
    ))

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "BreadboardMount":
        return cls(Cutout(
            params.breadboard_length, params.breadboard_width,
            params.breadboard_clearance, params.breadboard_offset_x,
            params.breadboard_offset_y, through=True,
        ))

    def configure(self, params: PlateParameters) -> None:
        params.breadboard_length = self.cutout.length
        params.breadboard_width = self.cutout.width
        params.breadboard_clearance = self.cutout.clearance
        params.breadboard_offset_x = self.cutout.offset_x
        params.breadboard_offset_y = self.cutout.offset_y

    def apply(self, body: Brep, params: PlateParameters) -> Brep:
        return cut_breadboard_opening(body, params)


@dataclass
class MotorDriverMount:
    cutout: Cutout = field(default_factory=lambda: Cutout(
        50.8, 22.9, 0.6, -55.0, 0.0, None, 2.5
    ))
    mount_hole_diameter: float = 2.54
    mount_hole_spacing_x: float = 17.78
    mount_hole_spacing_y: float = 45.72
    draft_deg: float = 1.5
    counterbore_diameter: float = 5.0
    counterbore_depth: float = 2.0
    cable_hole_diameter: float = 6.0
    cable_channel_width: float = 4.0
    cable_channel_depth: float = 1.2

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "MotorDriverMount":
        return cls(
            Cutout(
                params.motor_driver_length, params.motor_driver_width,
                params.motor_driver_clearance, params.motor_driver_offset_x,
                0.0, params.motor_driver_recess_depth,
                params.motor_driver_corner_radius,
            ),
            params.motor_driver_mount_hole_diameter,
            params.motor_driver_mount_hole_spacing_x,
            params.motor_driver_mount_hole_spacing_y,
            params.motor_driver_draft_deg,
            params.motor_driver_counterbore_diameter,
            params.motor_driver_counterbore_depth,
            params.motor_driver_cable_hole_diameter,
            params.motor_driver_cable_channel_width,
            params.motor_driver_cable_channel_depth,
        )

    def configure(self, params: PlateParameters) -> None:
        params.motor_driver_length = self.cutout.length
        params.motor_driver_width = self.cutout.width
        params.motor_driver_clearance = self.cutout.clearance
        params.motor_driver_offset_x = self.cutout.offset_x
        params.motor_driver_recess_depth = self.cutout.depth
        params.motor_driver_corner_radius = self.cutout.corner_radius
        params.motor_driver_mount_hole_diameter = self.mount_hole_diameter
        params.motor_driver_mount_hole_spacing_x = self.mount_hole_spacing_x
        params.motor_driver_mount_hole_spacing_y = self.mount_hole_spacing_y
        params.motor_driver_draft_deg = self.draft_deg
        params.motor_driver_counterbore_diameter = self.counterbore_diameter
        params.motor_driver_counterbore_depth = self.counterbore_depth
        params.motor_driver_cable_hole_diameter = self.cable_hole_diameter
        params.motor_driver_cable_channel_width = self.cable_channel_width
        params.motor_driver_cable_channel_depth = self.cable_channel_depth

    def apply(self, body: Brep, params: PlateParameters) -> Brep:
        return cut_motor_driver_slot(body, params)


@dataclass
class UpperMountPattern:
    hole_diameter: float = 4.2
    x_positions: Tuple[float, float] = (-48.0, 48.0)
    y_offset: float = 36.0

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "UpperMountPattern":
        return cls(params.upper_hole_diameter, params.upper_hole_x_positions,
                   params.upper_hole_y_offset)

    @property
    def centers(self) -> List[Tuple[float, float]]:
        return [(x, y) for x in self.x_positions for y in (-self.y_offset, self.y_offset)]

    def configure(self, params: PlateParameters) -> None:
        params.upper_hole_diameter = self.hole_diameter
        params.upper_hole_x_positions = self.x_positions
        params.upper_hole_y_offset = self.y_offset

    def apply(self, body: Brep, params: PlateParameters) -> Brep:
        return cut_upper_mount_holes(body, params)


@dataclass
class ChassisMountPattern:
    hole_diameter: float = 3.5
    counterbore_diameter: float = 6.5
    counterbore_depth: float = 2.0
    rear_x: float = -55.0
    rear_y_offset: float = 32.0
    front_x: float = 55.0
    front_y_offset: float = 32.0

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "ChassisMountPattern":
        return cls(
            params.chassis_hole_diameter,
            params.chassis_counterbore_diameter,
            params.chassis_counterbore_depth,
            params.chassis_rear_hole_x,
            params.chassis_rear_hole_y_offset, params.chassis_front_hole_x,
            params.chassis_front_hole_y_offset,
        )

    @property
    def centers(self) -> List[Tuple[float, float]]:
        return [
            (self.rear_x, -self.rear_y_offset),
            (self.rear_x, self.rear_y_offset),
            (self.front_x, -self.front_y_offset),
            (self.front_x, self.front_y_offset),
        ]

    def configure(self, params: PlateParameters) -> None:
        params.chassis_hole_diameter = self.hole_diameter
        params.chassis_counterbore_diameter = self.counterbore_diameter
        params.chassis_counterbore_depth = self.counterbore_depth
        params.chassis_rear_hole_x = self.rear_x
        params.chassis_rear_hole_y_offset = self.rear_y_offset
        params.chassis_front_hole_x = self.front_x
        params.chassis_front_hole_y_offset = self.front_y_offset

    def apply(self, body: Brep, params: PlateParameters) -> Brep:
        return cut_chassis_mount_holes(body, params)


@dataclass
class DisplayMount:
    width: float = 38.0
    height: float = 28.0
    depth: float = 4.0
    active_width: float = 25.0
    active_height: float = 14.0
    window_offset_y: float = 0.0
    clearance: float = 0.5
    tilt_deg: float = 55.0
    offset_y: float = 0.0
    recess_depth: float = 2.0
    support_margin: float = 4.0
    support_thickness: float = 5.0
    front_overlap: float = 5.0
    joint_width: float = 22.0
    joint_depth_x: float = 7.0
    joint_insertion_depth: float = 6.0
    joint_clearance: float = 0.35
    retainer_tab_size: float = 5.0
    retainer_overlap: float = 1.0
    retainer_thickness: float = 1.0
    bezel_border: float = 3.0
    bezel_thickness: float = 2.0

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "DisplayMount":
        names = (
            "display_width", "display_height", "display_depth",
            "display_active_width", "display_active_height",
            "display_window_offset_y", "display_clearance",
            "display_tilt_deg", "display_offset_y", "display_recess_depth",
            "display_support_margin", "display_support_thickness",
            "display_front_overlap", "display_joint_width",
            "display_joint_depth_x", "display_joint_insertion_depth",
            "display_joint_clearance", "display_retainer_tab_size",
            "display_retainer_overlap", "display_retainer_thickness",
            "bezel_border", "bezel_thickness",
        )
        return cls(*(getattr(params, name) for name in names))

    def configure(self, params: PlateParameters) -> None:
        mapping = {
            "display_width": self.width,
            "display_height": self.height,
            "display_depth": self.depth,
            "display_active_width": self.active_width,
            "display_active_height": self.active_height,
            "display_window_offset_y": self.window_offset_y,
            "display_clearance": self.clearance,
            "display_tilt_deg": self.tilt_deg,
            "display_offset_y": self.offset_y,
            "display_recess_depth": self.recess_depth,
            "display_support_margin": self.support_margin,
            "display_support_thickness": self.support_thickness,
            "display_front_overlap": self.front_overlap,
            "display_joint_width": self.joint_width,
            "display_joint_depth_x": self.joint_depth_x,
            "display_joint_insertion_depth": self.joint_insertion_depth,
            "display_joint_clearance": self.joint_clearance,
            "display_retainer_tab_size": self.retainer_tab_size,
            "display_retainer_overlap": self.retainer_overlap,
            "display_retainer_thickness": self.retainer_thickness,
            "bezel_border": self.bezel_border,
            "bezel_thickness": self.bezel_thickness,
        }
        for name, value in mapping.items():
            setattr(params, name, value)

    def apply(self, body: Brep, params: PlateParameters) -> Brep:
        return cut_display_holder_socket(body, params)

    def make_printable_part(self, params: PlateParameters) -> Brep:
        return make_display_holder(params)


@dataclass
class PlateAssembly:
    plate: LowerPlate = field(default_factory=LowerPlate)
    breadboard_mount: BreadboardMount = field(default_factory=BreadboardMount)
    motor_driver_mount: MotorDriverMount = field(default_factory=MotorDriverMount)
    display_mount: DisplayMount = field(default_factory=DisplayMount)
    upper_mount_pattern: UpperMountPattern = field(default_factory=UpperMountPattern)
    chassis_mount_pattern: ChassisMountPattern = field(default_factory=ChassisMountPattern)
    boolean_epsilon: float = 0.1

    @classmethod
    def from_parameters(cls, params: PlateParameters) -> "PlateAssembly":
        return cls(
            LowerPlate.from_parameters(params),
            BreadboardMount.from_parameters(params),
            MotorDriverMount.from_parameters(params),
            DisplayMount.from_parameters(params),
            UpperMountPattern.from_parameters(params),
            ChassisMountPattern.from_parameters(params),
            params.boolean_epsilon,
        )

    def to_parameters(self) -> PlateParameters:
        params = PlateParameters(boolean_epsilon=self.boolean_epsilon)
        self.plate.configure(params)
        self.breadboard_mount.configure(params)
        self.motor_driver_mount.configure(params)
        self.display_mount.configure(params)
        self.upper_mount_pattern.configure(params)
        self.chassis_mount_pattern.configure(params)
        return params


@dataclass
class DifferentialMechanismPlatform:
    """Simple upper platform whose mating pegs derive from an upper mount pattern."""

    mount_pattern: UpperMountPattern
    mount_plane_z: float = 8.0
    platform_thickness: float = 4.0
    standoff_height: float = 8.0
    insertion_depth: float = 4.0
    peg_clearance: float = 0.25
    edge_margin: float = 8.0

    def build(self) -> Brep:
        x0, x1 = sorted(self.mount_pattern.x_positions)
        length = x1 - x0 + 2.0 * self.edge_margin
        width = 2.0 * self.mount_pattern.y_offset + 2.0 * self.edge_margin
        center_x = 0.5 * (x0 + x1)
        platform = _box(
            length, width, self.platform_thickness,
            (center_x, 0.0, self.mount_plane_z + self.standoff_height
             + 0.5 * self.platform_thickness),
        )
        peg_diameter = self.mount_pattern.hole_diameter - self.peg_clearance
        if peg_diameter <= 0.0:
            raise ValueError("peg_clearance must be smaller than the mount hole diameter")
        post_height = self.standoff_height + self.insertion_depth
        post_center_z = self.mount_plane_z + 0.5 * (
            self.standoff_height - self.insertion_depth
        )
        posts = [
            BREP_BACKEND.from_cylinder(Cylinder(
                0.5 * peg_diameter,
                post_height,
                frame=Frame((x, y, post_center_z)),
            ))
            for x, y in self.mount_pattern.centers
        ]
        return _union([platform] + posts)


@dataclass
class HardwareItem:
    quantity: int
    specification: str
    purpose: str
    notes: str = ""


@dataclass
class FastenerBOM:
    items: List[HardwareItem]
    assumptions: List[str] = field(default_factory=list)

    @property
    def total_screws(self) -> int:
        return sum(
            item.quantity for item in self.items
            if "screw" in item.specification.lower()
        )

    def to_text(self) -> str:
        lines = ["VURP FASTENER SHOPPING LIST", ""]
        for item in self.items:
            line = "{} x {} — {}".format(
                item.quantity, item.specification, item.purpose
            )
            if item.notes:
                line += " ({})".format(item.notes)
            lines.append(line)
        lines.extend(["", "Total screws: {}".format(self.total_screws)])
        if self.assumptions:
            lines.extend(["", "ASSUMPTIONS"])
            lines.extend("- " + assumption for assumption in self.assumptions)
        return "\n".join(lines)


def _next_fastener_length(required: float) -> int:
    for length in (4, 5, 6, 8, 10, 12, 14, 16, 20, 25, 30):
        if length + 1e-9 >= required:
            return length
    return int(required + 4.0)


def _nominal_metric_size(clearance_hole: float) -> Tuple[str, float]:
    if clearance_hole <= 3.0:
        return "M2.5", 2.0
    if clearance_hole <= 3.8:
        return "M3", 2.4
    if clearance_hole <= 4.8:
        return "M4", 3.2
    return "M5", 4.0


def build_fastener_bom(
    assembly: PlateAssembly,
    platform: Optional[DifferentialMechanismPlatform] = None,
    deck_thickness: float = 1.5,
) -> FastenerBOM:
    """Derive a purchase list from the composed model and explicit assumptions."""
    if deck_thickness <= 0.0:
        raise ValueError("deck_thickness must be positive")

    plate = assembly.plate
    motor = assembly.motor_driver_mount
    chassis = assembly.chassis_mount_pattern

    # FeatherWing: PCB + remaining pocket floor + nut engagement.
    pcb_thickness = 1.6
    pocket_depth = motor.cutout.effective_depth(plate.thickness)
    pocket_floor = plate.thickness - pocket_depth
    motor_required = pcb_thickness + pocket_floor + motor.counterbore_depth
    motor_length = _next_fastener_length(motor_required)

    chassis_size, chassis_nut_thickness = _nominal_metric_size(
        chassis.hole_diameter
    )
    washer_thickness = 0.5
    chassis_required = (
        plate.thickness - chassis.counterbore_depth
        + deck_thickness + washer_thickness
        + chassis_nut_thickness
    )
    chassis_length = _next_fastener_length(chassis_required)

    items = [
        HardwareItem(
            4,
            "M2.5 x {} mm flat-head machine screws".format(motor_length),
            "Motor FeatherWing",
            "Flat heads leave more clearance around Feather components.",
        ),
        HardwareItem(
            4,
            "M2.5 hex nuts",
            "Motor FeatherWing underside counterbores",
            "5 mm across flats; test-fit the printed counterbores.",
        ),
        HardwareItem(
            4,
            "{} x {} mm button-head machine screws".format(
                chassis_size, chassis_length
            ),
            "Printed plate to Gladiator metal deck",
            "Head must fit a {:.1f} mm diameter x {:.1f} mm deep counterbore."
            .format(chassis.counterbore_diameter, chassis.counterbore_depth),
        ),
        HardwareItem(
            4,
            "{} flat washers".format(chassis_size),
            "Spread load over the Gladiator deck slits",
        ),
        HardwareItem(
            4,
            "{} nyloc nuts".format(chassis_size),
            "Retain the plate below the metal deck",
        ),
    ]
    assumptions = [
        "Gladiator metal deck thickness is {:.1f} mm; adjust the BOM input after measuring it."
        .format(deck_thickness),
        "The display holder is friction-fit and needs no screws.",
        "The differential platform uses four integral pegs derived from UpperMountPattern and needs no screws."
        if platform is not None else
        "No upper-platform fasteners are included because no platform definition was supplied.",
        "Buy one or two spare screws and nuts of each size for assembly losses.",
    ]
    return FastenerBOM(items, assumptions)


def _box(x: float, y: float, z: float, center: Tuple[float, float, float]) -> Brep:
    return BREP_BACKEND.from_box(Box(x, y, z, frame=Frame(center)))


def _oriented_box(
    x: float,
    y: float,
    z: float,
    frame: Frame,
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Brep:
    """Create a box aligned to a frame and offset in its local coordinates."""
    dx, dy, dz = offset
    point = (
        frame.point.x + dx * frame.xaxis.x + dy * frame.yaxis.x + dz * frame.zaxis.x,
        frame.point.y + dx * frame.xaxis.y + dy * frame.yaxis.y + dz * frame.zaxis.y,
        frame.point.z + dx * frame.xaxis.z + dy * frame.yaxis.z + dz * frame.zaxis.z,
    )
    return BREP_BACKEND.from_box(Box(x, y, z, frame=Frame(point, frame.xaxis, frame.yaxis)))


def _union(parts: Iterable[Brep]) -> Brep:
    """Boolean-kernel boundary: replace this function to swap backends."""
    parts = list(parts)
    if not parts:
        raise ValueError("Cannot union an empty list")
    result = parts[0]
    for part in parts[1:]:
        result = _single_result(BREP_BACKEND.from_boolean_union(result, part), "union")
    return result


def _difference(body: Brep, cutters: Iterable[Brep]) -> Brep:
    """Boolean-kernel boundary: replace this function to swap backends."""
    result = body
    for cutter in cutters:
        result = _single_result(BREP_BACKEND.from_boolean_difference(result, cutter), "difference")
    return result


def _intersection(a: Brep, b: Brep) -> Brep:
    """Boolean-kernel boundary for a single-solid intersection."""
    return _single_result(BREP_BACKEND.from_boolean_intersection(a, b), "intersection")


def _single_result(value, operation: str) -> Brep:
    """Normalize backends that return ``[Brep]`` for boolean operations."""
    if isinstance(value, (list, tuple)):
        if len(value) != 1:
            raise RuntimeError("Boolean {} produced {} solids; expected one".format(operation, len(value)))
        return value[0]
    return value


def make_base_plate(params: PlateParameters) -> Brep:
    return _box(
        params.plate_length,
        params.plate_width,
        params.plate_thickness,
        (0.0, 0.0, 0.5 * params.plate_thickness),
    )


def add_front_rear_walls(body: Brep, params: PlateParameters) -> Brep:
    # plate_length remains the measured metal-deck length. Walls sit OUTSIDE
    # that footprint, with only epsilon penetrating inward for a robust union.
    e = params.boolean_epsilon
    front = _box(
        params.wall_thickness + e,
        params.plate_width,
        params.front_wall_height + params.plate_thickness,
        (0.5 * params.plate_length + 0.5 * (params.wall_thickness - e), 0.0,
         0.5 * (params.plate_thickness - params.front_wall_height)),
    )
    rear = _box(
        params.wall_thickness + e,
        params.plate_width,
        params.rear_wall_height + params.plate_thickness,
        (-0.5 * params.plate_length - 0.5 * (params.wall_thickness - e), 0.0,
         0.5 * (params.plate_thickness - params.rear_wall_height)),
    )
    return _union([body, front, rear])


def fillet_front_rear_edges(body: Brep, params: PlateParameters) -> Brep:
    """Round edges on the two outward wall faces; leave openings untouched."""
    radius = params.fillet_radius
    if radius <= 0:
        return body
    front_x = 0.5 * params.plate_length + params.wall_thickness
    rear_x = -front_x
    tolerance = max(0.2, 2.0 * params.boolean_epsilon)
    edges = list(body.edges)
    selected = []
    for edge in edges:
        points = [vertex.point for vertex in edge.vertices]
        if points and all(
            abs(point.x - front_x) <= tolerance or abs(point.x - rear_x) <= tolerance
            for point in points
        ):
            selected.append(edge)
    if not selected:
        return body
    selected_ids = {id(edge) for edge in selected}
    excluded = [edge for edge in edges if id(edge) not in selected_ids]
    return body.filleted(radius, edges=excluded)


def cut_breadboard_opening(body: Brep, params: PlateParameters) -> Brep:
    e = params.boolean_epsilon
    cutter = _box(
        params.breadboard_length + params.breadboard_clearance,
        params.breadboard_width + params.breadboard_clearance,
        params.plate_thickness + 2.0 * e,
        (params.breadboard_offset_x, params.breadboard_offset_y,
         0.5 * params.plate_thickness),
    )
    return _difference(body, [cutter])


def _motor_driver_recess_depth(params: PlateParameters) -> float:
    if params.motor_driver_recess_depth is None:
        return 0.5 * params.plate_thickness
    return params.motor_driver_recess_depth


def _rounded_rectangle_curve(
    center_x: float,
    size_x: float,
    size_y: float,
    radius: float,
    z: float,
    segments: int = 8,
):
    """Return a closed, CAD-neutral polygonal approximation of a rounded rectangle."""
    radius = min(radius, 0.5 * size_x, 0.5 * size_y)
    corner_x = 0.5 * size_x - radius
    corner_y = 0.5 * size_y - radius
    points: List[Point] = []
    for cx, cy, start in (
        (corner_x, corner_y, 0.0),
        (-corner_x, corner_y, 0.5 * pi),
        (-corner_x, -corner_y, pi),
        (corner_x, -corner_y, 1.5 * pi),
    ):
        for index in range(segments + 1):
            angle = start + index * 0.5 * pi / segments
            points.append(Point(center_x + cx + radius * cos(angle),
                                cy + radius * sin(angle), z))
    points.append(points[0])
    return NurbsCurve.from_points(points, degree=1)


def _drafted_rounded_box(
    center_x: float,
    size_x: float,
    size_y: float,
    radius: float,
    bottom_z: float,
    top_z: float,
    draft_deg: float,
) -> Brep:
    """Create a capped rounded-rectangle frustum, wider at its open top."""
    expansion = (top_z - bottom_z) * tan(radians(draft_deg))
    bottom = _rounded_rectangle_curve(center_x, size_x, size_y, radius, bottom_z)
    top = _rounded_rectangle_curve(
        center_x,
        size_x + 2.0 * expansion,
        size_y + 2.0 * expansion,
        radius + expansion,
        top_z,
    )
    result = BREP_BACKEND.from_loft([bottom, top])
    result.cap_planar_holes()
    # Loft orientation depends on curve winding; enforce outward solid normals
    # so boolean difference keeps the plate rather than the cutter volume.
    if str(result.orientation).lower().endswith("inward"):
        result.flip()
    return result


def cut_motor_driver_slot(body: Brep, params: PlateParameters) -> Brep:
    """Cut a top-opening FeatherWing pocket plus its four mounting holes."""
    e = params.boolean_epsilon
    size_x = params.motor_driver_width + params.motor_driver_clearance
    size_y = params.motor_driver_length + params.motor_driver_clearance
    radius = min(
        params.fillet_radius,
        0.5 * size_x,
        0.5 * size_y,
    )
    depth = _motor_driver_recess_depth(params)
    pocket = _drafted_rounded_box(
        params.motor_driver_offset_x,
        size_x,
        size_y,
        radius,
        params.plate_thickness - depth,
        params.plate_thickness + e,
        params.motor_driver_draft_deg,
    )
    body = _difference(body, [pocket])

    hole_cutters: List[Brep] = []
    for dx in (-0.5 * params.motor_driver_mount_hole_spacing_x,
               0.5 * params.motor_driver_mount_hole_spacing_x):
        for dy in (-0.5 * params.motor_driver_mount_hole_spacing_y,
                   0.5 * params.motor_driver_mount_hole_spacing_y):
            hole_cutters.append(BREP_BACKEND.from_cylinder(Cylinder(
                0.5 * params.motor_driver_mount_hole_diameter,
                params.plate_thickness + 2.0 * e,
                frame=Frame((params.motor_driver_offset_x + dx, dy,
                             0.5 * params.plate_thickness)),
            )))
    body = _difference(body, hole_cutters)

    # Recess M2.5 nuts or bolt heads from the underside, keeping them flush.
    counterbores: List[Brep] = []
    counterbore_z = 0.5 * params.motor_driver_counterbore_depth - e
    for dx in (-0.5 * params.motor_driver_mount_hole_spacing_x,
               0.5 * params.motor_driver_mount_hole_spacing_x):
        for dy in (-0.5 * params.motor_driver_mount_hole_spacing_y,
                   0.5 * params.motor_driver_mount_hole_spacing_y):
            counterbores.append(BREP_BACKEND.from_cylinder(Cylinder(
                0.5 * params.motor_driver_counterbore_diameter,
                params.motor_driver_counterbore_depth + 2.0 * e,
                frame=Frame((params.motor_driver_offset_x + dx, dy, counterbore_z)),
            )))
    body = _difference(body, counterbores)

    # Two short-end cable drops and shallow lead-in channels.
    cable_y = 0.5 * size_y + 5.0
    cable_cutters: List[Brep] = []
    for y in (-cable_y, cable_y):
        cable_cutters.append(BREP_BACKEND.from_cylinder(Cylinder(
            0.5 * params.motor_driver_cable_hole_diameter,
            params.plate_thickness + 2.0 * e,
            frame=Frame((params.motor_driver_offset_x, y,
                         0.5 * params.plate_thickness)),
        )))
        channel_length = cable_y - 0.5 * size_y
        channel_center_y = (0.5 * size_y + 0.5 * channel_length) * (1.0 if y > 0 else -1.0)
        cable_cutters.append(_box(
            params.motor_driver_cable_channel_width,
            channel_length + 2.0 * e,
            params.motor_driver_cable_channel_depth + e,
            (params.motor_driver_offset_x, channel_center_y,
             params.plate_thickness - 0.5 * params.motor_driver_cable_channel_depth + 0.5 * e),
        ))
    return _difference(body, cable_cutters)


def cut_top_perimeter_reveal(body: Brep, params: PlateParameters) -> Brep:
    """Cut a restrained shallow groove around the plate's top perimeter."""
    e = params.boolean_epsilon
    width = params.top_reveal_width
    depth = params.top_reveal_depth
    z = params.plate_thickness - 0.5 * depth + 0.5 * e
    half_x = 0.5 * params.plate_length
    half_y = 0.5 * params.plate_width
    cutters = [
        _box(params.plate_length, width, depth + e, (0.0, half_y - 0.5 * width, z)),
        _box(params.plate_length, width, depth + e, (0.0, -half_y + 0.5 * width, z)),
        _box(width, params.plate_width, depth + e, (half_x - 0.5 * width, 0.0, z)),
        _box(width, params.plate_width, depth + e, (-half_x + 0.5 * width, 0.0, z)),
    ]
    return _difference(body, cutters)


def cut_upper_mount_holes(body: Brep, params: PlateParameters) -> Brep:
    e = params.boolean_epsilon
    cutters: List[Brep] = []
    for x in params.upper_hole_x_positions:
        for y in (-params.upper_hole_y_offset, params.upper_hole_y_offset):
            cylinder = Cylinder(
                0.5 * params.upper_hole_diameter,
                params.plate_thickness + 2.0 * e,
                frame=Frame((x, y, 0.5 * params.plate_thickness)),
            )
            cutters.append(BREP_BACKEND.from_cylinder(cylinder))
    return _difference(body, cutters)


def cut_chassis_mount_holes(body: Brep, params: PlateParameters) -> Brep:
    """Cut four deck-slit holes with top counterbores for flush screw heads."""
    e = params.boolean_epsilon
    cutters: List[Brep] = []
    centers = [
        (params.chassis_rear_hole_x, -params.chassis_rear_hole_y_offset),
        (params.chassis_rear_hole_x, params.chassis_rear_hole_y_offset),
        (params.chassis_front_hole_x, -params.chassis_front_hole_y_offset),
        (params.chassis_front_hole_x, params.chassis_front_hole_y_offset),
    ]
    for x, y in centers:
        cylinder = Cylinder(
            0.5 * params.chassis_hole_diameter,
            params.plate_thickness + 2.0 * e,
            frame=Frame((x, y, 0.5 * params.plate_thickness)),
        )
        cutters.append(BREP_BACKEND.from_cylinder(cylinder))
    body = _difference(body, cutters)
    counterbore_height = params.chassis_counterbore_depth + e
    counterbore_z = (
        params.plate_thickness
        - 0.5 * params.chassis_counterbore_depth
        + 0.5 * e
    )
    counterbores = [
        BREP_BACKEND.from_cylinder(Cylinder(
            0.5 * params.chassis_counterbore_diameter,
            counterbore_height,
            frame=Frame((x, y, counterbore_z)),
        ))
        for x, y in centers
    ]
    return _difference(body, counterbores)


def _display_frame(
    params: PlateParameters,
    normal_offset: float = 0.0,
    seat_height: float = 0.0,
) -> Frame:
    """Frame whose XY plane is the inclined display plane.

    Local X spans global Y. Local Y rises toward the chassis rear. Local Z is
    the outward/front-facing surface normal.
    """
    angle = radians(params.display_tilt_deg)
    xaxis = Vector(0.0, 1.0, 0.0)
    yaxis = Vector(-cos(angle), 0.0, sin(angle))
    normal = Vector(sin(angle), 0.0, cos(angle))
    support_h = params.display_height + params.display_clearance + 2.0 * params.display_support_margin
    # Anchor the lower/outward corner. The holder builder trims everything
    # below this seating plane, removing the opposite pointed corner.
    # Align the slab's actual lower/outward vertex directly with the tongue's
    # outer top edge. Do not compensate for the trimmed footprint projection.
    lower_x = (
        _display_joint_center_x(params)
        + 0.5 * params.display_joint_depth_x
    )
    lower_z = params.plate_thickness + seat_height
    center = (
        lower_x - 0.5 * support_h * cos(angle)
        - 0.5 * params.display_support_thickness * normal.x
        + normal_offset * normal.x,
        params.display_offset_y,
        lower_z + 0.5 * support_h * sin(angle)
        - 0.5 * params.display_support_thickness * normal.z
        + normal_offset * normal.z,
    )
    return Frame(center, xaxis, yaxis)


def _display_joint_center_x(params: PlateParameters) -> float:
    return 0.5 * params.plate_length - params.display_front_overlap


def cut_display_holder_socket(body: Brep, params: PlateParameters) -> Brep:
    """Cut the plate socket receiving the separate holder's tongue."""
    e = params.boolean_epsilon
    socket = _box(
        params.display_joint_depth_x + params.display_joint_clearance,
        params.display_joint_width + params.display_joint_clearance,
        params.display_joint_insertion_depth + 2.0 * e,
        (
            _display_joint_center_x(params),
            params.display_offset_y,
            params.plate_thickness - 0.5 * params.display_joint_insertion_depth,
        ),
    )
    return _difference(body, [socket])


def make_display_holder(params: PlateParameters) -> Brep:
    """Make the separately printable inclined cradle with an insertion tongue."""
    e = params.boolean_epsilon
    support_w = params.display_width + params.display_clearance + 2.0 * params.display_support_margin
    support_h = params.display_height + params.display_clearance + 2.0 * params.display_support_margin
    # The tongue stops at the plate top, exactly matching the socket height.
    # The slab and tongue meet on the same plane without protruding into the
    # plate outside the socket.
    shoulder_height = 0.0
    support_frame = _display_frame(params, seat_height=shoulder_height)
    support = BREP_BACKEND.from_box(Box(
        support_w,
        support_h,
        params.display_support_thickness,
        frame=support_frame,
    ))
    # With the outward corner seated on the tongue, the opposite corner of a
    # tilted rectangular slab would point down through it. Trim that triangular
    # prism at the tongue-top plane so the holder has a flat, supported foot.
    tongue_top_z = params.plate_thickness + shoulder_height
    clip_height = 4.0 * (
        params.plate_length + params.plate_width + params.display_height
    )
    clip = _box(
        clip_height,
        clip_height,
        clip_height,
        (0.0, 0.0, tongue_top_z + 0.5 * clip_height),
    )
    support = _intersection(support, clip)
    if params.fillet_radius > 0:
        # Fillet after trimming so the seating vertex cannot be pulled back.
        # Round exposed-face top/side borders, but keep the bottom joint edge
        # sharp and dimensionally aligned with the tongue.
        edges = list(support.edges)
        exposed = []
        target_local_z = 0.5 * params.display_support_thickness
        for edge in edges:
            world_points = [vertex.point for vertex in edge.vertices]
            local_points = [
                support_frame.to_local_coordinates(point)
                for point in world_points
            ]
            on_exposed_face = local_points and all(
                abs(point.z - target_local_z) <= 0.2
                for point in local_points
            )
            on_seating_plane = world_points and all(
                abs(point.z - tongue_top_z) <= 0.2
                for point in world_points
            )
            if on_exposed_face and not on_seating_plane:
                exposed.append(edge)
        exposed_ids = {id(edge) for edge in exposed}
        excluded = [edge for edge in edges if id(edge) not in exposed_ids]
        support = support.filleted(params.fillet_radius, edges=excluded)

    # Hollow the slab from behind, leaving only the integral front bezel and
    # a narrow perimeter wall around the display body.
    cavity_w = params.display_width + params.display_clearance
    cavity_h = params.display_height + params.display_clearance
    cavity_depth = (
        params.display_support_thickness - params.bezel_thickness + 2.0 * e
    )
    cavity = _oriented_box(
        cavity_w,
        cavity_h,
        cavity_depth,
        support_frame,
        offset=(0.0, 0.0, -0.5 * params.bezel_thickness),
    )
    aperture = _oriented_box(
        params.display_active_width + params.display_clearance,
        params.display_active_height + params.display_clearance,
        params.bezel_thickness + 2.0 * e,
        support_frame,
        offset=(
            0.0,
            params.display_window_offset_y,
            0.5 * params.display_support_thickness
            - 0.5 * params.bezel_thickness,
        ),
    )
    support = _difference(support, [cavity, aperture])

    # Four small rear tabs overlap the PCB/body envelope. They replace the
    # former solid backing and retain a display inserted from behind.
    tab = params.display_retainer_tab_size
    overlap = params.display_retainer_overlap
    tab_t = params.display_retainer_thickness
    margin = params.display_support_margin
    tab_z = (
        0.5 * params.display_support_thickness
        - params.bezel_thickness
        - params.display_depth
        - params.display_clearance
        - 0.5 * tab_t
    )
    tabs = []
    side_x = 0.5 * cavity_w + 0.5 * margin - 0.5 * overlap
    side_w = margin + overlap
    for sign in (-1.0, 1.0):
        tabs.append(_oriented_box(
            side_w, tab, tab_t, support_frame,
            offset=(sign * side_x, 0.0, tab_z),
        ))
    end_y = 0.5 * cavity_h + 0.5 * margin - 0.5 * overlap
    end_h = margin + overlap
    # One upper tab only; the display remains open at the lower edge for easy
    # rear insertion. The left and right tabs above provide the other retainers.
    tabs.append(_oriented_box(
        tab, end_h, tab_t, support_frame,
        offset=(0.0, end_y, tab_z),
    ))
    # The tongue rises above the seating plane to overlap the inclined slab;
    # only the lower insertion portion enters the cleared plate socket.
    tongue = _box(
        params.display_joint_depth_x,
        params.display_joint_width,
        params.display_joint_insertion_depth,
        (
            _display_joint_center_x(params),
            params.display_offset_y,
            params.plate_thickness - 0.5 * params.display_joint_insertion_depth,
        ),
    )
    return _union([support, tongue] + tabs)


def add_display_cradle(body: Brep, params: PlateParameters) -> Brep:
    """Compatibility wrapper: attach the holder to a body as one solid."""
    return _union([body, make_display_holder(params)])


def make_display_bezel(params: PlateParameters) -> Brep:
    """Make a separate assembly-positioned frame; active screen stays exposed."""
    outer_w = params.display_width + params.display_clearance + 2.0 * params.bezel_border
    outer_h = params.display_height + params.display_clearance + 2.0 * params.bezel_border
    # If the display is deeper than its recess, position the bezel over the
    # protruding body instead of intersecting it in the assembly preview.
    display_protrusion = max(0.0, params.display_depth - params.display_recess_depth)
    normal_offset = (
        0.5 * params.display_support_thickness
        + display_protrusion
        + 0.5 * params.bezel_thickness
    )
    outer = BREP_BACKEND.from_box(Box(
        outer_w, outer_h, params.bezel_thickness,
        frame=_display_frame(
            params,
            normal_offset,
            seat_height=min(2.0, 0.5 * params.display_support_thickness) - params.boolean_epsilon,
        ),
    ))
    # The aperture follows the measured active screen area. The former max()
    # fallback could silently enlarge it based on the PCB/body dimensions.
    opening_w = params.display_active_width + params.display_clearance
    opening_h = params.display_active_height + params.display_clearance
    opening = BREP_BACKEND.from_box(Box(
        opening_w, opening_h, params.bezel_thickness + 2.0 * params.boolean_epsilon,
        frame=_display_frame(
            params,
            normal_offset,
            seat_height=(
                min(2.0, 0.5 * params.display_support_thickness)
                - params.boolean_epsilon
                + params.display_window_offset_y
            ),
        ),
    ))
    return _difference(outer, [opening])


def build_plate_assembly(assembly: PlateAssembly) -> PlateResult:
    """Build a lower plate from independently authored typed feature objects."""
    params = assembly.to_parameters()
    params.validate()
    body = make_base_plate(params)
    body = add_front_rear_walls(body, params)
    body = fillet_front_rear_edges(body, params)
    body = assembly.breadboard_mount.apply(body, params)
    body = assembly.motor_driver_mount.apply(body, params)
    body = assembly.upper_mount_pattern.apply(body, params)
    body = assembly.chassis_mount_pattern.apply(body, params)
    body = assembly.display_mount.apply(body, params)
    body = cut_top_perimeter_reveal(body, params)
    holder = assembly.display_mount.make_printable_part(params)
    return PlateResult(
        body=body,
        display_holder=holder,
        parameters=params,
        features={
            "breadboard_opening": (
                params.breadboard_length + params.breadboard_clearance,
                params.breadboard_width + params.breadboard_clearance,
            ),
            "motor_driver_slot": {
                "center": (params.motor_driver_offset_x, 0.0),
                "size": (
                    params.motor_driver_width + params.motor_driver_clearance,
                    params.motor_driver_length + params.motor_driver_clearance,
                ),
                "corner_radius": params.fillet_radius,
                "recess_depth": _motor_driver_recess_depth(params),
                "draft_deg": params.motor_driver_draft_deg,
                "mount_hole_diameter": params.motor_driver_mount_hole_diameter,
                "mount_hole_centers": [
                    (params.motor_driver_offset_x + dx, dy)
                    for dx in (-0.5 * params.motor_driver_mount_hole_spacing_x,
                               0.5 * params.motor_driver_mount_hole_spacing_x)
                    for dy in (-0.5 * params.motor_driver_mount_hole_spacing_y,
                               0.5 * params.motor_driver_mount_hole_spacing_y)
                ],
                "counterbore": (
                    params.motor_driver_counterbore_diameter,
                    params.motor_driver_counterbore_depth,
                ),
                "cable_hole_centers": [
                    (params.motor_driver_offset_x, y)
                    for y in (
                        -0.5 * (params.motor_driver_length + params.motor_driver_clearance) - 5.0,
                        0.5 * (params.motor_driver_length + params.motor_driver_clearance) + 5.0,
                    )
                ],
            },
            "top_reveal": (params.top_reveal_width, params.top_reveal_depth),
            "upper_hole_centers": [
                (x, y) for x in params.upper_hole_x_positions
                for y in (-params.upper_hole_y_offset, params.upper_hole_y_offset)
            ],
            "chassis_hole_centers": [
                (params.chassis_rear_hole_x, -params.chassis_rear_hole_y_offset),
                (params.chassis_rear_hole_x, params.chassis_rear_hole_y_offset),
                (params.chassis_front_hole_x, -params.chassis_front_hole_y_offset),
                (params.chassis_front_hole_x, params.chassis_front_hole_y_offset),
            ],
            "chassis_counterbore": (
                params.chassis_counterbore_diameter,
                params.chassis_counterbore_depth,
            ),
        },
    )


def build_vurp_lower_plate(params: PlateParameters = None) -> PlateResult:
    """Compatibility wrapper for callers using the original flat parameter set."""
    params = params or PlateParameters()
    return build_plate_assembly(PlateAssembly.from_parameters(params))


def brep_to_stl_mesh(brep: Brep, name: str = "VURP part") -> Mesh:
    """Tessellate a backend Brep and return one triangular COMPAS mesh."""
    patches = brep.to_meshes()
    if not patches:
        raise RuntimeError("Brep tessellation produced no meshes")
    mesh = patches[0].copy()
    for patch in patches[1:]:
        mesh.join(patch, weld=False)
    mesh.quads_to_triangles()
    if not mesh.is_trimesh():
        raise RuntimeError("Brep tessellation contains non-triangular faces")
    mesh.name = name
    return mesh


def export_stls(result: PlateResult, output_directory: str, prefix: str = "vurp") -> Dict[str, str]:
    """Write the plate and integral-bezel holder as binary STLs using COMPAS."""
    import os

    os.makedirs(output_directory, exist_ok=True)
    paths = {
        "body": os.path.join(output_directory, "{}_lower_plate.stl".format(prefix)),
        "display_holder": os.path.join(output_directory, "{}_display_holder.stl".format(prefix)),
    }
    STL(paths["body"]).write(
        brep_to_stl_mesh(result.body, "VURP lower plate"), binary=True
    )
    STL(paths["display_holder"]).write(
        brep_to_stl_mesh(result.display_holder, "VURP display holder"), binary=True
    )
    return paths


if __name__ == "__main__":
    # CAD-agnostic smoke path. Display/export is intentionally delegated to an
    # adapter such as rhino_preview.py.
    result = build_vurp_lower_plate()
    print("Built VURP lower plate and rear-loading display holder with parameters:")
    print(result.parameters)
