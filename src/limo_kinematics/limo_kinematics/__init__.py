"""LIMO robot kinematics calculation and Nav2 validation package."""

from limo_kinematics.geometry import AckermannGeometry, load_geometry
from limo_kinematics.kinematics import (
    central_to_inner,
    inner_to_central,
    max_curvature,
    min_turning_radius_rear_axle,
    radius_at_offset,
    twist_to_inner_steering,
)
from limo_kinematics.footprint import (
    circumscribed_radius,
    inscribed_radius,
    pad_footprint,
    parse_footprint,
)
from limo_kinematics.report import Finding, check_nav2_yaml, recommend
from limo_kinematics.cli import main

__all__ = [
    "AckermannGeometry",
    "load_geometry",
    "central_to_inner",
    "inner_to_central",
    "max_curvature",
    "min_turning_radius_rear_axle",
    "radius_at_offset",
    "twist_to_inner_steering",
    "circumscribed_radius",
    "inscribed_radius",
    "pad_footprint",
    "parse_footprint",
    "Finding",
    "recommend",
    "check_nav2_yaml",
    "main",
]
