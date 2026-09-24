"""Nav2 parameter recommendation and YAML validation."""

import math
from dataclasses import dataclass
from typing import Sequence
import yaml

from limo_kinematics.footprint import Point, inscribed_radius


@dataclass
class Finding:
    """Report finding for a Nav2 parameter check."""

    param: str
    actual: float | None
    recommended: float
    ok: bool


def recommend(
    r_min_base: float,
    footprint: Sequence[Point],
    padding: float = 0.0,
) -> dict[str, float]:
    """Generate recommended Nav2 parameters based on robot geometry and footprint.

    Minimum turning radii are rounded UP to 0.01 m using a 1e-9 epsilon.
    """
    rounded_r = round(math.ceil((r_min_base - 1e-9) * 100.0) / 100.0, 2)
    inflation_min = inscribed_radius(footprint, padding)
    return {
        "minimum_turning_radius": rounded_r,
        "min_turning_r": rounded_r,
        "inflation_radius_min": inflation_min,
    }


CHECK_SPECS: list[tuple[str, str]] = [
    (
        "planner_server.ros__parameters.GridBased.minimum_turning_radius",
        "minimum_turning_radius",
    ),
    (
        "controller_server.ros__parameters.FollowPath.AckermannConstraints.min_turning_r",
        "min_turning_r",
    ),
    (
        "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius",
        "inflation_radius_min",
    ),
    (
        "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius",
        "inflation_radius_min",
    ),
]


def _lookup_path(data: object, dotted_path: str) -> float | None:
    """Safely traverse a nested dictionary and return float value or None."""
    curr = data
    for key in dotted_path.split("."):
        if not isinstance(curr, dict) or key not in curr:
            return None
        curr = curr[key]
    try:
        val = float(curr)
        if math.isnan(val):
            return None
        return val
    except (TypeError, ValueError):
        return None


def check_nav2_yaml(path: str, rec: dict[str, float]) -> list[Finding]:
    """Validate Nav2 configuration YAML against recommendations."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception:
        data = None

    findings: list[Finding] = []
    for param, rec_key in CHECK_SPECS:
        rec_val = rec[rec_key]
        actual_val = _lookup_path(data, param)
        ok = (actual_val is not None) and (actual_val >= rec_val - 1e-9)
        findings.append(
            Finding(
                param=param,
                actual=actual_val,
                recommended=rec_val,
                ok=ok,
            )
        )
    return findings
