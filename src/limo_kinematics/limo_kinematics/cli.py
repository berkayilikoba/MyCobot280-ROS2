"""Command line interface for LIMO kinematics analysis."""

import argparse
import math
import sys
from typing import Sequence
import xml.etree.ElementTree as ET

from limo_kinematics.footprint import (
    circumscribed_radius,
    inscribed_radius,
    parse_footprint,
)
from limo_kinematics.geometry import load_geometry
from limo_kinematics.kinematics import (
    max_curvature,
    min_turning_radius_rear_axle,
    radius_at_offset,
)
from limo_kinematics.report import Finding, check_nav2_yaml, recommend


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="LIMO kinematics and Nav2 parameter checker"
    )
    parser.add_argument("urdf", help="Path to robot URDF file")
    parser.add_argument(
        "--driver",
        nargs=3,
        type=float,
        metavar=("L", "T", "INNER"),
        default=None,
        help="Driver override parameters: wheelbase, track, max_inner_steer",
    )
    parser.add_argument(
        "--footprint",
        type=str,
        default="[[-0.08,-0.1],[0.22,-0.1],[0.22,0.1],[-0.08,0.1]]",
        help="Robot footprint polygon string or list",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.02,
        help="Footprint padding in meters",
    )
    parser.add_argument(
        "--nav2-yaml",
        type=str,
        default=None,
        help="Optional path to Nav2 parameters YAML to validate",
    )
    return parser.parse_args(argv)


def print_report(
    wheelbase: float,
    track: float,
    wheel_radius: float,
    max_inner_steer: float,
    r_rear: float,
    r_base: float,
    max_curv: float,
    r_inscribed: float,
    r_circumscribed: float,
    rec: dict[str, float],
    findings: list[Finding] | None,
) -> None:
    """Print readable report with exact decimal precisions to stdout."""
    deg = math.degrees(max_inner_steer)
    print("Ackermann Geometry & Kinematics:")
    print(f"  wheelbase: {wheelbase:.3f}")
    print(f"  track: {track:.3f}")
    print(f"  wheel_radius: {wheel_radius:.3f}")
    print(f"  max_inner_steer: {max_inner_steer:.3f} rad ({deg:.3f} deg)")
    print(f"  min turning radius at rear axle: {r_rear:.3f}")
    print(f"  min turning radius at base_link: {r_base:.3f}")
    print(f"  max curvature: {max_curv:.3f}")
    print(f"  inscribed radius: {r_inscribed:.3f}")
    print(f"  circumscribed radius: {r_circumscribed:.3f}")
    print("\nRecommendations:")
    print(f"  minimum_turning_radius: {rec['minimum_turning_radius']:.2f}")
    print(f"  min_turning_r: {rec['min_turning_r']:.2f}")
    print(f"  inflation_radius_min: {rec['inflation_radius_min']:.2f}")

    if findings is not None:
        print("\nNav2 Configuration Check:")
        for f in findings:
            status = "OK" if f.ok else "FAIL"
            act_str = f"{f.actual:.3f}" if f.actual is not None else "None"
            print(f"[{status}] {f.param}: actual={act_str}, recommended={f.recommended:.2f}")


def main(argv: Sequence[str] | None = None) -> int:
    """Run CLI analysis and validation."""
    args = parse_args(argv)

    try:
        geom = load_geometry(args.urdf)
    except (ValueError, OSError, ET.ParseError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if args.driver is not None:
        wheelbase = float(args.driver[0])
        track = float(args.driver[1])
        max_inner_steer = float(args.driver[2])
    else:
        wheelbase = geom.wheelbase
        track = geom.track
        max_inner_steer = geom.max_inner_steer

    wheel_radius = geom.wheel_radius
    rear_axle_x = geom.rear_axle_x

    r_rear = min_turning_radius_rear_axle(wheelbase, track, max_inner_steer)
    r_base = radius_at_offset(r_rear, abs(rear_axle_x))
    max_curv = max_curvature(r_base)

    footprint_points = parse_footprint(args.footprint)
    r_inscribed = inscribed_radius(footprint_points, args.padding)
    r_circumscribed = circumscribed_radius(footprint_points, args.padding)

    rec = recommend(r_base, footprint_points, args.padding)

    findings: list[Finding] | None = None
    if args.nav2_yaml is not None:
        findings = check_nav2_yaml(args.nav2_yaml, rec)

    print_report(
        wheelbase=wheelbase,
        track=track,
        wheel_radius=wheel_radius,
        max_inner_steer=max_inner_steer,
        r_rear=r_rear,
        r_base=r_base,
        max_curv=max_curv,
        r_inscribed=r_inscribed,
        r_circumscribed=r_circumscribed,
        rec=rec,
        findings=findings,
    )

    if findings is not None and any(not f.ok for f in findings):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
