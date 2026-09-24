"""URDF geometry extraction for Ackermann vehicles."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass(frozen=True)
class AckermannGeometry:
    """Robot Ackermann geometry parameters in base_link frame."""

    wheelbase: float
    track: float
    wheel_radius: float
    max_inner_steer: float
    rear_axle_x: float


def _parse_xyz(elem: ET.Element | None) -> tuple[float, float, float]:
    """Parse xyz attribute into float tuple."""
    if elem is None:
        return (0.0, 0.0, 0.0)
    xyz_str = elem.get("xyz")
    if not xyz_str:
        return (0.0, 0.0, 0.0)
    parts = [float(v) for v in xyz_str.split()]
    if len(parts) != 3:
        return (0.0, 0.0, 0.0)
    return (parts[0], parts[1], parts[2])


def _parse_axis(elem: ET.Element | None) -> tuple[float, float, float]:
    """Parse axis element xyz attribute into float tuple."""
    if elem is None:
        return (1.0, 0.0, 0.0)
    return _parse_xyz(elem)


def _is_z_axis(axis: tuple[float, float, float]) -> bool:
    """Check if axis is (0, 0, +-1)."""
    x, y, z = axis
    return abs(x) < 1e-6 and abs(y) < 1e-6 and (abs(abs(z) - 1.0) < 1e-6)


def load_geometry(urdf_path: str) -> AckermannGeometry:
    """Parse URDF file and extract Ackermann vehicle geometry."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = root.findall("joint")
    links = root.findall("link")
    link_map = {l.get("name"): l for l in links if l.get("name")}

    # Map parent link name to child joints
    parent_to_joints: dict[str, list[ET.Element]] = {}
    for j in joints:
        parent = j.find("parent")
        if parent is not None and parent.get("link"):
            parent_to_joints.setdefault(parent.get("link"), []).append(j)

    # Find steering joints:
    # type="revolute", axis is (0,0,+-1), and child link has continuous child joint
    steering_joints: list[ET.Element] = []
    for j in joints:
        if j.get("type") != "revolute":
            continue
        axis = _parse_axis(j.find("axis"))
        if not _is_z_axis(axis):
            continue
        child = j.find("child")
        if child is None or not child.get("link"):
            continue
        child_link = child.get("link")
        child_joints = parent_to_joints.get(child_link, [])
        if any(cj.get("type") == "continuous" for cj in child_joints):
            steering_joints.append(j)

    if not steering_joints:
        raise ValueError("No steering joints found in URDF")

    # Parent link from which steering joints hang (typically base_link)
    steer_parent = steering_joints[0].find("parent").get("link")

    # Rear wheels: continuous joints directly attached to steer_parent
    rear_wheel_joints: list[ET.Element] = []
    for j in parent_to_joints.get(steer_parent, []):
        if j.get("type") == "continuous":
            rear_wheel_joints.append(j)

    if not rear_wheel_joints:
        raise ValueError("No rear wheel joints found in URDF")

    # Front axle x and rear axle x
    front_xs = [_parse_xyz(j.find("origin"))[0] for j in steering_joints]
    front_x = sum(front_xs) / len(front_xs)

    rear_xs = [_parse_xyz(j.find("origin"))[0] for j in rear_wheel_joints]
    rear_axle_x = sum(rear_xs) / len(rear_xs)

    wheelbase = front_x - rear_axle_x

    rear_ys = [_parse_xyz(j.find("origin"))[1] for j in rear_wheel_joints]
    track = max(rear_ys) - min(rear_ys)

    # Wheel links: child links of rear wheels and front wheels
    wheel_link_names: list[str] = [
        j.find("child").get("link")
        for j in rear_wheel_joints
        if j.find("child") is not None and j.find("child").get("link")
    ]
    for sj in steering_joints:
        hinge_link = sj.find("child").get("link")
        for cj in parent_to_joints.get(hinge_link, []):
            if cj.get("type") == "continuous" and cj.find("child") is not None:
                wheel_link_names.append(cj.find("child").get("link"))

    # wheel_radius: prefer collision cylinder, fall back to visual
    wheel_radius: float | None = None
    for name in wheel_link_names:
        link = link_map.get(name)
        if link is None:
            continue
        for col in link.findall("collision"):
            cyl = col.find("geometry/cylinder")
            if cyl is not None and cyl.get("radius") is not None:
                wheel_radius = float(cyl.get("radius"))
                break
        if wheel_radius is not None:
            break

    if wheel_radius is None:
        for name in wheel_link_names:
            link = link_map.get(name)
            if link is None:
                continue
            for vis in link.findall("visual"):
                cyl = vis.find("geometry/cylinder")
                if cyl is not None and cyl.get("radius") is not None:
                    wheel_radius = float(cyl.get("radius"))
                    break
            if wheel_radius is not None:
                break

    if wheel_radius is None:
        raise ValueError("Could not determine wheel radius from URDF wheel links")

    # max_inner_steer: min over steering joints of min(upper, -lower) from <limit>
    steer_limits: list[float] = []
    for sj in steering_joints:
        limit = sj.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(
                f"Steering joint {sj.get('name')} missing valid limit element"
            )
        lower = float(limit.get("lower"))
        upper = float(limit.get("upper"))
        steer_limits.append(min(upper, -lower))

    max_inner_steer = min(steer_limits)

    return AckermannGeometry(
        wheelbase=wheelbase,
        track=track,
        wheel_radius=wheel_radius,
        max_inner_steer=max_inner_steer,
        rear_axle_x=rear_axle_x,
    )
