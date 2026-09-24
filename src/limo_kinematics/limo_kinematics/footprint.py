"""Nav2 footprint parsing, padding, and geometric radii calculations."""

import json
import math
from typing import Sequence, Union

Point = tuple[float, float]
FootprintInput = Union[str, Sequence[Sequence[Union[float, int]]]]


def parse_footprint(fp: FootprintInput) -> list[Point]:
    """Parse a Nav2 footprint string '[[x,y],...]' or a sequence of pairs.

    Returns a list of (x, y) float tuples.
    Raises ValueError if fewer than 3 points or malformed.
    """
    if isinstance(fp, str):
        try:
            raw = json.loads(fp)
        except Exception as e:
            raise ValueError(f"Malformed footprint string: {fp}") from e
    elif isinstance(fp, (list, tuple)):
        raw = fp
    else:
        raise ValueError(f"Footprint must be a string or sequence, got {type(fp)}")

    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"Footprint must be a sequence of points, got {type(raw)}")

    points: list[Point] = []
    for item in raw:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            raise ValueError(
                f"Each footprint vertex must be a 2-element pair, got {item}"
            )
        try:
            points.append((float(item[0]), float(item[1])))
        except (TypeError, ValueError) as e:
            raise ValueError(f"Coordinates must be numbers, got {item}") from e

    if len(points) < 3:
        raise ValueError(f"Footprint must have at least 3 points, got {len(points)}")

    return points


def pad_footprint(points: Sequence[Point], padding: float) -> list[Point]:
    """Apply Nav2 padFootprint rule: x += sign(x)*padding, y += sign(y)*padding."""
    def _sign(val: float) -> float:
        if val > 0.0:
            return 1.0
        elif val < 0.0:
            return -1.0
        return 0.0

    return [(x + _sign(x) * padding, y + _sign(y) * padding) for x, y in points]


def inscribed_radius(points: Sequence[Point], padding: float = 0.0) -> float:
    """Calculate inscribed radius as minimum distance from origin to polygon edges."""
    padded = pad_footprint(points, padding)
    n = len(padded)
    min_dist = float("inf")
    for i in range(n):
        x1, y1 = padded[i]
        x2, y2 = padded[(i + 1) % n]
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq == 0.0:
            dist = math.hypot(x1, y1)
        else:
            t = (-x1 * dx - y1 * dy) / seg_len_sq
            t_clamped = max(0.0, min(1.0, t))
            proj_x = x1 + t_clamped * dx
            proj_y = y1 + t_clamped * dy
            dist = math.hypot(proj_x, proj_y)
        if dist < min_dist:
            min_dist = dist
    return min_dist


def circumscribed_radius(points: Sequence[Point], padding: float = 0.0) -> float:
    """Calculate circumscribed radius as max vertex distance from origin."""
    padded = pad_footprint(points, padding)
    return max(math.hypot(x, y) for x, y in padded)
