"""Ackermann steering kinematics calculations matching limo_driver.cpp."""

import math


def min_turning_radius_rear_axle(
    wheelbase: float, track: float, max_inner_steer: float
) -> float:
    """Calculate minimum turning radius at the rear axle center.

    Formula: wheelbase / tan(max_inner_steer) + track / 2.
    """
    if not (
        math.isfinite(wheelbase)
        and math.isfinite(track)
        and math.isfinite(max_inner_steer)
    ):
        raise ValueError("wheelbase, track, and max_inner_steer must be finite")
    if wheelbase <= 0.0:
        raise ValueError(f"wheelbase must be positive, got {wheelbase}")
    if track < 0.0:
        raise ValueError(f"track must be non-negative, got {track}")
    if not (0.0 < max_inner_steer < (math.pi / 2.0)):
        raise ValueError(
            f"max_inner_steer must be in (0, pi/2), got {max_inner_steer}"
        )
    return wheelbase / math.tan(max_inner_steer) + track / 2.0


def radius_at_offset(r_rear: float, offset: float) -> float:
    """Calculate turning radius at a longitudinal offset from rear axle.

    Formula: hypot(r_rear, offset).
    """
    return math.hypot(r_rear, offset)


def max_curvature(r: float) -> float:
    """Calculate maximum path curvature from turning radius.

    Formula: 1 / r.
    """
    if not math.isfinite(r) or r <= 0.0:
        raise ValueError(f"Radius must be positive and finite, got {r}")
    return 1.0 / r


def inner_to_central(inner: float, L: float, T: float) -> float:
    """Convert inner wheel steer angle to central Ackermann steering angle.

    Formulas match limo_driver.cpp.
    """
    if inner == 0.0:
        return 0.0
    abs_inner = abs(inner)
    r = L / math.tan(abs_inner) + T / 2.0
    c = math.atan(L / r)
    return -c if inner < 0.0 else c


def central_to_inner(c: float, L: float, T: float) -> float:
    """Convert central Ackermann steering angle to inner wheel steer angle.

    Formulas match limo_driver.cpp.
    """
    if c == 0.0:
        return 0.0
    abs_c = abs(c)
    num = 2.0 * L * math.sin(abs_c)
    den = 2.0 * L * math.cos(abs_c) - T * math.sin(abs_c)
    inner = math.atan(num / den)
    return -inner if c < 0.0 else inner


def twist_to_inner_steering(v: float, w: float, L: float, T: float,
                            max_inner: float):
    """cmd_vel (v [m/s], w [rad/s] YAW HIZI) -> iç tekerlek direksiyon açısı [rad].

    limo_base C++ kinematics::twistToInnerSteering ile birebir aynı davranış
    (gerçek LIMO sürücüsünün cmd_vel yorumu):
      * w == 0              -> 0.0
      * v == 0 ve w != 0    -> sign(w) * max_inner (yerinde dönüş yok, tam kilit)
      * r = v / w; |r| < T/2 ise r = sign(r) * (T/2 + 0.01);
        iç açı = central_to_inner(atan(L / r)), [-max_inner, max_inner]'a kırpılır.
    Girdilerden biri sonlu değilse None döner.
    """
    if not (math.isfinite(v) and math.isfinite(w)):
        return None
    if w == 0.0:
        return 0.0
    if v == 0.0:
        return math.copysign(max_inner, w)
    r = v / w
    if abs(r) < T / 2.0:
        r = math.copysign(T / 2.0 + 0.01, r)
    inner = central_to_inner(math.atan(L / r), L, T)
    return max(-max_inner, min(max_inner, inner))
