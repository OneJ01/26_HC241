#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure aiming mathematics for the MMR engagement coordinator.

This module contains no ROS or hardware code.  It converts the solver's
body-relative yaw/elevation into roll/pitch-corrected commands.

Project conventions
-------------------
- ``target_yaw_deg`` is the relative body-heading error produced by the
  ballistic solver. Positive means the target is to the right, negative to
  the left.
- The returned ``corrected_yaw_deg`` is NOT a pan-motor command. In the
  tilt-only architecture it is the remaining body-rotation command.
- ``corrected_elevation_deg`` is the tilt command passed to the turret
  actuator only after the coordinator locks the final solution.
- The rotation equations intentionally preserve the convention already used
  and field-tested in the former turret_control_node implementation.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Any


@dataclass(frozen=True)
class AimCorrectionResult:
    """Complete result used by the future engagement coordinator."""

    valid: bool
    reason: str

    target_yaw_deg: float
    target_elevation_deg: float
    imu_roll_deg: float
    imu_pitch_deg: float

    corrected_yaw_deg: float
    corrected_elevation_deg: float
    limited_elevation_deg: float
    elevation_residual_deg: float

    correction_enabled: bool
    correction_applied: bool
    imu_safe: bool
    elevation_in_range: bool

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _is_finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalize_signed_angle_deg(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180], preserving positive 180 degrees."""

    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError("angle_deg must be finite")

    wrapped = (angle + 180.0) % 360.0 - 180.0
    if math.isclose(wrapped, -180.0, abs_tol=1e-12) and angle > 0.0:
        return 180.0
    return wrapped


def correct_aim_for_imu(
    target_yaw_deg: float,
    target_elevation_deg: float,
    imu_roll_deg: float,
    imu_pitch_deg: float,
    *,
    correction_enabled: bool = True,
    correction_threshold_deg: float = 0.5,
    roll_sign: float = 1.0,
    pitch_sign: float = 1.0,
) -> tuple[float, float, bool]:
    """Apply coupled 3-D roll/pitch correction.

    The level-frame target direction is transformed into the tilted body
    frame using the same equation previously embedded in turret_control_node::

        d_B = Rx(-roll) * Ry(pitch) * d_L

    Returns
    -------
    corrected_yaw_deg, corrected_elevation_deg, correction_applied

    Raises
    ------
    ValueError
        If an input or configuration value is not finite, or if the threshold
        is negative.
    """

    values = {
        "target_yaw_deg": target_yaw_deg,
        "target_elevation_deg": target_elevation_deg,
        "imu_roll_deg": imu_roll_deg,
        "imu_pitch_deg": imu_pitch_deg,
        "correction_threshold_deg": correction_threshold_deg,
        "roll_sign": roll_sign,
        "pitch_sign": pitch_sign,
    }
    invalid = [name for name, value in values.items() if not _is_finite_number(value)]
    if invalid:
        raise ValueError(f"non-finite value(s): {', '.join(invalid)}")

    threshold = float(correction_threshold_deg)
    if threshold < 0.0:
        raise ValueError("correction_threshold_deg must be >= 0")

    target_yaw = normalize_signed_angle_deg(float(target_yaw_deg))
    target_elevation = float(target_elevation_deg)
    roll_deg = float(imu_roll_deg)
    pitch_deg = float(imu_pitch_deg)

    if (
        not bool(correction_enabled)
        or (abs(roll_deg) < threshold and abs(pitch_deg) < threshold)
    ):
        return target_yaw, target_elevation, False

    az = math.radians(target_yaw)
    el = math.radians(target_elevation)
    roll = math.radians(roll_deg * float(roll_sign))
    pitch = math.radians(pitch_deg * float(pitch_sign))

    # Target direction in the level frame.
    dx = math.cos(el) * math.cos(az)
    dy = math.cos(el) * math.sin(az)
    dz = math.sin(el)

    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    # d_B = Rx(-roll) * Ry(pitch) * d_L
    px = cp * dx + sp * dz
    py = dy
    pz = -sp * dx + cp * dz

    bx = px
    by = cr * py + sr * pz
    bz = -sr * py + cr * pz

    corrected_yaw = normalize_signed_angle_deg(math.degrees(math.atan2(by, bx)))
    horizontal = math.hypot(bx, by)
    corrected_elevation = math.degrees(math.atan2(bz, horizontal))

    if not (math.isfinite(corrected_yaw) and math.isfinite(corrected_elevation)):
        raise ValueError("IMU correction produced a non-finite result")

    return corrected_yaw, corrected_elevation, True


def limit_angle_with_residual(
    value_deg: float,
    min_deg: float,
    max_deg: float,
    *,
    tolerance_deg: float = 0.0,
) -> tuple[float, bool, float]:
    """Limit an angle and report whether it exceeded the tolerated range.

    Returns ``limited_deg, exceeded, residual_deg`` where residual is
    ``raw - limited``. The function is mainly used for diagnostics; the
    coordinator must not silently aim at a clamped elevation.
    """

    for name, value in {
        "value_deg": value_deg,
        "min_deg": min_deg,
        "max_deg": max_deg,
        "tolerance_deg": tolerance_deg,
    }.items():
        if not _is_finite_number(value):
            raise ValueError(f"{name} must be finite")

    raw = float(value_deg)
    lower = float(min_deg)
    upper = float(max_deg)
    tolerance = float(tolerance_deg)

    if lower > upper:
        raise ValueError("min_deg must be <= max_deg")
    if tolerance < 0.0:
        raise ValueError("tolerance_deg must be >= 0")

    limited = max(lower, min(upper, raw))
    residual = raw - limited
    exceeded = raw < (lower - tolerance) or raw > (upper + tolerance)
    return limited, exceeded, residual


def evaluate_corrected_aim(
    target_yaw_deg: float,
    target_elevation_deg: float,
    imu_roll_deg: float,
    imu_pitch_deg: float,
    *,
    correction_enabled: bool = True,
    correction_threshold_deg: float = 0.5,
    roll_sign: float = 1.0,
    pitch_sign: float = 1.0,
    imu_roll_limit_deg: float = 15.0,
    imu_pitch_limit_deg: float = 15.0,
    elevation_min_deg: float = 45.0,
    elevation_max_deg: float = 61.0,
    mechanical_limit_tolerance_deg: float = 0.2,
) -> AimCorrectionResult:
    """Validate inputs, apply IMU correction and evaluate firing feasibility.

    ``valid`` becomes false when:
    - an input/configuration value is invalid,
    - roll or pitch exceeds its safety limit,
    - the corrected elevation exceeds the physical tilt range.

    Yaw is intentionally not clamped because the MMR body can rotate through
    the full signed range.
    """

    raw_inputs = {
        "target_yaw_deg": target_yaw_deg,
        "target_elevation_deg": target_elevation_deg,
        "imu_roll_deg": imu_roll_deg,
        "imu_pitch_deg": imu_pitch_deg,
        "correction_threshold_deg": correction_threshold_deg,
        "roll_sign": roll_sign,
        "pitch_sign": pitch_sign,
        "imu_roll_limit_deg": imu_roll_limit_deg,
        "imu_pitch_limit_deg": imu_pitch_limit_deg,
        "elevation_min_deg": elevation_min_deg,
        "elevation_max_deg": elevation_max_deg,
        "mechanical_limit_tolerance_deg": mechanical_limit_tolerance_deg,
    }
    invalid = [name for name, value in raw_inputs.items() if not _is_finite_number(value)]

    # A result object is returned even for invalid external data so the
    # coordinator can publish a deterministic fault reason instead of raising.
    if invalid:
        nan = float("nan")
        return AimCorrectionResult(
            valid=False,
            reason=f"NON_FINITE_INPUT:{','.join(invalid)}",
            target_yaw_deg=float(target_yaw_deg) if _is_finite_number(target_yaw_deg) else nan,
            target_elevation_deg=(
                float(target_elevation_deg) if _is_finite_number(target_elevation_deg) else nan
            ),
            imu_roll_deg=float(imu_roll_deg) if _is_finite_number(imu_roll_deg) else nan,
            imu_pitch_deg=float(imu_pitch_deg) if _is_finite_number(imu_pitch_deg) else nan,
            corrected_yaw_deg=nan,
            corrected_elevation_deg=nan,
            limited_elevation_deg=nan,
            elevation_residual_deg=nan,
            correction_enabled=bool(correction_enabled),
            correction_applied=False,
            imu_safe=False,
            elevation_in_range=False,
        )

    try:
        roll_limit = float(imu_roll_limit_deg)
        pitch_limit = float(imu_pitch_limit_deg)
        if roll_limit < 0.0 or pitch_limit < 0.0:
            raise ValueError("IMU safety limits must be >= 0")

        corrected_yaw, corrected_elevation, applied = correct_aim_for_imu(
            float(target_yaw_deg),
            float(target_elevation_deg),
            float(imu_roll_deg),
            float(imu_pitch_deg),
            correction_enabled=correction_enabled,
            correction_threshold_deg=float(correction_threshold_deg),
            roll_sign=float(roll_sign),
            pitch_sign=float(pitch_sign),
        )

        limited_elevation, elevation_exceeded, elevation_residual = limit_angle_with_residual(
            corrected_elevation,
            float(elevation_min_deg),
            float(elevation_max_deg),
            tolerance_deg=float(mechanical_limit_tolerance_deg),
        )
    except ValueError as exc:
        nan = float("nan")
        return AimCorrectionResult(
            valid=False,
            reason=f"INVALID_CONFIGURATION:{exc}",
            target_yaw_deg=float(target_yaw_deg),
            target_elevation_deg=float(target_elevation_deg),
            imu_roll_deg=float(imu_roll_deg),
            imu_pitch_deg=float(imu_pitch_deg),
            corrected_yaw_deg=nan,
            corrected_elevation_deg=nan,
            limited_elevation_deg=nan,
            elevation_residual_deg=nan,
            correction_enabled=bool(correction_enabled),
            correction_applied=False,
            imu_safe=False,
            elevation_in_range=False,
        )

    roll_safe = abs(float(imu_roll_deg)) <= roll_limit
    pitch_safe = abs(float(imu_pitch_deg)) <= pitch_limit
    imu_safe = roll_safe and pitch_safe
    elevation_in_range = not elevation_exceeded

    reasons: list[str] = []
    if not roll_safe:
        reasons.append("IMU_ROLL_LIMIT_EXCEEDED")
    if not pitch_safe:
        reasons.append("IMU_PITCH_LIMIT_EXCEEDED")
    if not elevation_in_range:
        reasons.append("CORRECTED_ELEVATION_LIMIT_EXCEEDED")

    return AimCorrectionResult(
        valid=imu_safe and elevation_in_range,
        reason="OK" if not reasons else "+".join(reasons),
        target_yaw_deg=normalize_signed_angle_deg(float(target_yaw_deg)),
        target_elevation_deg=float(target_elevation_deg),
        imu_roll_deg=float(imu_roll_deg),
        imu_pitch_deg=float(imu_pitch_deg),
        corrected_yaw_deg=corrected_yaw,
        corrected_elevation_deg=corrected_elevation,
        limited_elevation_deg=limited_elevation,
        elevation_residual_deg=elevation_residual,
        correction_enabled=bool(correction_enabled),
        correction_applied=applied,
        imu_safe=imu_safe,
        elevation_in_range=elevation_in_range,
    )


def main() -> None:
    """Small ROS-independent smoke test: ``python3 -m fire_solution.aim_math``."""

    sample = evaluate_corrected_aim(
        target_yaw_deg=8.0,
        target_elevation_deg=53.0,
        imu_roll_deg=2.0,
        imu_pitch_deg=1.0,
    )
    print(json.dumps(sample.as_dict(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
