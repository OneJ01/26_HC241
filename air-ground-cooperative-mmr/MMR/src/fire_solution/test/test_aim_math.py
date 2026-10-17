import math

import pytest

from fire_solution.aim_math import (
    correct_aim_for_imu,
    evaluate_corrected_aim,
    limit_angle_with_residual,
    normalize_signed_angle_deg,
)


def test_level_body_returns_solver_angles_without_correction():
    yaw, elevation, applied = correct_aim_for_imu(12.5, 53.0, 0.0, 0.0)
    assert yaw == pytest.approx(12.5)
    assert elevation == pytest.approx(53.0)
    assert applied is False


def test_below_threshold_does_not_apply_correction():
    yaw, elevation, applied = correct_aim_for_imu(
        -4.0,
        52.0,
        0.49,
        -0.49,
        correction_threshold_deg=0.5,
    )
    assert yaw == pytest.approx(-4.0)
    assert elevation == pytest.approx(52.0)
    assert applied is False


def test_positive_pitch_reduces_command_elevation_with_project_convention():
    yaw, elevation, applied = correct_aim_for_imu(0.0, 53.0, 0.0, 5.0)
    assert yaw == pytest.approx(0.0, abs=1e-10)
    assert elevation == pytest.approx(48.0, abs=1e-10)
    assert applied is True


def test_roll_couples_into_both_yaw_and_elevation():
    yaw, elevation, applied = correct_aim_for_imu(0.0, 53.0, 5.0, 0.0)
    assert applied is True
    assert yaw > 0.0
    assert elevation < 53.0


def test_rotation_preserves_unit_direction_geometry():
    yaw, elevation, _ = correct_aim_for_imu(17.0, 52.0, 4.0, -3.0)
    x = math.cos(math.radians(elevation)) * math.cos(math.radians(yaw))
    y = math.cos(math.radians(elevation)) * math.sin(math.radians(yaw))
    z = math.sin(math.radians(elevation))
    assert math.sqrt(x * x + y * y + z * z) == pytest.approx(1.0, abs=1e-12)


def test_yaw_is_not_pan_limited_and_is_available_for_body_rotation():
    result = evaluate_corrected_aim(120.0, 53.0, 0.0, 0.0)
    assert result.valid is True
    assert result.corrected_yaw_deg == pytest.approx(120.0)


def test_corrected_elevation_outside_mechanical_range_is_invalid():
    result = evaluate_corrected_aim(0.0, 45.0, 0.0, 5.0)
    assert result.valid is False
    assert result.elevation_in_range is False
    assert result.reason == "CORRECTED_ELEVATION_LIMIT_EXCEEDED"
    assert result.corrected_elevation_deg == pytest.approx(40.0)
    assert result.limited_elevation_deg == pytest.approx(45.0)
    assert result.elevation_residual_deg == pytest.approx(-5.0)


def test_imu_safety_limit_blocks_solution():
    result = evaluate_corrected_aim(0.0, 53.0, 16.0, 0.0)
    assert result.valid is False
    assert result.imu_safe is False
    assert "IMU_ROLL_LIMIT_EXCEEDED" in result.reason


def test_non_finite_external_input_returns_invalid_result():
    result = evaluate_corrected_aim(float("nan"), 53.0, 0.0, 0.0)
    assert result.valid is False
    assert result.reason.startswith("NON_FINITE_INPUT")
    assert math.isnan(result.corrected_yaw_deg)


def test_low_level_function_rejects_non_finite_value():
    with pytest.raises(ValueError):
        correct_aim_for_imu(0.0, 53.0, float("inf"), 0.0)


def test_limit_angle_tolerance_and_residual():
    limited, exceeded, residual = limit_angle_with_residual(
        61.1,
        45.0,
        61.0,
        tolerance_deg=0.2,
    )
    assert limited == pytest.approx(61.0)
    assert exceeded is False
    assert residual == pytest.approx(0.1)

    _, exceeded_far, _ = limit_angle_with_residual(
        61.3,
        45.0,
        61.0,
        tolerance_deg=0.2,
    )
    assert exceeded_far is True


def test_normalize_signed_angle():
    assert normalize_signed_angle_deg(190.0) == pytest.approx(-170.0)
    assert normalize_signed_angle_deg(-190.0) == pytest.approx(170.0)
    assert normalize_signed_angle_deg(180.0) == pytest.approx(180.0)
