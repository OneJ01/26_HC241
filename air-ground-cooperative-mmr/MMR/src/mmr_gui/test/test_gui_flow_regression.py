"""Regression checks for stable GUI state display and post-fire presentation."""

from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "mmr_gui" / "main_gui.py"
TEXT = SOURCE.read_text(encoding="utf-8")


def test_engagement_status_is_primary_fire_badge_authority():
    assert "def _engagement_status_is_authoritative" in TEXT
    assert "if self._engagement_status_is_authoritative():\n            return" in TEXT


def test_high_rate_labels_only_repaint_on_change():
    assert "def _set_label_if_changed" in TEXT
    assert "if label.text() != text:" in TEXT
    assert "if style is not None and label.styleSheet() != style:" in TEXT


def test_fired_banner_has_fixed_short_display_window():
    assert "self._fired_banner_duration_sec: float = 1.5" in TEXT
    assert 'state == "FIRED"' in TEXT
    assert "self._fired_banner_until" in TEXT


def test_long_reason_labels_cannot_expand_window_minimum_width():
    assert "self.lbl_engagement_reason.setMinimumWidth(0)" in TEXT
    assert "QSizePolicy.Policy.Ignored" in TEXT
    assert "self.lbl_turret_error.setMinimumWidth(0)" in TEXT


def test_motion_only_tilt_warning_is_hidden_without_disabling_real_limits():
    assert "motion_only_invalid = (" in TEXT
    assert "and stationary is False" in TEXT
    assert "and roll_ok is not False" in TEXT
    assert "and pitch_ok is not False" in TEXT
    assert "if motion_only_invalid:" in TEXT
    assert "self._tilt_motion_log_suppressed_ui = True" in TEXT
    assert "실제 roll/pitch 제한 초과는 그대로 기록한다" in TEXT


def test_drive_motion_imu_notices_are_filtered_only_in_gui():
    assert "_DRIVE_MOVING_HIDDEN_IMU_REASONS" in TEXT
    assert "def _drive_motion_recent_for_ui" in TEXT
    assert "hide_expected_moving_notice = hide_alignment_notice or hide_motion_imu_notice" in TEXT
    assert "안전 인터록은 변경하지 않는다" in TEXT

def test_estop_recovery_display_has_priority_over_retained_target_label():
    assert 'if awaiting_reengagement and state != "FAULT":' in TEXT
    assert 'state_ko = "시퀀스 복구 필요"' in TEXT

