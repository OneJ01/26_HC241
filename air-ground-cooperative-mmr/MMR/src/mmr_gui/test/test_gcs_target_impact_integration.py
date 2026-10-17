"""Regression checks for GCS target retention and impact visualization."""

from pathlib import Path


SRC = Path(__file__).resolve().parents[2]
WORKSPACE = SRC.parent
GUI_SOURCE = SRC / "mmr_gui" / "mmr_gui" / "main_gui.py"
LAUNCH_SOURCE = SRC / "mmr_status" / "mmr_status" / "mmr_launch.py"
PARAMS_SOURCE = SRC / "fire_solution" / "config" / "launch_params.yaml"
GUI_TEXT = GUI_SOURCE.read_text(encoding="utf-8")
LAUNCH_TEXT = LAUNCH_SOURCE.read_text(encoding="utf-8")
PARAMS_TEXT = PARAMS_SOURCE.read_text(encoding="utf-8")


def test_gui_subscribes_to_gcs_target_and_impact_topics():
    assert '"/target/point"' in GUI_TEXT
    assert '"/compute/impact_point"' in GUI_TEXT
    assert "def _impact_point_cb(msg):" in GUI_TEXT


def test_target_marker_includes_configurable_success_radius():
    assert "def update_drone_target(" in GUI_TEXT
    assert "radius_m: float = DEFAULT_IMPACT_THRESHOLD_M" in GUI_TEXT
    assert "targetRadiusCircle" in GUI_TEXT
    assert "self._impact_threshold_m" in GUI_TEXT


def test_impact_result_draws_marker_line_and_distance_label():
    assert "def show_impact_result(" in GUI_TEXT
    assert "impactLayers" in GUI_TEXT
    assert "window.showImpactResult" in GUI_TEXT
    assert "L.polyline" in GUI_TEXT
    assert 'result = "SUCCESS" if error_m <= self._impact_threshold_m else "REATTACK"' in GUI_TEXT


def test_delayed_and_duplicate_impact_guards_are_present():
    assert "t_click_ns" in GUI_TEXT
    assert "target_publish_ns" in GUI_TEXT
    assert "_impact_seen_keys" in GUI_TEXT
    assert "_pending_impact_points" in GUI_TEXT
    readme_text = (WORKSPACE / "README.md").read_text(encoding="utf-8")
    assert "t_click_ns < target.t_publish_ns" in readme_text


def test_same_id_with_changed_sequence_or_coordinates_is_new_target():
    assert "target_payload_changed" in GUI_TEXT
    assert "horizontal_shift_m > 0.05" in GUI_TEXT
    assert 'str(previous_seq) != str(current_seq)' in GUI_TEXT


def test_gui_receives_shared_parameter_file_from_integrated_launch():
    assert "parameters=[fire_cfg]" in LAUNCH_TEXT
    for key in (
        "main_gui:",
        "impact_threshold_m: 15.0",
        "impact_history_max_count: 20",
        "reject_stale_impact: true",
    ):
        assert key in PARAMS_TEXT
