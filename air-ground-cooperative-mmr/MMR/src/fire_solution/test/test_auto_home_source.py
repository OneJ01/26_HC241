"""Source-level regression checks for post-fire and E-Stop-reset auto home."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "fire_solution" / "turret_control_node.py").read_text(encoding="utf-8")
CONFIG = (ROOT / "config" / "launch_params.yaml").read_text(encoding="utf-8")


def test_auto_home_parameters_are_declared_and_enabled_in_launch_config():
    assert 'declare_parameter("auto_home_after_fire", True)' in SOURCE
    assert 'declare_parameter("auto_home_after_estop_reset", True)' in SOURCE
    assert "auto_home_after_fire: true" in CONFIG
    assert "auto_home_after_estop_reset: true" in CONFIG


def test_fire_completion_queues_home_only_after_fired_state_is_published():
    fired_index = SOURCE.index('self._set_state(\n            "FIRED"')
    home_index = SOURCE.index('"source": "auto_after_fire"', fired_index)
    assert fired_index < home_index


def test_estop_reset_serializes_stop_clear_and_home_in_that_order():
    function_start = SOURCE.index("def _handle_estop_reset")
    function_end = SOURCE.index("# ------------------------------------------------------------------\n    # Hardware helpers", function_start)
    body = SOURCE[function_start:function_end]
    assert body.index('self._send_uart_line("STOP"') < body.index("self._estop_event.clear()")
    assert body.index("self._estop_event.clear()") < body.index('"source": "estop_reset"')


def test_motor_direction_mapping_was_not_changed():
    assert 'positive_sign = "+" if invert else "-"' in SOURCE
    assert 'negative_sign = "-" if invert else "+"' in SOURCE
