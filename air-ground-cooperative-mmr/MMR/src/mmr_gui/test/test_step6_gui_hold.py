from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "mmr_gui" / "main_gui.py"
TEXT = SOURCE.read_text(encoding="utf-8")


def test_hold_ring_is_embedded_in_main_gui():
    assert "class HoldProgressRing(QWidget):" in TEXT
    assert "hold_progress_ring.py" not in TEXT


def test_gui_uses_coordinator_topics():
    assert '"/mmr/operator_input"' in TEXT
    assert '"/mmr/engagement_status"' in TEXT
    assert '"align_aim_pressed"' in TEXT
    assert '"fire_pressed"' in TEXT


def test_hold_buttons_use_press_release_not_direct_click_fire():
    assert "self.btn_align_aim.pressed.connect" in TEXT
    assert "self.btn_align_aim.released.connect" in TEXT
    assert "self.btn_fire.pressed.connect" in TEXT
    assert "self.btn_fire.released.connect" in TEXT
    assert "self.btn_fire.clicked.connect(self._on_fire_btn_clicked)" not in TEXT


def test_operator_input_has_20hz_heartbeat():
    assert "self._operator_input_timer.start(50)" in TEXT
    assert "def _publish_operator_input_heartbeat" in TEXT


def test_engagement_progress_fields_are_displayed():
    assert 'data.get("aim_hold_progress"' in TEXT
    assert 'data.get("fire_hold_progress"' in TEXT
    assert "solution_revision" in TEXT


def test_heading_alignment_fields_are_displayed():
    assert '"current_heading_deg"' in TEXT
    assert '"target_heading_deg"' in TEXT
    assert '"heading_error_deg"' in TEXT
    assert 'self.lbl_align_heading' in TEXT
    assert 'self.lbl_heading_error' in TEXT


def test_target_solution_panel_subscribes_to_live_topics():
    assert '"/target/point"' in TEXT
    assert '"/mmr/fire_solution"' in TEXT
    assert 'self.lbl_active_target' in TEXT
    assert 'self.lbl_target_coord' in TEXT
    assert 'self.lbl_solution_aim' in TEXT
    assert 'self.lbl_solution_rev' in TEXT
