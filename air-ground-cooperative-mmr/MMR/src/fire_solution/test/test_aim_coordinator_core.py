import pytest

from fire_solution.aim_coordinator_core import (
    AimCoordinatorCore,
    CoordinatorConfig,
    EngagementState,
)


def make_core(**overrides):
    cfg = CoordinatorConfig(
        max_solution_age_sec=100.0,
        max_imu_age_sec=100.0,
        max_drive_status_age_sec=100.0,
        max_turret_status_age_sec=100.0,
        align_settle_sec=0.2,
        aim_hold_sec=0.5,
        fire_hold_sec=1.0,
        **overrides,
    )
    return AimCoordinatorCore(cfg)


def feed_common(core, now=0.0, *, yaw=8.0, elev=53.0, in_range=True, target_id="T1"):
    core.update_turret(
        {
            "state": "HOME_READY",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "aim_request_accept_seq": 0,
            "fire_request_accept_seq": 0,
            "command_rejection_seq": 0,
            "estop_latched": False,
        },
        now,
    )
    core.update_imu(
        {
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "tilt_ok": True,
            "imu_valid": True,
            "stationary": True,
        },
        now,
    )
    core.update_drive(
        {
            "all_stopped": True,
            "feedback_all_fresh": True,
            "manual_override_active": False,
        },
        now,
    )
    core.update_solution(
        {
            "target_id": target_id,
            "in_range": in_range,
            "solution_type": "valid" if in_range else "out_of_range",
            "turret_yaw_deg": yaw,
            "elevation_deg": elev,
            "horizontal_dist_m": 20.0,
            "mmr_heading_deg": 100.0,
            "target_lat_deg": 37.0,
            "target_lon_deg": 127.0,
            "target_z_m": 100.0,
        },
        now,
    )


def run_to_wait_align(core, now=0.0):
    core.update_buttons(False, False)
    core.step(now)
    core.step(now + 0.01)
    assert core.state == EngagementState.WAIT_ALIGN


def run_to_ready_for_aim(core, now=0.0):
    run_to_wait_align(core, now)

    # Hold A. With zero yaw, alignment enters settle immediately.
    core.update_buttons(True, False)
    core.step(now + 0.02)
    assert core.state == EngagementState.BODY_ALIGNING
    core.update_buttons(True, False)
    core.step(now + 0.03)
    core.update_buttons(True, False)
    core.step(now + 0.25)
    assert core.state == EngagementState.ALIGN_COMPLETE_WAIT_RELEASE

    # Release A and perform final recheck.
    core.update_buttons(False, False)
    core.step(now + 0.26)
    assert core.state == EngagementState.FINAL_RECHECK
    core.update_buttons(False, False)
    core.step(now + 0.27)
    assert core.state == EngagementState.READY_FOR_AIM


def run_to_aim_ready(core, now=0.0):
    run_to_ready_for_aim(core, now)

    # Second A hold confirms atomic turret aim.
    core.update_buttons(True, False)
    core.step(now + 0.30)
    assert core.state == EngagementState.AIM_CONFIRMING
    core.update_buttons(True, False)
    actions = core.step(now + 0.81)
    assert core.state == EngagementState.TURRET_AIMING
    assert [a.kind for a in actions] == ["aim_request"]

    core.update_turret(
        {
            "state": "AIM_READY",
            "home_valid": True,
            "aim_complete": True,
            "fire_ready": True,
            "active_target_key": core.locked_target_key,
            "active_solution_revision": core.locked_solution_revision,
            "target_elevation_deg": core.locked_elevation_deg,
            "current_elevation_deg": core.locked_elevation_deg,
            "aim_request_accept_seq": core.locked_aim_accept_seq + 1,
            "fire_request_accept_seq": 0,
            "command_rejection_seq": 0,
            "estop_latched": False,
        },
        now + 0.82,
    )
    core.update_buttons(False, False)
    core.step(now + 0.82)
    assert core.state == EngagementState.FINAL_AIM_VERIFY
    core.step(now + 0.83)
    assert core.state == EngagementState.AIM_READY


def test_startup_home_and_valid_solution_reach_wait_align():
    core = make_core()
    feed_common(core, yaw=8.0)
    run_to_wait_align(core)
    assert core.corrected_result.corrected_yaw_deg == pytest.approx(8.0)


def test_body_alignment_publishes_signed_pivot_command():
    core = make_core()
    feed_common(core, yaw=12.0)
    run_to_wait_align(core)

    core.update_buttons(True, False)
    core.step(0.02)
    core.update_buttons(True, False)
    actions = core.step(0.03)

    align = [a for a in actions if a.kind == "align_cmd"]
    assert len(align) == 1
    assert align[0].payload["active"] is True
    assert align[0].payload["yaw_error_deg"] == pytest.approx(12.0)
    assert align[0].payload["pivot_rpm"] == pytest.approx(7.0)


def test_align_release_before_completion_returns_to_wait_align_and_stops():
    core = make_core()
    feed_common(core, yaw=12.0)
    run_to_wait_align(core)
    core.update_buttons(True, False)
    core.step(0.02)

    core.update_buttons(False, False)
    actions = core.step(0.03)
    assert core.state == EngagementState.WAIT_ALIGN
    assert any(a.kind == "align_cmd" and not a.payload["active"] for a in actions)


def test_full_align_aim_fire_hold_flow():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)

    core.update_buttons(False, True)
    core.step(0.90)
    assert core.state == EngagementState.FIRE_CONFIRMING
    core.update_buttons(False, True)
    actions = core.step(1.91)
    assert core.state == EngagementState.FIRING
    assert [a.kind for a in actions] == ["fire_confirm"]
    assert actions[0].payload["command"] == "FIRE"

    core.update_turret(
        {
            "state": "FIRED",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "aim_request_accept_seq": core.locked_aim_accept_seq + 1,
            "fire_request_accept_seq": core.locked_fire_accept_seq + 1,
            "estop_latched": False,
        },
        1.92,
    )
    core.update_buttons(False, False)
    core.step(1.92)
    assert core.state == EngagementState.FIRED


def test_aim_hold_release_cancels_without_request():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_ready_for_aim(core)

    core.update_buttons(True, False)
    core.step(0.30)
    core.update_buttons(False, False)
    actions = core.step(0.40)
    assert core.state == EngagementState.READY_FOR_AIM
    assert not any(a.kind == "aim_request" for a in actions)
    assert core.aim_hold_progress == 0.0


def test_manual_override_cancels_auto_alignment():
    core = make_core()
    feed_common(core, yaw=10.0)
    run_to_wait_align(core)
    core.update_buttons(True, False)
    core.step(0.02)

    core.update_drive(
        {
            "all_stopped": False,
            "feedback_all_fresh": True,
            "manual_override_active": True,
        },
        0.03,
    )
    core.update_buttons(True, False)
    actions = core.step(0.03)
    assert core.state == EngagementState.WAIT_ALIGN
    assert any(a.kind == "align_cmd" and not a.payload["active"] for a in actions)


def test_out_of_range_requires_manual_reposition():
    core = make_core()
    feed_common(core, yaw=0.0, in_range=False)
    core.update_buttons(False, False)
    core.step(0.0)
    core.step(0.01)
    assert core.state == EngagementState.REPOSITION_REQUIRED


def test_target_change_during_atomic_turret_motion_invalidates_ready():
    core = make_core()
    feed_common(core, yaw=0.0, target_id="T1")
    run_to_ready_for_aim(core)
    core.update_buttons(True, False)
    core.step(0.30)
    core.update_buttons(True, False)
    core.step(0.81)
    assert core.state == EngagementState.TURRET_AIMING

    # A new target arrives while the turret is executing the locked move.
    core.update_solution(
        {
            "target_id": "T2",
            "in_range": True,
            "solution_type": "valid",
            "turret_yaw_deg": 0.0,
            "elevation_deg": 53.0,
            "horizontal_dist_m": 20.0,
        },
        0.82,
    )
    core.update_turret(
        {
            "state": "AIM_READY",
            "home_valid": True,
            "aim_complete": True,
            "fire_ready": True,
            "estop_latched": False,
        },
        0.83,
    )
    core.update_buttons(False, False)
    core.step(0.83)
    assert core.state == EngagementState.TARGET_CHECK
    assert core.locked_target_key == ""


def test_status_reports_alignment_heading_fields():
    core = make_core()
    feed_common(core, yaw=12.0)
    run_to_wait_align(core)

    status = core.status(0.05)
    assert status["current_heading_deg"] == pytest.approx(100.0)
    assert status["heading_error_deg"] == pytest.approx(12.0)
    assert status["target_heading_deg"] == pytest.approx(112.0)
    assert status["heading_tolerance_deg"] == pytest.approx(3.0)
    assert status["heading_within_tolerance"] is False


def run_to_turret_aiming(core, now=0.0):
    run_to_ready_for_aim(core, now)
    core.update_buttons(True, False)
    core.step(now + 0.30)
    core.update_buttons(True, False)
    actions = core.step(now + 0.81)
    assert core.state == EngagementState.TURRET_AIMING
    assert [a.kind for a in actions] == ["aim_request"]


def matching_aim_ready_payload(core, **updates):
    payload = {
        "state": "AIM_READY",
        "busy": False,
        "home_valid": True,
        "aim_complete": True,
        "fire_ready": True,
        "active_target_key": core.locked_target_key,
        "active_solution_revision": core.locked_solution_revision,
        "target_elevation_deg": core.locked_elevation_deg,
        "current_elevation_deg": core.locked_elevation_deg,
        "aim_request_accept_seq": core.locked_aim_accept_seq + 1,
        "fire_request_accept_seq": core.locked_fire_accept_seq,
        "command_rejection_seq": core.locked_aim_rejection_seq,
        "estop_latched": False,
    }
    payload.update(updates)
    return payload


def test_old_aim_ready_for_different_target_is_not_accepted():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_turret_aiming(core)

    core.update_turret(
        matching_aim_ready_payload(core, active_target_key="id:OLD"),
        0.82,
    )
    core.step(0.82)
    assert core.state == EngagementState.READY_FOR_AIM
    assert core.reason == "TURRET_AIM_RESULT_MISMATCH"


def test_matching_aim_rejection_returns_to_ready_for_aim():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_turret_aiming(core)

    core.update_turret(
        {
            "state": "HOME_READY",
            "busy": False,
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "command_rejection_seq": core.locked_aim_rejection_seq + 1,
            "last_rejected_command": "AIM",
            "last_rejection_reason": "TURRET_BUSY",
            "last_rejection_target_key": core.locked_target_key,
            "last_rejection_solution_revision": core.locked_solution_revision,
            "estop_latched": False,
        },
        0.82,
    )
    core.step(0.82)
    assert core.state == EngagementState.READY_FOR_AIM
    assert core.reason == "AIM_REQUEST_REJECTED:TURRET_BUSY"


def test_final_verify_home_ready_returns_to_reaim():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_turret_aiming(core)
    core.update_turret(matching_aim_ready_payload(core), 0.82)
    core.step(0.82)
    assert core.state == EngagementState.FINAL_AIM_VERIFY

    core.update_turret(
        {
            "state": "HOME_READY",
            "busy": False,
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "estop_latched": False,
        },
        0.83,
    )
    core.step(0.83)
    assert core.state == EngagementState.READY_FOR_AIM
    assert core.reason == "TURRET_READY_LOST_REAIM"


def test_firing_home_required_returns_to_startup_home():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)
    core.update_buttons(False, True)
    core.step(0.90)
    core.update_buttons(False, True)
    core.step(1.91)
    assert core.state == EngagementState.FIRING

    core.update_turret(
        {
            "state": "HOME_REQUIRED",
            "home_valid": False,
            "aim_complete": False,
            "fire_ready": False,
            "last_error": "FIRE_BLOCKED_AFTER_STOP:INP_NOT_READY",
            "estop_latched": False,
        },
        1.92,
    )
    core.update_buttons(False, False)
    core.step(1.92)
    assert core.state == EngagementState.STARTUP_WAIT_HOME
    assert "INP_NOT_READY" in core.reason


def test_matching_fire_rejection_returns_to_aim_ready():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)
    core.update_buttons(False, True)
    core.step(0.90)
    core.update_buttons(False, True)
    core.step(1.91)
    assert core.state == EngagementState.FIRING

    core.update_turret(
        matching_aim_ready_payload(
            core,
            command_rejection_seq=core.locked_fire_rejection_seq + 1,
            last_rejected_command="FIRE",
            last_rejection_reason="TURRET_BUSY",
            last_rejection_target_key=core.locked_target_key,
            last_rejection_solution_revision=core.locked_solution_revision,
        ),
        1.92,
    )
    core.update_buttons(False, False)
    core.step(1.92)
    assert core.state == EngagementState.AIM_READY
    assert core.reason == "FIRE_REQUEST_REJECTED:TURRET_BUSY"


def test_pre_accept_home_ready_does_not_cancel_aim_request():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_turret_aiming(core)

    # The coordinator may tick before the turret subscriber consumes AIM.
    core.step(0.815)
    assert core.state == EngagementState.TURRET_AIMING
    assert core.reason == "WAITING_FOR_TURRET_AIM_ACCEPT"


def test_old_matching_aim_ready_without_new_accept_is_not_accepted():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_turret_aiming(core)

    core.update_turret(
        matching_aim_ready_payload(
            core,
            aim_request_accept_seq=core.locked_aim_accept_seq,
        ),
        0.82,
    )
    core.step(0.82)
    assert core.state == EngagementState.TURRET_AIMING
    assert core.reason == "WAITING_FOR_TURRET_AIM_ACCEPT"


def test_pre_accept_aim_ready_does_not_cancel_fire_request():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)
    core.update_buttons(False, True)
    core.step(0.90)
    core.update_buttons(False, True)
    core.step(1.91)
    assert core.state == EngagementState.FIRING

    # Latest status can still be AIM_READY before FIRE callback acceptance.
    core.step(1.915)
    assert core.state == EngagementState.FIRING
    assert core.reason == "WAITING_FOR_TURRET_FIRE_ACCEPT"


def test_old_fired_without_new_fire_accept_is_not_accepted():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)
    core.update_buttons(False, True)
    core.step(0.90)
    core.update_buttons(False, True)
    core.step(1.91)
    assert core.state == EngagementState.FIRING

    core.update_turret(
        {
            "state": "FIRED",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "aim_request_accept_seq": core.locked_aim_accept_seq + 1,
            "fire_request_accept_seq": core.locked_fire_accept_seq,
            "estop_latched": False,
        },
        1.92,
    )
    core.step(1.92)
    assert core.state == EngagementState.FIRING
    assert core.reason == "WAITING_FOR_TURRET_FIRE_ACCEPT"


def test_fired_waits_during_auto_home_then_retains_target_and_returns_idle():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)

    core.update_buttons(False, True)
    core.step(0.90)
    core.update_buttons(False, True)
    core.step(1.91)
    assert core.state == EngagementState.FIRING

    fired_payload = {
        "state": "FIRED",
        "home_valid": True,
        "aim_complete": False,
        "fire_ready": False,
        "aim_request_accept_seq": core.locked_aim_accept_seq + 1,
        "fire_request_accept_seq": core.locked_fire_accept_seq + 1,
        "estop_latched": False,
    }
    core.update_turret(fired_payload, 1.92)
    core.update_buttons(False, False)
    core.step(1.92)
    assert core.state == EngagementState.FIRED

    core.update_turret(
        {
            **fired_payload,
            "state": "HOMING",
            "home_valid": False,
            "busy": True,
        },
        2.0,
    )
    actions = core.step(2.0)
    assert core.state == EngagementState.FIRED
    assert core.reason == "POST_FIRE_AUTO_HOMING"
    assert not any(a.kind == "clear_target" for a in actions)

    original_target = core.active_target_key
    original_solution = dict(core.solution)
    core.update_turret(
        {
            **fired_payload,
            "state": "HOME_READY",
            "home_valid": True,
            "busy": False,
        },
        2.1,
    )
    actions = core.step(2.1)
    assert core.state == EngagementState.IDLE
    assert core.reason == "FIRE_COMPLETE_TARGET_RETAINED"
    assert core.solution == original_solution
    assert core.active_target_key == original_target
    assert core.status(2.1)["awaiting_reengagement"] is True
    assert not any(a.kind == "clear_target" for a in actions)


def test_post_fire_reengagement_requires_new_a_press_and_new_revision():
    core = make_core()
    feed_common(core, yaw=0.0)
    core.state = EngagementState.IDLE
    core._awaiting_reengagement = True
    core._completed_solution_signature = core._solution_signature
    old_revision = core.solution_revision

    # Heartbeat와 A 미입력만으로는 상태가 진행되지 않는다.
    heartbeat = dict(core.solution)
    core.update_solution(heartbeat, 1.0)
    core.update_buttons(False, False)
    core.step(1.0)
    assert core.state == EngagementState.IDLE
    assert core.reason == "TARGET_RETAINED_WAIT_A"

    # 동일 표적 재사용은 새 A rising edge에서만 시작한다.
    core.update_buttons(True, False)
    core.step(1.1)
    assert core.state == EngagementState.TARGET_CHECK
    assert core.reason == "REENGAGEMENT_REQUESTED"
    assert core.solution_revision == old_revision + 1


def test_next_target_received_during_firing_is_activated_only_after_home():
    core = make_core()
    feed_common(core, yaw=0.0, target_id="T1")
    core.state = EngagementState.FIRING
    core.locked_target_key = core.active_target_key
    core.locked_solution_revision = core.solution_revision

    next_solution = dict(core.solution)
    next_solution.update(
        {
            "target_id": "T2",
            "target_lat_deg": 37.0003,
            "target_lon_deg": 127.0003,
            "turret_yaw_deg": 5.0,
        }
    )
    core.update_solution(next_solution, 1.0)
    assert core.state == EngagementState.FIRING
    assert core._next_target_received_during_post_fire is True

    core.update_turret(
        {
            "state": "FIRED",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "fire_request_accept_seq": core.locked_fire_accept_seq + 1,
            "estop_latched": False,
        },
        1.1,
    )
    core.step(1.1)
    assert core.state == EngagementState.FIRED

    core.update_turret(
        {
            "state": "HOME_READY",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "estop_latched": False,
        },
        1.2,
    )
    actions = core.step(1.2)
    assert core.state == EngagementState.TARGET_CHECK
    assert core.active_target_key == "id:T2"
    assert not any(a.kind == "clear_target" for a in actions)


def test_next_target_received_during_post_fire_waits_for_home_then_activates():
    core = make_core()
    feed_common(core, yaw=0.0, target_id="T1")
    core.state = EngagementState.FIRED
    core.locked_target_key = core.active_target_key

    next_solution = dict(core.solution)
    next_solution.update(
        {
            "target_id": "T2",
            "target_lat_deg": 37.0002,
            "target_lon_deg": 127.0002,
            "turret_yaw_deg": 4.0,
        }
    )
    core.update_solution(next_solution, 1.0)
    assert core.state == EngagementState.FIRED
    assert core.active_target_key == "id:T2"
    assert core.status(1.0)["next_target_buffered_during_post_fire"] is True

    core.update_turret(
        {
            "state": "HOME_READY",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "estop_latched": False,
        },
        1.1,
    )
    actions = core.step(1.1)
    assert core.state == EngagementState.TARGET_CHECK
    assert core.reason == "NEXT_TARGET"
    assert core.active_target_key == "id:T2"
    assert not any(a.kind == "clear_target" for a in actions)


def test_post_fire_clear_mode_remains_available_by_parameter():
    core = make_core(retain_target_after_fire=False)
    feed_common(core, yaw=0.0)
    core.state = EngagementState.FIRED
    core.locked_target_key = core.active_target_key
    core.update_turret(
        {
            "state": "HOME_READY",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "estop_latched": False,
        },
        2.0,
    )
    actions = core.step(2.0)
    assert core.state == EngagementState.IDLE
    assert core.reason == "FIRE_COMPLETE_HOME_READY"
    assert core.solution is None
    assert core.active_target_key == ""
    clear_actions = [a for a in actions if a.kind == "clear_target"]
    assert len(clear_actions) == 1

def test_completed_solution_heartbeat_updates_freshness_without_restart():
    core = make_core()
    feed_common(core, yaw=0.0)
    original = dict(core.solution)
    core.state = EngagementState.IDLE
    core._awaiting_reengagement = True
    core._completed_solution_signature = core._solution_signature

    core.update_solution(original, 1.0)
    assert core.state == EngagementState.IDLE
    assert core.solution == original
    assert core.solution_rx_time == pytest.approx(1.0)
    assert core.reason == "TARGET_RETAINED_WAIT_A"

def test_fired_state_does_not_finish_before_turret_enters_home_ready():
    core = make_core()
    feed_common(core, yaw=0.0)
    run_to_aim_ready(core)
    core.update_buttons(False, True)
    core.step(0.90)
    core.update_buttons(False, True)
    core.step(1.91)

    core.update_turret(
        {
            "state": "FIRED",
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "aim_request_accept_seq": core.locked_aim_accept_seq + 1,
            "fire_request_accept_seq": core.locked_fire_accept_seq + 1,
            "estop_latched": False,
        },
        1.92,
    )
    core.update_buttons(False, False)
    core.step(1.92)
    assert core.state == EngagementState.FIRED

    actions = core.step(1.93)
    assert core.state == EngagementState.FIRED
    assert core.reason == "FIRE_COMPLETE_WAITING_AUTO_HOME"
    assert not any(a.kind == "clear_target" for a in actions)


def test_body_alignment_can_continue_with_fresh_moving_imu_when_enabled():
    core = make_core(allow_motion_during_body_align=True)
    feed_common(core, yaw=12.0)
    run_to_wait_align(core)

    core.update_buttons(True, False)
    core.step(0.02)

    # tilt_monitor_node는 이동 중 stationary/imu_valid/tilt_ok를 false로 내보내지만
    # roll_ok/pitch_ok와 데이터 freshness는 유지한다.
    core.update_imu(
        {
            "roll_deg": 0.2,
            "pitch_deg": -0.1,
            "roll_ok": True,
            "pitch_ok": True,
            "tilt_ok": False,
            "imu_valid": False,
            "stationary": False,
        },
        0.03,
    )
    core.update_drive(
        {
            "all_stopped": False,
            "feedback_all_fresh": True,
            "manual_override_active": False,
        },
        0.03,
    )
    core.update_buttons(True, False)
    actions = core.step(0.03)

    align = [a for a in actions if a.kind == "align_cmd"]
    assert len(align) == 1
    assert align[0].payload["active"] is True
    assert align[0].payload["yaw_error_deg"] == pytest.approx(12.0)
