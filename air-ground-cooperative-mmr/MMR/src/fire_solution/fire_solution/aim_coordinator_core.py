#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure state machine for MMR body alignment, turret aiming, and fire approval.

The class in this module has no ROS dependency.  ROS callbacks only feed the
latest JSON payloads and button state into :class:`AimCoordinatorCore`; the
core returns actions that the ROS wrapper publishes.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Any, Optional

from .aim_math import AimCorrectionResult, evaluate_corrected_aim


class EngagementState(str, Enum):
    STARTUP_WAIT_HOME = "STARTUP_WAIT_HOME"
    IDLE = "IDLE"
    TARGET_CHECK = "TARGET_CHECK"
    REPOSITION_REQUIRED = "REPOSITION_REQUIRED"
    WAIT_ALIGN = "WAIT_ALIGN"
    BODY_ALIGNING = "BODY_ALIGNING"
    ALIGN_COMPLETE_WAIT_RELEASE = "ALIGN_COMPLETE_WAIT_RELEASE"
    FINAL_RECHECK = "FINAL_RECHECK"
    READY_FOR_AIM = "READY_FOR_AIM"
    AIM_CONFIRMING = "AIM_CONFIRMING"
    TURRET_AIMING = "TURRET_AIMING"
    FINAL_AIM_VERIFY = "FINAL_AIM_VERIFY"
    AIM_READY = "AIM_READY"
    FIRE_CONFIRMING = "FIRE_CONFIRMING"
    FIRING = "FIRING"
    FIRED = "FIRED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class CoordinatorConfig:
    align_enter_tolerance_deg: float = 2.0
    align_exit_tolerance_deg: float = 3.0
    align_settle_sec: float = 0.5

    aim_hold_sec: float = 0.5
    fire_hold_sec: float = 1.0

    align_fast_error_deg: float = 10.0
    align_slow_error_deg: float = 3.0
    align_fast_rpm: float = 7.0
    align_medium_rpm: float = 6.0
    align_slow_rpm: float = 5.0

    max_solution_age_sec: float = 1.0
    max_imu_age_sec: float = 0.5
    max_drive_status_age_sec: float = 0.75
    max_turret_status_age_sec: float = 1.0

    require_drive_feedback: bool = True
    require_imu_stationary: bool = True
    # true이면 BODY_ALIGNING 중에는 GNSS 기반 raw yaw로 계속 주행하고,
    # 정지 후에만 stationary IMU 보정값으로 최종 재검증한다.
    allow_motion_during_body_align: bool = False

    correction_enabled: bool = True
    correction_threshold_deg: float = 0.5
    imu_roll_sign: float = 1.0
    imu_pitch_sign: float = 1.0
    imu_roll_limit_deg: float = 15.0
    imu_pitch_limit_deg: float = 15.0
    elevation_min_deg: float = 45.0
    elevation_max_deg: float = 61.0
    mechanical_limit_tolerance_deg: float = 0.2

    final_elevation_tolerance_deg: float = 0.5
    home_request_retry_sec: float = 2.0

    # 발사 완료 후에도 마지막 표적/탄도해를 보관한다. 같은 표적 재교전은
    # 자동으로 시작하지 않고 IDLE 상태에서 새로운 A 버튼 rising edge를 요구한다.
    retain_target_after_fire: bool = True


@dataclass(frozen=True)
class CoordinatorAction:
    kind: str
    payload: dict[str, Any]


class AimCoordinatorCore:
    """Deterministic engagement state machine.

    Button A is used twice:
    - hold-to-run body alignment,
    - after release, hold again for ``aim_hold_sec`` to start atomic turret aim.

    Button Y is hold-to-confirm fire for ``fire_hold_sec``.
    """

    def __init__(self, config: Optional[CoordinatorConfig] = None) -> None:
        self.cfg = config or CoordinatorConfig()
        self.state = EngagementState.STARTUP_WAIT_HOME
        self.reason = "WAITING_FOR_TURRET_HOME"

        self.solution: Optional[dict[str, Any]] = None
        self.solution_rx_time: Optional[float] = None
        self.solution_revision = 0
        self._solution_signature: Optional[tuple[Any, ...]] = None
        # 발사 완료 표적을 보관하는 동안 같은 표적의 heartbeat가 상태 머신을
        # 자동 재시작하지 않도록 완료 signature와 재교전 대기 플래그를 둔다.
        self._completed_solution_signature: Optional[tuple[Any, ...]] = None
        self._awaiting_reengagement = False
        self._next_target_received_during_post_fire = False

        self.imu: Optional[dict[str, Any]] = None
        self.imu_rx_time: Optional[float] = None
        self.drive: Optional[dict[str, Any]] = None
        self.drive_rx_time: Optional[float] = None
        self.turret: Optional[dict[str, Any]] = None
        self.turret_rx_time: Optional[float] = None

        self.active_target_key = ""
        self.locked_target_key = ""
        self.locked_solution_revision: Optional[int] = None
        self.locked_elevation_deg: Optional[float] = None
        self.locked_yaw_deg: Optional[float] = None
        self.locked_aim_rejection_seq = 0
        self.locked_fire_rejection_seq = 0
        self.locked_aim_accept_seq = 0
        self.locked_fire_accept_seq = 0
        self.target_changed_during_atomic_action = False

        self.corrected_result: Optional[AimCorrectionResult] = None

        self.a_pressed = False
        self.y_pressed = False
        self._prev_a_pressed = False
        self._prev_y_pressed = False
        self.a_rising = False
        self.a_falling = False
        self.y_rising = False
        self.y_falling = False

        self.align_settle_started_at: Optional[float] = None
        self.aim_hold_started_at: Optional[float] = None
        self.fire_hold_started_at: Optional[float] = None
        self.aim_hold_progress = 0.0
        self.fire_hold_progress = 0.0

        self._last_home_request_at: Optional[float] = None
        self._reposition_entry_revision = 0
        self.last_action = ""

    # ------------------------------------------------------------------
    # Input updates
    # ------------------------------------------------------------------
    def update_solution(self, payload: dict[str, Any], now: float) -> None:
        if not isinstance(payload, dict):
            return

        target_key = self._make_target_key(payload)
        signature = self._make_solution_signature(payload, target_key)
        old_target = self.active_target_key
        target_changed = bool(old_target and target_key and old_target != target_key)

        if signature != self._solution_signature:
            self.solution_revision += 1
            self._solution_signature = signature

        # 완료 표적도 solver heartbeat로 freshness가 계속 갱신되어야 같은 좌표를
        # 다시 사용할 수 있다. 단, 재교전 대기 중에는 동일 표적 heartbeat나
        # GNSS/풍속에 따른 탄도해 미세 갱신만으로 시퀀스를 자동 재시작하지 않는다.
        self.solution = dict(payload)
        self.solution_rx_time = float(now)
        self.active_target_key = target_key

        # 자동 홈 복귀가 끝나기 전에는 새 표적이 들어와도 FIRED 상태를 벗어나지
        # 않는다. 최신 표적/탄도해만 버퍼링하고 HOME_READY 이후 활성화한다.
        if self.state == EngagementState.FIRED:
            if target_changed:
                self._next_target_received_during_post_fire = True
                self.reason = "NEXT_TARGET_BUFFERED_DURING_POST_FIRE"
            return

        if self._awaiting_reengagement:
            if target_changed:
                self._awaiting_reengagement = False
                self._completed_solution_signature = None
                self._clear_locked_solution()
                self._set_state(EngagementState.TARGET_CHECK, "NEXT_TARGET")
            else:
                self._completed_solution_signature = signature
                if self.state == EngagementState.IDLE:
                    self.reason = "TARGET_RETAINED_WAIT_A"
            return

        if self._completed_solution_signature is not None:
            self._completed_solution_signature = None

        if target_changed:
            if self.state in {
                EngagementState.TURRET_AIMING,
                EngagementState.FIRING,
            }:
                self.target_changed_during_atomic_action = True
                if self.state == EngagementState.FIRING:
                    self._next_target_received_during_post_fire = True
            else:
                self._clear_locked_solution()
                self._set_state(EngagementState.TARGET_CHECK, "NEW_TARGET")

        if self.state == EngagementState.IDLE:
            self._set_state(EngagementState.TARGET_CHECK, "TARGET_RECEIVED")
        elif self.state == EngagementState.FIRED and target_changed:
            self._clear_locked_solution()
            self._set_state(EngagementState.TARGET_CHECK, "NEXT_TARGET")

    def update_imu(self, payload: dict[str, Any], now: float) -> None:
        if isinstance(payload, dict):
            self.imu = dict(payload)
            self.imu_rx_time = float(now)

    def update_drive(self, payload: dict[str, Any], now: float) -> None:
        if isinstance(payload, dict):
            self.drive = dict(payload)
            self.drive_rx_time = float(now)

    def update_turret(self, payload: dict[str, Any], now: float) -> None:
        if isinstance(payload, dict):
            self.turret = dict(payload)
            self.turret_rx_time = float(now)

    def update_buttons(self, a_pressed: bool, y_pressed: bool) -> None:
        self._prev_a_pressed = self.a_pressed
        self._prev_y_pressed = self.y_pressed
        self.a_pressed = bool(a_pressed)
        self.y_pressed = bool(y_pressed)
        self.a_rising = self.a_pressed and not self._prev_a_pressed
        self.a_falling = (not self.a_pressed) and self._prev_a_pressed
        self.y_rising = self.y_pressed and not self._prev_y_pressed
        self.y_falling = (not self.y_pressed) and self._prev_y_pressed

    # ------------------------------------------------------------------
    # Main state-machine tick
    # ------------------------------------------------------------------
    def step(self, now: float) -> list[CoordinatorAction]:
        now = float(now)
        actions: list[CoordinatorAction] = []

        try:
            self._global_turret_guard(now, actions)

            if self.state == EngagementState.STARTUP_WAIT_HOME:
                self._step_wait_home(now, actions)
            elif self.state == EngagementState.IDLE:
                self._step_idle()
            elif self.state == EngagementState.TARGET_CHECK:
                self._step_target_check(now)
            elif self.state == EngagementState.REPOSITION_REQUIRED:
                self._step_reposition(now)
            elif self.state == EngagementState.WAIT_ALIGN:
                self._step_wait_align(now)
            elif self.state == EngagementState.BODY_ALIGNING:
                self._step_body_aligning(now, actions)
            elif self.state == EngagementState.ALIGN_COMPLETE_WAIT_RELEASE:
                self._step_align_complete(now, actions)
            elif self.state == EngagementState.FINAL_RECHECK:
                self._step_final_recheck(now)
            elif self.state == EngagementState.READY_FOR_AIM:
                self._step_ready_for_aim(now)
            elif self.state == EngagementState.AIM_CONFIRMING:
                self._step_aim_confirming(now, actions)
            elif self.state == EngagementState.TURRET_AIMING:
                self._step_turret_aiming(now)
            elif self.state == EngagementState.FINAL_AIM_VERIFY:
                self._step_final_aim_verify(now)
            elif self.state == EngagementState.AIM_READY:
                self._step_aim_ready(now)
            elif self.state == EngagementState.FIRE_CONFIRMING:
                self._step_fire_confirming(now, actions)
            elif self.state == EngagementState.FIRING:
                self._step_firing()
            elif self.state == EngagementState.FIRED:
                self._step_fired(actions)
            elif self.state == EngagementState.FAULT:
                self._step_fault(now)
        finally:
            # Edges apply to exactly one state-machine cycle.
            self.a_rising = False
            self.a_falling = False
            self.y_rising = False
            self.y_falling = False

        return actions

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------
    def _global_turret_guard(
        self,
        now: float,
        actions: list[CoordinatorAction],
    ) -> None:
        if not self._is_fresh(self.turret_rx_time, self.cfg.max_turret_status_age_sec, now):
            # Never issue a new HOME while an atomic turret move or fire
            # sequence may still be running.  Wait for status to recover.
            if self.state in {EngagementState.TURRET_AIMING, EngagementState.FIRING}:
                self.reason = "TURRET_STATUS_STALE_DURING_ATOMIC_ACTION"
            elif self.state not in {EngagementState.IDLE, EngagementState.STARTUP_WAIT_HOME}:
                self._set_state(EngagementState.STARTUP_WAIT_HOME, "TURRET_STATUS_STALE")
            return

        assert self.turret is not None
        turret_state = str(self.turret.get("state", "")).upper()
        estop = bool(self.turret.get("estop_latched", False)) or turret_state == "ESTOP"
        if estop:
            self._stop_align(actions)
            self._set_state(EngagementState.FAULT, "TURRET_ESTOP")
            return

        if turret_state == "FAULT":
            self._stop_align(actions)
            self._set_state(
                EngagementState.FAULT,
                str(self.turret.get("last_error", "TURRET_FAULT")),
            )
            return

        home_valid = bool(self.turret.get("home_valid", False))
        if not home_valid and self.state not in {
            EngagementState.TURRET_AIMING,
            EngagementState.FIRING,
            EngagementState.FIRED,
            EngagementState.FAULT,
            EngagementState.STARTUP_WAIT_HOME,
        }:
            self._stop_align(actions)
            self._set_state(EngagementState.STARTUP_WAIT_HOME, "TURRET_HOME_INVALID")

    def _step_wait_home(
        self,
        now: float,
        actions: list[CoordinatorAction],
    ) -> None:
        if self._turret_home_ready(now):
            next_state = EngagementState.TARGET_CHECK if self.solution else EngagementState.IDLE
            self._set_state(next_state, "HOME_READY")
            return

        turret_state = str((self.turret or {}).get("state", "")).upper()
        if turret_state == "ESTOP":
            self._set_state(EngagementState.FAULT, "TURRET_ESTOP")
            return

        if turret_state in {"HOME_REQUIRED", "FAULT", ""} or self.turret is None:
            if (
                self._last_home_request_at is None
                or now - self._last_home_request_at >= self.cfg.home_request_retry_sec
            ):
                actions.append(
                    CoordinatorAction(
                        "home_request",
                        {"command": "HOME", "source": "aim_coordinator"},
                    )
                )
                self._last_home_request_at = now
                self.last_action = "HOME_REQUEST"

    def _step_idle(self) -> None:
        if self.solution is None:
            self._awaiting_reengagement = False
            return

        if self._awaiting_reengagement:
            # 발사 완료 후 남아 있는 좌표만으로 정렬이 자동 재개되지 않도록 한다.
            # 반드시 홈 복귀 이후 새로 누른 A 버튼 edge가 있어야 동일 표적을
            # 새로운 교전 revision으로 다시 사용할 수 있다.
            if self.a_rising:
                self._awaiting_reengagement = False
                self._completed_solution_signature = None
                self.solution_revision += 1
                self._set_state(EngagementState.TARGET_CHECK, "REENGAGEMENT_REQUESTED")
            else:
                self.reason = "TARGET_RETAINED_WAIT_A"
            return

        self._set_state(EngagementState.TARGET_CHECK, "TARGET_AVAILABLE")

    def _step_target_check(self, now: float) -> None:
        result, reason = self._evaluate_current_aim(now)
        if result is None:
            if reason == "NO_SOLUTION":
                self._set_state(EngagementState.IDLE, reason)
            elif reason in {"OUT_OF_RANGE", "ELEVATION_OUT_OF_RANGE"}:
                self._enter_reposition(reason)
            else:
                self.reason = reason
            return

        self.corrected_result = result
        self._set_state(EngagementState.WAIT_ALIGN, "SOLUTION_READY")

    def _step_reposition(self, now: float) -> None:
        if not self._drive_stopped(now):
            self.reason = "MOVE_WITH_JOYSTICK"
            return

        result, reason = self._evaluate_current_aim(now)
        if result is not None:
            self.corrected_result = result
            self._set_state(EngagementState.WAIT_ALIGN, "POSITION_NOW_VALID")
            return

        self.reason = reason

    def _step_wait_align(self, now: float) -> None:
        result, reason = self._evaluate_current_aim(now)
        if result is None:
            self._route_invalid_aim(reason)
            return

        self.corrected_result = result
        if self.a_pressed:
            self.align_settle_started_at = None
            self._set_state(EngagementState.BODY_ALIGNING, "ALIGN_BUTTON_HELD")

    def _step_body_aligning(
        self,
        now: float,
        actions: list[CoordinatorAction],
    ) -> None:
        if not self.a_pressed:
            self._stop_align(actions)
            self.align_settle_started_at = None
            self._set_state(EngagementState.WAIT_ALIGN, "ALIGN_BUTTON_RELEASED")
            return

        if self._manual_override_active(now):
            self._stop_align(actions)
            self.align_settle_started_at = None
            self._set_state(EngagementState.WAIT_ALIGN, "MANUAL_DRIVE_OVERRIDE")
            return

        result: Optional[AimCorrectionResult] = None
        if (
            self.cfg.allow_motion_during_body_align
            and self._imu_is_fresh_motion_state(now)
        ):
            yaw_error, reason = self._evaluate_raw_alignment_yaw(now)
            if yaw_error is None:
                self._stop_align(actions)
                self.align_settle_started_at = None
                self._route_invalid_aim(reason)
                return
        else:
            result, reason = self._evaluate_current_aim(now)
            if result is None:
                self._stop_align(actions)
                self.align_settle_started_at = None
                self._route_invalid_aim(reason)
                return
            self.corrected_result = result
            yaw_error = result.corrected_yaw_deg
        if abs(yaw_error) > self.cfg.align_enter_tolerance_deg:
            self.align_settle_started_at = None
            rpm = self._pivot_rpm(abs(yaw_error))
            actions.append(
                CoordinatorAction(
                    "align_cmd",
                    {
                        "active": True,
                        "yaw_error_deg": yaw_error,
                        "pivot_rpm": rpm,
                        "source": "aim_coordinator",
                    },
                )
            )
            self.last_action = "AUTO_ALIGN"
            self.reason = "BODY_ROTATING"
            return

        self._stop_align(actions)
        if not self._drive_stopped(now):
            self.align_settle_started_at = None
            self.reason = "WAITING_FOR_WHEELS_TO_STOP"
            return

        if not self._imu_stable(now):
            self.align_settle_started_at = None
            self.reason = "WAITING_FOR_IMU_STABLE"
            return

        # 이동 중에는 raw GNSS yaw를 사용했으므로, 바퀴 정지와 IMU 안정 후
        # 반드시 롤/피치 보정값으로 한 번 더 확인한다.
        final_result, reason = self._evaluate_current_aim(now)
        if final_result is None:
            self.align_settle_started_at = None
            self._route_invalid_aim(reason)
            return
        self.corrected_result = final_result
        if abs(final_result.corrected_yaw_deg) > self.cfg.align_enter_tolerance_deg:
            self.align_settle_started_at = None
            self.reason = "ALIGN_FINAL_CORRECTION_REQUIRED"
            return

        if self.align_settle_started_at is None:
            self.align_settle_started_at = now
            self.reason = "ALIGN_SETTLING"
            return

        if now - self.align_settle_started_at >= self.cfg.align_settle_sec:
            self._set_state(
                EngagementState.ALIGN_COMPLETE_WAIT_RELEASE,
                "ALIGN_COMPLETE_RELEASE_BUTTON",
            )

    def _step_align_complete(
        self,
        now: float,
        actions: list[CoordinatorAction],
    ) -> None:
        self._stop_align(actions)
        result, reason = self._evaluate_current_aim(now)
        if result is None:
            self._route_invalid_aim(reason)
            return

        self.corrected_result = result
        if abs(result.corrected_yaw_deg) > self.cfg.align_exit_tolerance_deg:
            self.align_settle_started_at = None
            if self.a_pressed:
                self._set_state(EngagementState.BODY_ALIGNING, "ALIGN_DRIFTED")
            else:
                self._set_state(EngagementState.WAIT_ALIGN, "ALIGN_DRIFTED")
            return

        if self.a_falling or not self.a_pressed:
            self._set_state(EngagementState.FINAL_RECHECK, "ALIGN_BUTTON_RELEASED")

    def _step_final_recheck(self, now: float) -> None:
        result, reason = self._evaluate_current_aim(now)
        if result is None:
            self._route_invalid_aim(reason)
            return
        self.corrected_result = result

        if abs(result.corrected_yaw_deg) > self.cfg.align_exit_tolerance_deg:
            self._set_state(EngagementState.WAIT_ALIGN, "YAW_OUT_AFTER_RELEASE")
            return
        if not self._drive_stopped(now):
            self._set_state(EngagementState.WAIT_ALIGN, "DRIVE_NOT_STOPPED")
            return
        if not self._imu_stable(now):
            self.reason = "WAITING_FOR_IMU_STABLE"
            return

        self._set_state(EngagementState.READY_FOR_AIM, "HOLD_A_TO_AIM")

    def _step_ready_for_aim(self, now: float) -> None:
        result, reason = self._evaluate_aligned_ready(now)
        if result is None:
            self._route_invalid_aim(reason)
            return
        self.corrected_result = result

        if self.a_pressed:
            self.aim_hold_started_at = now
            self.aim_hold_progress = 0.0
            self._set_state(EngagementState.AIM_CONFIRMING, "AIM_HOLD_STARTED")

    def _step_aim_confirming(
        self,
        now: float,
        actions: list[CoordinatorAction],
    ) -> None:
        if not self.a_pressed:
            self.aim_hold_started_at = None
            self.aim_hold_progress = 0.0
            self._set_state(EngagementState.READY_FOR_AIM, "AIM_HOLD_CANCELLED")
            return

        result, reason = self._evaluate_aligned_ready(now)
        if result is None:
            self.aim_hold_started_at = None
            self.aim_hold_progress = 0.0
            self._route_invalid_aim(reason)
            return

        self.corrected_result = result
        if self.aim_hold_started_at is None:
            self.aim_hold_started_at = now
        elapsed = max(0.0, now - self.aim_hold_started_at)
        self.aim_hold_progress = min(1.0, elapsed / max(1e-6, self.cfg.aim_hold_sec))
        self.reason = "HOLD_TO_CONFIRM_AIM"

        if elapsed < self.cfg.aim_hold_sec:
            return

        self.locked_target_key = self.active_target_key
        self.locked_solution_revision = self.solution_revision
        self.locked_elevation_deg = result.corrected_elevation_deg
        self.locked_yaw_deg = result.corrected_yaw_deg
        self.target_changed_during_atomic_action = False
        self.locked_aim_rejection_seq = self._turret_rejection_seq()
        self.locked_aim_accept_seq = self._turret_accept_seq("AIM")

        actions.append(
            CoordinatorAction(
                "aim_request",
                {
                    "target_key": self.locked_target_key,
                    "solution_revision": self.locked_solution_revision,
                    "elevation_deg": self.locked_elevation_deg,
                    "corrected_elevation_deg": self.locked_elevation_deg,
                    "corrected_yaw_deg": self.locked_yaw_deg,
                    "source": "aim_coordinator",
                },
            )
        )
        self.last_action = "AIM_REQUEST"
        self.aim_hold_started_at = None
        self.aim_hold_progress = 1.0
        self._set_state(EngagementState.TURRET_AIMING, "TURRET_AIM_REQUESTED")

    def _step_turret_aiming(self, now: float) -> None:
        if not self._is_fresh(self.turret_rx_time, self.cfg.max_turret_status_age_sec, now):
            self.reason = "TURRET_STATUS_STALE"
            return

        if self.target_changed_during_atomic_action:
            self._clear_locked_solution()
            self._set_state(EngagementState.TARGET_CHECK, "TARGET_CHANGED_DURING_AIM")
            return

        turret = self.turret or {}
        turret_state = str(turret.get("state", "")).upper()

        rejected_reason = self._matching_new_rejection(
            "AIM",
            self.locked_aim_rejection_seq,
        )
        if rejected_reason:
            if rejected_reason == "ESTOP_LATCHED" or turret_state == "ESTOP":
                self._set_state(EngagementState.FAULT, "AIM_REQUEST_REJECTED:ESTOP_LATCHED")
            elif turret_state == "HOME_REQUIRED" or not bool(turret.get("home_valid", False)):
                self._set_state(
                    EngagementState.STARTUP_WAIT_HOME,
                    f"AIM_REQUEST_REJECTED:{rejected_reason}",
                )
            else:
                self._set_state(
                    EngagementState.READY_FOR_AIM,
                    f"AIM_REQUEST_REJECTED:{rejected_reason}",
                )
            return

        if turret_state in {"FAULT", "ESTOP", "HOME_REQUIRED"}:
            next_state = (
                EngagementState.STARTUP_WAIT_HOME
                if turret_state == "HOME_REQUIRED"
                else EngagementState.FAULT
            )
            self._set_state(next_state, str(turret.get("last_error", turret_state)))
            return

        # The most recent status can still be the pre-request HOME_READY
        # snapshot until the turret callback accepts or rejects the request.
        # Rejections are handled through the persistent rejection sequence;
        # therefore HOME_READY here is a wait condition, not a failure.
        if turret_state == "HOME_READY" and not bool(turret.get("busy", False)):
            self.reason = "WAITING_FOR_TURRET_AIM_ACCEPT"
            return

        if turret_state == "AIM_READY":
            if self._turret_accept_seq("AIM") <= self.locked_aim_accept_seq:
                self.reason = "WAITING_FOR_TURRET_AIM_ACCEPT"
                return
            if not self._turret_matches_locked_aim(turret):
                self._set_state(
                    EngagementState.READY_FOR_AIM,
                    "TURRET_AIM_RESULT_MISMATCH",
                )
                return
            if bool(turret.get("aim_complete", False)):
                self._set_state(EngagementState.FINAL_AIM_VERIFY, "TURRET_AIM_COMPLETE")

    def _step_final_aim_verify(self, now: float) -> None:
        if self.target_changed_during_atomic_action:
            self._clear_locked_solution()
            self._set_state(EngagementState.TARGET_CHECK, "TARGET_CHANGED_DURING_AIM")
            return

        if not self._is_fresh(self.turret_rx_time, self.cfg.max_turret_status_age_sec, now):
            self.reason = "TURRET_STATUS_STALE"
            return

        turret = self.turret or {}
        turret_state = str(turret.get("state", "")).upper()
        if turret_state in {"FAULT", "ESTOP", "HOME_REQUIRED"}:
            next_state = (
                EngagementState.STARTUP_WAIT_HOME
                if turret_state == "HOME_REQUIRED"
                else EngagementState.FAULT
            )
            self._set_state(next_state, str(turret.get("last_error", turret_state)))
            return
        if turret_state == "HOME_READY":
            self._set_state(EngagementState.READY_FOR_AIM, "TURRET_READY_LOST_REAIM")
            return
        if turret_state != "AIM_READY":
            self._set_state(EngagementState.READY_FOR_AIM, "TURRET_NOT_AIM_READY_REAIM")
            return
        if self._turret_accept_seq("AIM") <= self.locked_aim_accept_seq:
            self._set_state(EngagementState.READY_FOR_AIM, "TURRET_AIM_ACCEPT_NOT_CONFIRMED")
            return
        if not self._turret_matches_locked_aim(turret):
            self._set_state(EngagementState.READY_FOR_AIM, "TURRET_AIM_RESULT_MISMATCH")
            return

        result, reason = self._evaluate_aligned_ready(now)
        if result is None:
            self._route_invalid_aim(reason)
            return

        if self.active_target_key != self.locked_target_key:
            self._clear_locked_solution()
            self._set_state(EngagementState.TARGET_CHECK, "TARGET_CHANGED")
            return

        if self.locked_elevation_deg is None:
            self._set_state(EngagementState.FAULT, "LOCKED_ELEVATION_MISSING")
            return

        elevation_delta = abs(result.corrected_elevation_deg - self.locked_elevation_deg)
        if elevation_delta > self.cfg.final_elevation_tolerance_deg:
            self.corrected_result = result
            self._set_state(EngagementState.READY_FOR_AIM, "ELEVATION_CHANGED_REAIM")
            return

        if not (
            bool(turret.get("home_valid", False))
            and bool(turret.get("aim_complete", False))
            and bool(turret.get("fire_ready", False))
        ):
            self._set_state(EngagementState.READY_FOR_AIM, "TURRET_READY_LOST_REAIM")
            return

        self.corrected_result = result
        self._set_state(EngagementState.AIM_READY, "HOLD_Y_TO_FIRE")

    def _step_aim_ready(self, now: float) -> None:
        result, reason = self._evaluate_fire_ready(now)
        if result is None:
            self._route_invalid_aim(reason)
            return
        self.corrected_result = result

        if self.y_pressed:
            self.fire_hold_started_at = now
            self.fire_hold_progress = 0.0
            self._set_state(EngagementState.FIRE_CONFIRMING, "FIRE_HOLD_STARTED")

    def _step_fire_confirming(
        self,
        now: float,
        actions: list[CoordinatorAction],
    ) -> None:
        if not self.y_pressed:
            self.fire_hold_started_at = None
            self.fire_hold_progress = 0.0
            self._set_state(EngagementState.AIM_READY, "FIRE_HOLD_CANCELLED")
            return

        result, reason = self._evaluate_fire_ready(now)
        if result is None:
            self.fire_hold_started_at = None
            self.fire_hold_progress = 0.0
            self._route_invalid_aim(reason)
            return

        self.corrected_result = result
        if self.fire_hold_started_at is None:
            self.fire_hold_started_at = now
        elapsed = max(0.0, now - self.fire_hold_started_at)
        self.fire_hold_progress = min(1.0, elapsed / max(1e-6, self.cfg.fire_hold_sec))
        self.reason = "HOLD_TO_CONFIRM_FIRE"

        if elapsed < self.cfg.fire_hold_sec:
            return

        self.locked_fire_rejection_seq = self._turret_rejection_seq()
        self.locked_fire_accept_seq = self._turret_accept_seq("FIRE")
        actions.append(
            CoordinatorAction(
                "fire_confirm",
                {
                    "command": "FIRE",
                    "target_key": self.locked_target_key,
                    "solution_revision": self.locked_solution_revision,
                    "source": "aim_coordinator",
                },
            )
        )
        self.last_action = "FIRE_CONFIRM"
        self.fire_hold_started_at = None
        self.fire_hold_progress = 1.0
        self._set_state(EngagementState.FIRING, "FIRE_REQUESTED")

    def _step_firing(self) -> None:
        turret = self.turret or {}
        turret_state = str(turret.get("state", "")).upper()

        rejected_reason = self._matching_new_rejection(
            "FIRE",
            self.locked_fire_rejection_seq,
        )
        if rejected_reason:
            if rejected_reason == "ESTOP_LATCHED" or turret_state == "ESTOP":
                self._set_state(EngagementState.FAULT, "FIRE_REQUEST_REJECTED:ESTOP_LATCHED")
            elif turret_state == "HOME_REQUIRED" or not bool(turret.get("home_valid", False)):
                self._set_state(
                    EngagementState.STARTUP_WAIT_HOME,
                    f"FIRE_REQUEST_REJECTED:{rejected_reason}",
                )
            elif self._turret_matches_locked_aim(turret) and bool(turret.get("fire_ready", False)):
                self._set_state(
                    EngagementState.AIM_READY,
                    f"FIRE_REQUEST_REJECTED:{rejected_reason}",
                )
            else:
                self._set_state(
                    EngagementState.READY_FOR_AIM,
                    f"FIRE_REQUEST_REJECTED:{rejected_reason}",
                )
            return

        if turret_state == "FIRED":
            if self._turret_accept_seq("FIRE") <= self.locked_fire_accept_seq:
                self.reason = "WAITING_FOR_TURRET_FIRE_ACCEPT"
                return
            self._set_state(EngagementState.FIRED, "FIRE_COMPLETE")
        elif turret_state in {"FAULT", "ESTOP"}:
            self._set_state(
                EngagementState.FAULT,
                str(turret.get("last_error", turret_state)),
            )
        elif turret_state == "HOME_REQUIRED":
            self._set_state(
                EngagementState.STARTUP_WAIT_HOME,
                str(turret.get("last_error") or "HOME_REQUIRED_DURING_FIRE"),
            )
        elif turret_state == "HOME_READY":
            self._set_state(EngagementState.READY_FOR_AIM, "TURRET_RETURNED_HOME_READY_DURING_FIRE")
        elif turret_state == "AIM_READY":
            # The fire request may not have been consumed by the turret callback
            # yet. A real rejection is detected by command_rejection_seq.
            self.reason = "WAITING_FOR_TURRET_FIRE_ACCEPT"

    def _step_fired(self, actions: list[CoordinatorAction]) -> None:
        """Finish one engagement only after the turret returns HOME.

        The turret keeps ``FIRED`` visible briefly, then performs automatic
        HOME.  During that movement the coordinator intentionally remains in
        FIRED so a residual solution cannot restart alignment.
        """
        turret = self.turret or {}
        turret_state = str(turret.get("state", "")).upper()

        if turret_state in {"FAULT", "ESTOP"}:
            self._set_state(
                EngagementState.FAULT,
                str(turret.get("last_error") or turret_state),
            )
            return

        if turret_state == "HOME_REQUIRED":
            self._set_state(
                EngagementState.STARTUP_WAIT_HOME,
                str(turret.get("last_error") or "POST_FIRE_HOME_REQUIRED"),
            )
            return

        if turret_state in {"HOMING", "STARTUP_HOMING"}:
            self.reason = "POST_FIRE_AUTO_HOMING"
            return

        if (
            turret_state == "HOME_READY"
            and self._turret_home_ready(self.turret_rx_time or 0.0)
        ):
            completed_target = self.locked_target_key or self.active_target_key

            if self._next_target_received_during_post_fire:
                # 발사한 표적과 다음 표적을 섞지 않는다. 포신이 HOME_READY가 된
                # 시점에만 버퍼링된 최신 표적을 정식 다음 교전으로 활성화한다.
                self._next_target_received_during_post_fire = False
                self._completed_solution_signature = None
                self._awaiting_reengagement = False
                self.corrected_result = None
                self._clear_locked_solution()
                self.last_action = "ACTIVATE_NEXT_TARGET_AFTER_FIRE"
                self._set_state(EngagementState.TARGET_CHECK, "NEXT_TARGET")
                return

            self._completed_solution_signature = self._solution_signature
            self.corrected_result = None
            self._clear_locked_solution()

            if self.cfg.retain_target_after_fire:
                # 표적/탄도해는 GUI 표시와 동일 좌표 재교전을 위해 유지한다.
                # IDLE에서는 A rising edge 전까지 어떠한 자동 정렬도 시작하지 않는다.
                self._awaiting_reengagement = True
                self._next_target_received_during_post_fire = False
                self.last_action = "RETAIN_TARGET_AFTER_FIRE"
                self._set_state(
                    EngagementState.IDLE,
                    "FIRE_COMPLETE_TARGET_RETAINED",
                )
                return

            actions.append(
                CoordinatorAction(
                    "clear_target",
                    {
                        "clear_target": True,
                        "active": False,
                        "status": "CLEARED",
                        "source": "aim_coordinator_post_fire",
                        "target_key": completed_target,
                    },
                )
            )
            self.last_action = "CLEAR_TARGET_AFTER_FIRE"
            self._awaiting_reengagement = False
            self._next_target_received_during_post_fire = False
            self.solution = None
            self.solution_rx_time = None
            self._solution_signature = None
            self.active_target_key = ""
            self._set_state(EngagementState.IDLE, "FIRE_COMPLETE_HOME_READY")
            return

        self.reason = "FIRE_COMPLETE_WAITING_AUTO_HOME"

    def _step_fault(self, now: float) -> None:
        if self._turret_home_ready(now):
            next_state = EngagementState.TARGET_CHECK if self.solution else EngagementState.IDLE
            self._set_state(next_state, "FAULT_CLEARED_HOME_VALID")

    # ------------------------------------------------------------------
    # Validation helpers
    # ------------------------------------------------------------------
    def _imu_is_fresh_motion_state(self, now: float) -> bool:
        """Return true only for a fresh, within-limit IMU moving indication."""
        if self.imu is None or not self._is_fresh(
            self.imu_rx_time,
            self.cfg.max_imu_age_sec,
            now,
        ):
            return False
        if self.imu.get("stationary") is not False:
            return False
        if self.imu.get("roll_ok") is False or self.imu.get("pitch_ok") is False:
            return False
        roll = self.imu.get("roll_deg", self.imu.get("roll_used_deg"))
        pitch = self.imu.get("pitch_deg", self.imu.get("pitch_used_deg"))
        return self._finite(roll) and self._finite(pitch)

    def _evaluate_raw_alignment_yaw(
        self,
        now: float,
    ) -> tuple[Optional[float], str]:
        """Return fresh GNSS/body-heading yaw without stationary IMU gating.

        This helper is used only while the drivetrain is moving. Final alignment,
        aim, and fire readiness still pass through ``_evaluate_current_aim`` and
        therefore retain the stationary-IMU and drive-stop interlocks.
        """
        if self.solution is None:
            return None, "NO_SOLUTION"
        if not self._is_fresh(self.solution_rx_time, self.cfg.max_solution_age_sec, now):
            return None, "SOLUTION_STALE"
        if not bool(self.solution.get("in_range", False)):
            return None, "OUT_OF_RANGE"

        yaw = self.solution.get("turret_yaw_deg")
        elevation = self.solution.get("elevation_deg")
        if not self._finite(yaw) or not self._finite(elevation):
            return None, "INVALID_FIRE_SOLUTION"
        return self._normalize_signed_deg(float(yaw)), "OK"

    def _evaluate_current_aim(
        self,
        now: float,
    ) -> tuple[Optional[AimCorrectionResult], str]:
        if self.solution is None:
            return None, "NO_SOLUTION"
        if not self._is_fresh(self.solution_rx_time, self.cfg.max_solution_age_sec, now):
            return None, "SOLUTION_STALE"
        if not bool(self.solution.get("in_range", False)):
            return None, "OUT_OF_RANGE"

        yaw = self.solution.get("turret_yaw_deg")
        elevation = self.solution.get("elevation_deg")
        if not self._finite(yaw) or not self._finite(elevation):
            return None, "INVALID_FIRE_SOLUTION"

        if self.imu is None:
            return None, "NO_IMU"
        if not self._is_fresh(self.imu_rx_time, self.cfg.max_imu_age_sec, now):
            return None, "IMU_STALE"
        if not self._imu_stable(now):
            return None, "IMU_NOT_STABLE"

        roll = self.imu.get("roll_deg", self.imu.get("roll_used_deg"))
        pitch = self.imu.get("pitch_deg", self.imu.get("pitch_used_deg"))
        if not self._finite(roll) or not self._finite(pitch):
            return None, "INVALID_IMU_ANGLES"

        result = evaluate_corrected_aim(
            float(yaw),
            float(elevation),
            float(roll),
            float(pitch),
            correction_enabled=self.cfg.correction_enabled,
            correction_threshold_deg=self.cfg.correction_threshold_deg,
            roll_sign=self.cfg.imu_roll_sign,
            pitch_sign=self.cfg.imu_pitch_sign,
            imu_roll_limit_deg=self.cfg.imu_roll_limit_deg,
            imu_pitch_limit_deg=self.cfg.imu_pitch_limit_deg,
            elevation_min_deg=self.cfg.elevation_min_deg,
            elevation_max_deg=self.cfg.elevation_max_deg,
            mechanical_limit_tolerance_deg=self.cfg.mechanical_limit_tolerance_deg,
        )
        if not result.valid:
            if not result.elevation_in_range:
                return None, "ELEVATION_OUT_OF_RANGE"
            return None, result.reason
        return result, "OK"

    def _evaluate_aligned_ready(
        self,
        now: float,
    ) -> tuple[Optional[AimCorrectionResult], str]:
        result, reason = self._evaluate_current_aim(now)
        if result is None:
            return None, reason
        if abs(result.corrected_yaw_deg) > self.cfg.align_exit_tolerance_deg:
            return None, "YAW_OUT_OF_TOLERANCE"
        if not self._drive_stopped(now):
            return None, "DRIVE_NOT_STOPPED"
        return result, "OK"

    def _evaluate_fire_ready(
        self,
        now: float,
    ) -> tuple[Optional[AimCorrectionResult], str]:
        result, reason = self._evaluate_aligned_ready(now)
        if result is None:
            return None, reason
        if self.active_target_key != self.locked_target_key:
            return None, "TARGET_CHANGED"
        turret = self.turret or {}
        if not self._is_fresh(self.turret_rx_time, self.cfg.max_turret_status_age_sec, now):
            return None, "TURRET_STATUS_STALE"
        if not self._turret_matches_locked_aim(turret):
            return None, "TURRET_AIM_RESULT_MISMATCH"
        if not (
            bool(turret.get("home_valid", False))
            and bool(turret.get("aim_complete", False))
            and bool(turret.get("fire_ready", False))
            and str(turret.get("state", "")).upper() == "AIM_READY"
        ):
            return None, "TURRET_NOT_FIRE_READY"
        return result, "OK"

    def _imu_stable(self, now: float) -> bool:
        if self.imu is None or not self._is_fresh(
            self.imu_rx_time,
            self.cfg.max_imu_age_sec,
            now,
        ):
            return False
        if self.imu.get("tilt_ok") is False:
            return False
        if self.imu.get("imu_valid") is False:
            return False
        if self.cfg.require_imu_stationary and self.imu.get("stationary") is False:
            return False
        return True

    def _drive_stopped(self, now: float) -> bool:
        if self.drive is None or not self._is_fresh(
            self.drive_rx_time,
            self.cfg.max_drive_status_age_sec,
            now,
        ):
            return False
        if self.cfg.require_drive_feedback and not bool(
            self.drive.get("feedback_all_fresh", False)
        ):
            return False
        return bool(self.drive.get("all_stopped", False))

    def _manual_override_active(self, now: float) -> bool:
        if self.drive is None or not self._is_fresh(
            self.drive_rx_time,
            self.cfg.max_drive_status_age_sec,
            now,
        ):
            return False
        return bool(self.drive.get("manual_override_active", False))

    def _turret_home_ready(self, now: float) -> bool:
        if self.turret is None or not self._is_fresh(
            self.turret_rx_time,
            self.cfg.max_turret_status_age_sec,
            now,
        ):
            return False
        return bool(self.turret.get("home_valid", False)) and not bool(
            self.turret.get("estop_latched", False)
        )

    def _turret_rejection_seq(self) -> int:
        return self._as_int((self.turret or {}).get("command_rejection_seq"), default=0)

    def _turret_accept_seq(self, command: str) -> int:
        key = {
            "AIM": "aim_request_accept_seq",
            "FIRE": "fire_request_accept_seq",
        }.get(str(command).upper(), "")
        if not key:
            return 0
        return self._as_int((self.turret or {}).get(key), default=0)

    def _matching_new_rejection(self, command: str, baseline_seq: int) -> str:
        turret = self.turret or {}
        seq = self._as_int(turret.get("command_rejection_seq"), default=0)
        if seq <= int(baseline_seq):
            return ""
        if str(turret.get("last_rejected_command", "")).upper() != str(command).upper():
            return ""

        rejected_target = str(turret.get("last_rejection_target_key", ""))
        rejected_revision = turret.get("last_rejection_solution_revision")
        if str(command).upper() in {"AIM", "FIRE"}:
            # Only a rejection carrying the exact locked request identity may
            # affect the active sequence. Unrelated terminal/test commands are
            # ignored even when they are rejected at the same time.
            if rejected_target != self.locked_target_key:
                return ""
            if rejected_revision is None or not self._same_revision(
                rejected_revision,
                self.locked_solution_revision,
            ):
                return ""
        return str(turret.get("last_rejection_reason", "COMMAND_REJECTED"))

    def _turret_matches_locked_aim(self, turret: dict[str, Any]) -> bool:
        if self._as_int(turret.get("aim_request_accept_seq"), default=0) <= self.locked_aim_accept_seq:
            return False
        if not self.locked_target_key or self.locked_solution_revision is None:
            return False
        if str(turret.get("active_target_key", "")) != self.locked_target_key:
            return False
        if not self._same_revision(
            turret.get("active_solution_revision"),
            self.locked_solution_revision,
        ):
            return False
        if self.locked_elevation_deg is None:
            return False

        target_elevation = self._as_finite_float(turret.get("target_elevation_deg"))
        current_elevation = self._as_finite_float(turret.get("current_elevation_deg"))
        if target_elevation is None or current_elevation is None:
            return False
        tolerance = max(0.0, self.cfg.final_elevation_tolerance_deg)
        return (
            abs(target_elevation - self.locked_elevation_deg) <= tolerance
            and abs(current_elevation - self.locked_elevation_deg) <= tolerance
        )

    # ------------------------------------------------------------------
    # State/action utilities
    # ------------------------------------------------------------------
    def _route_invalid_aim(self, reason: str) -> None:
        if reason in {"OUT_OF_RANGE", "ELEVATION_OUT_OF_RANGE"}:
            self._enter_reposition(reason)
        elif reason in {"YAW_OUT_OF_TOLERANCE", "DRIVE_NOT_STOPPED"}:
            self._set_state(EngagementState.WAIT_ALIGN, reason)
        elif reason == "TARGET_CHANGED":
            self._set_state(EngagementState.TARGET_CHECK, reason)
        elif reason in {"TURRET_AIM_RESULT_MISMATCH", "TURRET_NOT_FIRE_READY"}:
            self._set_state(EngagementState.READY_FOR_AIM, reason)
        else:
            self.reason = reason

    def _enter_reposition(self, reason: str) -> None:
        self._reposition_entry_revision = self.solution_revision
        self._clear_locked_solution()
        self._set_state(EngagementState.REPOSITION_REQUIRED, reason)

    def _set_state(self, state: EngagementState, reason: str) -> None:
        if state != self.state:
            if state != EngagementState.AIM_CONFIRMING:
                self.aim_hold_started_at = None
                if state not in {EngagementState.TURRET_AIMING, EngagementState.FINAL_AIM_VERIFY}:
                    self.aim_hold_progress = 0.0
            if state != EngagementState.FIRE_CONFIRMING:
                self.fire_hold_started_at = None
                if state not in {EngagementState.FIRING, EngagementState.FIRED}:
                    self.fire_hold_progress = 0.0
        self.state = state
        self.reason = reason

    def _stop_align(self, actions: list[CoordinatorAction]) -> None:
        actions.append(
            CoordinatorAction(
                "align_cmd",
                {"active": False, "source": "aim_coordinator"},
            )
        )
        self.last_action = "ALIGN_STOP"

    def _pivot_rpm(self, abs_error_deg: float) -> float:
        if abs_error_deg > self.cfg.align_fast_error_deg:
            return self.cfg.align_fast_rpm
        if abs_error_deg > self.cfg.align_slow_error_deg:
            return self.cfg.align_medium_rpm
        return self.cfg.align_slow_rpm

    def _clear_locked_solution(self) -> None:
        self.locked_target_key = ""
        self.locked_solution_revision = None
        self.locked_elevation_deg = None
        self.locked_yaw_deg = None
        self.locked_aim_rejection_seq = self._turret_rejection_seq()
        self.locked_fire_rejection_seq = self._turret_rejection_seq()
        self.locked_aim_accept_seq = self._turret_accept_seq("AIM")
        self.locked_fire_accept_seq = self._turret_accept_seq("FIRE")
        self.target_changed_during_atomic_action = False


    @staticmethod
    def _as_int(value: Any, default: int = 0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return int(default)

    @staticmethod
    def _same_revision(left: Any, right: Any) -> bool:
        if left is None or right is None:
            return False
        try:
            return int(left) == int(right)
        except (TypeError, ValueError):
            return str(left) == str(right)

    @staticmethod
    def _as_finite_float(value: Any) -> Optional[float]:
        try:
            value = float(value)
            return value if math.isfinite(value) else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _normalize_heading_deg(value: float) -> float:
        return float(value) % 360.0

    @staticmethod
    def _normalize_signed_deg(value: float) -> float:
        return ((float(value) + 180.0) % 360.0) - 180.0

    @staticmethod
    def _finite(value: Any) -> bool:
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _is_fresh(rx_time: Optional[float], max_age: float, now: float) -> bool:
        return rx_time is not None and 0.0 <= now - rx_time <= max(0.0, max_age)

    @staticmethod
    def _make_target_key(payload: dict[str, Any]) -> str:
        target_id = payload.get("target_id")
        if target_id not in {None, ""}:
            return f"id:{target_id}"

        lat = payload.get("target_lat_deg")
        lon = payload.get("target_lon_deg")
        z = payload.get("target_z_m")
        try:
            if math.isfinite(float(lat)) and math.isfinite(float(lon)):
                z_text = "nan" if z is None else f"{float(z):.2f}"
                return f"geo:{float(lat):.7f},{float(lon):.7f},{z_text}"
        except (TypeError, ValueError):
            pass

        seq = payload.get("target_seq")
        if seq not in {None, ""}:
            return f"seq:{seq}"
        return "target:anonymous"

    @staticmethod
    def _make_solution_signature(
        payload: dict[str, Any],
        target_key: str,
    ) -> tuple[Any, ...]:
        def rounded(name: str, digits: int = 4) -> Any:
            value = payload.get(name)
            try:
                value = float(value)
                return round(value, digits) if math.isfinite(value) else str(value)
            except (TypeError, ValueError):
                return value

        return (
            target_key,
            bool(payload.get("in_range", False)),
            rounded("turret_yaw_deg"),
            rounded("elevation_deg"),
            rounded("horizontal_dist_m", 3),
            str(payload.get("solution_type", "")),
        )

    # ------------------------------------------------------------------
    # Status snapshot
    # ------------------------------------------------------------------
    def status(self, now: float) -> dict[str, Any]:
        result = self.corrected_result
        solution = self.solution or {}

        current_heading_deg = self._as_finite_float(solution.get("mmr_heading_deg"))
        raw_yaw_error_deg = self._as_finite_float(solution.get("turret_yaw_deg"))
        corrected_yaw_deg = (
            result.corrected_yaw_deg if result is not None else raw_yaw_error_deg
        )
        corrected_elevation_deg = (
            result.corrected_elevation_deg if result is not None
            else self._as_finite_float(solution.get("elevation_deg"))
        )

        target_heading_deg = None
        heading_error_deg = None
        if current_heading_deg is not None and corrected_yaw_deg is not None:
            heading_error_deg = self._normalize_signed_deg(corrected_yaw_deg)
            target_heading_deg = self._normalize_heading_deg(
                current_heading_deg + heading_error_deg
            )

        return {
            "state": self.state.value,
            "reason": self.reason,
            "active_target_key": self.active_target_key,
            "solution_revision": self.solution_revision,
            "target_retained": bool(self.solution is not None),
            "awaiting_reengagement": bool(self._awaiting_reengagement),
            "next_target_buffered_during_post_fire": bool(
                self._next_target_received_during_post_fire
            ),
            "locked_target_key": self.locked_target_key,
            "locked_solution_revision": self.locked_solution_revision,
            "locked_elevation_deg": self.locked_elevation_deg,
            "current_heading_deg": current_heading_deg,
            "target_heading_deg": target_heading_deg,
            "heading_error_deg": heading_error_deg,
            "raw_yaw_error_deg": raw_yaw_error_deg,
            "heading_tolerance_deg": self.cfg.align_exit_tolerance_deg,
            "heading_within_tolerance": (
                abs(heading_error_deg) <= self.cfg.align_exit_tolerance_deg
                if heading_error_deg is not None else None
            ),
            "corrected_yaw_deg": corrected_yaw_deg,
            "corrected_elevation_deg": corrected_elevation_deg,
            "aim_hold_progress": round(self.aim_hold_progress, 4),
            "fire_hold_progress": round(self.fire_hold_progress, 4),
            "a_pressed": self.a_pressed,
            "y_pressed": self.y_pressed,
            "drive_stopped": self._drive_stopped(float(now)),
            "imu_stable": self._imu_stable(float(now)),
            "turret_home_valid": bool((self.turret or {}).get("home_valid", False)),
            "turret_fire_ready": bool((self.turret or {}).get("fire_ready", False)),
            "last_action": self.last_action,
            "solution_fresh": self._is_fresh(
                self.solution_rx_time,
                self.cfg.max_solution_age_sec,
                float(now),
            ),
            "imu_fresh": self._is_fresh(
                self.imu_rx_time,
                self.cfg.max_imu_age_sec,
                float(now),
            ),
            "drive_status_fresh": self._is_fresh(
                self.drive_rx_time,
                self.cfg.max_drive_status_age_sec,
                float(now),
            ),
            "turret_status_fresh": self._is_fresh(
                self.turret_rx_time,
                self.cfg.max_turret_status_age_sec,
                float(now),
            ),
            "config": asdict(self.cfg),
        }
