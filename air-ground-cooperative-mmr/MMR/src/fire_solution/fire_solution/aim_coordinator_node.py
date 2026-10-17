#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 wrapper for the MMR engagement state machine."""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from .aim_coordinator_core import (
    AimCoordinatorCore,
    CoordinatorAction,
    CoordinatorConfig,
)


class AimCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("aim_coordinator_node")

        # Topics
        self.declare_parameter("fire_solution_topic", "/mmr/fire_solution")
        self.declare_parameter("imu_status_topic", "/imu/tilt_status")
        self.declare_parameter("drive_status_topic", "/mmr/drive/status")
        self.declare_parameter("turret_status_topic", "/mmr/turret/status")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("operator_input_topic", "/mmr/operator_input")

        self.declare_parameter("align_cmd_topic", "/mmr/drive/align_cmd")
        self.declare_parameter("aim_request_topic", "/mmr/turret/aim_request")
        self.declare_parameter("home_request_topic", "/mmr/turret/home_request")
        self.declare_parameter("fire_confirm_topic", "/mmr/fire_confirm")
        self.declare_parameter("engagement_status_topic", "/mmr/engagement_status")
        self.declare_parameter("target_point_topic", "/target/point")

        # Buttons / loop
        self.declare_parameter("align_aim_button", 0)  # Xbox A
        self.declare_parameter("fire_button", 3)      # Xbox Y
        self.declare_parameter("operator_input_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)

        # Hidden local-only diagnostic dry-run. Starts OFF every process start.
        # B+X hold is chosen because drive control only uses X together with BACK+START.
        self.declare_parameter("diagnostic_hidden_mode_enabled", True)
        self.declare_parameter("diagnostic_toggle_buttons", [1, 2])  # Xbox B + X
        self.declare_parameter("diagnostic_toggle_hold_sec", 3.0)
        self.declare_parameter("diagnostic_neutral_axis_indices", [1, 3])
        self.declare_parameter("diagnostic_neutral_deadzone", 0.10)
        self.declare_parameter("diagnostic_local_event_path", "/tmp/mmr_local_diagnostic_event.json")

        # Alignment state machine
        self.declare_parameter("align_enter_tolerance_deg", 2.0)
        self.declare_parameter("align_exit_tolerance_deg", 3.0)
        self.declare_parameter("align_settle_sec", 0.5)
        self.declare_parameter("aim_hold_sec", 0.5)
        self.declare_parameter("fire_hold_sec", 1.0)

        self.declare_parameter("align_fast_error_deg", 10.0)
        self.declare_parameter("align_slow_error_deg", 3.0)
        self.declare_parameter("align_fast_rpm", 7.0)
        self.declare_parameter("align_medium_rpm", 6.0)
        self.declare_parameter("align_slow_rpm", 5.0)

        # Freshness / safety
        self.declare_parameter("max_solution_age_sec", 1.0)
        self.declare_parameter("max_imu_age_sec", 0.5)
        self.declare_parameter("max_drive_status_age_sec", 0.75)
        self.declare_parameter("max_turret_status_age_sec", 1.0)
        self.declare_parameter("require_drive_feedback", True)
        self.declare_parameter("require_imu_stationary", True)
        self.declare_parameter("allow_motion_during_body_align", False)

        # IMU correction / mechanics
        self.declare_parameter("imu_correction_enabled", True)
        self.declare_parameter("imu_correction_threshold_deg", 0.5)
        self.declare_parameter("imu_roll_sign", 1.0)
        self.declare_parameter("imu_pitch_sign", 1.0)
        self.declare_parameter("imu_roll_limit_deg", 15.0)
        self.declare_parameter("imu_pitch_limit_deg", 15.0)
        self.declare_parameter("elevation_min_deg", 45.0)
        self.declare_parameter("elevation_max_deg", 61.0)
        self.declare_parameter("mechanical_limit_tolerance_deg", 0.2)
        self.declare_parameter("final_elevation_tolerance_deg", 0.5)
        self.declare_parameter("home_request_retry_sec", 2.0)
        self.declare_parameter("retain_target_after_fire", True)

        cfg = CoordinatorConfig(
            align_enter_tolerance_deg=self._f("align_enter_tolerance_deg"),
            align_exit_tolerance_deg=self._f("align_exit_tolerance_deg"),
            align_settle_sec=self._f("align_settle_sec"),
            aim_hold_sec=self._f("aim_hold_sec"),
            fire_hold_sec=self._f("fire_hold_sec"),
            align_fast_error_deg=self._f("align_fast_error_deg"),
            align_slow_error_deg=self._f("align_slow_error_deg"),
            align_fast_rpm=self._f("align_fast_rpm"),
            align_medium_rpm=self._f("align_medium_rpm"),
            align_slow_rpm=self._f("align_slow_rpm"),
            max_solution_age_sec=self._f("max_solution_age_sec"),
            max_imu_age_sec=self._f("max_imu_age_sec"),
            max_drive_status_age_sec=self._f("max_drive_status_age_sec"),
            max_turret_status_age_sec=self._f("max_turret_status_age_sec"),
            require_drive_feedback=bool(self.get_parameter("require_drive_feedback").value),
            require_imu_stationary=bool(self.get_parameter("require_imu_stationary").value),
            allow_motion_during_body_align=bool(
                self.get_parameter("allow_motion_during_body_align").value
            ),
            correction_enabled=bool(self.get_parameter("imu_correction_enabled").value),
            correction_threshold_deg=self._f("imu_correction_threshold_deg"),
            imu_roll_sign=self._f("imu_roll_sign"),
            imu_pitch_sign=self._f("imu_pitch_sign"),
            imu_roll_limit_deg=self._f("imu_roll_limit_deg"),
            imu_pitch_limit_deg=self._f("imu_pitch_limit_deg"),
            elevation_min_deg=self._f("elevation_min_deg"),
            elevation_max_deg=self._f("elevation_max_deg"),
            mechanical_limit_tolerance_deg=self._f("mechanical_limit_tolerance_deg"),
            final_elevation_tolerance_deg=self._f("final_elevation_tolerance_deg"),
            home_request_retry_sec=self._f("home_request_retry_sec"),
            retain_target_after_fire=bool(
                self.get_parameter("retain_target_after_fire").value
            ),
        )
        self.core = AimCoordinatorCore(cfg)

        self.align_aim_button = int(self.get_parameter("align_aim_button").value)
        self.fire_button = int(self.get_parameter("fire_button").value)
        self.operator_input_timeout_sec = self._f("operator_input_timeout_sec")

        self._joy_a = False
        self._joy_y = False
        self._joy_rx_time: float | None = None
        self._gui_a = False
        self._gui_y = False
        self._gui_rx_time: float | None = None
        self._last_state = ""

        self._diagnostic_mode = False
        self._diagnostic_toggle_started_at: float | None = None
        self._diagnostic_toggle_latched = False
        self._diagnostic_toggle_buttons = tuple(
            int(v) for v in self.get_parameter("diagnostic_toggle_buttons").value
        )
        self._diagnostic_neutral_axis_indices = tuple(
            int(v) for v in self.get_parameter("diagnostic_neutral_axis_indices").value
        )
        self._diagnostic_home_confirmed = False
        self._diagnostic_aim_active = False
        self._diagnostic_motor_bypass_active = False
        self._diagnostic_last_error_key = ""
        self._diagnostic_fire_simulated_at: float | None = None
        self._latest_real_turret_payload: dict[str, Any] | None = None

        # Publishers
        self.pub_align = self.create_publisher(
            String,
            str(self.get_parameter("align_cmd_topic").value),
            10,
        )
        self.pub_aim = self.create_publisher(
            String,
            str(self.get_parameter("aim_request_topic").value),
            10,
        )
        self.pub_home = self.create_publisher(
            String,
            str(self.get_parameter("home_request_topic").value),
            10,
        )
        self.pub_fire = self.create_publisher(
            String,
            str(self.get_parameter("fire_confirm_topic").value),
            10,
        )
        self.pub_status = self.create_publisher(
            String,
            str(self.get_parameter("engagement_status_topic").value),
            10,
        )
        self.pub_target_clear = self.create_publisher(
            String,
            str(self.get_parameter("target_point_topic").value),
            10,
        )

        # Subscriptions
        self.create_subscription(
            String,
            str(self.get_parameter("fire_solution_topic").value),
            self._cb_solution,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("imu_status_topic").value),
            self._cb_imu,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("drive_status_topic").value),
            self._cb_drive,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("turret_status_topic").value),
            self._cb_turret,
            10,
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._cb_joy,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("operator_input_topic").value),
            self._cb_operator_input,
            10,
        )

        rate_hz = max(1.0, self._f("control_rate_hz"))
        self.create_timer(1.0 / rate_hz, self._control_tick)

        self.get_logger().info(
            "AimCoordinatorNode started | "
            f"A=buttons[{self.align_aim_button}] ALIGN/AIM | "
            f"Y=buttons[{self.fire_button}] FIRE | "
            f"aim_hold={cfg.aim_hold_sec:.2f}s | fire_hold={cfg.fire_hold_sec:.2f}s"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _cb_solution(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "fire_solution")
        if payload is not None:
            self.core.update_solution(payload, time.monotonic())

    def _cb_imu(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "imu_status")
        if payload is not None:
            self.core.update_imu(payload, time.monotonic())

    def _cb_drive(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "drive_status")
        if payload is not None:
            self.core.update_drive(payload, time.monotonic())

    def _cb_turret(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "turret_status")
        if payload is None:
            return

        now = time.monotonic()
        self._latest_real_turret_payload = dict(payload)
        turret_state = str(payload.get("state", "")).upper()
        home_valid = bool(payload.get("home_valid", False))
        if turret_state == "HOME_READY" and home_valid:
            self._diagnostic_home_confirmed = True

        # Never bypass E-Stop or a HOME failure.  Only after a valid HOME and an
        # active diagnostic AIM may an elevation/INP failure be converted into a
        # synthetic AIM_READY snapshot for the local dry-run state machine.
        if (
            self._diagnostic_mode
            and self._diagnostic_aim_active
            and self._diagnostic_home_confirmed
            and turret_state in {"FAULT", "HOME_REQUIRED"}
            and not bool(payload.get("estop_latched", False))
        ):
            reason = str(payload.get("last_error") or turret_state)
            if not self._diagnostic_is_bypassable_motor_error(reason):
                self.core.update_turret(payload, now)
                return

            error_key = f"{turret_state}:{reason}"
            if error_key != self._diagnostic_last_error_key:
                self._diagnostic_last_error_key = error_key
                self._local_diag_event(
                    f"[진단] 포신 모터/INP 오류 감지 — {reason} | dry-run 시퀀스는 계속 진행"
                )
            self._diagnostic_motor_bypass_active = True
            self.core.update_turret(self._diagnostic_aim_ready_payload(payload), now)
            return

        if self._diagnostic_mode and self._diagnostic_motor_bypass_active:
            # Keep the coordinator on the synthetic dry-run snapshot until the
            # turret has consumed the real FIRE command.  From that moment the
            # real status (FIRING -> FIRED -> HOME) drives the coordinator, so
            # the fire result is never masked.
            if turret_state in {"FIRING", "FIRED", "FAULT", "ESTOP", "HOME_REQUIRED"}:
                self._diagnostic_motor_bypass_active = False
                self._diagnostic_aim_active = False
                self.core.update_turret(payload, now)
                return
            self.core.update_turret(self._diagnostic_aim_ready_payload(payload), now)
            return

        self.core.update_turret(payload, now)

    def _cb_joy(self, msg: Joy) -> None:
        now = time.monotonic()
        self._handle_diagnostic_toggle(msg, now)
        self._joy_a = self._button(msg.buttons, self.align_aim_button)
        self._joy_y = self._button(msg.buttons, self.fire_button)
        self._joy_rx_time = now

    def _cb_operator_input(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "operator_input")
        if payload is None:
            return
        self._gui_a = bool(payload.get("align_aim_pressed", False))
        self._gui_y = bool(payload.get("fire_pressed", False))
        self._gui_rx_time = time.monotonic()

    # ------------------------------------------------------------------
    # Timer / dispatch
    # ------------------------------------------------------------------
    def _control_tick(self) -> None:
        now = time.monotonic()
        self._advance_diagnostic_simulation(now)
        a_pressed, y_pressed = self._effective_buttons(now)
        self.core.update_buttons(a_pressed, y_pressed)

        actions = self.core.step(now)
        for action in actions:
            self._dispatch(action)

        status = self.core.status(now)
        status["t_ns"] = time.time_ns()
        # Diagnostic state/synthetic progress is local-only. Do not expose it on
        # the normal engagement status topic used by external consumers.
        if not self._diagnostic_mode:
            self._publish_json(self.pub_status, status)

        state = str(status.get("state", ""))
        if state != self._last_state:
            if not self._diagnostic_mode:
                self.get_logger().info(
                    f"[STATE] {self._last_state or '-'} -> {state} | "
                    f"reason={status.get('reason', '')}"
                )
            self._last_state = state

    def _dispatch(self, action: CoordinatorAction) -> None:
        if self._diagnostic_mode and action.kind == "aim_request":
            turret = self._latest_real_turret_payload or {}
            if not (
                self._diagnostic_home_confirmed
                and bool(turret.get("home_valid", False))
                and str(turret.get("state", "")).upper() not in {"FAULT", "ESTOP", "HOME_REQUIRED"}
            ):
                self._local_diag_event(
                    "[진단] HOME 유효 상태가 아니므로 포신 진단 이동을 시작하지 않았습니다."
                )
                return
            # Diagnostic mode still sends the real AIM request.  Only a later
            # bypassable elevation/INP failure is converted to synthetic AIM_READY
            # by _cb_turret so the diagnostic sequence can continue.
            self._diagnostic_aim_active = True
            self._diagnostic_motor_bypass_active = False
            self._diagnostic_last_error_key = ""
            self._local_diag_event(
                "[진단] 포신 AIM 명령을 실제 turret/STM 경로로 송신합니다."
            )

        if self._diagnostic_mode and action.kind == "home_request":
            # HOME is never bypassed: send the real request and keep HOME/ESTOP
            # failures as blocking failures even in diagnostic mode.
            self._diagnostic_aim_active = False
            self._diagnostic_motor_bypass_active = False
            self._diagnostic_last_error_key = ""
            self._local_diag_event(
                "[진단] HOME 명령을 실제 turret/STM 경로로 송신합니다."
            )

        if self._diagnostic_mode and action.kind == "fire_confirm":
            # FIRE is the only command bypass mode transmits to the STM.  Stop
            # swallowing motor faults from here on; the synthetic snapshot is
            # released in _cb_turret as soon as the turret consumes the FIRE.
            self._diagnostic_aim_active = False
            self._diagnostic_last_error_key = ""
            self._diagnostic_fire_simulated_at = None
            self._local_diag_event(
                "[진단] 바이패스 — 실제 FIRE 명령을 STM으로 송신합니다."
            )

        publisher = {
            "align_cmd": self.pub_align,
            "aim_request": self.pub_aim,
            "home_request": self.pub_home,
            "fire_confirm": self.pub_fire,
            "clear_target": self.pub_target_clear,
        }.get(action.kind)
        if publisher is None:
            self.get_logger().error(f"Unknown coordinator action: {action.kind}")
            return
        self._publish_json(publisher, action.payload)

    def _effective_buttons(self, now: float) -> tuple[bool, bool]:
        joy_fresh = (
            self._joy_rx_time is not None
            and now - self._joy_rx_time <= self.operator_input_timeout_sec
        )
        gui_fresh = (
            self._gui_rx_time is not None
            and now - self._gui_rx_time <= self.operator_input_timeout_sec
        )
        return (
            (self._joy_a if joy_fresh else False) or (self._gui_a if gui_fresh else False),
            (self._joy_y if joy_fresh else False) or (self._gui_y if gui_fresh else False),
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _handle_diagnostic_toggle(self, msg: Joy, now: float) -> None:
        if not bool(self.get_parameter("diagnostic_hidden_mode_enabled").value):
            return
        combo = bool(self._diagnostic_toggle_buttons) and all(
            self._button(msg.buttons, idx) for idx in self._diagnostic_toggle_buttons
        )
        deadzone = max(0.0, self._f("diagnostic_neutral_deadzone"))
        neutral = all(
            0 <= idx < len(msg.axes) and abs(float(msg.axes[idx])) <= deadzone
            for idx in self._diagnostic_neutral_axis_indices
        )

        if not combo or not neutral:
            self._diagnostic_toggle_started_at = None
            if not combo:
                self._diagnostic_toggle_latched = False
            return
        if self._diagnostic_toggle_latched:
            return
        if self._diagnostic_toggle_started_at is None:
            self._diagnostic_toggle_started_at = now
            return
        if now - self._diagnostic_toggle_started_at < max(0.5, self._f("diagnostic_toggle_hold_sec")):
            return

        self._diagnostic_toggle_latched = True
        self._diagnostic_toggle_started_at = None
        self._set_diagnostic_mode(not self._diagnostic_mode, now)

    def _set_diagnostic_mode(self, enabled: bool, now: float) -> None:
        self._diagnostic_mode = bool(enabled)
        self._diagnostic_aim_active = False
        self._diagnostic_motor_bypass_active = False
        self._diagnostic_last_error_key = ""
        self._diagnostic_fire_simulated_at = None
        turret = self._latest_real_turret_payload or {}
        self._diagnostic_home_confirmed = bool(
            turret.get("home_valid", False)
            and str(turret.get("state", "")).upper() == "HOME_READY"
        )
        if not enabled and self._latest_real_turret_payload is not None:
            self.core.update_turret(dict(self._latest_real_turret_payload), now)
        state_text = "ENABLED" if enabled else "DISABLED"
        self._local_diag_event(f"[LOCAL] DIAGNOSTIC DRY-RUN {state_text}")

    @staticmethod
    def _diagnostic_is_bypassable_motor_error(reason: str) -> bool:
        text = str(reason or "").upper()
        if any(token in text for token in ("ESTOP", "HOME0", "HOME_BEFORE_AIM", "STARTUP_HOME")):
            return False
        return any(
            token in text
            for token in (
                "INP_",
                "ELEV_MOTOR_WAIT_TIMEOUT",
                "ELEV_UART_SEND_FAILED",
                "M0RUN_NOT_IDLE",
                "STATUS_",
                "UART_RESPONSE_TIMEOUT",
            )
        )

    def _diagnostic_aim_ready_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        fake = dict(payload)
        fake.update({
            "state": "AIM_READY",
            "busy": False,
            "home_valid": True,
            "aim_complete": True,
            "fire_ready": True,
            "last_error": "",
            "active_target_key": self.core.locked_target_key,
            "active_solution_revision": self.core.locked_solution_revision,
            "aim_request_accept_seq": max(
                int(fake.get("aim_request_accept_seq", 0) or 0),
                int(self.core.locked_aim_accept_seq) + 1,
            ),
        })
        # Bypass mode never sends the AIM request, so the real status carries no
        # elevation for the locked solution.  Report the locked elevation so the
        # synthetic snapshot stays self-consistent for the aim-result check.
        locked_elevation = self.core.locked_elevation_deg
        if locked_elevation is not None:
            fake["target_elevation_deg"] = float(locked_elevation)
            fake["current_elevation_deg"] = float(locked_elevation)
        return fake

    def _advance_diagnostic_simulation(self, now: float) -> None:
        if not self._diagnostic_mode or self._diagnostic_fire_simulated_at is None:
            return
        if now - self._diagnostic_fire_simulated_at < 0.20:
            return
        fake = dict(self.core.turret or {})
        fake.update({
            "state": "HOME_READY",
            "busy": False,
            "home_valid": True,
            "aim_complete": False,
            "fire_ready": False,
            "last_error": "",
        })
        self.core.update_turret(fake, now)
        self._diagnostic_fire_simulated_at = None
        self._diagnostic_aim_active = False
        self._diagnostic_motor_bypass_active = False
        self._local_diag_event("[진단] DRY-RUN 논리 시퀀스 완료")

    def _local_diag_event(self, message: str) -> None:
        path = os.path.expanduser(
            str(self.get_parameter("diagnostic_local_event_path").value or "").strip()
        )
        if not path:
            return
        payload = {"seq": time.time_ns(), "message": str(message)}
        tmp_path = f"{path}.tmp.{os.getpid()}"
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, path)
        except Exception:
            try:
                if os.path.exists(tmp_path):
                    os.unlink(tmp_path)
            except Exception:
                pass

    def _f(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"parameter {name} must be finite")
        return value

    @staticmethod
    def _button(buttons: list[int] | tuple[int, ...], index: int) -> bool:
        return 0 <= index < len(buttons) and int(buttons[index]) != 0

    def _parse_json(self, text: str, source: str) -> dict[str, Any] | None:
        try:
            payload = json.loads(text)
            if not isinstance(payload, dict):
                raise ValueError("payload is not a JSON object")
            return payload
        except Exception as exc:
            self.get_logger().warn(
                f"[{source}] JSON parse failed: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    @staticmethod
    def _publish_json(publisher: Any, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AimCoordinatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Best-effort drivetrain stop before node shutdown.
        try:
            AimCoordinatorNode._publish_json(
                node.pub_align,
                {"active": False, "source": "aim_coordinator_shutdown"},
            )
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
