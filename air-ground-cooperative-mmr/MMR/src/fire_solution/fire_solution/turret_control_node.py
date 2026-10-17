#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tilt-only turret actuator node.

Role
----
- Home the tilt axis with STM command HOME0.
- Accept a locked elevation command on /mmr/turret/aim_request.
- Move the tilt axis atomically to the requested pulse position.
- Publish hardware state on /mmr/turret/status.
- Accept FIRE only after AIM_READY.
- Treat emergency interruption or uncertain motion as HOME_REQUIRED.

This node intentionally does NOT:
- calculate ballistic solutions,
- apply IMU correction,
- control a pan/yaw motor,
- read joystick buttons,
- manage the overall engagement state machine.
Those responsibilities are handled by the coordinator added in a later step.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - depends on deployment environment
    serial = None


class TurretControlNode(Node):
    """Hardware-facing tilt turret controller."""

    VALID_STATES = {
        "STARTUP",
        "STARTUP_HOMING",
        "HOME_READY",
        "HOMING",
        "TURRET_AIMING",
        "AIM_READY",
        "FIRING",
        "FIRED",
        "HOME_REQUIRED",
        "ESTOP",
        "FAULT",
        "SHUTDOWN",
    }

    def __init__(self) -> None:
        super().__init__("turret_control_node")

        # Topics
        self.declare_parameter("aim_request_topic", "/mmr/turret/aim_request")
        self.declare_parameter("home_request_topic", "/mmr/turret/home_request")
        self.declare_parameter("fire_confirm_topic", "/mmr/fire_confirm")
        self.declare_parameter("estop_topic", "/mmr/emergency_stop")
        self.declare_parameter("status_topic", "/mmr/turret/status")
        self.declare_parameter("legacy_cmd_topic", "/mmr/turret_cmd")
        self.declare_parameter("legacy_fire_ready_topic", "/mmr/fire_ready")

        # UART / STM
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.2)
        self.declare_parameter("uart_enabled", True)
        self.declare_parameter("home_on_startup", True)
        # USB-CDC STM boards may reset when the serial port is opened.
        # Wait before sending startup HOME0 so the STM firmware is fully ready.
        self.declare_parameter("startup_uart_settle_delay_sec", 2.0)
        # STM UART line parser synchronization.
        # When enabled, the node sends SYNC once after serial-open settle time
        # and waits briefly for ACK SYNC before sending the first real command.
        self.declare_parameter("uart_sync_on_startup", True)
        self.declare_parameter("uart_sync_command", "SYNC")
        self.declare_parameter("uart_sync_timeout_sec", 1.0)
        self.declare_parameter("motor_wait_timeout_sec", 8.0)
        self.declare_parameter("motor_wait_poll_sec", 0.10)
        self.declare_parameter("stop_to_fire_delay_sec", 0.05)
        self.declare_parameter("require_in_position_for_aim_ready", True)
        self.declare_parameter("require_in_position_for_fire", True)
        self.declare_parameter("in_position_stable_count", 2)

        # HOME0 robustness.  A re-home request can arrive immediately after a
        # previous command, so stop the motor and verify M0RUN=0 before HOME0.
        # A rejected HOME0 is retried only after another idle check.
        self.declare_parameter("home_prepare_stop_enabled", True)
        self.declare_parameter("home_idle_timeout_sec", 2.0)
        self.declare_parameter("home_idle_poll_sec", 0.10)
        self.declare_parameter("home_stop_settle_sec", 0.10)
        self.declare_parameter("home_command_retry_count", 1)
        self.declare_parameter("home_retry_delay_sec", 0.25)
        self.declare_parameter("home_accept_valid_status_after_reject", True)

        # Tilt geometry / calibration mapping
        # Physical HOME is 43 deg at POS=0, while normal aim commands remain 45..61 deg.
        self.declare_parameter("tilt_home_deg", 43.0)
        self.declare_parameter("tilt_command_min_deg", 45.0)
        self.declare_parameter("tilt_max_deg", 61.0)
        self.declare_parameter("tilt_max_pulse", 950)
        self.declare_parameter("tilt_mapping_mode", "lut")
        self.declare_parameter(
            "tilt_mapping_angles_deg",
            [43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0,
             52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0],
        )
        self.declare_parameter(
            "tilt_mapping_pulses",
            [0, 50, 100, 150, 200, 250, 310, 360, 420, 470,
             520, 590, 640, 690, 740, 790, 840, 900, 950],
        )
        self.declare_parameter("elevation_motor_id", 0)
        self.declare_parameter("invert_elevation_dir", False)
        self.declare_parameter("deadband_deg", 0.20)
        self.declare_parameter("mechanical_limit_tolerance_deg", 0.20)

        # Backlash / trusted-position policy
        self.declare_parameter("rehome_before_each_aim", False)
        self.declare_parameter("rehome_if_target_below_current", True)

        # Miscellaneous
        self.declare_parameter("status_publish_rate_hz", 5.0)
        self.declare_parameter("terminal_input_enabled", True)
        self.declare_parameter("allow_fire_without_aim", False)

        # Post-action recovery.  Keep the fired state visible briefly, then
        # return the tilt axis to the trusted HOME position.  E-Stop reset also
        # queues HOME automatically so normal operation cannot resume from an
        # unknown pulse position.
        self.declare_parameter("auto_home_after_fire", True)
        self.declare_parameter("post_fire_home_delay_sec", 1.5)
        self.declare_parameter("auto_home_after_estop_reset", True)

        self._tilt_mapping_mode = str(
            self.get_parameter("tilt_mapping_mode").value
        ).strip().lower()
        self._tilt_mapping_angles_deg: list[float] = []
        self._tilt_mapping_pulses: list[int] = []
        self._configure_tilt_mapping()

        # State guarded by _state_lock
        self._state_lock = threading.RLock()
        self._state = "STARTUP"
        self._busy = False
        self._home_valid = False
        self._aim_complete = False
        self._fire_ready = False
        self._current_tilt_pulse = 0
        self._target_tilt_pulse: Optional[int] = None
        self._target_elevation_deg: Optional[float] = None
        self._active_target_key = ""
        self._active_solution_revision: Any = None
        self._last_error = ""
        self._last_command = ""
        self._command_rejection_seq = 0
        self._aim_request_accept_seq = 0
        self._fire_request_accept_seq = 0
        self._last_rejected_command = ""
        self._last_rejection_reason = ""
        self._last_rejection_target_key = ""
        self._last_rejection_solution_revision: Any = None
        self._aim_count_since_home = 0
        self._last_stm_status: dict[str, Any] = {}
        self._last_stm_status_time_sec = 0.0

        self._command_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        self._shutdown_event = threading.Event()
        self._estop_event = threading.Event()
        self._serial_lock = threading.Lock()
        self.ser = None

        status_topic = str(self.get_parameter("status_topic").value)
        legacy_cmd_topic = str(self.get_parameter("legacy_cmd_topic").value)
        legacy_ready_topic = str(self.get_parameter("legacy_fire_ready_topic").value)

        self.pub_status = self.create_publisher(String, status_topic, 10)
        self.pub_legacy_cmd = self.create_publisher(String, legacy_cmd_topic, 10)
        self.pub_legacy_fire_ready = self.create_publisher(String, legacy_ready_topic, 10)

        self.sub_aim = self.create_subscription(
            String,
            str(self.get_parameter("aim_request_topic").value),
            self._cb_aim_request,
            10,
        )
        self.sub_home = self.create_subscription(
            String,
            str(self.get_parameter("home_request_topic").value),
            self._cb_home_request,
            10,
        )
        self.sub_fire = self.create_subscription(
            String,
            str(self.get_parameter("fire_confirm_topic").value),
            self._cb_fire_confirm,
            10,
        )
        self.sub_estop = self.create_subscription(
            String,
            str(self.get_parameter("estop_topic").value),
            self._cb_estop,
            10,
        )

        self._open_serial_if_enabled()
        self._startup_uart_settle_delay()

        rate_hz = max(0.5, float(self.get_parameter("status_publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish_status)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        if bool(self.get_parameter("terminal_input_enabled").value):
            threading.Thread(target=self._terminal_input_loop, daemon=True).start()

        if bool(self.get_parameter("home_on_startup").value):
            self._set_state(
                "STARTUP_HOMING",
                busy=True,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_command="HOME_QUEUED",
            )
            self._enqueue({"type": "HOME", "source": "startup", "startup": True})
        else:
            self._set_state(
                "HOME_REQUIRED",
                busy=False,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_error="STARTUP_HOME_DISABLED",
            )

        self.get_logger().info(
            "TurretControlNode started | tilt-only actuator | "
            f"aim={self.get_parameter('aim_request_topic').value} | "
            f"status={status_topic} | "
            f"uart={self.get_parameter('uart_enabled').value} | "
            f"port={self.get_parameter('serial_port').value}"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _cb_aim_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("aim_request must be a JSON object")
        except Exception as exc:
            self._reject(
                "AIM_REQUEST_PARSE_ERROR",
                {"message": str(exc), "rejected_command": "AIM"},
            )
            return

        aim_reject_meta = {
            "rejected_command": "AIM",
            "rejected_target_key": str(payload.get("target_key", "")),
            "rejected_solution_revision": payload.get("solution_revision"),
            "request": payload,
        }

        elevation = payload.get(
            "elevation_deg",
            payload.get("corrected_elevation_deg", payload.get("target_elevation_deg")),
        )
        if not self._is_finite(elevation):
            self._reject("INVALID_ELEVATION", aim_reject_meta)
            return

        elevation = float(elevation)
        min_deg = float(self.get_parameter("tilt_command_min_deg").value)
        max_deg = float(self.get_parameter("tilt_max_deg").value)
        tol = float(self.get_parameter("mechanical_limit_tolerance_deg").value)
        if elevation < (min_deg - tol) or elevation > (max_deg + tol):
            self._reject(
                "ELEVATION_LIMIT_EXCEEDED",
                {
                    **aim_reject_meta,
                    "requested_elevation_deg": elevation,
                    "min_elevation_deg": min_deg,
                    "max_elevation_deg": max_deg,
                },
            )
            return

        with self._state_lock:
            if self._estop_event.is_set():
                self._reject_locked("ESTOP_LATCHED", aim_reject_meta)
                return
            if self._busy:
                self._reject_locked("TURRET_BUSY", aim_reject_meta)
                return

            self._busy = True
            self._aim_request_accept_seq += 1
            self._aim_complete = False
            self._fire_ready = False
            self._target_elevation_deg = elevation
            self._target_tilt_pulse = self._tilt_deg_to_abs_pulse(elevation)
            self._active_target_key = str(payload.get("target_key", ""))
            self._active_solution_revision = payload.get("solution_revision")
            self._last_error = ""
            self._last_command = "AIM_REQUEST_ACCEPTED"
            self._state = "TURRET_AIMING"

        request = dict(payload)
        request["type"] = "AIM"
        request["elevation_deg"] = elevation
        request["target_tilt_pulse"] = self._target_tilt_pulse
        self._enqueue(request)
        self._publish_status(extra={"event": "AIM_REQUEST_ACCEPTED"})

    def _cb_home_request(self, msg: String) -> None:
        cmd, payload = self._parse_command(msg.data)
        if cmd not in {"HOME", "REHOME"}:
            self._reject(
                "UNKNOWN_HOME_COMMAND",
                {"raw": msg.data, "rejected_command": "HOME"},
            )
            return

        home_reject_meta = {"rejected_command": "HOME", "request": payload}
        with self._state_lock:
            if self._estop_event.is_set():
                self._reject_locked("ESTOP_LATCHED", home_reject_meta)
                return
            if self._busy:
                self._reject_locked("TURRET_BUSY", home_reject_meta)
                return
            self._busy = True
            self._aim_complete = False
            self._fire_ready = False
            self._state = "HOMING"
            self._last_error = ""
            self._last_command = "HOME_REQUEST_ACCEPTED"

        self._enqueue({"type": "HOME", "source": payload.get("source", "topic")})
        self._publish_status(extra={"event": "HOME_REQUEST_ACCEPTED"})

    def _cb_fire_confirm(self, msg: String) -> None:
        cmd, payload = self._parse_command(msg.data)
        if cmd != "FIRE":
            self._reject(
                "UNKNOWN_FIRE_COMMAND",
                {"raw": msg.data, "rejected_command": "FIRE"},
            )
            return

        fire_reject_meta = {
            "rejected_command": "FIRE",
            "rejected_target_key": str(payload.get("target_key", "")),
            "rejected_solution_revision": payload.get("solution_revision"),
            "request": payload,
        }
        allow_without_aim = bool(self.get_parameter("allow_fire_without_aim").value)
        with self._state_lock:
            if self._estop_event.is_set():
                self._reject_locked("ESTOP_LATCHED", fire_reject_meta)
                return
            if self._busy:
                self._reject_locked("TURRET_BUSY", fire_reject_meta)
                return
            if not allow_without_aim and not self._fire_ready:
                self._reject_locked("AIM_NOT_READY", fire_reject_meta)
                return

            self._busy = True
            self._fire_request_accept_seq += 1
            self._fire_ready = False
            self._state = "FIRING"
            self._last_error = ""
            self._last_command = "FIRE_REQUEST_ACCEPTED"

        self._enqueue({"type": "FIRE", **payload})
        self._publish_status(extra={"event": "FIRE_REQUEST_ACCEPTED"})

    def _cb_estop(self, msg: String) -> None:
        cmd, payload = self._parse_command(msg.data)
        active = cmd in {"STOP", "ESTOP", "E_STOP", "TRUE", "1", "ON"}
        reset = cmd in {"RESET", "CLEAR", "FALSE", "0", "OFF"}

        if active:
            self._estop_event.set()
            self._discard_pending_commands()
            with self._state_lock:
                self._home_valid = False
                self._aim_complete = False
                self._fire_ready = False
                self._state = "ESTOP"
                self._last_error = "ESTOP_LATCHED"
                self._last_command = "ESTOP_REQUESTED"
            # The active worker notices _estop_event and sends STOP from the
            # same serial thread. This avoids concurrent UART transactions.
            self._enqueue({"type": "ESTOP", **payload})
            self._publish_status(extra={"event": "ESTOP_LATCHED"})
            return

        if reset:
            # Do not clear the latch in the ROS callback.  RESET is serialized
            # behind the worker's ESTOP/STOP handling so a rapid operator reset
            # cannot let an interrupted AIM continue before STOP is sent.
            with self._state_lock:
                self._busy = True
                self._home_valid = False
                self._aim_complete = False
                self._fire_ready = False
                self._state = "ESTOP"
                self._last_error = "ESTOP_RESET_PENDING"
                self._last_command = "ESTOP_RESET_QUEUED"
            self._enqueue({"type": "RESET", **payload})
            self._publish_status(extra={"event": "ESTOP_RESET_QUEUED"})
            return

        self._reject("UNKNOWN_ESTOP_COMMAND", {"raw": msg.data})

    # ------------------------------------------------------------------
    # Worker and command handlers
    # ------------------------------------------------------------------
    def _enqueue(self, item: dict[str, Any]) -> None:
        self._command_queue.put(item)

    def _discard_pending_commands(self) -> None:
        """Discard queued motion/fire commands after an emergency transition.

        A command already executing is stopped through ``_estop_event``.  This
        helper prevents commands that were queued before ESTOP/RESET from being
        executed after the latch is cleared.  SHUTDOWN is preserved.
        """
        preserved: list[dict[str, Any]] = []
        while True:
            try:
                item = self._command_queue.get_nowait()
            except queue.Empty:
                break
            try:
                if str(item.get("type", "")).upper() == "SHUTDOWN":
                    preserved.append(item)
            finally:
                self._command_queue.task_done()
        for item in preserved:
            self._command_queue.put(item)

    def _wait_recovery_delay(self, delay_sec: float) -> bool:
        """Wait without hiding an E-Stop or node shutdown request."""
        deadline = time.monotonic() + max(0.0, float(delay_sec))
        while time.monotonic() < deadline:
            if self._shutdown_event.is_set() or self._estop_event.is_set():
                return False
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return not self._shutdown_event.is_set() and not self._estop_event.is_set()

    def _worker_loop(self) -> None:
        while not self._shutdown_event.is_set():
            try:
                item = self._command_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            cmd_type = str(item.get("type", "")).upper()
            try:
                if cmd_type == "HOME":
                    self._handle_home(item)
                elif cmd_type == "AIM":
                    self._handle_aim(item)
                elif cmd_type == "FIRE":
                    self._handle_fire(item)
                elif cmd_type == "ESTOP":
                    self._handle_estop(item)
                elif cmd_type == "RESET":
                    self._handle_estop_reset(item)
                elif cmd_type == "SHUTDOWN":
                    break
                else:
                    self._fault("UNKNOWN_WORKER_COMMAND", {"item": item})
            except Exception as exc:  # defensive boundary around hardware worker
                self.get_logger().exception(f"[WORKER] unhandled exception: {exc}")
                self._fault("WORKER_EXCEPTION", {"message": str(exc)})
            finally:
                self._command_queue.task_done()

    def _handle_home(self, item: dict[str, Any]) -> None:
        startup = bool(item.get("startup", False))
        if self._estop_event.is_set():
            self._set_state(
                "ESTOP",
                busy=False,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_error="ESTOP_LATCHED",
                last_command="HOME_ABORTED",
            )
            return

        self._set_state(
            "STARTUP_HOMING" if startup else "HOMING",
            busy=True,
            home_valid=False,
            aim_complete=False,
            fire_ready=False,
            last_error="",
            last_command="HOME0",
        )
        self._publish_status(extra={"event": "HOME_STARTED"})

        ok, reason = self._home_tilt()
        if ok:
            self._set_state(
                "HOME_READY",
                busy=False,
                home_valid=True,
                aim_complete=False,
                fire_ready=False,
                current_tilt_pulse=0,
                target_tilt_pulse=None,
                target_elevation_deg=None,
                last_error="",
                last_command="HOME_COMPLETE",
                aim_count_since_home=0,
            )
            self._publish_status(extra={"event": "HOME_COMPLETE"})
        else:
            state = "ESTOP" if self._estop_event.is_set() else "HOME_REQUIRED"
            self._set_state(
                state,
                busy=False,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_error=reason,
                last_command="HOME_FAILED",
            )
            self._publish_status(extra={"event": "HOME_FAILED", "reason": reason})

    def _handle_aim(self, item: dict[str, Any]) -> None:
        elevation = float(item["elevation_deg"])
        target_pulse = int(item["target_tilt_pulse"])

        if self._estop_event.is_set():
            self._abort_motion("ESTOP_LATCHED")
            return

        with self._state_lock:
            current_pulse = self._current_tilt_pulse
            home_valid = self._home_valid

        rehome_each = bool(self.get_parameter("rehome_before_each_aim").value)
        rehome_reverse = bool(self.get_parameter("rehome_if_target_below_current").value)
        must_home = (not home_valid) or rehome_each or (rehome_reverse and target_pulse < current_pulse)

        if must_home:
            self.get_logger().info(
                f"[AIM] re-home required | home_valid={home_valid} | "
                f"current={current_pulse}p target={target_pulse}p"
            )
            self._set_state(
                "HOMING",
                busy=True,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_command="HOME_BEFORE_AIM",
            )
            ok, reason = self._home_tilt()
            if not ok:
                self._abort_motion(f"HOME_BEFORE_AIM_FAILED:{reason}")
                return
            current_pulse = 0
            self._set_state(
                "TURRET_AIMING",
                busy=True,
                home_valid=True,
                current_tilt_pulse=0,
                aim_count_since_home=0,
                last_command="AIM_AFTER_HOME",
            )

        delta = target_pulse - current_pulse
        deadband_pulse = self._deadband_pulse()

        if abs(delta) <= deadband_pulse:
            self._complete_aim(elevation, target_pulse, "AIM_COMPLETE_NO_MOVE")
            return

        line = self._format_elevation_cmd(delta)
        if line is None:
            self._complete_aim(elevation, target_pulse, "AIM_COMPLETE_NO_MOVE")
            return

        self._set_state(
            "TURRET_AIMING",
            busy=True,
            aim_complete=False,
            fire_ready=False,
            target_tilt_pulse=target_pulse,
            target_elevation_deg=elevation,
            last_command=line,
        )
        self._publish_status(
            extra={
                "event": "TURRET_MOVE_STARTED",
                "delta_tilt_pulse": delta,
                "uart_command": line,
            }
        )

        if not self._send_uart_line(line):
            self._abort_motion("ELEV_UART_SEND_FAILED")
            return

        ok, reason = self._wait_m0_idle()
        if not ok:
            self._abort_motion(reason)
            return

        self._complete_aim(elevation, target_pulse, "AIM_COMPLETE")

    def _handle_fire(self, item: dict[str, Any]) -> None:
        if self._estop_event.is_set():
            self._set_state(
                "ESTOP",
                busy=False,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_error="ESTOP_LATCHED",
                last_command="FIRE_ABORTED",
            )
            return

        self._set_state("FIRING", busy=True, fire_ready=False, last_command="FIRE_READY_VERIFY")
        self._publish_status(extra={"event": "FIRE_SEQUENCE_STARTED"})

        ready_ok, ready_reason = self._verify_stm_ready_for_fire()
        if not ready_ok:
            self._block_fire_due_to_ready_check(ready_reason, "FIRE_BLOCKED")
            return

        self._set_state("FIRING", busy=True, fire_ready=False, last_command="STOP")

        stop_ok = self._send_uart_line("STOP")
        if not stop_ok:
            self._fault("STOP_UART_FAILED", {"fire_sent": False})
            return

        delay = max(0.0, float(self.get_parameter("stop_to_fire_delay_sec").value))
        if delay > 0:
            time.sleep(delay)

        if self._estop_event.is_set():
            self._set_state(
                "ESTOP",
                busy=False,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_error="ESTOP_DURING_FIRE_SEQUENCE",
                last_command="FIRE_ABORTED",
            )
            return

        ready_ok, ready_reason = self._verify_stm_ready_for_fire()
        if not ready_ok:
            self._block_fire_due_to_ready_check(ready_reason, "FIRE_BLOCKED_AFTER_STOP")
            return

        fire_ok = self._send_uart_line("FIRE")
        if not fire_ok:
            self._fault("FIRE_UART_FAILED", {"stop_sent": True, "fire_sent": False})
            return

        self._set_state(
            "FIRED",
            busy=False,
            aim_complete=False,
            fire_ready=False,
            last_error="",
            last_command="FIRE_COMPLETE",
        )
        auto_home = bool(self.get_parameter("auto_home_after_fire").value)
        post_fire_delay = max(
            0.0,
            float(self.get_parameter("post_fire_home_delay_sec").value),
        )
        self._publish_status(
            extra={
                "event": "FIRED",
                "stop_sent": True,
                "fire_sent": True,
                "request": item,
                "auto_home_queued": auto_home,
                "post_fire_home_delay_sec": post_fire_delay,
            }
        )
        self.get_logger().info(f"[FIRED] target={self._active_target_key or '-'}")

        if auto_home:
            # Keep FIRED visible long enough for the coordinator/GUI to consume
            # it, then execute HOME in the same serial worker.
            if not self._wait_recovery_delay(post_fire_delay):
                return
            self.get_logger().info("[AUTO_HOME] starting HOME after fire")
            self._handle_home({
                "type": "HOME",
                "source": "auto_after_fire",
                "auto_home": True,
            })

    def _handle_estop(self, item: dict[str, Any]) -> None:
        if not self._estop_event.is_set():
            return
        # If another command was interrupted, send STOP once more from the same
        # UART worker to guarantee an idle output state.
        self._send_uart_line("STOP", require_ack=False)
        self._set_state(
            "ESTOP",
            busy=False,
            home_valid=False,
            aim_complete=False,
            fire_ready=False,
            last_error="ESTOP_LATCHED",
            last_command="STOP",
        )
        self._publish_status(extra={"event": "ESTOP_STOP_SENT", "request": item})

    def _handle_estop_reset(self, item: dict[str, Any]) -> None:
        """Clear E-Stop only after STOP has been serialized by the worker."""
        self._send_uart_line("STOP", require_ack=False)
        self._discard_pending_commands()
        self._estop_event.clear()

        auto_home = bool(self.get_parameter("auto_home_after_estop_reset").value)
        if not auto_home:
            self._set_state(
                "HOME_REQUIRED",
                busy=False,
                home_valid=False,
                aim_complete=False,
                fire_ready=False,
                last_error="HOME_REQUIRED_AFTER_ESTOP",
                last_command="ESTOP_RESET",
            )
            self._publish_status(extra={"event": "ESTOP_RESET", "auto_home_queued": False})
            return

        self._set_state(
            "HOMING",
            busy=True,
            home_valid=False,
            aim_complete=False,
            fire_ready=False,
            last_error="",
            last_command="AUTO_HOME_AFTER_ESTOP_RESET",
        )
        self._publish_status(
            extra={
                "event": "AUTO_HOME_AFTER_ESTOP_RESET_STARTED",
                "auto_home_queued": True,
                "request": item,
            }
        )
        self._handle_home({
            "type": "HOME",
            "source": "estop_reset",
            "auto_home": True,
        })

    # ------------------------------------------------------------------
    # Hardware helpers
    # ------------------------------------------------------------------
    def _startup_uart_settle_delay(self) -> None:
        """Allow STM USB-CDC firmware to finish reset/init before the first STM command."""
        if not bool(self.get_parameter("uart_enabled").value):
            return

        delay_sec = max(0.0, float(self.get_parameter("startup_uart_settle_delay_sec").value))
        if delay_sec > 0.0:
            self.get_logger().info(
                f"[UART] startup settle delay {delay_sec:.2f}s before first STM command"
            )
            time.sleep(delay_sec)

        # Drop boot banners, stale partial lines, or noise generated during reset
        # from the PC-side input/output buffers. This does not clear STM rx_line.
        if self.ser is not None and self.ser.is_open:
            try:
                with self._serial_lock:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
            except Exception as exc:
                self.get_logger().warn(f"[UART] startup buffer reset skipped: {exc}")

        self._sync_stm_line_parser()

    def _sync_stm_line_parser(self) -> bool:
        """Synchronize STM line parser before the first real command.

        ROS-side reset_input_buffer() only clears bytes waiting on the PC side.
        It cannot clear a partial command already accumulated in STM rx_line.
        The STM firmware patch below adds a SYNC/FLUSH command that clears
        rx_line/rx_pos and replies with ACK SYNC.
        """
        if not bool(self.get_parameter("uart_enabled").value):
            return True
        if not bool(self.get_parameter("uart_sync_on_startup").value):
            return True
        if self.ser is None or not self.ser.is_open:
            return False

        command = str(self.get_parameter("uart_sync_command").value).strip() or "SYNC"
        timeout_sec = max(0.1, float(self.get_parameter("uart_sync_timeout_sec").value))
        deadline = time.monotonic() + timeout_sec

        try:
            with self._serial_lock:
                # Keep the host-side buffer clean immediately before sending SYNC.
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.ser.write((command + "\n").encode("ascii"))
                self.ser.flush()
                self.get_logger().info(f"[UART_TX] {command}")

                while time.monotonic() < deadline:
                    raw = self.ser.readline()
                    line = raw.decode("ascii", errors="ignore").strip()
                    if not line:
                        continue

                    self.get_logger().info(f"[UART_RX] {line}")
                    if line.startswith("ACK SYNC") or line.startswith(f"ACK {command}"):
                        self.ser.reset_input_buffer()
                        self.get_logger().info("[UART_SYNC] STM line parser synchronized")
                        return True
                    if line.startswith("NAK") or line.startswith("ERR"):
                        # Older STM firmware may not know SYNC. Even then the newline
                        # terminates any partial STM rx_line, so this is still a useful
                        # resync attempt. Continue startup but make it visible in logs.
                        self.ser.reset_input_buffer()
                        self.get_logger().warn(
                            f"[UART_SYNC] STM rejected {command}: {line}; "
                            "continuing after parser flush attempt"
                        )
                        return False

                self.get_logger().warn(f"[UART_SYNC] {command} ACK timeout; continuing")
                return False
        except Exception as exc:
            self.get_logger().warn(f"[UART_SYNC] failed: {exc}")
            return False

    def _open_serial_if_enabled(self) -> None:
        if not bool(self.get_parameter("uart_enabled").value):
            self.get_logger().warn("[UART] disabled; using dry-run mode")
            return
        if serial is None:
            self.get_logger().error("[UART] pyserial is not installed")
            return

        port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = float(self.get_parameter("serial_timeout_sec").value)
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )
            self.get_logger().info(f"[UART] opened {port} @ {baudrate}")
        except Exception as exc:
            self.ser = None
            self.get_logger().error(f"[UART] open failed: {exc}")

    def _send_uart_line(self, line: str, require_ack: bool = True) -> bool:
        if not bool(self.get_parameter("uart_enabled").value):
            self.get_logger().info(f"[UART_DRY_RUN] {line}")
            return True
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("[UART] serial is not open")
            return False

        with self._serial_lock:
            try:
                self.ser.write((line + "\n").encode("ascii"))
                self.ser.flush()
                self.get_logger().info(f"[UART_TX] {line}")
            except Exception as exc:
                self.get_logger().error(f"[UART] send failed: {exc}")
                return False

            if not require_ack:
                return True

            try:
                raw = self.ser.readline()
                response = raw.decode("ascii", errors="ignore").strip()
            except Exception as exc:
                self.get_logger().error(f"[UART] ACK read failed: {exc}")
                return False

        if response.startswith("ACK"):
            self.get_logger().info(f"[UART_RX] {response}")
            return True
        if response.startswith("NAK") or response.startswith("ERR"):
            self.get_logger().error(f"[UART_RX] {response} <- rejected: {line}")
            return False
        if response:
            self.get_logger().error(f"[UART] unexpected response '{response}' <- {line}")
        else:
            self.get_logger().error(f"[UART] ACK timeout: {line}")
        return False

    def _drain_uart_input(self) -> None:
        if self.ser is None or not self.ser.is_open:
            return
        try:
            time.sleep(0.2)
            with self._serial_lock:
                self.ser.reset_input_buffer()
        except Exception:
            pass

    def _read_until(self, targets: tuple[str, ...], timeout_sec: float) -> tuple[Optional[str], str]:
        if not bool(self.get_parameter("uart_enabled").value):
            return targets[0], "OK"
        if self.ser is None or not self.ser.is_open:
            return None, "SERIAL_NOT_OPEN"

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._estop_event.is_set():
                self._send_uart_line("STOP", require_ack=False)
                return None, "ESTOP_DURING_UART_WAIT"

            try:
                with self._serial_lock:
                    raw = self.ser.readline()
                line = raw.decode("ascii", errors="ignore").strip()
            except Exception as exc:
                return None, f"UART_READ_FAILED:{exc}"

            if not line:
                continue

            self.get_logger().info(f"[UART_RX] {line}")
            if any(line.startswith(target) for target in targets):
                return line, "OK"
            if line.startswith("ERR") or line.startswith("NAK"):
                return None, f"STM_REJECTED:{line}"

        return None, "UART_RESPONSE_TIMEOUT"

    def _query_stm_status(self) -> tuple[Optional[dict[str, Any]], str]:
        if not bool(self.get_parameter("uart_enabled").value):
            return {"POS": self._current_tilt_pulse, "HOMED": 1, "HOME45": 1, "M0RUN": 0, "INP": 1}, "OK"
        if self.ser is None or not self.ser.is_open:
            return None, "SERIAL_NOT_OPEN"

        with self._serial_lock:
            try:
                self.ser.write(b"STATUS\n")
                self.ser.flush()
                self.get_logger().info("[UART_TX] STATUS")
            except Exception as exc:
                return None, f"STATUS_SEND_FAILED:{exc}"

        line, reason = self._read_until(("POS=",), timeout_sec=2.0)
        if line is None:
            return None, reason
        return self._parse_status_line(line), "OK"

    @staticmethod
    def _parse_status_line(line: str) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for token in line.split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            try:
                result[key] = int(value)
            except ValueError:
                result[key] = value
        return result

    def _remember_stm_status(self, status: dict[str, Any]) -> None:
        with self._state_lock:
            self._last_stm_status = dict(status)
            self._last_stm_status_time_sec = time.time()

    @staticmethod
    def _status_int(status: dict[str, Any], key: str, default: Optional[int] = None) -> Optional[int]:
        value = status.get(key, default)
        if value is None:
            return default
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _status_has_ready_position(self, status: dict[str, Any], require_inp: bool) -> tuple[bool, str]:
        m0run = self._status_int(status, "M0RUN", 1)
        if m0run != 0:
            return False, f"M0RUN_NOT_IDLE:{status}"

        if not require_inp:
            return True, "OK"

        if "INP" not in status:
            return False, f"INP_MISSING_IN_STATUS:{status}"

        inp = self._status_int(status, "INP", None)
        if inp != 1:
            return False, f"INP_NOT_READY:{status}"

        return True, "OK"

    def _verify_stm_ready_for_fire(self) -> tuple[bool, str]:
        if not bool(self.get_parameter("uart_enabled").value):
            return True, "OK"

        status, reason = self._query_stm_status()
        if status is None:
            return False, f"FIRE_STATUS_FAILED:{reason}"

        self._remember_stm_status(status)
        require_inp = bool(self.get_parameter("require_in_position_for_fire").value)
        return self._status_has_ready_position(status, require_inp)

    def _block_fire_due_to_ready_check(self, reason: str, event: str) -> None:
        with self._state_lock:
            home_valid = self._home_valid

        next_state = "HOME_READY" if home_valid else "HOME_REQUIRED"
        self._set_state(
            next_state,
            busy=False,
            home_valid=home_valid,
            aim_complete=False,
            fire_ready=False,
            last_error=f"{event}:{reason}",
            last_command="FIRE_BLOCKED",
        )
        self._publish_status(extra={"event": event, "reason": reason})
        self.get_logger().error(f"[{event}] {reason}")

    def _wait_m0_idle(self) -> tuple[bool, str]:
        if not bool(self.get_parameter("uart_enabled").value):
            return True, "OK"

        timeout_sec = max(0.5, float(self.get_parameter("motor_wait_timeout_sec").value))
        poll_sec = max(0.02, float(self.get_parameter("motor_wait_poll_sec").value))
        require_inp = bool(self.get_parameter("require_in_position_for_aim_ready").value)
        stable_required = max(1, int(self.get_parameter("in_position_stable_count").value))

        deadline = time.monotonic() + timeout_sec
        last_status: Optional[dict[str, Any]] = None
        stable_count = 0

        while time.monotonic() < deadline:
            if self._estop_event.is_set():
                self._send_uart_line("STOP", require_ack=False)
                return False, "ESTOP_DURING_TURRET_MOTION"

            status, reason = self._query_stm_status()
            if status is None:
                self.get_logger().warn(f"[MOTOR_WAIT] status unavailable: {reason}")
                stable_count = 0
                time.sleep(poll_sec)
                continue

            last_status = status
            self._remember_stm_status(status)

            ready, ready_reason = self._status_has_ready_position(status, require_inp)
            if ready:
                stable_count += 1
                self.get_logger().info(
                    f"[MOTOR_WAIT] ready {stable_count}/{stable_required} | {status}"
                )
                if stable_count >= stable_required:
                    return True, "OK"
            else:
                stable_count = 0
                m0run = self._status_int(status, "M0RUN", 1)
                if m0run == 0 and "INP" not in status and require_inp:
                    return False, ready_reason

            time.sleep(poll_sec)

        return False, f"ELEV_MOTOR_WAIT_TIMEOUT:last_status={last_status}"

    def _status_is_valid_home(self, status: Optional[dict[str, Any]]) -> bool:
        if not status:
            return False
        return (
            self._status_int(status, "POS", -9999) == 0
            and self._status_int(status, "HOMED", 0) == 1
            and self._status_int(status, "HOME45", 0) == 1
            and self._status_int(status, "M0RUN", 1) == 0
        )

    def _wait_home_idle(self) -> tuple[bool, str]:
        """Stop the tilt axis and verify that HOME0 will start from idle."""
        if not bool(self.get_parameter("uart_enabled").value):
            return True, "OK"
        if self.ser is None or not self.ser.is_open:
            return False, "SERIAL_NOT_OPEN"

        if bool(self.get_parameter("home_prepare_stop_enabled").value):
            if not self._send_uart_line("STOP", require_ack=False):
                return False, "HOME_PREP_STOP_SEND_FAILED"
            settle_sec = max(0.0, float(self.get_parameter("home_stop_settle_sec").value))
            if settle_sec > 0.0:
                time.sleep(settle_sec)

        timeout_sec = max(0.2, float(self.get_parameter("home_idle_timeout_sec").value))
        poll_sec = max(0.02, float(self.get_parameter("home_idle_poll_sec").value))
        deadline = time.monotonic() + timeout_sec
        last_status: Optional[dict[str, Any]] = None
        last_reason = "NO_STATUS"

        while time.monotonic() < deadline:
            if self._estop_event.is_set():
                return False, "ESTOP_DURING_HOME_PREP"

            status, reason = self._query_stm_status()
            if status is None:
                last_reason = reason
                time.sleep(poll_sec)
                continue

            last_status = status
            self._remember_stm_status(status)
            if self._status_int(status, "M0RUN", 1) == 0:
                self.get_logger().info(f"[HOME_PREP] motor idle | {status}")
                return True, "OK"

            time.sleep(poll_sec)

        return False, f"HOME_PREP_IDLE_TIMEOUT:last_status={last_status}:last_reason={last_reason}"

    def _send_home0_once(self) -> tuple[bool, str]:
        """Send one HOME0 transaction and validate the final STM status."""
        self._drain_uart_input()
        with self._serial_lock:
            try:
                self.ser.write(b"HOME0\n")
                self.ser.flush()
                self.get_logger().info("[UART_TX] HOME0")
            except Exception as exc:
                return False, f"HOME0_SEND_FAILED:{exc}"

        ack, reason = self._read_until(("ACK HOME0",), timeout_sec=2.0)
        if ack is None:
            return False, f"HOME0_ACK_FAILED:{reason}"

        done, reason = self._read_until(("DONE HOME0",), timeout_sec=15.0)
        if done is None:
            return False, f"HOME0_DONE_FAILED:{reason}"

        status, reason = self._query_stm_status()
        if status is None:
            return False, f"HOME0_STATUS_FAILED:{reason}"

        self._remember_stm_status(status)
        if self._status_is_valid_home(status):
            self.get_logger().info("[HOME0] complete | POS=0 HOMED=1 HOME45=1 M0RUN=0")
            return True, "OK"
        return False, f"HOME0_INVALID_STATUS:{status}"

    def _home_tilt(self) -> tuple[bool, str]:
        if self._estop_event.is_set():
            return False, "ESTOP_LATCHED"
        if not bool(self.get_parameter("uart_enabled").value):
            self.get_logger().warn("[HOME0] dry-run complete")
            return True, "OK"
        if self.ser is None or not self.ser.is_open:
            return False, "SERIAL_NOT_OPEN"

        retry_count = max(0, int(self.get_parameter("home_command_retry_count").value))
        retry_delay = max(0.0, float(self.get_parameter("home_retry_delay_sec").value))
        accept_valid_status = bool(
            self.get_parameter("home_accept_valid_status_after_reject").value
        )
        last_reason = "HOME0_NOT_ATTEMPTED"

        for attempt in range(retry_count + 1):
            idle_ok, idle_reason = self._wait_home_idle()
            if not idle_ok:
                last_reason = idle_reason
                self.get_logger().error(
                    f"[HOME0] preparation failed {attempt + 1}/{retry_count + 1}: {idle_reason}"
                )
            else:
                ok, reason = self._send_home0_once()
                if ok:
                    return True, "OK"
                last_reason = reason
                self.get_logger().warn(
                    f"[HOME0] attempt {attempt + 1}/{retry_count + 1} failed: {reason}"
                )

                # Some STM firmware rejects a repeated HOME0 while already at a
                # valid home.  Accept only the complete, idle home status.
                if accept_valid_status:
                    status, status_reason = self._query_stm_status()
                    if status is not None:
                        self._remember_stm_status(status)
                    if self._status_is_valid_home(status):
                        self.get_logger().warn(
                            "[HOME0] command rejected, but STM reports a valid idle home; "
                            "continuing safely"
                        )
                        return True, "ALREADY_AT_VALID_HOME"
                    if status is None:
                        last_reason = f"{reason}:POST_REJECT_STATUS_FAILED:{status_reason}"

            if attempt < retry_count:
                self._send_uart_line("STOP", require_ack=False)
                if retry_delay > 0.0:
                    time.sleep(retry_delay)

        return False, last_reason

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------
    def _set_state(self, state: str, **updates: Any) -> None:
        if state not in self.VALID_STATES:
            raise ValueError(f"invalid turret state: {state}")
        with self._state_lock:
            self._state = state
            for key, value in updates.items():
                attr = f"_{key}"
                if not hasattr(self, attr):
                    raise AttributeError(f"unknown state field: {key}")
                setattr(self, attr, value)

    def _complete_aim(self, elevation: float, target_pulse: int, event: str) -> None:
        self._set_state(
            "AIM_READY",
            busy=False,
            home_valid=True,
            aim_complete=True,
            fire_ready=True,
            current_tilt_pulse=target_pulse,
            target_tilt_pulse=target_pulse,
            target_elevation_deg=elevation,
            last_error="",
            last_command=event,
            aim_count_since_home=self._aim_count_since_home + 1,
        )
        self._publish_status(extra={"event": event})
        self.get_logger().info(
            f"[AIM_READY] target={self._active_target_key or '-'} | "
            f"elevation={elevation:.2f} deg | pulse={target_pulse}"
        )

    def _abort_motion(self, reason: str) -> None:
        if not self._estop_event.is_set():
            self._send_uart_line("STOP", require_ack=False)
        state = "ESTOP" if self._estop_event.is_set() else "HOME_REQUIRED"
        self._set_state(
            state,
            busy=False,
            home_valid=False,
            aim_complete=False,
            fire_ready=False,
            last_error=reason,
            last_command="MOTION_ABORTED",
        )
        self._publish_status(extra={"event": "MOTION_ABORTED", "reason": reason})
        self.get_logger().error(f"[MOTION_ABORTED] {reason} | re-home required")

    def _fault(self, reason: str, extra: Optional[dict[str, Any]] = None) -> None:
        self._set_state(
            "FAULT",
            busy=False,
            home_valid=False,
            aim_complete=False,
            fire_ready=False,
            last_error=reason,
            last_command="FAULT",
        )
        payload = {"event": "FAULT", "reason": reason}
        if extra:
            payload.update(extra)
        self._publish_status(extra=payload)
        self.get_logger().error(f"[FAULT] {reason}")

    def _reject(self, reason: str, extra: Optional[dict[str, Any]] = None) -> None:
        with self._state_lock:
            self._reject_locked(reason, extra)

    def _reject_locked(self, reason: str, extra: Optional[dict[str, Any]] = None) -> None:
        extra = dict(extra or {})
        rejected_command = str(extra.get("rejected_command", "")).upper()
        rejected_target_key = str(extra.get("rejected_target_key", ""))
        rejected_revision = extra.get("rejected_solution_revision")

        self._command_rejection_seq += 1
        self._last_rejected_command = rejected_command
        self._last_rejection_reason = reason
        self._last_rejection_target_key = rejected_target_key
        self._last_rejection_solution_revision = rejected_revision

        payload = self._status_payload_locked()
        payload.update({"event": "COMMAND_REJECTED", "reason": reason})
        payload.update(extra)
        self._publish_payload(payload)
        self.get_logger().warn(
            f"[COMMAND_REJECTED] command={rejected_command or '-'} reason={reason}"
        )

    def _publish_status(self, extra: Optional[dict[str, Any]] = None) -> None:
        with self._state_lock:
            payload = self._status_payload_locked()
        if extra:
            payload.update(extra)
        self._publish_payload(payload)

    def _status_payload_locked(self) -> dict[str, Any]:
        uart_enabled = bool(self.get_parameter("uart_enabled").value)
        uart_open = (self.ser is not None and self.ser.is_open) if uart_enabled else True
        return {
            "state": self._state,
            "busy": self._busy,
            "home_valid": self._home_valid,
            "aim_complete": self._aim_complete,
            "fire_ready": self._fire_ready,
            "estop_latched": self._estop_event.is_set(),
            "current_tilt_pulse": self._current_tilt_pulse,
            "current_elevation_deg": round(self._pulse_to_tilt_deg(self._current_tilt_pulse), 3),
            "tilt_mapping_mode": self._tilt_mapping_mode,
            "tilt_physical_home_deg": float(self.get_parameter("tilt_home_deg").value),
            "tilt_command_min_deg": float(self.get_parameter("tilt_command_min_deg").value),
            "tilt_max_deg": float(self.get_parameter("tilt_max_deg").value),
            "tilt_max_pulse": int(self.get_parameter("tilt_max_pulse").value),
            "target_tilt_pulse": self._target_tilt_pulse,
            "target_elevation_deg": self._target_elevation_deg,
            "active_target_key": self._active_target_key,
            "active_solution_revision": self._active_solution_revision,
            "aim_count_since_home": self._aim_count_since_home,
            "last_error": self._last_error,
            "last_command": self._last_command,
            "command_rejection_seq": self._command_rejection_seq,
            "aim_request_accept_seq": self._aim_request_accept_seq,
            "fire_request_accept_seq": self._fire_request_accept_seq,
            "last_rejected_command": self._last_rejected_command,
            "last_rejection_reason": self._last_rejection_reason,
            "last_rejection_target_key": self._last_rejection_target_key,
            "last_rejection_solution_revision": self._last_rejection_solution_revision,
            "stm_last_status": dict(self._last_stm_status),
            "stm_last_status_time_sec": self._last_stm_status_time_sec,
            "stm_in_position": self._last_stm_status.get("INP"),
            "stm_m0run": self._last_stm_status.get("M0RUN"),
            "uart_enabled": uart_enabled,
            "uart_open": uart_open,
            "t_ns": time.time_ns(),
        }

    def _publish_payload(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(msg)
        self.pub_legacy_cmd.publish(msg)

        ready_msg = String()
        ready_msg.data = json.dumps(
            {
                "fire_ready": bool(payload.get("fire_ready", False)),
                "status": payload.get("state"),
                "target_key": payload.get("active_target_key", ""),
                "home_valid": bool(payload.get("home_valid", False)),
                "aim_complete": bool(payload.get("aim_complete", False)),
                "reason": payload.get("reason", payload.get("last_error", "")),
                "t_ns": payload.get("t_ns", time.time_ns()),
            },
            ensure_ascii=False,
        )
        self.pub_legacy_fire_ready.publish(ready_msg)

    # ------------------------------------------------------------------
    # Numeric / protocol helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _is_finite(value: Any) -> bool:
        try:
            return math.isfinite(float(value))
        except Exception:
            return False

    def _configure_tilt_mapping(self) -> None:
        mode = self._tilt_mapping_mode
        if mode not in {"linear", "lut"}:
            raise ValueError("tilt_mapping_mode must be 'linear' or 'lut'")

        if mode == "linear":
            return

        angles = [float(v) for v in self.get_parameter("tilt_mapping_angles_deg").value]
        pulses = [int(v) for v in self.get_parameter("tilt_mapping_pulses").value]
        if len(angles) != len(pulses) or len(angles) < 2:
            raise ValueError("tilt LUT angle/pulse arrays must have the same length >= 2")
        if any(not math.isfinite(v) for v in angles):
            raise ValueError("tilt LUT angles must be finite")
        if any(angles[i] >= angles[i + 1] for i in range(len(angles) - 1)):
            raise ValueError("tilt LUT angles must be strictly increasing")
        if any(pulses[i] >= pulses[i + 1] for i in range(len(pulses) - 1)):
            raise ValueError("tilt LUT pulses must be strictly increasing")

        home = float(self.get_parameter("tilt_home_deg").value)
        max_deg = float(self.get_parameter("tilt_max_deg").value)
        max_pulse = int(self.get_parameter("tilt_max_pulse").value)
        eps = 1e-6
        if abs(angles[0] - home) > eps or pulses[0] != 0:
            raise ValueError("tilt LUT must start at tilt_home_deg and pulse 0")
        if abs(angles[-1] - max_deg) > eps or pulses[-1] != max_pulse:
            raise ValueError("tilt LUT must end at tilt_max_deg and tilt_max_pulse")

        self._tilt_mapping_angles_deg = angles
        self._tilt_mapping_pulses = pulses

    @staticmethod
    def _interp(x: float, xs: list[float], ys: list[float]) -> float:
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        for i in range(len(xs) - 1):
            x0, x1 = xs[i], xs[i + 1]
            if x <= x1:
                ratio = (x - x0) / (x1 - x0)
                return ys[i] + ratio * (ys[i + 1] - ys[i])
        return ys[-1]

    def _tilt_deg_to_abs_pulse(self, angle_deg: float) -> int:
        home = float(self.get_parameter("tilt_home_deg").value)
        max_deg = float(self.get_parameter("tilt_max_deg").value)
        max_pulse = int(self.get_parameter("tilt_max_pulse").value)
        angle = max(home, min(max_deg, float(angle_deg)))

        if self._tilt_mapping_mode == "lut":
            pulse = self._interp(
                angle,
                self._tilt_mapping_angles_deg,
                [float(v) for v in self._tilt_mapping_pulses],
            )
        else:
            if max_deg <= home:
                raise ValueError("tilt_max_deg must be greater than tilt_home_deg")
            pulse = (angle - home) * max_pulse / (max_deg - home)
        return int(round(max(0.0, min(float(max_pulse), pulse))))

    def _pulse_to_tilt_deg(self, pulse: int) -> float:
        home = float(self.get_parameter("tilt_home_deg").value)
        max_deg = float(self.get_parameter("tilt_max_deg").value)
        max_pulse = int(self.get_parameter("tilt_max_pulse").value)
        p = max(0, min(max_pulse, int(pulse)))

        if self._tilt_mapping_mode == "lut":
            return self._interp(
                float(p),
                [float(v) for v in self._tilt_mapping_pulses],
                self._tilt_mapping_angles_deg,
            )
        if max_pulse <= 0:
            return home
        return home + p * (max_deg - home) / max_pulse

    def _deadband_pulse(self) -> int:
        deadband_deg = max(0.0, float(self.get_parameter("deadband_deg").value))
        if deadband_deg <= 0.0:
            return 0

        if self._tilt_mapping_mode == "lut":
            reference = self._target_elevation_deg
            if reference is None:
                reference = self._pulse_to_tilt_deg(self._current_tilt_pulse)
            lo = self._tilt_deg_to_abs_pulse(float(reference) - deadband_deg / 2.0)
            hi = self._tilt_deg_to_abs_pulse(float(reference) + deadband_deg / 2.0)
            return max(1, abs(hi - lo))

        home = float(self.get_parameter("tilt_home_deg").value)
        max_deg = float(self.get_parameter("tilt_max_deg").value)
        max_pulse = int(self.get_parameter("tilt_max_pulse").value)
        if max_deg <= home:
            return 0
        return max(0, int(round(deadband_deg * max_pulse / (max_deg - home))))

    def _format_elevation_cmd(self, delta_steps: int) -> Optional[str]:
        if delta_steps == 0:
            return None
        motor_id = int(self.get_parameter("elevation_motor_id").value)
        invert = bool(self.get_parameter("invert_elevation_dir").value)

        # Existing STM convention:
        # Python +delta = larger POS / higher elevation.
        # M0- increases POS, M0+ decreases POS.
        positive_sign = "+" if invert else "-"
        negative_sign = "-" if invert else "+"
        sign = positive_sign if delta_steps > 0 else negative_sign
        return f"M{motor_id}{sign}{abs(int(delta_steps))}"

    @staticmethod
    def _parse_command(raw: str) -> tuple[str, dict[str, Any]]:
        text = raw.strip()
        if not text:
            return "", {}
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                cmd = str(payload.get("command", payload.get("cmd", ""))).strip().upper()
                return cmd, payload
        except Exception:
            pass
        return text.upper(), {}

    # ------------------------------------------------------------------
    # Terminal test interface
    # ------------------------------------------------------------------
    def _terminal_input_loop(self) -> None:
        print("\n" + "=" * 52)
        print("  Tilt Turret Actuator Test")
        print("=" * 52)
        print("  h              : HOME0")
        print("  a <deg> [key]  : aim request, e.g. a 53.2 T001")
        print("  f              : FIRE (only when AIM_READY)")
        print("  s              : latch E-STOP")
        print("  r              : reset E-STOP; HOME required")
        print("  q              : shutdown")
        print("=" * 52 + "\n")

        while not self._shutdown_event.is_set():
            try:
                line = input("turret > ").strip()
            except EOFError:
                return
            except Exception:
                return
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            if cmd in {"q", "quit"}:
                rclpy.shutdown()
                return
            if cmd == "h":
                msg = String()
                msg.data = "HOME"
                self._cb_home_request(msg)
                continue
            if cmd == "a" and len(parts) >= 2:
                try:
                    elevation = float(parts[1])
                except ValueError:
                    print("invalid elevation")
                    continue
                target_key = parts[2] if len(parts) >= 3 else "terminal"
                msg = String()
                msg.data = json.dumps({"elevation_deg": elevation, "target_key": target_key})
                self._cb_aim_request(msg)
                continue
            if cmd == "f":
                msg = String()
                msg.data = "FIRE"
                self._cb_fire_confirm(msg)
                continue
            if cmd == "s":
                msg = String()
                msg.data = "ESTOP"
                self._cb_estop(msg)
                continue
            if cmd == "r":
                msg = String()
                msg.data = "RESET"
                self._cb_estop(msg)
                continue
            print("unknown command")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._shutdown_event.set()
        self._estop_event.set()
        self._enqueue({"type": "SHUTDOWN"})
        try:
            if hasattr(self, "_worker") and self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass

        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("[UART] closed")
        except Exception:
            pass

        with self._state_lock:
            self._state = "SHUTDOWN"
            self._busy = False
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TurretControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
