#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mmr_drive_node / bldc_drive_node.py

BLC-400R4E 6축 RS-485 BLDC 드라이버를 Joy 토픽으로 제어하는 ROS2 노드.

제약 사항 (blc6m_test.py 및 누리로봇 프로토콜 V1.1 기반):
  - 허용 명령: 0x03 가감속 속도제어, 0x0C 제어 On/Off
  - 금지: 파라미터 설정 명령(0x04~0x10), 피드백 요청 명령(0xA0~0xCD)
  - 최대 선속도 RPM: HARD_MAX_RPM=19.0 RPM (모터축 기준)
  - 조향: 내측 바퀴만 최대 50% 감속 (외측 유지)

Joy 축/버튼 매핑:
  - axes[1]   : 왼쪽 스틱 Y  → 전후진 (전진: +1.0 / 후진: -1.0)
  - axes[3]   : 오른쪽 스틱 X → 조향   (좌: +1.0 / 우: -1.0)
  - buttons[4]: LB 유지 → 제자리 좌회전
  - buttons[5]: RB 유지 → 제자리 우회전
  - LB + RB   : 0RPM 정지
  (axes[0] 왼쪽 스틱 X, axes[4] 오른쪽 스틱 Y 미사용)

모터 ID 매핑 (blc6m_test.py 동일):
  - 좌측: ID 1, 3, 5  전진 방향 = CCW (0x00)
  - 우측: ID 2, 4, 6  전진 방향 = CW  (0x01)

조향 로직 (skid-steer 차동):
  base_rpm = 운용 max_rpm × |linear_y|
  steer    = axes[3]   (-1.0 ~ +1.0)
  감속율   = 0.5 × |steer|   (0% ~ 50%)

  steer > 0 (좌회전):
    left_rpm  = base_rpm × (1 - 감속율)
    right_rpm = base_rpm

  steer < 0 (우회전):
    left_rpm  = base_rpm
    right_rpm = base_rpm × (1 - 감속율)

  linear_y == 0 (정지):
    left_rpm = right_rpm = 0  (조향 입력 무관)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import serial

from mmr_drive_node.load_observer import LoadObserver
from mmr_drive_node.turn_strategy import (
    PivotPulseConfig,
    PivotPulseController,
    TurnStrategyConfig,
    TurnStrategyController,
)


# ─────────────────────────────────────────────
# 상수
# ─────────────────────────────────────────────

LEFT_IDS  = [1, 3, 5]
RIGHT_IDS = [2, 4, 6]
ALL_IDS   = LEFT_IDS + RIGHT_IDS

DIR_CCW = 0x00   # 좌측 전진
DIR_CW  = 0x01   # 우측 전진

HARD_MAX_RPM  = 19.0   # 모터 보호용 절대 상한 (20RPM 정격 대비 보수적 제한)
MAX_STEER_RED = 0.50   # 내측 최대 감속 비율 (50%)
ARRIVE_S      = 0.15   # 가감속 도달 시간 [s] — 응답성과 안정성 사이 조정값

# Joy 축/버튼 인덱스
AXIS_LINEAR  = 1   # 왼쪽 스틱 Y
AXIS_STEER   = 3   # 오른쪽 스틱 X
BUTTON_LB    = 4   # LB: 제자리 좌회전
BUTTON_RB    = 5   # RB: 제자리 우회전
BUTTON_X     = 2   # X: 조이스틱 E-Stop Reset 콤보 기본 버튼

DEFAULT_MANUAL_PIVOT_RPM = 1.0

WHEEL_LABEL_BY_ID = {
    1: 'FL',  # Front Left
    3: 'ML',  # Middle Left
    5: 'RL',  # Rear Left
    2: 'FR',  # Front Right
    4: 'MR',  # Middle Right
    6: 'RR',  # Rear Right
}

# 데드존 — 스틱 중립 오차 흡수
DEADZONE_LINEAR = 0.05
DEADZONE_STEER  = 0.05


# ─────────────────────────────────────────────
# 누리로봇 프로토콜 헬퍼 (허용 명령만 구현)
# ─────────────────────────────────────────────

def _calc_checksum(dev_id: int, size: int, mode: int, data: bytes) -> int:
    """CheckSum = ~((ID + Size + Mode + Data...) & 0xFF) & 0xFF"""
    return (~((dev_id + size + mode + sum(data)) & 0xFF)) & 0xFF


def _build_packet(dev_id: int, mode: int, data: bytes = b'') -> bytes:
    """
    Frame: FF FE | ID | SIZE | CHECKSUM | MODE | DATA...
    SIZE  = 2 (CHECKSUM 1 + MODE 1) + len(data)
    """
    size = 2 + len(data)
    chk  = _calc_checksum(dev_id, size, mode, data)
    return bytes([0xFF, 0xFE, dev_id & 0xFF, size & 0xFF, chk, mode & 0xFF]) + data


def _to_be16(value: int) -> bytes:
    value = max(0, min(65533, int(value)))
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def make_acc_speed_packet(dev_id: int, direction: int, rpm: float, arrive_s: float) -> bytes:
    """
    Mode 0x03 가감속 속도제어 패킷 생성.
    data: direction[1B] | speed[2B, 0.1RPM 단위] | arrive_time[1B, 0.1s 단위]
    """
    spd_raw = max(0, min(65533, int(round(rpm * 10.0))))
    arr_raw = max(1, min(255,   int(round(arrive_s * 10.0))))
    data    = bytes([direction & 0xFF]) + _to_be16(spd_raw) + bytes([arr_raw])
    return _build_packet(dev_id, 0x03, data)


def make_control_onoff_packet(dev_id: int, on: bool) -> bytes:
    """
    Mode 0x0C 제어 On/Off 패킷 생성.
    0x00 = On, 0x01 = Off
    """
    return _build_packet(dev_id, 0x0C, bytes([0x00 if on else 0x01]))


def make_speed_feedback_request_packet(dev_id: int) -> bytes:
    """Mode 0xA2 속도 피드백 요청 패킷 생성."""
    return _build_packet(dev_id, 0xA2)


# ─────────────────────────────────────────────
# RS-485 시리얼 링크
# ─────────────────────────────────────────────

class RS485Link:
    """
    pyserial 래퍼.
    Half-duplex RS-485이므로 쓰기만 수행하고 읽기는 하지 않음.
    모든 송신은 lock으로 직렬화.
    """

    def __init__(self, port: str, baudrate: int):
        self._port     = port
        self._baudrate = baudrate
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()

    def open(self) -> None:
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.5,
        )

    def close(self) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def send_packets(self, packets: list[bytes], inter_delay_s: float = 0.005) -> None:
        """
        패킷 목록을 순서대로 송신.
        inter_delay_s: ID 간 최소 전송 간격 (기본 5ms)
        """
        with self._lock:
            if not self.is_open:
                return
            for pkt in packets:
                self._ser.write(pkt)
                self._ser.flush()
                time.sleep(inter_delay_s)

    def _recv_packet_locked(self, timeout_s: float) -> dict | None:
        """lock을 잡은 상태에서 응답 패킷 1개를 수신한다."""

        if not self.is_open:
            return None

        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            b1 = self._ser.read(1)

            if not b1 or b1[0] != 0xFF:
                continue

            b2 = self._ser.read(1)

            if not b2 or b2[0] != 0xFE:
                continue

            header = self._ser.read(2)

            if len(header) != 2:
                continue

            rx_id = header[0]
            size = header[1]
            rest = bytearray()

            while len(rest) < size and time.monotonic() < deadline:
                chunk = self._ser.read(size - len(rest))

                if chunk:
                    rest.extend(chunk)

            if len(rest) != size or size < 2:
                return None

            checksum = rest[0]
            mode = rest[1]
            data = bytes(rest[2:])

            checksum_calc = _calc_checksum(
                rx_id,
                size,
                mode,
                data,
            )

            return {
                "ok": checksum == checksum_calc,
                "id": rx_id,
                "mode": mode,
                "data": data,
            }

        return None

    def request_speed_feedback(
        self,
        dev_id: int,
        timeout_s: float = 0.15,
    ) -> dict | None:
        """0xA2 요청 후 0xD2 응답을 파싱하여 반환한다."""

        with self._lock:
            if not self.is_open:
                return None

            self._ser.reset_input_buffer()

            self._ser.write(
                make_speed_feedback_request_packet(dev_id)
            )
            self._ser.flush()

            response = self._recv_packet_locked(timeout_s)

        if not response or not response["ok"]:
            return None

        if response["id"] != dev_id:
            return None

        if response["mode"] != 0xD2:
            return None

        data = response["data"]

        if len(data) < 6:
            return None

        return {
            "id": dev_id,
            "direction": data[0],
            "rpm": ((data[1] << 8) | data[2]) / 10.0,
            "position_deg": ((data[3] << 8) | data[4]) / 10.0,
            "current_a": data[5] / 10.0,
        }


# ─────────────────────────────────────────────
# 조향 계산
# ─────────────────────────────────────────────

def apply_deadzone(value: float, threshold: float) -> float:
    if abs(value) < threshold:
        return 0.0
    # 데드존 외부 구간을 0~1 로 재정규화
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold) / (1.0 - threshold)


def signed_rpm_from_direction(dev_id: int, direction: int, rpm: float) -> float:
    """로봇 전진 방향을 +RPM으로 통일한다."""
    value = abs(float(rpm))

    if dev_id in LEFT_IDS:
        # 좌측 전진 방향 = CCW
        return value if direction == DIR_CCW else -value

    if dev_id in RIGHT_IDS:
        # 우측 전진 방향 = CW
        return value if direction == DIR_CW else -value

    return 0.0


def compute_wheel_rpms(
    linear: float,
    steer: float,
    max_rpm: float,
) -> tuple[float, float]:
    """
    기존 차동 조향 계산.

    adaptive_steering_enabled=false일 때 이 함수를 그대로 사용하므로
    패치 비활성화 시 기존 동작으로 완전히 복귀한다.
    """
    base_rpm = max(0.0, float(max_rpm)) * abs(linear)
    reduction = MAX_STEER_RED * abs(steer)   # 0.0 ~ 0.5

    if steer > 0.0:           # 좌회전 → 좌측 감속
        left_rpm = base_rpm * (1.0 - reduction)
        right_rpm = base_rpm
    elif steer < 0.0:         # 우회전 → 우측 감속
        left_rpm = base_rpm
        right_rpm = base_rpm * (1.0 - reduction)
    else:                     # 직진
        left_rpm = base_rpm
        right_rpm = base_rpm

    return left_rpm, right_rpm


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(float(value), high))


def _lerp(x: float, x0: float, y0: float, x1: float, y1: float) -> float:
    """두 점 사이 선형 보간. x0 == x1이면 y1을 반환한다."""
    if x1 <= x0:
        return float(y1)
    t = _clamp((x - x0) / (x1 - x0), 0.0, 1.0)
    return y0 + (y1 - y0) * t


def _piecewise_linear(x: float, points: list[tuple[float, float]]) -> float:
    """x 오름차순 점 목록을 이용한 연속 구간 선형 보간."""
    if not points:
        raise ValueError('points must not be empty')

    x = float(x)
    if x <= points[0][0]:
        return float(points[0][1])

    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return _lerp(x, x0, y0, x1, y1)

    return float(points[-1][1])


def slew_towards(current: float, target: float, rate_per_sec: float, dt: float) -> float:
    """현재값을 목표값 방향으로 rate_per_sec 이하 속도로 이동시킨다."""
    current = _clamp(current, -1.0, 1.0)
    target = _clamp(target, -1.0, 1.0)
    rate_per_sec = max(0.0, float(rate_per_sec))
    dt = max(0.0, float(dt))

    if rate_per_sec <= 0.0 or dt <= 0.0:
        return current

    max_step = rate_per_sec * dt
    delta = _clamp(target - current, -max_step, max_step)
    return _clamp(current + delta, -1.0, 1.0)


def compute_adaptive_wheel_rpms(
    linear: float,
    steer: float,
    max_rpm: float,
    low_speed_rpm: float,
    mid_speed_rpm: float,
    high_speed_rpm: float,
    transition_rpm: float,
    reduction_low: float,
    reduction_mid: float,
    reduction_high: float,
    reduction_very_high: float,
    speed_cap_mid_rpm: float,
    speed_cap_high_rpm: float,
    speed_cap_full_rpm: float,
) -> tuple[float, float, dict]:
    """속도·조향량 연동 제한이 적용된 일반 차동 조향 계산.

    일반 주행에서만 사용하며 내측 휠도 진행 방향을 유지한다.
    반환 RPM은 방향 부호가 없는 크기이고, 방향은 기존 _send_velocity가 결정한다.
    """
    max_rpm = max(0.0, float(max_rpm))
    linear_abs = _clamp(abs(linear), 0.0, 1.0)
    steer_abs = _clamp(abs(steer), 0.0, 1.0)

    requested_base_rpm = max_rpm * linear_abs

    # 큰 조향일수록 전체 기준 속도를 낮춘다.
    # 0~20% 조향은 기존 max_rpm을 유지하고 50/80/100%에서 단계적으로 제한한다.
    cap_points = [
        (0.0, max_rpm),
        (0.2, max_rpm),
        (0.5, min(max_rpm, max(0.0, speed_cap_mid_rpm))),
        (0.8, min(max_rpm, max(0.0, speed_cap_high_rpm))),
        (1.0, min(max_rpm, max(0.0, speed_cap_full_rpm))),
    ]
    speed_cap_rpm = _piecewise_linear(steer_abs, cap_points)
    base_rpm = min(requested_base_rpm, speed_cap_rpm)

    low_speed_rpm = max(0.0, float(low_speed_rpm))
    mid_speed_rpm = max(low_speed_rpm, float(mid_speed_rpm))
    high_speed_rpm = max(mid_speed_rpm, float(high_speed_rpm))

    max_transition = min(
        max(0.1, mid_speed_rpm - low_speed_rpm),
        max(0.1, high_speed_rpm - mid_speed_rpm),
    )
    transition_rpm = _clamp(transition_rpm, 0.1, max_transition)

    reduction_low = _clamp(reduction_low, 0.0, 0.95)
    reduction_mid = _clamp(reduction_mid, 0.0, reduction_low)
    reduction_high = _clamp(reduction_high, 0.0, reduction_mid)
    reduction_very_high = _clamp(
        reduction_very_high,
        0.0,
        reduction_high,
    )

    # 각 속도대 시작 후 transition_rpm 동안만 부드럽게 다음 감속률로 전환한다.
    reduction_points = [
        (0.0, reduction_low),
        (low_speed_rpm, reduction_low),
        (low_speed_rpm + transition_rpm, reduction_mid),
        (mid_speed_rpm, reduction_mid),
        (mid_speed_rpm + transition_rpm, reduction_high),
        (high_speed_rpm, reduction_high),
        (high_speed_rpm + transition_rpm, reduction_very_high),
        (max(HARD_MAX_RPM, high_speed_rpm + transition_rpm), reduction_very_high),
    ]
    max_reduction = _piecewise_linear(base_rpm, reduction_points)
    reduction = max_reduction * steer_abs

    if steer > 0.0:           # 좌회전 → 좌측 내측 감속
        left_rpm = base_rpm * (1.0 - reduction)
        right_rpm = base_rpm
    elif steer < 0.0:         # 우회전 → 우측 내측 감속
        left_rpm = base_rpm
        right_rpm = base_rpm * (1.0 - reduction)
    else:
        left_rpm = base_rpm
        right_rpm = base_rpm

    debug = {
        'requested_base_rpm': requested_base_rpm,
        'speed_cap_rpm': speed_cap_rpm,
        'base_rpm': base_rpm,
        'max_reduction': max_reduction,
        'applied_reduction': reduction,
    }
    return left_rpm, right_rpm, debug


# ─────────────────────────────────────────────
# ROS2 노드
# ─────────────────────────────────────────────

class BldcDriveNode(Node):

    def __init__(self):
        super().__init__('bldc_drive_node')

        # ── 파라미터 선언 ──────────────────────────────────
        self.declare_parameter('serial_port',    '/dev/ttyUSB0')
        self.declare_parameter('baudrate',       115200)
        self.declare_parameter('inter_delay_ms', 5)
        self.declare_parameter('max_rpm',        HARD_MAX_RPM)
        self.declare_parameter('manual_pivot_rpm', DEFAULT_MANUAL_PIVOT_RPM)
        self.declare_parameter('align_cmd_topic', '/mmr/drive/align_cmd')
        self.declare_parameter('default_pivot_rpm', DEFAULT_MANUAL_PIVOT_RPM)
        # 자동 정렬 구동 방식:
        #   pivot       = 기존 제자리 회전
        #   arc_forward = 양쪽 바퀴를 모두 전진시키고 내측만 감속하는 저속 곡선 조향
        self.declare_parameter('auto_align_motion_mode', 'pivot')
        self.declare_parameter('auto_align_arc_inner_ratio', 0.50)
        self.declare_parameter('align_cmd_timeout_sec', 0.5)
        self.declare_parameter('arrive_s',       ARRIVE_S)
        self.declare_parameter('joy_topic',      '/joy')
        self.declare_parameter('feedback_enabled', True)
        self.declare_parameter(
            'feedback_ids',
            [1, 2, 3, 4, 5, 6],
        )
        self.declare_parameter('feedback_period_s', 0.15)
        self.declare_parameter('feedback_timeout_s', 0.12)
        self.declare_parameter('drive_status_topic', '/mmr/drive/status')
        self.declare_parameter('stop_rpm_threshold', 0.3)
        self.declare_parameter('feedback_fresh_timeout_s', 1.5)

        # ── RPM·전류 통합 부하 관측기(Shadow mode) ───────────
        # 관측 결과는 상태 토픽에만 기록하며 모터 명령에는 적용하지 않는다.
        self.declare_parameter('load_observer_enabled', True)
        self.declare_parameter('load_protection_enabled', False)
        self.declare_parameter('load_min_target_rpm', 1.0)
        self.declare_parameter('load_feedback_max_age_s', 0.5)
        self.declare_parameter('load_progressing_min_ratio', 0.45)
        self.declare_parameter('load_stall_max_ratio', 0.20)
        self.declare_parameter('load_unload_current_ratio', 0.40)
        self.declare_parameter('load_over_current_ratio', 1.50)
        self.declare_parameter('load_turning_threshold', 0.20)
        self.declare_parameter('load_suspect_confirm_s', 0.30)
        self.declare_parameter('load_stall_confirm_s', 0.60)
        self.declare_parameter('load_recovery_confirm_s', 0.50)

        # ── 일반 주행 적응형 차동 조향(비침습 옵션) ──────────
        # false이면 기존 compute_wheel_rpms() 경로를 그대로 사용한다.
        self.declare_parameter('adaptive_steering_enabled', False)
        self.declare_parameter('adaptive_command_period_s', 0.2)
        self.declare_parameter('steer_slew_rate_per_sec', 1.0)
        self.declare_parameter('steer_low_speed_rpm', 4.0)
        self.declare_parameter('steer_mid_speed_rpm', 8.0)
        self.declare_parameter('steer_high_speed_rpm', 12.0)
        self.declare_parameter('steer_transition_rpm', 1.0)
        self.declare_parameter('steer_reduction_low', 0.50)
        self.declare_parameter('steer_reduction_mid', 0.30)
        self.declare_parameter('steer_reduction_high', 0.20)
        self.declare_parameter('steer_reduction_very_high', 0.12)
        self.declare_parameter('steer_speed_cap_mid_rpm', 10.0)
        self.declare_parameter('steer_speed_cap_high_rpm', 7.0)
        self.declare_parameter('steer_speed_cap_full_rpm', 5.0)

        # ── 선택형 조향 시험 전략(모두 기본 OFF) ─────────────
        # 기존 좌/우 부호, 모터 ID, 전/후진 방향표는 변경하지 않고
        # 기존 계산이 만든 좌/우 RPM 크기만 선택적으로 완화한다.
        self.declare_parameter('turn_strategy_command_period_s', 0.10)
        self.declare_parameter('turn_strategy_start_threshold', 0.15)
        self.declare_parameter('turn_strategy_release_threshold', 0.08)

        self.declare_parameter('wide_turn_enabled', False)
        self.declare_parameter('wide_turn_min_inner_ratio', 0.70)

        # 큰 조향 입력에서 펄스 ON 구간의 내측 휠을 더 강하게 감속한다.
        # 모터 방향은 바꾸지 않으므로 기존 전/후진 및 좌/우 방향표를 침범하지 않는다.
        self.declare_parameter('tight_turn_enabled', False)
        self.declare_parameter('tight_turn_start_steer', 0.55)
        self.declare_parameter('tight_turn_full_steer', 0.90)
        self.declare_parameter('tight_turn_min_inner_ratio', 0.20)
        self.declare_parameter('tight_turn_outer_speed_cap_rpm', 15.0)
        self.declare_parameter('tight_turn_ramp_s', 0.15)
        self.declare_parameter('tight_turn_only_with_pulse', True)

        self.declare_parameter('turn_entry_enabled', False)
        self.declare_parameter('turn_entry_speed_cap_rpm', 3.0)
        self.declare_parameter('turn_entry_hold_s', 0.30)
        self.declare_parameter('turn_entry_ramp_s', 0.50)

        self.declare_parameter('pulse_turn_enabled', False)
        self.declare_parameter('pulse_turn_on_s', 0.30)
        self.declare_parameter('pulse_turn_relief_s', 0.20)
        self.declare_parameter('pulse_turn_relief_speed_cap_rpm', 3.0)

        # LB/RB 제자리 회전 전용 펄스. RELIEF 구간은 완전 0RPM이다.
        self.declare_parameter('pivot_pulse_enabled', False)
        self.declare_parameter('pivot_pulse_rpm', 4.0)
        self.declare_parameter('pivot_pulse_on_s', 0.20)
        self.declare_parameter('pivot_pulse_relief_s', 0.20)
        self.declare_parameter('pivot_pulse_arrive_s', 0.10)

        self.declare_parameter('turn_release_enabled', False)
        self.declare_parameter('turn_release_straight_s', 0.40)
        self.declare_parameter('turn_release_speed_cap_rpm', 2.0)
        # true이면 조향 해제/카운터 스티어 중에도 현재 전진 요청 속도를 유지한다.
        # 기존 2~3RPM 강제 제한 때문에 발생하던 '조향 해제 직후 잠시 멈춤'을 방지한다.
        self.declare_parameter('turn_release_keep_drive_speed', False)

        self.declare_parameter('counter_steer_enabled', False)
        self.declare_parameter('counter_steer_ratio', 0.05)
        self.declare_parameter('counter_steer_duration_s', 0.15)
        self.declare_parameter('counter_steer_speed_cap_rpm', 1.5)
        self.declare_parameter('counter_steer_forward_only', True)

        self.declare_parameter('load_adaptive_turn_enabled', False)
        self.declare_parameter('load_turn_unload_diff_scale', 0.70)
        self.declare_parameter('load_turn_unload_speed_scale', 0.85)
        self.declare_parameter('load_turn_local_stall_diff_scale', 0.50)
        self.declare_parameter('load_turn_local_stall_speed_scale', 0.70)

        # ── Emergency Stop ─────────────────────────────────
        # 공통 토픽 규약:
        #   ESTOP / STOP / TRUE / 1 / ON  → E-Stop latch
        #   RESET / CLEAR / FALSE / 0 / OFF → latch 해제
        self.declare_parameter('emergency_stop_topic', '/mmr/emergency_stop')
        self.declare_parameter('joy_estop_enabled', True)
        self.declare_parameter('joy_estop_button_indices', [6, 7])
        self.declare_parameter('joy_estop_publish_period_sec', 0.5)
        self.declare_parameter('joy_reset_enabled', True)
        self.declare_parameter('joy_reset_button_indices', [6, 7, BUTTON_X])
        self.declare_parameter('joy_reset_hold_sec', 3.0)
        self.declare_parameter('joy_reset_require_neutral', True)
        self.declare_parameter('joy_reset_neutral_axis_indices', [0, 1, 3, 4])
        self.declare_parameter('joy_reset_neutral_deadzone', 0.15)

        port         = self.get_parameter('serial_port').value
        baudrate     = self.get_parameter('baudrate').value
        inter_ms     = self.get_parameter('inter_delay_ms').value

        requested_max_rpm = float(self.get_parameter('max_rpm').value)
        self._max_rpm = max(0.0, min(requested_max_rpm, HARD_MAX_RPM))

        self._manual_pivot_rpm = float(
            self.get_parameter('manual_pivot_rpm').value
        )
        self._manual_pivot_rpm = max(
            0.0,
            min(self._manual_pivot_rpm, self._max_rpm),
        )

        self._default_pivot_rpm = float(
            self.get_parameter('default_pivot_rpm').value
        )
        self._default_pivot_rpm = max(
            0.0,
            min(self._default_pivot_rpm, self._max_rpm),
        )
        self._auto_align_motion_mode = str(
            self.get_parameter('auto_align_motion_mode').value
        ).strip().lower()
        if self._auto_align_motion_mode not in {'pivot', 'arc_forward'}:
            self.get_logger().warn(
                'auto_align_motion_mode 값이 유효하지 않아 pivot으로 대체합니다: '
                f'{self._auto_align_motion_mode}'
            )
            self._auto_align_motion_mode = 'pivot'
        self._auto_align_arc_inner_ratio = _clamp(
            float(self.get_parameter('auto_align_arc_inner_ratio').value),
            0.0,
            1.0,
        )
        self._align_cmd_timeout_sec = max(
            0.1,
            float(self.get_parameter('align_cmd_timeout_sec').value),
        )

        self._arrive_s  = float(self.get_parameter('arrive_s').value)
        self._inter_s   = inter_ms / 1000.0
        joy_topic    = self.get_parameter('joy_topic').value
        align_cmd_topic = str(self.get_parameter('align_cmd_topic').value)
        drive_status_topic = str(
            self.get_parameter('drive_status_topic').value
        )
        self._emergency_stop_topic = str(
            self.get_parameter('emergency_stop_topic').value
        )
        self._joy_estop_enabled = bool(
            self.get_parameter('joy_estop_enabled').value
        )
        self._joy_estop_button_indices = self._parse_button_index_list(
            self.get_parameter('joy_estop_button_indices').value,
            default=[6, 7],
        )
        self._joy_estop_publish_period_sec = max(
            0.1,
            float(self.get_parameter('joy_estop_publish_period_sec').value),
        )
        self._joy_reset_enabled = bool(
            self.get_parameter('joy_reset_enabled').value
        )
        self._joy_reset_button_indices = self._parse_button_index_list(
            self.get_parameter('joy_reset_button_indices').value,
            default=[6, 7, BUTTON_X],
        )
        self._joy_reset_hold_sec = max(
            0.5,
            float(self.get_parameter('joy_reset_hold_sec').value),
        )
        self._joy_reset_require_neutral = bool(
            self.get_parameter('joy_reset_require_neutral').value
        )
        self._joy_reset_neutral_axis_indices = self._parse_button_index_list(
            self.get_parameter('joy_reset_neutral_axis_indices').value,
            default=[0, 1, 3, 4],
        )
        self._joy_reset_neutral_deadzone = max(
            0.01,
            float(self.get_parameter('joy_reset_neutral_deadzone').value),
        )
        self._stop_rpm_threshold = float(
            self.get_parameter('stop_rpm_threshold').value
        )
        self._feedback_fresh_timeout_s = float(
            self.get_parameter('feedback_fresh_timeout_s').value
        )

        self._load_observer_enabled = bool(
            self.get_parameter('load_observer_enabled').value
        )
        requested_load_protection = bool(
            self.get_parameter('load_protection_enabled').value
        )
        # 이번 패치는 Shadow mode 전용이다. 보호 요청이 실수로 true여도
        # 기존 모터 명령에 개입하지 않도록 강제로 비활성화한다.
        self._load_protection_enabled = False
        if requested_load_protection:
            self.get_logger().warn(
                'load_protection_enabled=true 요청됨. 현재 패치는 Shadow mode 전용이므로 '
                '실제 감속/정지 명령에는 적용하지 않습니다.'
            )
        self._load_observer = LoadObserver(
            min_target_rpm=float(
                self.get_parameter('load_min_target_rpm').value
            ),
            feedback_max_age_s=float(
                self.get_parameter('load_feedback_max_age_s').value
            ),
            progressing_min_ratio=float(
                self.get_parameter('load_progressing_min_ratio').value
            ),
            stall_max_ratio=float(
                self.get_parameter('load_stall_max_ratio').value
            ),
            unload_current_ratio=float(
                self.get_parameter('load_unload_current_ratio').value
            ),
            over_current_ratio=float(
                self.get_parameter('load_over_current_ratio').value
            ),
            turning_threshold=float(
                self.get_parameter('load_turning_threshold').value
            ),
            suspect_confirm_s=float(
                self.get_parameter('load_suspect_confirm_s').value
            ),
            stall_confirm_s=float(
                self.get_parameter('load_stall_confirm_s').value
            ),
            recovery_confirm_s=float(
                self.get_parameter('load_recovery_confirm_s').value
            ),
        )

        self._adaptive_steering_enabled = bool(
            self.get_parameter('adaptive_steering_enabled').value
        )
        self._adaptive_command_period_s = max(
            0.1,
            float(self.get_parameter('adaptive_command_period_s').value),
        )
        self._steer_slew_rate_per_sec = max(
            0.1,
            float(self.get_parameter('steer_slew_rate_per_sec').value),
        )
        self._steer_low_speed_rpm = max(
            0.0,
            float(self.get_parameter('steer_low_speed_rpm').value),
        )
        self._steer_mid_speed_rpm = max(
            self._steer_low_speed_rpm + 0.1,
            float(self.get_parameter('steer_mid_speed_rpm').value),
        )
        self._steer_high_speed_rpm = max(
            self._steer_mid_speed_rpm + 0.1,
            float(self.get_parameter('steer_high_speed_rpm').value),
        )
        self._steer_transition_rpm = max(
            0.1,
            float(self.get_parameter('steer_transition_rpm').value),
        )
        self._steer_reduction_low = _clamp(
            float(self.get_parameter('steer_reduction_low').value),
            0.0,
            0.95,
        )
        self._steer_reduction_mid = _clamp(
            float(self.get_parameter('steer_reduction_mid').value),
            0.0,
            self._steer_reduction_low,
        )
        self._steer_reduction_high = _clamp(
            float(self.get_parameter('steer_reduction_high').value),
            0.0,
            self._steer_reduction_mid,
        )
        self._steer_reduction_very_high = _clamp(
            float(self.get_parameter('steer_reduction_very_high').value),
            0.0,
            self._steer_reduction_high,
        )
        self._steer_speed_cap_mid_rpm = _clamp(
            float(self.get_parameter('steer_speed_cap_mid_rpm').value),
            0.0,
            self._max_rpm,
        )
        self._steer_speed_cap_high_rpm = _clamp(
            float(self.get_parameter('steer_speed_cap_high_rpm').value),
            0.0,
            self._steer_speed_cap_mid_rpm,
        )
        self._steer_speed_cap_full_rpm = _clamp(
            float(self.get_parameter('steer_speed_cap_full_rpm').value),
            0.0,
            self._steer_speed_cap_high_rpm,
        )

        self._turn_strategy_command_period_s = max(
            0.05,
            float(self.get_parameter('turn_strategy_command_period_s').value),
        )
        self._turn_strategy = TurnStrategyController(
            TurnStrategyConfig(
                start_threshold=float(
                    self.get_parameter('turn_strategy_start_threshold').value
                ),
                release_threshold=float(
                    self.get_parameter('turn_strategy_release_threshold').value
                ),
                wide_turn_enabled=bool(
                    self.get_parameter('wide_turn_enabled').value
                ),
                wide_turn_min_inner_ratio=float(
                    self.get_parameter('wide_turn_min_inner_ratio').value
                ),
                tight_turn_enabled=bool(
                    self.get_parameter('tight_turn_enabled').value
                ),
                tight_turn_start_steer=float(
                    self.get_parameter('tight_turn_start_steer').value
                ),
                tight_turn_full_steer=float(
                    self.get_parameter('tight_turn_full_steer').value
                ),
                tight_turn_min_inner_ratio=float(
                    self.get_parameter('tight_turn_min_inner_ratio').value
                ),
                tight_turn_outer_speed_cap_rpm=float(
                    self.get_parameter(
                        'tight_turn_outer_speed_cap_rpm'
                    ).value
                ),
                tight_turn_ramp_s=float(
                    self.get_parameter('tight_turn_ramp_s').value
                ),
                tight_turn_only_with_pulse=bool(
                    self.get_parameter(
                        'tight_turn_only_with_pulse'
                    ).value
                ),
                turn_entry_enabled=bool(
                    self.get_parameter('turn_entry_enabled').value
                ),
                turn_entry_speed_cap_rpm=float(
                    self.get_parameter('turn_entry_speed_cap_rpm').value
                ),
                turn_entry_hold_s=float(
                    self.get_parameter('turn_entry_hold_s').value
                ),
                turn_entry_ramp_s=float(
                    self.get_parameter('turn_entry_ramp_s').value
                ),
                pulse_turn_enabled=bool(
                    self.get_parameter('pulse_turn_enabled').value
                ),
                pulse_turn_on_s=float(
                    self.get_parameter('pulse_turn_on_s').value
                ),
                pulse_turn_relief_s=float(
                    self.get_parameter('pulse_turn_relief_s').value
                ),
                pulse_turn_relief_speed_cap_rpm=float(
                    self.get_parameter(
                        'pulse_turn_relief_speed_cap_rpm'
                    ).value
                ),
                turn_release_enabled=bool(
                    self.get_parameter('turn_release_enabled').value
                ),
                turn_release_straight_s=float(
                    self.get_parameter('turn_release_straight_s').value
                ),
                turn_release_speed_cap_rpm=float(
                    self.get_parameter('turn_release_speed_cap_rpm').value
                ),
                turn_release_keep_drive_speed=bool(
                    self.get_parameter(
                        'turn_release_keep_drive_speed'
                    ).value
                ),
                counter_steer_enabled=bool(
                    self.get_parameter('counter_steer_enabled').value
                ),
                counter_steer_ratio=float(
                    self.get_parameter('counter_steer_ratio').value
                ),
                counter_steer_duration_s=float(
                    self.get_parameter('counter_steer_duration_s').value
                ),
                counter_steer_speed_cap_rpm=float(
                    self.get_parameter('counter_steer_speed_cap_rpm').value
                ),
                counter_steer_forward_only=bool(
                    self.get_parameter('counter_steer_forward_only').value
                ),
                load_adaptive_turn_enabled=(
                    bool(
                        self.get_parameter(
                            'load_adaptive_turn_enabled'
                        ).value
                    )
                    and self._load_observer_enabled
                ),
                load_turn_unload_diff_scale=float(
                    self.get_parameter('load_turn_unload_diff_scale').value
                ),
                load_turn_unload_speed_scale=float(
                    self.get_parameter('load_turn_unload_speed_scale').value
                ),
                load_turn_local_stall_diff_scale=float(
                    self.get_parameter(
                        'load_turn_local_stall_diff_scale'
                    ).value
                ),
                load_turn_local_stall_speed_scale=float(
                    self.get_parameter(
                        'load_turn_local_stall_speed_scale'
                    ).value
                ),
            )
        )

        self._pivot_pulse = PivotPulseController(
            PivotPulseConfig(
                enabled=bool(
                    self.get_parameter('pivot_pulse_enabled').value
                ),
                rpm=float(self.get_parameter('pivot_pulse_rpm').value),
                on_s=float(self.get_parameter('pivot_pulse_on_s').value),
                relief_s=float(
                    self.get_parameter('pivot_pulse_relief_s').value
                ),
            )
        )
        self._pivot_pulse_arrive_s = max(
            0.1,
            float(self.get_parameter('pivot_pulse_arrive_s').value),
        )

        self._feedback_enabled = bool(
            self.get_parameter('feedback_enabled').value
        )
        self._feedback_ids = [
            int(dev_id)
            for dev_id in self.get_parameter(
                'feedback_ids'
            ).value
        ]

        if not self._feedback_ids:
            self._feedback_ids = [1, 2, 3, 4, 5, 6]

        invalid_ids = [
            dev_id
            for dev_id in self._feedback_ids
            if dev_id not in ALL_IDS
        ]

        if invalid_ids:
            raise ValueError(
                f'허용되지 않은 피드백 ID: {invalid_ids}'
            )

        self._feedback_index = 0
        self._feedback_period_s = float(
            self.get_parameter('feedback_period_s').value
        )
        self._feedback_timeout_s = float(
            self.get_parameter('feedback_timeout_s').value
        )

        # ── RS-485 링크 ────────────────────────────────────
        self._link = RS485Link(port, baudrate)
        try:
            self._link.open()
            self.get_logger().info(
                f'RS-485 포트 열림: {port} @ {baudrate}bps'
            )
        except Exception as e:
            self.get_logger().error(f'RS-485 포트 열기 실패: {e}')

        # ── 드라이버 초기 제어 On ───────────────────────────
        self._send_control_on()

        # ── 상태 추적 (중복 전송 억제) ────────────────────────
        # 직전 전송한 (left_rpm, right_rpm, forward) 를 기억해
        # 값이 동일한 경우 재전송하지 않음
        self._prev_state: object | None = None

        # 적응형 조향 상태. Joy autorepeat(통합 런치 기본 20Hz)를 받아
        # 조향값은 연속 갱신하되 RS-485 명령은 adaptive_command_period_s로 제한한다.
        adaptive_now = time.monotonic()
        self._adaptive_target_steer = 0.0
        self._adaptive_applied_steer = 0.0
        self._adaptive_last_update_time = adaptive_now
        self._adaptive_last_send_time = 0.0
        self._adaptive_last_forward: bool | None = None
        self._adaptive_debug = {
            'requested_base_rpm': 0.0,
            'speed_cap_rpm': self._max_rpm,
            'base_rpm': 0.0,
            'max_reduction': self._steer_reduction_low,
            'applied_reduction': 0.0,
        }
        self._observer_steer = 0.0
        self._latest_load_observation = self._load_observer.latest
        self._turn_strategy_last_send_time = 0.0
        self._turn_strategy_last_forward: bool | None = None
        self._turn_strategy_debug = self._turn_strategy.latest_debug
        self._pivot_pulse_debug = self._pivot_pulse.latest_debug

        # ── 구동 상태 추적 (/mmr/drive/status 발행용) ──────
        now_mono = time.monotonic()
        self._mode = 'STOPPED'
        self._last_command_time = now_mono
        self._last_manual_command_time = 0.0
        self._auto_align_active = False

        # E-Stop latch 상태.
        # E-Stop은 한 번 걸리면 RESET 메시지가 오기 전까지 주행/자동정렬 명령을 모두 차단한다.
        self._estop_latched = False
        self._estop_stop_sent = False
        self._estop_release_seen = True
        self._last_estop_publish_time = 0.0
        # Joy RESET이 완료된 직후에도 BACK+START가 계속 눌려 있으면
        # 다음 Joy 프레임에서 다시 ESTOP으로 해석될 수 있다.
        # RESET 완료 후에는 버튼을 한 번 모두 뗄 때까지 E-Stop 콤보를 무시한다.
        self._joy_estop_inhibit_until_release = False
        self._joy_reset_hold_started_at: float | None = None
        self._joy_reset_hold_progress = 0.0
        self._last_align_cmd_time = 0.0
        self._last_align_yaw_error_deg = 0.0
        self._target_rpm_by_id = {mid: 0.0 for mid in ALL_IDS}
        self._feedback_by_id = {
            mid: {
                'ok': False,
                'raw_rpm': 0.0,
                'actual_rpm': 0.0,
                'direction': None,
                'position_deg': None,
                'current_a': None,
                'last_feedback_time': 0.0,
                'timeout_count': 0,
            }
            for mid in ALL_IDS
        }

        # ── 구동 상태 토픽 발행 ─────────────────────────────
        self._drive_status_pub = self.create_publisher(
            String,
            drive_status_topic,
            10,
        )
        self._emergency_stop_pub = self.create_publisher(
            String,
            self._emergency_stop_topic,
            10,
        )
        self._emergency_stop_sub = self.create_subscription(
            String,
            self._emergency_stop_topic,
            self._emergency_stop_callback,
            10,
        )
        self._status_timer = self.create_timer(
            0.1,
            self._publish_drive_status,
        )

        # ── Joy 구독 ───────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_callback, qos
        )

        # ── 자동 정렬 명령 구독 ──────────────────────────────
        # aim_coordinator가 발행하는 /mmr/drive/align_cmd를 받아
        # 기존 BLDC 제자리 회전 명령으로만 연결한다.
        self._align_cmd_sub = self.create_subscription(
            String,
            align_cmd_topic,
            self._align_cmd_callback,
            10,
        )

        # ── Watchdog: 조이스틱/자동 정렬 명령 끊김 감시 ───────
        self._last_joy_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(
            0.5,
            self._watchdog_cb,
        )

        self._feedback_timer = None

        if self._feedback_enabled and self._link.is_open:
            self._feedback_timer = self.create_timer(
                self._feedback_period_s,
                self._feedback_cb,
            )

            self.get_logger().info(
                '엔코더 피드백 활성화: '
                f'ID {self._feedback_ids}, '
                f'요청 간격 {self._feedback_period_s:.2f}s'
            )

        if self._adaptive_steering_enabled:
            self.get_logger().info(
                '적응형 차동 조향 활성화: '
                f'cmd_period={self._adaptive_command_period_s:.2f}s, '
                f'steer_slew={self._steer_slew_rate_per_sec:.2f}/s, '
                f'full_steer_cap={self._steer_speed_cap_full_rpm:.1f}RPM'
            )
        else:
            self.get_logger().info(
                '적응형 차동 조향 비활성화 — 기존 차동 조향 사용'
            )

        if self._load_observer_enabled:
            self.get_logger().info(
                'RPM·전류 부하 관측기 활성화 — Shadow mode, 모터 명령 개입 없음'
            )
        else:
            self.get_logger().info('RPM·전류 부하 관측기 비활성화')

        if self._turn_strategy.enabled:
            self.get_logger().warn(
                '선택형 조향 시험 전략 활성화: '
                + ', '.join(self._turn_strategy.enabled_features)
                + '. 기존 방향/부호는 유지하고 좌우 RPM 크기만 조정합니다.'
            )
        else:
            self.get_logger().info(
                '선택형 조향 시험 전략 모두 OFF — 기존 조향 출력 그대로 사용'
            )

        if self._turn_strategy.config.turn_release_keep_drive_speed:
            self.get_logger().warn(
                '조향 해제 속도 유지 활성화 — 전진 입력 유지 시 '
                'RELEASE/COUNTER 구간에서 저속 강제 제한을 사용하지 않습니다.'
            )

        if self._pivot_pulse.config.enabled:
            self.get_logger().warn(
                '수동 제자리 펄스 회전 활성화: '
                f'{self._pivot_pulse.config.rpm:.1f}RPM, '
                f'ON={self._pivot_pulse.config.on_s:.2f}s, '
                f'RELIEF={self._pivot_pulse.config.relief_s:.2f}s'
            )
        else:
            self.get_logger().info('수동 제자리 회전은 기존 연속 방식 사용')

        self.get_logger().info('bldc_drive_node 준비 완료.')

    # ── 내부 헬퍼 ───────────────────────────────────────────

    @staticmethod
    def _parse_button_index_list(value, default: list[int]) -> list[int]:
        """ROS 파라미터에서 버튼/축 인덱스 리스트를 안전하게 정수 리스트로 변환한다."""
        try:
            parsed = [int(v) for v in list(value)]
        except Exception:
            parsed = list(default)

        parsed = [v for v in parsed if v >= 0]
        return parsed if parsed else list(default)

    @staticmethod
    def _buttons_pressed(buttons, indices: list[int]) -> bool:
        if not indices:
            return False
        for idx in indices:
            if len(buttons) <= idx or int(buttons[idx]) != 1:
                return False
        return True

    @staticmethod
    def _all_buttons_released(buttons) -> bool:
        return all(int(v) == 0 for v in buttons)

    def _reset_axes_neutral(self, axes) -> bool:
        if not self._joy_reset_require_neutral:
            return True

        for idx in self._joy_reset_neutral_axis_indices:
            if len(axes) <= idx:
                continue
            try:
                if abs(float(axes[idx])) > self._joy_reset_neutral_deadzone:
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def _publish_emergency_stop(self, command: str, source: str) -> None:
        """공통 E-Stop 토픽으로 ESTOP 또는 RESET 명령을 발행한다."""
        cmd = str(command).strip().upper()
        if cmd not in {'ESTOP', 'RESET'}:
            return

        msg = String()
        msg.data = cmd
        self._emergency_stop_pub.publish(msg)
        self.get_logger().warn(f'[E-STOP] {cmd} publish from {source}')

    def _activate_estop(self, source: str) -> None:
        """구동부 E-Stop latch를 걸고 모든 BLDC 드라이버를 정지/제어 Off한다."""
        if self._estop_latched and self._estop_stop_sent:
            self._mode = 'ESTOP'
            return

        self._estop_latched = True
        self._estop_stop_sent = True
        self._estop_release_seen = False
        self._joy_reset_hold_started_at = None
        self._joy_reset_hold_progress = 0.0
        self._auto_align_active = False
        self._last_align_yaw_error_deg = 0.0
        self._set_all_targets_zero()
        self._reset_adaptive_steering_state()
        self._reset_turn_strategy_state()
        self._reset_pivot_pulse_state()

        self.get_logger().error(f'[E-STOP] ACTIVATED by {source} — stop all BLDC')
        self._send_stop()
        self._mode = 'ESTOP'
        self._last_command_time = time.monotonic()

    def _clear_estop(self, source: str) -> None:
        """E-Stop latch를 해제하되, 즉시 이동하지 않고 0RPM 정지 상태로만 복귀한다."""
        if not self._estop_latched:
            return

        self.get_logger().warn(f'[E-STOP] RESET by {source} — control ON, hold 0RPM')
        self._estop_latched = False
        self._estop_stop_sent = False
        self._estop_release_seen = True
        self._joy_reset_hold_started_at = None
        self._joy_reset_hold_progress = 0.0
        self._auto_align_active = False
        self._last_align_yaw_error_deg = 0.0
        self._prev_state = None
        self._reset_adaptive_steering_state()
        self._reset_turn_strategy_state()
        self._reset_pivot_pulse_state()

        self._send_control_on()
        self._send_zero_velocity()
        self._mode = 'STOPPED'
        self._last_command_time = time.monotonic()

    def _emergency_stop_callback(self, msg: String) -> None:
        command = str(msg.data).strip().upper()

        if command in {'ESTOP', 'STOP', 'TRUE', '1', 'ON', 'EMERGENCY_STOP'}:
            self._activate_estop(source='topic')
            return

        if command in {'RESET', 'CLEAR', 'FALSE', '0', 'OFF'}:
            self._clear_estop(source='topic')
            return

        self.get_logger().warn(
            f'[E-STOP] 알 수 없는 명령: {msg.data}',
            throttle_duration_sec=2.0,
        )

    def _handle_joy_estop_reset(self, axes, buttons) -> bool:
        """Joy 콜백 앞단에서 E-Stop/Reset 콤보를 처리한다.

        반환값 True는 현재 Joy 메시지로 일반 주행 처리를 계속하지 말라는 뜻이다.
        """
        now = time.monotonic()

        estop_combo = self._buttons_pressed(
            buttons,
            self._joy_estop_button_indices,
        )
        reset_combo = self._buttons_pressed(
            buttons,
            self._joy_reset_button_indices,
        )

        if self._joy_estop_inhibit_until_release:
            if self._all_buttons_released(buttons):
                self._joy_estop_inhibit_until_release = False
                self._joy_reset_hold_started_at = None
                self._joy_reset_hold_progress = 0.0
                return True
            return True

        # 이미 E-Stop 상태에서는 RESET 콤보가 BACK+START를 포함하므로,
        # ESTOP 콤보보다 RESET 판정을 먼저 수행해야 한다.
        if self._estop_latched:
            if self._all_buttons_released(buttons):
                self._estop_release_seen = True
                self._joy_reset_hold_started_at = None
                self._joy_reset_hold_progress = 0.0
                return True

            if not self._joy_reset_enabled:
                return True

            neutral = self._reset_axes_neutral(axes)
            reset_allowed = self._estop_release_seen and neutral and reset_combo

            if reset_allowed:
                if self._joy_reset_hold_started_at is None:
                    self._joy_reset_hold_started_at = now
                    self._joy_reset_hold_progress = 0.0
                    self.get_logger().warn(
                        '[E-STOP] Joy RESET hold started',
                        throttle_duration_sec=1.0,
                    )
                else:
                    elapsed = now - self._joy_reset_hold_started_at
                    self._joy_reset_hold_progress = min(
                        1.0,
                        elapsed / self._joy_reset_hold_sec,
                    )
                    if elapsed >= self._joy_reset_hold_sec:
                        self._publish_emergency_stop('RESET', source='joy')
                        # RESET 콤보에는 E-STOP 콤보(BACK+START)가 포함된다.
                        # RESET 직후 버튼이 계속 눌려 있는 다음 Joy 프레임에서
                        # 다시 ESTOP이 걸리지 않도록 버튼 전체 release를 요구한다.
                        self._joy_estop_inhibit_until_release = True
                        self._clear_estop(source='joy')
                        return True
            else:
                self._joy_reset_hold_started_at = None
                self._joy_reset_hold_progress = 0.0

            return True

        if self._joy_estop_enabled and estop_combo:
            if now - self._last_estop_publish_time >= self._joy_estop_publish_period_sec:
                self._last_estop_publish_time = now
                self._publish_emergency_stop('ESTOP', source='joy')
            self._activate_estop(source='joy')
            return True

        self._joy_reset_hold_started_at = None
        self._joy_reset_hold_progress = 0.0
        return False

    def _reset_adaptive_steering_state(self) -> None:
        """정지/Pivot/E-Stop 전환 시 일반 조향 필터 상태를 중립으로 되돌린다."""
        now = time.monotonic()
        self._adaptive_target_steer = 0.0
        self._adaptive_applied_steer = 0.0
        self._adaptive_last_update_time = now
        self._adaptive_last_send_time = 0.0
        self._adaptive_last_forward = None
        self._adaptive_debug = {
            'requested_base_rpm': 0.0,
            'speed_cap_rpm': self._max_rpm,
            'base_rpm': 0.0,
            'max_reduction': self._steer_reduction_low,
            'applied_reduction': 0.0,
        }

    def _reset_turn_strategy_state(self) -> None:
        """정지/Pivot/E-Stop 전환 시 선택형 조향 상태를 초기화한다."""
        now = time.monotonic()
        self._turn_strategy.reset(now)
        self._turn_strategy_last_send_time = 0.0
        self._turn_strategy_last_forward = None
        self._turn_strategy_debug = self._turn_strategy.latest_debug

    def _reset_pivot_pulse_state(self) -> None:
        """LB/RB 제자리 펄스 상태를 IDLE로 초기화한다."""
        now = time.monotonic()
        self._pivot_pulse.reset(now)
        self._pivot_pulse_debug = self._pivot_pulse.latest_debug

    def _compute_adaptive_drive_command(
        self,
        linear: float,
        target_steer: float,
        now: float,
    ) -> tuple[float, float]:
        """조향 slew와 속도 연동 제한을 적용해 좌우 RPM을 계산한다."""
        dt = _clamp(now - self._adaptive_last_update_time, 0.0, 0.5)
        self._adaptive_last_update_time = now
        self._adaptive_target_steer = _clamp(target_steer, -1.0, 1.0)
        self._adaptive_applied_steer = slew_towards(
            self._adaptive_applied_steer,
            self._adaptive_target_steer,
            self._steer_slew_rate_per_sec,
            dt,
        )

        left_rpm, right_rpm, debug = compute_adaptive_wheel_rpms(
            linear=linear,
            steer=self._adaptive_applied_steer,
            max_rpm=self._max_rpm,
            low_speed_rpm=self._steer_low_speed_rpm,
            mid_speed_rpm=self._steer_mid_speed_rpm,
            high_speed_rpm=self._steer_high_speed_rpm,
            transition_rpm=self._steer_transition_rpm,
            reduction_low=self._steer_reduction_low,
            reduction_mid=self._steer_reduction_mid,
            reduction_high=self._steer_reduction_high,
            reduction_very_high=self._steer_reduction_very_high,
            speed_cap_mid_rpm=self._steer_speed_cap_mid_rpm,
            speed_cap_high_rpm=self._steer_speed_cap_high_rpm,
            speed_cap_full_rpm=self._steer_speed_cap_full_rpm,
        )
        self._adaptive_debug = debug
        return left_rpm, right_rpm

    def _send_control_on(self) -> None:
        """전체 모터 드라이버 제어 On (0x0C 0x00)."""
        pkts = [make_control_onoff_packet(mid, on=True) for mid in ALL_IDS]
        self._link.send_packets(pkts, self._inter_s)
        self.get_logger().info('모든 드라이버 제어 ON 전송')

    def _set_all_targets_zero(self) -> None:
        """GUI/상태 토픽용 목표 RPM을 0으로 갱신한다."""
        for mid in ALL_IDS:
            self._target_rpm_by_id[mid] = 0.0

    def _set_group_targets(
        self,
        left_dir: int,
        left_rpm: float,
        right_dir: int,
        right_rpm: float,
    ) -> None:
        """좌/우 그룹 목표 RPM을 로봇 기준 signed RPM으로 저장한다."""
        for mid in LEFT_IDS:
            self._target_rpm_by_id[mid] = signed_rpm_from_direction(
                mid, left_dir, left_rpm
            )
        for mid in RIGHT_IDS:
            self._target_rpm_by_id[mid] = signed_rpm_from_direction(
                mid, right_dir, right_rpm
            )

    def _send_stop(self) -> None:
        """전체 모터 0RPM 전송 후 제어 Off."""
        self._set_all_targets_zero()
        self._mode = 'STOPPED_OFF'
        self._last_command_time = time.monotonic()

        # 1단계: 0RPM
        pkts = []
        for mid in LEFT_IDS:
            pkts.append(make_acc_speed_packet(mid, DIR_CCW, 0.0, 0.3))
        for mid in RIGHT_IDS:
            pkts.append(make_acc_speed_packet(mid, DIR_CW, 0.0, 0.3))
        self._link.send_packets(pkts, self._inter_s)
        time.sleep(0.35)

        # 2단계: 제어 Off
        off_pkts = [make_control_onoff_packet(mid, on=False) for mid in ALL_IDS]
        self._link.send_packets(off_pkts, self._inter_s)
        self._prev_state = None
        self.get_logger().info('정지 및 제어 Off 전송')

    def _send_velocity(self, left_rpm: float, right_rpm: float, forward: bool) -> None:
        """
        양쪽 휠에 0x03 패킷 전송.
        전진(forward=True) : 좌=CCW / 우=CW
        후진(forward=False): 좌=CW  / 우=CCW
        """
        if self._estop_latched:
            self._set_all_targets_zero()
            self._mode = 'ESTOP'
            return

        # 일반 주행으로 전환되면 다음 LB/RB 입력은 항상 PIVOT ON부터 시작한다.
        self._reset_pivot_pulse_state()

        if forward:
            l_dir, r_dir = DIR_CCW, DIR_CW
        else:
            l_dir, r_dir = DIR_CW, DIR_CCW

        self._set_group_targets(l_dir, left_rpm, r_dir, right_rpm)
        self._mode = 'MANUAL_DRIVE'
        self._last_command_time = time.monotonic()
        self._last_manual_command_time = self._last_command_time

        # 중복 전송 억제 (소수점 1자리 반올림 수준에서 비교)
        state = (round(left_rpm, 1), round(right_rpm, 1), forward)
        if state == self._prev_state:
            return
        self._prev_state = state

        pkts = []
        for mid in LEFT_IDS:
            pkts.append(
                make_acc_speed_packet(mid, l_dir, left_rpm, self._arrive_s)
            )
        for mid in RIGHT_IDS:
            pkts.append(
                make_acc_speed_packet(mid, r_dir, right_rpm, self._arrive_s)
            )

        self._link.send_packets(pkts, self._inter_s)

    def _send_zero_velocity(self) -> None:
        """제어 Off 없이 전체 모터에 0RPM만 전송한다."""
        self._observer_steer = 0.0
        self._reset_adaptive_steering_state()
        self._reset_turn_strategy_state()
        self._reset_pivot_pulse_state()
        if self._estop_latched:
            self._set_all_targets_zero()
            self._mode = 'ESTOP'
            return

        self._set_all_targets_zero()
        self._mode = 'STOPPED'
        self._last_command_time = time.monotonic()

        zero_state = (0.0, 0.0, True)
        if self._prev_state == zero_state:
            return

        pkts = []
        for mid in LEFT_IDS:
            pkts.append(make_acc_speed_packet(mid, DIR_CCW, 0.0, self._arrive_s))
        for mid in RIGHT_IDS:
            pkts.append(make_acc_speed_packet(mid, DIR_CW, 0.0, self._arrive_s))

        self._link.send_packets(pkts, self._inter_s)
        self._prev_state = zero_state

    def _send_pivot_relief_zero(self, turn_left: bool) -> None:
        """제자리 펄스 RELIEF 구간용 0RPM 전송.

        일반 ``_send_zero_velocity``와 달리 펄스 상태를 초기화하지 않는다.
        버튼을 계속 누르고 있으면 다음 ON 구간으로 정상 복귀한다.
        """
        if self._estop_latched:
            self._set_all_targets_zero()
            self._mode = 'ESTOP'
            return

        self._set_all_targets_zero()
        self._mode = (
            'MANUAL_PIVOT_LEFT_RELIEF'
            if turn_left
            else 'MANUAL_PIVOT_RIGHT_RELIEF'
        )
        self._last_command_time = time.monotonic()
        self._last_manual_command_time = self._last_command_time

        state = ('manual_pivot_relief', bool(turn_left))
        if self._prev_state == state:
            return
        self._prev_state = state

        # 0RPM에서는 방향 비트가 운동에 영향을 주지 않지만, 기존 pivot
        # 방향표를 그대로 사용해 로그/패킷 일관성을 유지한다.
        if turn_left:
            left_dir = DIR_CW
            right_dir = DIR_CW
        else:
            left_dir = DIR_CCW
            right_dir = DIR_CCW

        pkts = []
        for mid in LEFT_IDS:
            pkts.append(
                make_acc_speed_packet(
                    mid, left_dir, 0.0, self._pivot_pulse_arrive_s
                )
            )
        for mid in RIGHT_IDS:
            pkts.append(
                make_acc_speed_packet(
                    mid, right_dir, 0.0, self._pivot_pulse_arrive_s
                )
            )
        self._link.send_packets(pkts, self._inter_s)

    def _send_manual_pivot(self, turn_left: bool) -> None:
        """LB/RB 수동 제자리 회전 명령을 전송한다.

        turn_left=True  : 좌측 바퀴 후진, 우측 바퀴 전진 → 제자리 좌회전
        turn_left=False : 좌측 바퀴 전진, 우측 바퀴 후진 → 제자리 우회전

        ``pivot_pulse_enabled=true``이면 기존 방향표는 그대로 두고
        PIVOT/0RPM RELIEF만 반복한다.
        """
        self._observer_steer = 1.0 if turn_left else -1.0
        self._reset_adaptive_steering_state()
        self._reset_turn_strategy_state()
        if self._estop_latched:
            self._mode = 'ESTOP'
            return

        if self._pivot_pulse.config.enabled:
            turn_sign = 1 if turn_left else -1
            rpm, debug = self._pivot_pulse.apply(
                turn_sign=turn_sign,
                now=time.monotonic(),
            )
            self._pivot_pulse_debug = debug
            rpm = max(0.0, min(rpm, self._max_rpm))
            if rpm <= 0.0:
                self._send_pivot_relief_zero(turn_left=turn_left)
                return
            arrive_s = self._pivot_pulse_arrive_s
            phase_tag = str(debug.get('phase', 'PIVOT')).lower()
        else:
            self._reset_pivot_pulse_state()
            rpm = max(0.0, min(self._manual_pivot_rpm, self._max_rpm))
            if rpm == 0.0:
                self._send_zero_velocity()
                return
            arrive_s = self._arrive_s
            phase_tag = 'continuous'

        if turn_left:
            state = ('manual_pivot_left', phase_tag, round(rpm, 1))
            left_dir = DIR_CW    # 좌측 후진
            right_dir = DIR_CW   # 우측 전진
        else:
            state = ('manual_pivot_right', phase_tag, round(rpm, 1))
            left_dir = DIR_CCW   # 좌측 전진
            right_dir = DIR_CCW  # 우측 후진

        self._set_group_targets(left_dir, rpm, right_dir, rpm)
        self._mode = 'MANUAL_PIVOT_LEFT' if turn_left else 'MANUAL_PIVOT_RIGHT'
        self._last_command_time = time.monotonic()
        self._last_manual_command_time = self._last_command_time

        if self._prev_state == state:
            return
        self._prev_state = state

        pkts = []
        for mid in LEFT_IDS:
            pkts.append(make_acc_speed_packet(mid, left_dir, rpm, arrive_s))
        for mid in RIGHT_IDS:
            pkts.append(make_acc_speed_packet(mid, right_dir, rpm, arrive_s))

        self._link.send_packets(pkts, self._inter_s)

    def _send_auto_pivot(self, turn_left: bool, rpm: float) -> None:
        """자동 정렬용 제자리 회전 명령을 전송한다.

        기존 수동 제자리 회전과 같은 방향 테이블을 사용하되,
        _last_manual_command_time은 갱신하지 않는다.
        aim_coordinator가 자동 정렬을 수동 override로 오판하지 않게 하기 위함이다.
        """
        self._observer_steer = 1.0 if turn_left else -1.0
        self._reset_adaptive_steering_state()
        self._reset_turn_strategy_state()
        self._reset_pivot_pulse_state()
        if self._estop_latched:
            self._auto_align_active = False
            self._last_align_yaw_error_deg = 0.0
            self._mode = 'ESTOP'
            return

        rpm = max(0.0, min(float(rpm), self._max_rpm))
        if rpm == 0.0:
            self._send_zero_velocity()
            return

        if turn_left:
            state = ('auto_pivot_left', round(rpm, 1))
            left_dir = DIR_CW    # 좌측 후진
            right_dir = DIR_CW   # 우측 전진
            self._mode = 'AUTO_ALIGN_LEFT'
        else:
            state = ('auto_pivot_right', round(rpm, 1))
            left_dir = DIR_CCW   # 좌측 전진
            right_dir = DIR_CCW  # 우측 후진
            self._mode = 'AUTO_ALIGN_RIGHT'

        self._set_group_targets(left_dir, rpm, right_dir, rpm)
        self._last_command_time = time.monotonic()

        if self._prev_state == state:
            return
        self._prev_state = state

        pkts = []
        for mid in LEFT_IDS:
            pkts.append(make_acc_speed_packet(mid, left_dir, rpm, self._arrive_s))
        for mid in RIGHT_IDS:
            pkts.append(make_acc_speed_packet(mid, right_dir, rpm, self._arrive_s))

        self._link.send_packets(pkts, self._inter_s)

    def _send_auto_arc(self, turn_left: bool, rpm: float) -> None:
        """자동 정렬용 저속 전진 곡선 조향 명령을 전송한다.

        양쪽 바퀴는 모두 전진 방향으로 구동하고, 회전 안쪽 바퀴 RPM만
        ``auto_align_arc_inner_ratio``만큼 낮춘다. 따라서 제자리 회전과 달리
        차체 위치가 전방으로 이동한다.
        """
        self._observer_steer = 1.0 if turn_left else -1.0
        self._reset_adaptive_steering_state()
        self._reset_turn_strategy_state()
        self._reset_pivot_pulse_state()
        if self._estop_latched:
            self._auto_align_active = False
            self._last_align_yaw_error_deg = 0.0
            self._mode = 'ESTOP'
            return

        outer_rpm = max(0.0, min(float(rpm), self._max_rpm))
        if outer_rpm == 0.0:
            self._send_zero_velocity()
            return

        inner_rpm = outer_rpm * self._auto_align_arc_inner_ratio
        if turn_left:
            left_rpm = inner_rpm
            right_rpm = outer_rpm
            state = ('auto_arc_left', round(left_rpm, 2), round(right_rpm, 2))
            self._mode = 'AUTO_ALIGN_ARC_LEFT'
        else:
            left_rpm = outer_rpm
            right_rpm = inner_rpm
            state = ('auto_arc_right', round(left_rpm, 2), round(right_rpm, 2))
            self._mode = 'AUTO_ALIGN_ARC_RIGHT'

        # 전진 방향표: 좌측 CCW, 우측 CW
        left_dir = DIR_CCW
        right_dir = DIR_CW
        self._set_group_targets(left_dir, left_rpm, right_dir, right_rpm)
        self._last_command_time = time.monotonic()

        if self._prev_state == state:
            return
        self._prev_state = state

        pkts = []
        for mid in LEFT_IDS:
            pkts.append(
                make_acc_speed_packet(mid, left_dir, left_rpm, self._arrive_s)
            )
        for mid in RIGHT_IDS:
            pkts.append(
                make_acc_speed_packet(mid, right_dir, right_rpm, self._arrive_s)
            )
        self._link.send_packets(pkts, self._inter_s)

    # ── 자동 정렬 명령 콜백 ─────────────────────────────────

    def _align_cmd_callback(self, msg: String) -> None:
        """aim_coordinator의 /mmr/drive/align_cmd를 처리한다.

        기대 JSON 예:
          {"active": true, "yaw_error_deg": 10.0, "pivot_rpm": 5.0}
          {"active": false}

        convention:
          yaw_error_deg > 0 : 표적이 차체 기준 오른쪽 → 우회전
          yaw_error_deg < 0 : 표적이 차체 기준 왼쪽   → 좌회전
        """
        if self._estop_latched:
            self._auto_align_active = False
            self._last_align_yaw_error_deg = 0.0
            self._mode = 'ESTOP'
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f'align_cmd JSON 파싱 실패: {msg.data}',
                throttle_duration_sec=2.0,
            )
            return

        now = time.monotonic()
        self._last_align_cmd_time = now

        active = bool(data.get('active', False))
        if not active:
            if self._auto_align_active:
                self.get_logger().info(
                    'align_cmd active=false 수신 → 자동 정렬 정지',
                    throttle_duration_sec=2.0,
                )
            self._auto_align_active = False
            self._last_align_yaw_error_deg = 0.0
            self._send_zero_velocity()
            return

        try:
            yaw_error = float(data.get('yaw_error_deg', 0.0))
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'align_cmd yaw_error_deg 값 오류: {data.get("yaw_error_deg")}',
                throttle_duration_sec=2.0,
            )
            self._auto_align_active = False
            self._send_zero_velocity()
            return

        try:
            rpm = float(data.get('pivot_rpm', self._default_pivot_rpm))
        except (TypeError, ValueError):
            rpm = self._default_pivot_rpm

        rpm = max(0.0, min(rpm, self._max_rpm))
        if rpm <= 0.0:
            self._auto_align_active = False
            self._send_zero_velocity()
            return

        self._auto_align_active = True
        self._last_align_yaw_error_deg = yaw_error

        # yaw_error_deg > 0이면 표적이 오른쪽에 있으므로 우회전한다.
        # 실제 장비에서 반대로 돌 경우 이 한 줄의 부호만 반대로 바꾸면 된다.
        turn_left = yaw_error < 0.0
        if self._auto_align_motion_mode == 'arc_forward':
            self._send_auto_arc(turn_left=turn_left, rpm=rpm)
        else:
            self._send_auto_pivot(turn_left=turn_left, rpm=rpm)

    # ── Joy 콜백 ────────────────────────────────────────────

    def _joy_callback(self, msg: Joy) -> None:
        self._last_joy_time = self.get_clock().now()

        axes = msg.axes
        if len(axes) < 4:
            self.get_logger().warn(
                f'Joy axes 부족: {len(axes)}개 수신 (최소 4개 필요)',
                throttle_duration_sec=5.0
            )
            return

        buttons = msg.buttons

        if self._handle_joy_estop_reset(axes, buttons):
            return

        lb_pressed = len(buttons) > BUTTON_LB and buttons[BUTTON_LB] == 1
        rb_pressed = len(buttons) > BUTTON_RB and buttons[BUTTON_RB] == 1

        raw_linear = float(axes[AXIS_LINEAR])
        raw_steer  = float(axes[AXIS_STEER])

        linear = apply_deadzone(raw_linear, DEADZONE_LINEAR)
        steer  = apply_deadzone(raw_steer,  DEADZONE_STEER)
        self._observer_steer = float(steer)

        manual_input_active = (
            lb_pressed
            or rb_pressed
            or linear != 0.0
            or steer != 0.0
        )

        # 자동 정렬 중에는 Joy 중립 메시지를 정지 명령으로 보지 않는다.
        # 실제 버튼/스틱 입력이 있을 때만 수동 override로 자동 정렬을 해제한다.
        if self._auto_align_active and not manual_input_active:
            return

        if manual_input_active:
            if self._auto_align_active:
                self.get_logger().info(
                    'Joy 수동 입력 감지 → 자동 정렬 해제',
                    throttle_duration_sec=1.0,
                )
            self._auto_align_active = False
            self._last_manual_command_time = time.monotonic()

        # LB/RB 수동 제자리 회전은 스틱 주행보다 우선한다.
        if lb_pressed and rb_pressed:
            self._send_zero_velocity()
            return

        if lb_pressed:
            self._send_manual_pivot(turn_left=True)
            return

        if rb_pressed:
            self._send_manual_pivot(turn_left=False)
            return

        # 정지 판정
        if linear == 0.0:
            # 전후진 없음 → 무조건 정지 명령
            self._send_zero_velocity()
            return

        # 전진/후진 방향
        forward = linear > 0.0

        # RPM 계산
        # 옵션이 꺼져 있으면 기존 계산 경로를 한 줄도 바꾸지 않고 그대로 사용한다.
        if not self._adaptive_steering_enabled:
            self._reset_adaptive_steering_state()
            left_rpm, right_rpm = compute_wheel_rpms(
                linear,
                steer,
                self._max_rpm,
            )
        else:
            now = time.monotonic()
            left_rpm, right_rpm = self._compute_adaptive_drive_command(
                linear=linear,
                target_steer=steer,
                now=now,
            )

            # 9600bps에서 6대 명령을 과도하게 반복하지 않도록 전송률을 제한한다.
            # 최초 주행, 전/후진 방향 변경은 즉시 전송하고 이후 slew 갱신은 주기 제한한다.
            force_send = (
                self._adaptive_last_forward is None
                or self._adaptive_last_forward != forward
                or self._mode != 'MANUAL_DRIVE'
            )
            # 선택형 조향 전략이 켜진 경우에는 아래 전략 상태 머신의
            # command_period/state_changed 기준으로 전송률을 제어한다.
            if not self._turn_strategy.enabled:
                if (
                    not force_send
                    and now - self._adaptive_last_send_time
                    < self._adaptive_command_period_s
                ):
                    return

                self._adaptive_last_send_time = now
                self._adaptive_last_forward = forward

        # 선택형 조향 시험 전략은 기존 계산 결과의 RPM 크기만 조정한다.
        # 모든 옵션이 false이면 입력 RPM을 그대로 반환하므로 기존 동작과 동일하다.
        strategy_now = time.monotonic()
        latest_load_state = str(
            self._latest_load_observation.get('state', 'DISABLED')
        )
        left_rpm, right_rpm, strategy_debug = self._turn_strategy.apply(
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            steer=steer,
            forward=forward,
            load_state=latest_load_state,
            now=strategy_now,
        )
        self._turn_strategy_debug = strategy_debug

        if self._turn_strategy.enabled:
            force_strategy_send = (
                bool(strategy_debug.get('state_changed', False))
                or self._turn_strategy_last_forward is None
                or self._turn_strategy_last_forward != forward
                or self._mode != 'MANUAL_DRIVE'
            )
            if (
                not force_strategy_send
                and strategy_now - self._turn_strategy_last_send_time
                < self._turn_strategy_command_period_s
            ):
                return
            self._turn_strategy_last_send_time = strategy_now
            self._turn_strategy_last_forward = forward

        # 안전상 한 번 더 클램핑한다.
        left_rpm = _clamp(left_rpm, 0.0, self._max_rpm)
        right_rpm = _clamp(right_rpm, 0.0, self._max_rpm)

        self._send_velocity(left_rpm, right_rpm, forward)

    # ── 엔코더 피드백 ───────────────────────────────────────

    def _feedback_cb(self) -> None:
        # 호출될 때마다 설정된 피드백 ID 중 하나를 순서대로 요청한다.
        dev_id = self._feedback_ids[
            self._feedback_index
        ]

        self._feedback_index = (
            self._feedback_index + 1
        ) % len(self._feedback_ids)

        feedback = self._link.request_speed_feedback(
            dev_id,
            self._feedback_timeout_s,
        )

        if feedback is None:
            fb = self._feedback_by_id[dev_id]
            fb['ok'] = False
            fb['timeout_count'] += 1

            self.get_logger().warn(
                f'[FB ID {dev_id}] '
                '응답 없음 또는 패킷 오류',
                throttle_duration_sec=2.0,
            )
            return

        actual_signed_rpm = signed_rpm_from_direction(
            feedback['id'],
            feedback['direction'],
            feedback['rpm'],
        )

        fb = self._feedback_by_id[feedback['id']]
        fb.update({
            'ok': True,
            'raw_rpm': float(feedback['rpm']),
            'actual_rpm': actual_signed_rpm,
            'direction': int(feedback['direction']),
            'position_deg': float(feedback['position_deg']),
            'current_a': float(feedback['current_a']),
            'last_feedback_time': time.monotonic(),
        })

        self.get_logger().info(
            f"[FB ID {feedback['id']}] "
            f"RPM={actual_signed_rpm:.1f} "
            f"RAW={feedback['rpm']:.1f} "
            f"POS={feedback['position_deg']:.1f}deg "
            f"CURRENT={feedback['current_a']:.1f}A",
            throttle_duration_sec=2.0,
        )

    def _publish_drive_status(self) -> None:
        """바퀴별 목표/실제 RPM과 구동부 상태를 JSON 토픽으로 발행한다."""
        now = time.monotonic()

        wheels = {}
        left_actual = []
        right_actual = []
        observer_feedback = {}
        target_snapshot = {
            mid: float(self._target_rpm_by_id.get(mid, 0.0))
            for mid in ALL_IDS
        }

        if self._feedback_enabled:
            feedback_all_fresh = True
        else:
            feedback_all_fresh = False

        for mid in ALL_IDS:
            fb = self._feedback_by_id[mid]
            target = float(self._target_rpm_by_id.get(mid, 0.0))

            age = None
            if fb['last_feedback_time'] > 0.0:
                age = now - fb['last_feedback_time']

            fresh = (
                self._feedback_enabled
                and bool(fb['ok'])
                and age is not None
                and age <= self._feedback_fresh_timeout_s
            )

            if mid in self._feedback_ids and not fresh:
                feedback_all_fresh = False

            actual = float(fb['actual_rpm'])
            error = target - actual
            label = WHEEL_LABEL_BY_ID.get(mid, f'ID{mid}')

            wheels[label] = {
                'id': mid,
                'target_rpm': target,
                'actual_rpm': actual,
                'error_rpm': error,
                'raw_rpm': float(fb['raw_rpm']),
                'current_a': fb['current_a'],
                'position_deg': fb['position_deg'],
                'ok': bool(fresh),
                'age_sec': age,
                'timeout_count': int(fb['timeout_count']),
            }
            observer_feedback[mid] = {
                'ok': bool(fresh),
                'actual_rpm': actual,
                'current_a': fb['current_a'],
                'age_sec': age,
            }

            if fresh:
                if mid in LEFT_IDS:
                    left_actual.append(actual)
                elif mid in RIGHT_IDS:
                    right_actual.append(actual)

        if self._load_observer_enabled:
            observation = self._load_observer.update(
                targets=target_snapshot,
                feedback=observer_feedback,
                steer=self._observer_steer,
                now=now,
            )
        else:
            observation = self._load_observer.empty_result()

        wheel_metrics = observation.get('wheel_metrics', {})
        for wheel in wheels.values():
            metric = wheel_metrics.get(int(wheel['id']), {})
            wheel.update({
                'observer_valid': bool(metric.get('observer_valid', False)),
                'progress_ratio': metric.get('progress_ratio'),
                'filtered_progress_ratio': metric.get(
                    'filtered_progress_ratio'
                ),
                'current_ratio': metric.get('current_ratio'),
                'filtered_current_a': metric.get('filtered_current_a'),
                'load_state': metric.get('state', 'DISABLED'),
                'load_candidate_state': metric.get(
                    'candidate_state', 'NONE'
                ),
                'unload_duration_s': metric.get('unload_duration_s', 0.0),
                'local_stall_duration_s': metric.get(
                    'local_stall_duration_s', 0.0
                ),
            })

        self._latest_load_observation = observation

        if self._feedback_enabled and feedback_all_fresh:
            stopped_values = [
                float(self._feedback_by_id[mid]['actual_rpm'])
                for mid in self._feedback_ids
            ]
        else:
            # 피드백이 아직 모두 신선하지 않으면 목표값 기준으로 보수적으로 판단한다.
            stopped_values = list(self._target_rpm_by_id.values())

        all_stopped = (
            bool(stopped_values)
            and all(abs(v) <= self._stop_rpm_threshold for v in stopped_values)
        )

        manual_override_active = (
            now - self._last_manual_command_time <= 0.5
        )

        payload = {
            't_ns': time.time_ns(),
            'mode': self._mode,
            'estop_latched': bool(self._estop_latched),
            'estop_release_seen': bool(self._estop_release_seen),
            'joy_reset_hold_progress': float(self._joy_reset_hold_progress),
            'joy_estop_inhibit_until_release': bool(self._joy_estop_inhibit_until_release),
            'joy_estop_button_indices': list(self._joy_estop_button_indices),
            'joy_reset_button_indices': list(self._joy_reset_button_indices),
            'all_stopped': bool(all_stopped),
            'feedback_all_fresh': bool(feedback_all_fresh),
            'manual_override_active': bool(manual_override_active),
            'auto_align_active': bool(self._auto_align_active),
            'auto_align_motion_mode': self._auto_align_motion_mode,
            'auto_align_arc_inner_ratio': self._auto_align_arc_inner_ratio,
            'last_align_yaw_error_deg': float(self._last_align_yaw_error_deg),
            'align_cmd_timeout_sec': self._align_cmd_timeout_sec,
            'max_rpm': self._max_rpm,
            'hard_max_rpm': HARD_MAX_RPM,
            'stop_rpm_threshold': self._stop_rpm_threshold,
            'adaptive_steering_enabled': bool(self._adaptive_steering_enabled),
            'adaptive_target_steer': float(self._adaptive_target_steer),
            'adaptive_applied_steer': float(self._adaptive_applied_steer),
            'adaptive_requested_base_rpm': float(
                self._adaptive_debug['requested_base_rpm']
            ),
            'adaptive_speed_cap_rpm': float(
                self._adaptive_debug['speed_cap_rpm']
            ),
            'adaptive_base_rpm': float(self._adaptive_debug['base_rpm']),
            'adaptive_max_reduction': float(
                self._adaptive_debug['max_reduction']
            ),
            'adaptive_applied_reduction': float(
                self._adaptive_debug['applied_reduction']
            ),
            'turn_strategy_enabled': bool(self._turn_strategy.enabled),
            'turn_strategy_features': list(
                self._turn_strategy_debug.get('features', [])
            ),
            'turn_strategy_state': str(
                self._turn_strategy_debug.get('state', 'DISABLED')
            ),
            'turn_strategy_input_steer': float(
                self._turn_strategy_debug.get('input_steer', 0.0)
            ),
            'turn_strategy_turning': bool(
                self._turn_strategy_debug.get('turning', False)
            ),
            'turn_strategy_entry_progress': float(
                self._turn_strategy_debug.get('entry_progress', 0.0)
            ),
            'turn_strategy_pulse_phase': str(
                self._turn_strategy_debug.get('pulse_phase', 'IDLE')
            ),
            'turn_strategy_release_phase': str(
                self._turn_strategy_debug.get('release_phase', 'IDLE')
            ),
            'turn_strategy_counter_active': bool(
                self._turn_strategy_debug.get(
                    'counter_steer_active', False
                )
            ),
            'turn_strategy_release_speed_preserved': bool(
                self._turn_strategy_debug.get(
                    'release_speed_preserved', False
                )
            ),
            'turn_strategy_tight_active': bool(
                self._turn_strategy_debug.get('tight_turn_active', False)
            ),
            'turn_strategy_tight_progress': float(
                self._turn_strategy_debug.get('tight_turn_progress', 0.0)
            ),
            'turn_strategy_tight_inner_ratio': float(
                self._turn_strategy_debug.get(
                    'tight_turn_inner_ratio', 1.0
                )
            ),
            'pivot_pulse_enabled': bool(
                self._pivot_pulse.config.enabled
            ),
            'pivot_pulse_phase': str(
                self._pivot_pulse_debug.get('phase', 'IDLE')
            ),
            'pivot_pulse_turn_sign': int(
                self._pivot_pulse_debug.get('turn_sign', 0)
            ),
            'pivot_pulse_command_rpm': float(
                self._pivot_pulse_debug.get('command_rpm', 0.0)
            ),
            'turn_strategy_load_overlay': str(
                self._turn_strategy_debug.get('load_overlay', 'NONE')
            ),
            'turn_strategy_speed_scale': float(
                self._turn_strategy_debug.get('speed_scale', 1.0)
            ),
            'turn_strategy_differential_scale': float(
                self._turn_strategy_debug.get(
                    'differential_scale', 1.0
                )
            ),
            'turn_strategy_input_left_rpm': float(
                self._turn_strategy_debug.get('input_left_rpm', 0.0)
            ),
            'turn_strategy_input_right_rpm': float(
                self._turn_strategy_debug.get('input_right_rpm', 0.0)
            ),
            'turn_strategy_output_left_rpm': float(
                self._turn_strategy_debug.get('output_left_rpm', 0.0)
            ),
            'turn_strategy_output_right_rpm': float(
                self._turn_strategy_debug.get('output_right_rpm', 0.0)
            ),
            'load_observer_enabled': bool(self._load_observer_enabled),
            'load_protection_enabled': bool(self._load_protection_enabled),
            'load_observer_mode': (
                'SHADOW' if self._load_observer_enabled else 'OFF'
            ),
            'load_state': observation.get('state', 'DISABLED'),
            'load_candidate_state': observation.get(
                'candidate_state', 'DISABLED'
            ),
            'load_score': int(observation.get('load_score', 0)),
            'suspect_wheel_ids': observation.get('suspect_wheel_ids', []),
            'observer_confidence': float(
                observation.get('observer_confidence', 0.0)
            ),
            'load_turning': bool(observation.get('turning', False)),
            'load_active_wheel_count': int(
                observation.get('active_wheel_count', 0)
            ),
            'load_fresh_active_wheel_count': int(
                observation.get('fresh_active_wheel_count', 0)
            ),
            'load_progressing_wheel_count': int(
                observation.get('progressing_wheel_count', 0)
            ),
            'load_stalled_wheel_count': int(
                observation.get('stalled_wheel_count', 0)
            ),
            'load_side_current_median_a': observation.get(
                'side_current_median_a',
                {'left': None, 'right': None},
            ),
            'left_avg_rpm': (
                sum(left_actual) / len(left_actual) if left_actual else 0.0
            ),
            'right_avg_rpm': (
                sum(right_actual) / len(right_actual) if right_actual else 0.0
            ),
            'wheels': wheels,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._drive_status_pub.publish(msg)

    # ── Watchdog ────────────────────────────────────────────

    def _watchdog_cb(self) -> None:
        """Joy/자동 정렬 명령 끊김을 감시하고 안전 정지한다."""
        now_mono = time.monotonic()

        if self._estop_latched:
            self._mode = 'ESTOP'
            return

        if self._auto_align_active:
            elapsed_align = now_mono - self._last_align_cmd_time
            if elapsed_align > self._align_cmd_timeout_sec:
                self.get_logger().warn(
                    f'align_cmd {elapsed_align:.2f}s 동안 없음 → 자동 정렬 정지',
                    throttle_duration_sec=2.0,
                )
                self._auto_align_active = False
                self._last_align_yaw_error_deg = 0.0
                self._send_zero_velocity()
            return

        elapsed = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if elapsed > 1.0:
            if self._prev_state != (0.0, 0.0, True):
                self.get_logger().warn(
                    f'Joy 신호 {elapsed:.1f}s 동안 없음 → 자동 정지',
                    throttle_duration_sec=3.0
                )
                self._send_zero_velocity()

    # ── 종료 ────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info('노드 종료 — 안전 정지 수행')
        self._send_stop()
        self._link.close()
        super().destroy_node()


# ─────────────────────────────────────────────
# 엔트리포인트
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BldcDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()