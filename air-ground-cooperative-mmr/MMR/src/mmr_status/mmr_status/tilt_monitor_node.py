#!/usr/bin/env python3
"""
tilt_monitor_node.py
====================
MW-AHRS-X1 기반 MMR 기울기 감시 노드

구독:
  /imu/data       (sensor_msgs/Imu)
  /imu/calibrate  (std_msgs/String)  — 영점 보정 명령 ("cal")

발행:
  /imu/tilt        (geometry_msgs/Vector3) — x=roll_used, y=pitch_used, z=0.0 (deg)
  /imu/tilt_status (std_msgs/String)       — 안전 판정 결과 JSON

핵심 정책:
  - 제조사 AHRS orientation 값은 최종 roll/pitch 기준으로 직접 사용하지 않음.
  - /imu/data.linear_acceleration 기반으로 정지 상태 roll/pitch를 직접 계산함.
  - /imu/data.angular_velocity와 acc norm으로 정지/가속 상태를 판단함.
  - orientation 기반 roll/pitch는 진단 및 drift 비교용으로만 사용함.
  - 기존 GUI/상부 제어 호환을 위해 roll_deg, pitch_deg, tilt_ok 필드는 유지함.
"""

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

# ================================================================
# 튜닝 파라미터 기본값
# ================================================================
ROLL_LIMIT_DEG = 10.0
PITCH_LIMIT_DEG = 10.0
G_REF = 9.80665


class TiltMonitorNode(Node):

    def __init__(self):
        super().__init__('tilt_monitor_node')

        # 기존 파라미터 유지
        self.declare_parameter('roll_limit_deg', ROLL_LIMIT_DEG)
        self.declare_parameter('pitch_limit_deg', PITCH_LIMIT_DEG)
        self.declare_parameter('print_hz', 1.0)

        # 기존 lpf_alpha는 호환용으로 남김. 기본 동작은 시간상수 기반 필터 사용.
        self.declare_parameter('lpf_alpha', 0.3)

        # 새 파라미터
        self.declare_parameter('use_time_constant_lpf', True)
        self.declare_parameter('tilt_lpf_tau_sec', 0.3)

        # 센서 축 부호 보정. 실제 기울임 테스트 후 필요하면 -1.0으로 변경.
        self.declare_parameter('roll_sign', -1.0)
        self.declare_parameter('pitch_sign', 1.0)

        # 정지 판정 기준
        self.declare_parameter('gyro_stationary_threshold_rad_s', 0.02)
        self.declare_parameter('acc_norm_tolerance_m_s2', 0.20)
        self.declare_parameter('acc_norm_ref_m_s2', G_REF)

        # 제조사 orientation과 acc 기반 tilt 차이 진단 기준
        self.declare_parameter('orient_acc_drift_limit_deg', 1.5)

        # 안전 정책
        # True: stationary가 아니면 tilt_ok=false
        # False: stationary가 아니어도 제한각만 만족하면 tilt_ok=true
        self.declare_parameter('require_stationary_for_tilt_ok', True)

        # True: orientation과 acc 차이가 커도 acc 기반 roll/pitch가 안정적이면 tilt_ok 유지
        # False: orientation drift가 크면 tilt_ok=false
        self.declare_parameter('allow_tilt_ok_when_orient_drift', True)

        self._roll_deg = 0.0
        self._pitch_deg = 0.0

        self._roll_filtered = 0.0
        self._pitch_filtered = 0.0
        self._filter_initialized = False

        self._roll_offset = 0.0
        self._pitch_offset = 0.0

        self._last_imu_time_sec = None
        self._last_status = {
            'state': 'NO_IMU',
            'reason': 'WAITING_FOR_IMU',
            'stationary': False,
            'imu_valid': False,
        }

        # 구독
        self.create_subscription(Imu, '/imu/data', self._cb_imu, 10)
        self.create_subscription(String, '/imu/calibrate', self._cb_calibrate, 10)

        # 발행
        self.pub_tilt = self.create_publisher(Vector3, '/imu/tilt', 10)
        self.pub_status = self.create_publisher(String, '/imu/tilt_status', 10)

        # 터미널 출력 타이머
        print_hz = max(0.1, float(self.get_parameter('print_hz').value))
        self.create_timer(1.0 / print_hz, self._print_tilt)

        self.get_logger().info(
            'TiltMonitorNode 시작 | '
            f'roll_limit={self.get_parameter("roll_limit_deg").value}° | '
            f'pitch_limit={self.get_parameter("pitch_limit_deg").value}° | '
            f'출력주기={print_hz}Hz | '
            f'LPF tau={self.get_parameter("tilt_lpf_tau_sec").value}s | '
            'roll/pitch source=linear_acceleration'
        )
        self.get_logger().info(
            '영점 보정 명령: '
            'ros2 topic pub --once /imu/calibrate std_msgs/msg/String "data: \'cal\'"'
        )

    # ------------------------------------------------------------------
    # IMU 콜백
    # ------------------------------------------------------------------
    def _cb_imu(self, msg: Imu):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # 1) 제조사 orientation 기반 roll/pitch 계산: 진단용
        roll_orient, pitch_orient, yaw_orient = self._quat_to_rpy_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        # 2) linear_acceleration 기반 roll/pitch 계산: 최종 사용 후보
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        roll_acc, pitch_acc = self._acc_to_roll_pitch_deg(ax, ay, az)
        roll_acc *= float(self.get_parameter('roll_sign').value)
        pitch_acc *= float(self.get_parameter('pitch_sign').value)

        # 3) gyro/acc norm 계산
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)

        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)

        gyro_thr = float(self.get_parameter('gyro_stationary_threshold_rad_s').value)
        acc_ref = float(self.get_parameter('acc_norm_ref_m_s2').value)
        acc_tol = float(self.get_parameter('acc_norm_tolerance_m_s2').value)

        stationary = (
            gyro_norm <= gyro_thr and
            abs(acc_norm - acc_ref) <= acc_tol
        )

        # 4) 최종 roll/pitch는 acc 기반 raw 값을 필터링해서 사용
        raw_roll_used = roll_acc
        raw_pitch_used = pitch_acc

        if self._last_imu_time_sec is None:
            dt = 0.01
        else:
            dt = max(1e-4, min(0.2, now_sec - self._last_imu_time_sec))
        self._last_imu_time_sec = now_sec

        if not self._filter_initialized:
            self._roll_filtered = raw_roll_used
            self._pitch_filtered = raw_pitch_used
            self._filter_initialized = True
        else:
            alpha = self._get_lpf_alpha(dt)
            self._roll_filtered = alpha * raw_roll_used + (1.0 - alpha) * self._roll_filtered
            self._pitch_filtered = alpha * raw_pitch_used + (1.0 - alpha) * self._pitch_filtered

        # 5) 영점 보정 오프셋 적용
        self._roll_deg = self._roll_filtered - self._roll_offset
        self._pitch_deg = self._pitch_filtered - self._pitch_offset

        # 6) orientation drift 진단
        roll_drift = roll_orient - roll_acc
        pitch_drift = pitch_orient - pitch_acc
        drift_limit = float(self.get_parameter('orient_acc_drift_limit_deg').value)
        imu_drift = (
            abs(roll_drift) > drift_limit or
            abs(pitch_drift) > drift_limit
        )

        # 7) 제한 판정
        roll_limit = float(self.get_parameter('roll_limit_deg').value)
        pitch_limit = float(self.get_parameter('pitch_limit_deg').value)
        roll_ok = abs(self._roll_deg) <= roll_limit
        pitch_ok = abs(self._pitch_deg) <= pitch_limit

        require_stationary = bool(self.get_parameter('require_stationary_for_tilt_ok').value)
        allow_orient_drift = bool(self.get_parameter('allow_tilt_ok_when_orient_drift').value)

        imu_valid = stationary
        tilt_ok = roll_ok and pitch_ok

        if require_stationary and not imu_valid:
            tilt_ok = False
        if imu_drift and not allow_orient_drift:
            tilt_ok = False

        # 8) 상태 문자열 구성
        state = 'OK'
        reason = 'OK'

        if not stationary:
            state = 'IMU_MOVING_OR_ACCEL'
            reason = 'NOT_STATIONARY_OR_ACC_NORM_BAD'
        elif not roll_ok or not pitch_ok:
            state = 'TILT_LIMIT'
            reason = 'ROLL_OR_PITCH_LIMIT'
        elif imu_drift:
            # 정책상 tilt_ok는 true일 수 있음. 단, 진단 상태는 남김.
            state = 'IMU_DRIFT'
            reason = 'ORIENT_ACC_MISMATCH'

        # 9) /imu/tilt 발행: 기존 호환. x/y는 최종 used roll/pitch.
        tilt = Vector3()
        tilt.x = self._roll_deg
        tilt.y = self._pitch_deg
        tilt.z = 0.0
        self.pub_tilt.publish(tilt)

        # 10) /imu/tilt_status 발행: 기존 필드 유지 + 진단 필드 추가
        status = {
            # 기존 호환 필드
            'tilt_ok': bool(tilt_ok),
            'roll_deg': round(self._roll_deg, 2),
            'pitch_deg': round(self._pitch_deg, 2),
            'roll_ok': bool(roll_ok),
            'pitch_ok': bool(pitch_ok),
            'roll_limit': roll_limit,
            'pitch_limit': pitch_limit,
            'roll_offset': round(self._roll_offset, 2),
            'pitch_offset': round(self._pitch_offset, 2),
            't_ns': time.time_ns(),

            # 새 진단 필드
            'roll_used_deg': round(self._roll_deg, 2),
            'pitch_used_deg': round(self._pitch_deg, 2),
            'roll_acc_deg': round(roll_acc, 2),
            'pitch_acc_deg': round(pitch_acc, 2),
            'roll_orient_deg': round(roll_orient, 2),
            'pitch_orient_deg': round(pitch_orient, 2),
            'yaw_orient_deg': round(yaw_orient, 2),
            'roll_drift_deg': round(roll_drift, 2),
            'pitch_drift_deg': round(pitch_drift, 2),
            'gyro_norm_rad_s': round(gyro_norm, 6),
            'acc_norm_m_s2': round(acc_norm, 4),
            'stationary': bool(stationary),
            'imu_valid': bool(imu_valid),
            'imu_drift': bool(imu_drift),
            'state': state,
            'reason': reason,
            'source': 'linear_acceleration',
        }
        self._last_status = status

        msg_out = String()
        msg_out.data = json.dumps(status, ensure_ascii=False)
        self.pub_status.publish(msg_out)

        if not tilt_ok:
            self.get_logger().warn(
                f'[{state}] roll={self._roll_deg:+.2f}° '
                f'pitch={self._pitch_deg:+.2f}° | {reason}'
            )

    # ------------------------------------------------------------------
    # 쿼터니언 → roll/pitch/yaw 변환, deg
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_rpy_deg(qx: float, qy: float, qz: float, qw: float):
        # roll
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)

        # pitch
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # yaw
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    # ------------------------------------------------------------------
    # 가속도 → roll/pitch 변환, deg
    # 정지 상태에서 중력 방향을 이용한 tilt 계산.
    # 센서 축 방향에 따라 roll_sign/pitch_sign으로 부호 보정 가능.
    # ------------------------------------------------------------------
    @staticmethod
    def _acc_to_roll_pitch_deg(ax: float, ay: float, az: float):
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return math.degrees(roll), math.degrees(pitch)

    # ------------------------------------------------------------------
    # LPF alpha 계산
    # ------------------------------------------------------------------
    def _get_lpf_alpha(self, dt: float) -> float:
        use_tau = bool(self.get_parameter('use_time_constant_lpf').value)
        if use_tau:
            tau = max(1e-3, float(self.get_parameter('tilt_lpf_tau_sec').value))
            return max(0.0, min(1.0, dt / (tau + dt)))

        alpha = float(self.get_parameter('lpf_alpha').value)
        return max(0.0, min(1.0, alpha))

    # ------------------------------------------------------------------
    # 영점 보정 콜백
    # MMR을 수평 평지에 놓고 아래 명령 실행:
    # ros2 topic pub --once /imu/calibrate std_msgs/msg/String "data: 'cal'"
    # ------------------------------------------------------------------
    def _cb_calibrate(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'cal':
            if not self._filter_initialized:
                self.get_logger().warn('[CALIBRATE] 아직 IMU 데이터가 수신되지 않았습니다.')
                return

            self._roll_offset = self._roll_filtered
            self._pitch_offset = self._pitch_filtered
            self.get_logger().info(
                f'[CALIBRATE] 영점 보정 완료 | '
                f'roll_offset={self._roll_offset:+.2f}° | '
                f'pitch_offset={self._pitch_offset:+.2f}°'
            )
        elif cmd in ('reset', 'clear'):
            self._roll_offset = 0.0
            self._pitch_offset = 0.0
            self.get_logger().info('[CALIBRATE] 영점 보정값 초기화 완료')
        else:
            self.get_logger().warn(
                f"[CALIBRATE] 알 수 없는 명령: '{msg.data}' "
                "(유효 명령: 'cal', 'reset')"
            )

    # ------------------------------------------------------------------
    # 터미널 주기 출력
    # ------------------------------------------------------------------
    def _print_tilt(self):
        roll_limit = float(self.get_parameter('roll_limit_deg').value)
        pitch_limit = float(self.get_parameter('pitch_limit_deg').value)
        roll_ok = abs(self._roll_deg) <= roll_limit
        pitch_ok = abs(self._pitch_deg) <= pitch_limit

        state = self._last_status.get('state', 'NO_IMU')
        stationary = self._last_status.get('stationary', False)
        roll_acc = self._last_status.get('roll_acc_deg', None)
        pitch_acc = self._last_status.get('pitch_acc_deg', None)
        roll_orient = self._last_status.get('roll_orient_deg', None)
        pitch_orient = self._last_status.get('pitch_orient_deg', None)

        if roll_acc is None:
            self.get_logger().info('[TILT] IMU 데이터 대기 중')
            return

        self.get_logger().info(
            f'[TILT:{state}] '
            f'used roll={self._roll_deg:+6.2f}° {"OK" if roll_ok else "NG"} | '
            f'used pitch={self._pitch_deg:+6.2f}° {"OK" if pitch_ok else "NG"} | '
            f'acc=({roll_acc:+.2f}, {pitch_acc:+.2f})° | '
            f'orient=({roll_orient:+.2f}, {pitch_orient:+.2f})° | '
            f'stationary={stationary} | '
            f'offset=({self._roll_offset:+.2f}, {self._pitch_offset:+.2f})°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TiltMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
