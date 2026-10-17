#!/usr/bin/env python3
"""
mmr_status_node.py
────────────────────────────────────────────────────────────────────────────
GNSS driver 토픽을 GUI용 통합 상태 토픽(/mmr/status)으로 변환하는 노드.

구독:
  /gnss/fix_pvt   sensor_msgs/NavSatFix   NAV-PVT 기반 fix/RTK 상태 판단용
  /gnss/fix_hp    sensor_msgs/NavSatFix   NAV-HPPOSLLH 기반 실제 고정밀 위치
  /gnss/heading   std_msgs/Float64        Moving Baseline heading [deg]

발행:
  /mmr/status     std_msgs/String         JSON 상태, 기본 5 Hz

주의:
  - /fix는 fire_solution 호환을 위해 gnss_driver에서 아직 발행 유지 가능
  - mmr_status_node에서는 /fix를 사용하지 않음
  - 상태 제한 로직은 마지막 안정화 단계에서 추가 예정
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String


def _fix_quality(msg: NavSatFix) -> str:
    """
    NavSatFix status/covariance 기반 fix 품질 문자열 변환.

    현재 gnss_driver에서:
      - NO_FIX
      - GNSS
      - RTK_FLOAT
      - RTK_FIX
    형태로 표현하기 위한 단순 매핑.
    """
    s = msg.status.status
    c = msg.position_covariance_type

    if s == NavSatStatus.STATUS_NO_FIX:
        return "NO_FIX"

    if s == NavSatStatus.STATUS_SBAS_FIX:
        return "RTK_FLOAT"

    if s == NavSatStatus.STATUS_GBAS_FIX:
        return "RTK_FIX" if c == NavSatFix.COVARIANCE_TYPE_KNOWN else "RTK_FLOAT"

    if s == NavSatStatus.STATUS_FIX:
        return "GNSS"

    return "UNKNOWN"


def _hdop(msg: NavSatFix) -> float:
    """
    position_covariance[0] 수평 분산 m² → HDOP 근사값.

    정확한 HDOP가 아니라 GUI 표시용 근사값.
    covariance가 없으면 -1.0 반환.
    """
    cov0 = msg.position_covariance[0]
    return round(math.sqrt(cov0), 2) if cov0 > 0 else -1.0


class MmrStatusNode(Node):
    """
    /gnss/fix_pvt + /gnss/fix_hp + /gnss/heading → /mmr/status

    데이터 역할:
      /gnss/fix_pvt  : fix_quality, hdop 상태 판단용
      /gnss/fix_hp   : 실제 lat/lon 위치값
      /gnss/heading  : MMR heading 값
    """

    # 현재 RELPOSNED/heading이 느리게 들어오는 상황을 고려해 넉넉하게 유지
    HEADING_STALE_SEC = 12.0

    def __init__(self):
        super().__init__("mmr_status_node")

        # ── 파라미터 ────────────────────────────────────────────────
        self.declare_parameter("fix_pvt_topic", "/gnss/fix_pvt")
        self.declare_parameter("fix_hp_topic", "/gnss/fix_hp")
        self.declare_parameter("gnss_heading_topic", "/gnss/heading")
        self.declare_parameter("status_topic", "/mmr/status")
        self.declare_parameter("publish_rate_hz", 5.0)

        fix_pvt_topic = str(self.get_parameter("fix_pvt_topic").value)
        fix_hp_topic = str(self.get_parameter("fix_hp_topic").value)
        gnss_heading_topic = str(self.get_parameter("gnss_heading_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # ── 내부 상태 ────────────────────────────────────────────────
        self._lat: float | None = None
        self._lon: float | None = None

        self._fix_quality: str = "NO_FIX"
        self._hdop: float = -1.0
        self._fix_source: str = "NONE"
        self._fix_t: float = 0.0

        self._heading_deg: float = 0.0
        self._heading_t: float = 0.0

        # ── QoS ─────────────────────────────────────────────────────
        # gnss_driver 기본 create_publisher(..., 10)와 맞추기 위해 RELIABLE 사용
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(
            NavSatFix,
            fix_pvt_topic,
            self._cb_fix_pvt,
            qos_reliable,
        )

        self.create_subscription(
            NavSatFix,
            fix_hp_topic,
            self._cb_fix_hp,
            qos_reliable,
        )

        self.create_subscription(
            Float64,
            gnss_heading_topic,
            self._cb_heading,
            qos_reliable,
        )

        # ── 발행 ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            "mmr_status_node 시작\n"
            f"  구독: {fix_pvt_topic} (NavSatFix, PVT 상태 판단용)\n"
            f"        {fix_hp_topic} (NavSatFix, HPPOSLLH 위치값)\n"
            f"        {gnss_heading_topic} (Float64, Moving Baseline heading)\n"
            f"  발행: {status_topic} @ {rate_hz:.1f} Hz"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_fix_pvt(self, msg: NavSatFix):
        """
        /gnss/fix_pvt 콜백.

        PVT는 실제 위치값으로 쓰지 않고,
        fix/RTK 상태 판단용으로만 사용한다.
        """
        self._fix_quality = _fix_quality(msg)
        self._hdop = _hdop(msg)

    def _cb_fix_hp(self, msg: NavSatFix):
        """
        /gnss/fix_hp 콜백.

        HPPOSLLH 기반 고정밀 위경도를 실제 MMR 위치로 사용한다.
        """
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._fix_source = "/gnss/fix_hp"
        self._fix_t = self._now_sec()

    def _cb_heading(self, msg: Float64):
        """
        /gnss/heading 콜백.

        Moving Baseline heading [deg].
        """
        self._heading_deg = float(msg.data) % 360.0
        self._heading_t = self._now_sec()

    def _publish(self):
        if self._lat is None or self._lon is None:
            return

        now = self._now_sec()

        heading_age = now - self._heading_t if self._heading_t > 0 else 9999.0
        heading_valid = heading_age <= self.HEADING_STALE_SEC

        if self._heading_t > 0 and not heading_valid:
            self.get_logger().warn(
                f"[GNSS_HDG] /gnss/heading 수신 끊김: {heading_age:.1f}s 경과",
                throttle_duration_sec=5.0,
            )

        payload = {
            "lat_deg": round(self._lat, 8),
            "lon_deg": round(self._lon, 8),
            "heading_deg": round(self._heading_deg, 2),
            "heading_valid": bool(heading_valid),
            "heading_age_sec": round(heading_age, 2) if self._heading_t > 0 else -1.0,
            "fix_quality": self._fix_quality,
            "fix_source": self._fix_source,
            "hdop": self._hdop,
        }

        self._pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args)
    node = MmrStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()