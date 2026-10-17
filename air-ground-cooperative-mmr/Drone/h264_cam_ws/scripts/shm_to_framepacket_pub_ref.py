#!/usr/bin/env python3
import json
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cam_msgs.msg import FramePacket

BASE = Path("/dev/shm")
LATEST = BASE / "latest_seq.txt"
FRAMES = BASE / "frames"
META   = BASE / "meta"

TOPIC = "/camera/frame_packet"

class ShmToFramePacket(Node):
    def __init__(self, rate_hz: float):
        super().__init__("shm_to_framepacket_pub")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(FramePacket, TOPIC, qos)
        self.last_seq = None
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.tick)

        self.get_logger().info(f"watch latest: {LATEST}")
        self.get_logger().info(f"publish -> {TOPIC} @ {rate_hz:.2f} Hz")
        self.last_latest_mtime = 0.0

    def _read_latest_seq(self):
        try:
            return int(LATEST.read_text().strip())
        except Exception:
            return None

    def tick(self):
        # ✅ latest_seq.txt 갱신(mtime) 기반으로 새 프레임 여부 판단
        try:
            st = LATEST.stat()
        except Exception:
            return

        if st.st_mtime == self.last_latest_mtime:
            return
        self.last_latest_mtime = st.st_mtime

        seq1 = self._read_latest_seq()
        if seq1 is None:
            return
        
        # 중복 seq 재발행 방지
        if self.last_seq is not None and seq1 <= self.last_seq:
            return
        jpg_path  = FRAMES / f"{seq1:06d}.jpg"
        meta_path = META   / f"{seq1:06d}.json"
        if (not jpg_path.exists()) or (not meta_path.exists()):
            return

        # 안정성: latest 한 번 더 확인
        seq2 = self._read_latest_seq()
        if seq2 != seq1:
            return

        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            meta = {}

        try:
            jpg_bytes = jpg_path.read_bytes()
        except Exception:
            return
        if not jpg_bytes:
            return

        msg = FramePacket()
        msg.seq = int(meta.get("seq", seq1))
        msg.width = int(meta.get("width", 0) or 0)
        msg.height = int(meta.get("height", 0) or 0)

        frame_wc = int(meta.get("FrameWallClock", 0) or 0)
        t_cam_ns = int(meta.get("t_cam_ns", 0) or 0) or frame_wc
        msg.t_cam_ns = int(t_cam_ns)
        msg.frame_wallclock_ns = int(frame_wc)
        msg.sensor_timestamp_ns = int(meta.get("SensorTimestamp", 0) or 0)

        msg.exposure_us = int(meta.get("ExposureTime_us", 0) or 0)
        msg.analogue_gain = float(meta.get("AnalogueGain", 0.0) or 0.0)
        msg.digital_gain = float(meta.get("DigitalGain", 0.0) or 0.0)
        msg.frame_duration_ns = int(meta.get("FrameDuration", 0) or 0)
        msg.host_time_ns = int(meta.get("host_time_ns", 0) or 0)

        try:
            msg.jpg = jpg_bytes
        except Exception:
            msg.jpg = list(jpg_bytes)

        self.pub.publish(msg)
        self.last_seq = seq1

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = ShmToFramePacket(rate_hz=args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
