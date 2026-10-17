#!/usr/bin/env python3
#real file
import os, json, time, argparse

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

FRAME_JPG = "/dev/shm/cam_frame.jpg"
META_JSON = "/dev/shm/cam_meta.json"

IMG_TOPIC  = "/camera/image_compressed"
META_TOPIC = "/camera/metadata"


class ShmJpegToCompressed(Node):
    def __init__(self, rate_hz: float):
        super().__init__("shm_cam_pub_compressed")

        # 최신 프레임 우선 (지연 누적 방지)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_img  = self.create_publisher(CompressedImage, IMG_TOPIC,  qos)
        self.pub_meta = self.create_publisher(String,          META_TOPIC, 10)

        self.period = 1.0 / max(rate_hz, 0.1)
        self.last_mtime = 0.0
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(f"watch frame: {FRAME_JPG}")
        self.get_logger().info(f"watch meta : {META_JSON}")
        self.get_logger().info(f"publish compressed jpeg -> {IMG_TOPIC} @ {rate_hz:.2f} Hz (QoS: BestEffort, depth=1)")

    def tick(self):
        # 프레임 파일 존재 확인
        try:
            st = os.stat(FRAME_JPG)
        except FileNotFoundError:
            self.get_logger().warn("frame not found yet")
            return

        # 파일 갱신 없으면 스킵(부하 감소)
        if st.st_mtime == self.last_mtime:
            return
        self.last_mtime = st.st_mtime

        # 메타 읽기(없으면 빈 dict)
        meta = {}
        try:
            with open(META_JSON, "r") as f:
                meta = json.load(f)
        except Exception:
            meta = {}

        # JPEG bytes 읽기
        try:
            with open(FRAME_JPG, "rb") as f:
                jpg_bytes = f.read()
        except Exception as e:
            self.get_logger().warn(f"failed to read jpg: {e}")
            return

        if not jpg_bytes:
            return

        # stamp: FrameWallClock(ns) 우선, 없으면 time.time_ns()
        # cam_capture_to_shm.py에서 out_md에 "t_cam_ns"도 넣고 있으니 둘 다 지원
        t_ns = int(meta.get("FrameWallClock", 0) or meta.get("t_cam_ns", 0) or 0)
        if t_ns <= 0:
            t_ns = time.time_ns()

        sec  = t_ns // 1_000_000_000
        nsec = t_ns %  1_000_000_000

        msg = CompressedImage()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = str(meta.get("frame_id", "imx296"))
        msg.format = "jpeg"
        msg.data = jpg_bytes

        self.pub_img.publish(msg)
        self.pub_meta.publish(String(data=json.dumps(meta, separators=(",",":"))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate (Hz)")
    args = ap.parse_args()

    rclpy.init()
    node = ShmJpegToCompressed(rate_hz=args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
