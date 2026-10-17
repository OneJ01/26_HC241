#!/usr/bin/env python3
import os, json, time, argparse, subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

FRAME_JPG = "/dev/shm/cam_frame.jpg"
META_JSON = "/dev/shm/cam_meta.json"

IMG_TOPIC  = "/camera/image_raw"
META_TOPIC = "/camera/metadata"

class ShmJpegToRaw(Node):
    def __init__(self, rate_hz: float):
        super().__init__("shm_cam_pub_raw")
        self.pub_img  = self.create_publisher(Image,  IMG_TOPIC,  10)
        self.pub_meta = self.create_publisher(String, META_TOPIC, 10)
        self.period = 1.0 / max(rate_hz, 0.1)
        self.last_mtime = 0.0
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(f"watch frame: {FRAME_JPG}")
        self.get_logger().info(f"watch meta : {META_JSON}")
        self.get_logger().info(f"publish raw rgb8 -> {IMG_TOPIC} @ {rate_hz:.2f} Hz")

    def _jpg_to_rgb24(self, jpg_path: str) -> bytes:
        # ffmpeg: jpg -> rawvideo rgb24 to stdout
        cmd = [
            "ffmpeg",
            "-hide_banner", "-loglevel", "error",
            "-i", jpg_path,
            "-f", "rawvideo",
            "-pix_fmt", "rgb24",
            "-"
        ]
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if p.returncode != 0:
            self.get_logger().warn(f"ffmpeg decode failed: {p.stderr.decode(errors='ignore')[:200]}")
            return b""
        return p.stdout

    def tick(self):
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

        width  = int(meta.get("width", 1456))
        height = int(meta.get("height", 1088))

        raw = self._jpg_to_rgb24(FRAME_JPG)
        if not raw:
            return

        expected = width * height * 3
        if len(raw) != expected:
            self.get_logger().warn(f"raw size mismatch: got={len(raw)} expected={expected} (w={width},h={height})")
            return

        # stamp: FrameWallClock(ns) 우선, 없으면 time.time_ns()
        t_ns = int(meta.get("FrameWallClock", 0)) or time.time_ns()
        sec  = t_ns // 1_000_000_000
        nsec = t_ns %  1_000_000_000

        msg = Image()
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)
        msg.header.frame_id = str(meta.get("frame_id", "imx296"))
        msg.height = height
        msg.width  = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = raw

        self.pub_img.publish(msg)
        self.pub_meta.publish(String(data=json.dumps(meta, separators=(",",":"))))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate (Hz)")
    args = ap.parse_args()

    rclpy.init()
    node = ShmJpegToRaw(rate_hz=args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
