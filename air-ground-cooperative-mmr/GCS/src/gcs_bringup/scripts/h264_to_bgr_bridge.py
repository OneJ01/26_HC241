#!/usr/bin/env python3
import argparse
import json
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

from camera_frame_buffer import CameraFrameBuffer
from h264_receiver import H264Receiver


FRAME_META_TOPIC = "/camera/frame_meta"
IMAGE_TOPIC = "/camera/image_bgr"
DISPLAY_IMAGE_TOPIC = "/camera/image_jpeg_display"
ENABLE_CALC_BGR_PUBLISH = False


def _to_int(v, default=0):
    try:
        if v is None:
            return int(default)
        return int(v)
    except Exception:
        return int(default)


class H264ToBgrBridge(Node):
    def __init__(self, udp_port, publish_rate_hz, width, height, max_match_dt_ms):
        super().__init__("h264_to_bgr_bridge")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_img = self.create_publisher(Image, IMAGE_TOPIC, qos)
        self.pub_img_display = self.create_publisher(CompressedImage, DISPLAY_IMAGE_TOPIC, qos)

        self.sub_meta = self.create_subscription(
            String,
            FRAME_META_TOPIC,
            self.on_frame_meta,
            10,
        )

        self.frame_buffer = CameraFrameBuffer()

        # GStreamer/H264Receiver는 UI 프로세스가 아니라 이 bridge 프로세스에서만 실행
        self.h264_rx = H264Receiver(
            frame_buffer=self.frame_buffer,
            udp_port=int(udp_port),
            width=int(width),
            height=int(height),
            max_match_dt_ms=float(max_match_dt_ms),
            log_fn=lambda msg: print(str(msg), flush=True),
        )

        self.timer = self.create_timer(
            1.0 / max(float(publish_rate_hz), 0.1),
            self.tick_publish,
        )

        self.pub_count = 0
        self.display_count = 0
        self.display_count = 0
        self.skip_count = 0
        self.t0 = time.time()

        self.h264_rx.start()

        self.get_logger().info(
            f"[H264_BGR_BRIDGE] started | UDP:{udp_port} -> {IMAGE_TOPIC} "
            f"rate={publish_rate_hz:.1f}Hz"
        )

    def on_frame_meta(self, msg: String):
        try:
            meta = json.loads(msg.data)
        except Exception:
            return

        if meta.get("pts_ns") is None:
            return

        if meta.get("frame_t_cam_ns", meta.get("t_cam_ns")) is None:
            return

        self.h264_rx.add_meta(meta)

    def tick_publish(self):
        payload, meta = self.frame_buffer.take_latest()

        if payload is None or not isinstance(meta, dict):
            return

        if meta.get("_frame_format") != "bgr":
            return

        bgr = payload
        try:
            h, w = bgr.shape[:2]
        except Exception:
            return

        bgr = np.ascontiguousarray(bgr)

        seq = _to_int(meta.get("seq", -1), -1)

        # ------------------------------------------------------------
        # 1) UI 표시용 영상
        # ------------------------------------------------------------
        # 표시용만 1/2 해상도로 줄여 JPEG 압축한다.
        # UI에서는 다시 원본 크기로 resize해서 클릭 좌표계는 1456x1088 기준을 유지한다.
        try:
            display_scale = 0.4
            display_w = max(1, int(w * display_scale))
            display_h = max(1, int(h * display_scale))

            display_bgr = cv2.resize(
                bgr,
                (display_w, display_h),
                interpolation=cv2.INTER_AREA
            )

            ok_jpg, enc = cv2.imencode(
                ".jpg",
                display_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), 45]
            )
        except Exception:
            ok_jpg = False
            enc = None
            display_w = 0
            display_h = 0

        if ok_jpg:
            display_img = CompressedImage()
            display_img.header.frame_id = str(seq)
            display_img.header.stamp = self.get_clock().now().to_msg()
            display_img.format = (
                f"jpeg;orig_width={int(w)};orig_height={int(h)};"
                f"display_width={int(display_w)};display_height={int(display_h)}"
            )
            display_img.data = enc.tobytes()

            self.pub_img_display.publish(display_img)
            self.display_count += 1

        # ------------------------------------------------------------
        # 2) 계산용 raw BGR 영상
        # ------------------------------------------------------------
        # /camera/image_bgr는 원본 1456x1088 BGR이라 매우 무겁다.
        # 현재 compute_logger는 /camera/frame_meta + /ui/click_event 기반으로 계산하므로
        # raw BGR를 상시 publish하지 않는다.
        #
        # 필요 시 ENABLE_CALC_BGR_PUBLISH=True로 바꾸면
        # sync_ok 프레임만 /camera/image_bgr로 발행할 수 있다.
        if not ENABLE_CALC_BGR_PUBLISH:
            self._print_stats_if_needed()
            return

        # /camera/image_bgr는 기존처럼 sync_ok일 때만 원본 해상도 BGR로 publish한다.
        # sync 실패 시 계산용 publish를 막아 잘못된 역투영을 방지한다.
        sync_ok = bool(meta.get("sync_ok", False))
        frame_t_cam_ns = meta.get("frame_t_cam_ns", meta.get("t_cam_ns"))

        if (not sync_ok) or frame_t_cam_ns is None:
            self.skip_count += 1
            self._print_stats_if_needed()
            return

        t_ns = _to_int(frame_t_cam_ns, 0)

        data_bytes = bgr.tobytes()

        img = Image()
        img.header.frame_id = str(seq)
        img.header.stamp.sec = int(t_ns // 1_000_000_000)
        img.header.stamp.nanosec = int(t_ns % 1_000_000_000)
        img.height = int(h)
        img.width = int(w)
        img.encoding = "bgr8"
        img.is_bigendian = 0
        img.step = int(w * 3)
        img.data = data_bytes

        self.pub_img.publish(img)
        self.pub_count += 1

        self._print_stats_if_needed()

    def _print_stats_if_needed(self):
        now = time.time()
        if now - self.t0 < 1.0:
            return

        dt = now - self.t0
        pub_fps = self.pub_count / dt if dt > 0 else 0.0
        display_fps = self.display_count / dt if dt > 0 else 0.0

        self.get_logger().info(
            f"[H264_BGR_BRIDGE] pub_fps={pub_fps:.1f} "
            f"display_fps={display_fps:.1f} "
            f"skip={self.skip_count}"
        )

        self.t0 = now
        self.pub_count = 0
        self.display_count = 0
        self.skip_count = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--width", type=int, default=1456)
    ap.add_argument("--height", type=int, default=1088)
    ap.add_argument("--max-match-dt-ms", type=float, default=100.0)
    args = ap.parse_args()

    rclpy.init()
    node = H264ToBgrBridge(
        udp_port=args.port,
        publish_rate_hz=args.rate,
        width=args.width,
        height=args.height,
        max_match_dt_ms=args.max_match_dt_ms,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()