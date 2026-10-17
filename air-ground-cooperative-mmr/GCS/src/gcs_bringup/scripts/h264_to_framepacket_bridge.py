#!/usr/bin/env python3
# h264_to_framepacket_bridge.py
#
# PC side bridge:
#   Pi H.264 UDP stream + /camera/frame_meta
#   → decoded BGR frame + matched timestamp meta
#   → JPEG encode on PC
#   → publish /camera/frame_packet
#
# 목적:
# - 무선망 구간은 H.264로 최적화
# - PC 내부에서는 기존 gcs_ui.py / compute_logger.py 구조를 살리기 위해 FramePacket 재발행
# - UI 프로세스에는 GStreamer를 넣지 않음

import json
import time
import argparse

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from cam_msgs.msg import FramePacket

from camera_frame_buffer import CameraFrameBuffer
from h264_receiver import H264Receiver


FRAME_META_TOPIC = "/camera/frame_meta"
FRAME_PACKET_TOPIC = "/camera/frame_packet"


def _to_int(value, default=0):
    try:
        if value is None:
            return int(default)
        return int(value)
    except Exception:
        return int(default)


def _to_float(value, default=0.0):
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


class H264ToFramePacketBridge(Node):
    def __init__(
        self,
        udp_port: int,
        jpeg_quality: int,
        publish_rate_hz: float,
        width: int,
        height: int,
        max_match_dt_ms: float,
    ):
        super().__init__("h264_to_framepacket_bridge")

        self.jpeg_quality = int(jpeg_quality)
        self.publish_rate_hz = float(publish_rate_hz)

        qos_frame = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_frame = self.create_publisher(
            FramePacket,
            FRAME_PACKET_TOPIC,
            qos_frame,
        )

        self.sub_meta = self.create_subscription(
            String,
            FRAME_META_TOPIC,
            self.on_frame_meta,
            10,
        )

        self.frame_buffer = CameraFrameBuffer()

        # H.264 receiver는 별도 thread로 돈다.
        # UI 프로세스가 아니므로 GStreamer + rclpy 조합만 사용한다.
        self.h264_rx = H264Receiver(
            frame_buffer=self.frame_buffer,
            udp_port=int(udp_port),
            width=int(width),
            height=int(height),
            max_match_dt_ms=float(max_match_dt_ms),
            log_fn=lambda msg: print(str(msg), flush=True),
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.1),
            self.tick_publish,
        )

        self.pub_count = 0
        self.skip_unsync_count = 0
        self.encode_fail_count = 0
        self.t0 = time.time()

        self.h264_rx.start()

        self.get_logger().info(
            f"[H264_BRIDGE] started | UDP:{udp_port} -> {FRAME_PACKET_TOPIC} "
            f"rate={publish_rate_hz:.1f}Hz quality={jpeg_quality}"
        )
        self.get_logger().info(
            f"[H264_BRIDGE] subscribe meta: {FRAME_META_TOPIC}"
        )

    def on_frame_meta(self, msg: String):
        try:
            meta = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(
                f"[H264_BRIDGE] frame_meta JSON parse fail: {type(e).__name__}: {e}"
            )
            return

        # 필수 필드 확인
        if meta.get("pts_ns") is None:
            return

        if meta.get("frame_t_cam_ns", meta.get("t_cam_ns")) is None:
            return

        self.h264_rx.add_meta(meta)

    def tick_publish(self):
        payload, meta = self.frame_buffer.take_latest()
        if payload is None or not isinstance(meta, dict):
            return

        fmt = meta.get("_frame_format")
        if fmt != "bgr":
            return

        sync_ok = bool(meta.get("sync_ok", False))
        frame_t_cam_ns = meta.get("frame_t_cam_ns", meta.get("t_cam_ns"))

        # 좌표 계산 신뢰성을 위해 sync 안 된 frame은 FramePacket으로 내보내지 않음.
        if (not sync_ok) or frame_t_cam_ns is None:
            self.skip_unsync_count += 1
            self._print_stats_if_needed()
            return

        bgr = payload

        try:
            h, w = bgr.shape[:2]
        except Exception:
            return

        encode_params = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            int(self.jpeg_quality),
        ]

        ok, enc = cv2.imencode(".jpg", bgr, encode_params)
        if not ok:
            self.encode_fail_count += 1
            self._print_stats_if_needed()
            return

        jpg_bytes = enc.tobytes()
        if not jpg_bytes:
            self.encode_fail_count += 1
            self._print_stats_if_needed()
            return

        seq = _to_int(meta.get("seq", -1), -1)
        t_cam_ns = _to_int(frame_t_cam_ns, 0)

        frame_duration_ns = _to_int(meta.get("frame_duration_ns", meta.get("FrameDuration", 0)), 0)
        # libcamera/Picamera2에서 FrameDuration이 us 단위처럼 66666으로 들어오는 경우 보정
        if 0 < frame_duration_ns < 1_000_000:
            frame_duration_ns *= 1000

        msg = FramePacket()
        msg.seq = int(seq)
        msg.width = int(_to_int(meta.get("width", w), w))
        msg.height = int(_to_int(meta.get("height", h), h))

        msg.t_cam_ns = int(t_cam_ns)
        msg.frame_wallclock_ns = int(_to_int(meta.get("FrameWallClock", t_cam_ns), t_cam_ns))
        msg.sensor_timestamp_ns = int(
            _to_int(meta.get("SensorTimestamp", meta.get("sensor_timestamp_ns", 0)), 0)
        )

        msg.exposure_us = int(_to_int(meta.get("ExposureTime_us", meta.get("exposure_us", 0)), 0))
        msg.analogue_gain = float(_to_float(meta.get("AnalogueGain", meta.get("analogue_gain", 0.0)), 0.0))
        msg.digital_gain = float(_to_float(meta.get("DigitalGain", meta.get("digital_gain", 0.0)), 0.0))
        msg.frame_duration_ns = int(frame_duration_ns)
        msg.host_time_ns = int(_to_int(meta.get("host_time_ns", 0), 0))

        try:
            msg.jpg = jpg_bytes
        except Exception:
            msg.jpg = list(jpg_bytes)

        self.pub_frame.publish(msg)
        self.pub_count += 1

        if self.pub_count % 30 == 0:
            match_dt = meta.get("pts_match_dt_ms")
            offset_ms = meta.get("pts_offset_ms")
            self.get_logger().info(
                "[H264_BRIDGE/PUB] "
                f"seq={seq} "
                f"t_cam_ns={t_cam_ns} "
                f"match_dt_ms={match_dt} "
                f"offset_ms={offset_ms} "
                f"jpg={len(jpg_bytes) / 1024:.1f}KB"
            )

        self._print_stats_if_needed()

    def _print_stats_if_needed(self):
        now = time.time()
        if now - self.t0 < 1.0:
            return

        elapsed = now - self.t0
        pub_fps = self.pub_count / elapsed if elapsed > 0 else 0.0

        self.get_logger().info(
            f"[H264_BRIDGE] pub_fps={pub_fps:.1f} "
            f"skip_unsync={self.skip_unsync_count} "
            f"encode_fail={self.encode_fail_count}"
        )

        self.pub_count = 0
        self.skip_unsync_count = 0
        self.encode_fail_count = 0
        self.t0 = now

    def destroy_node(self):
        try:
            if self.h264_rx is not None:
                self.h264_rx.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--quality", type=int, default=60)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--width", type=int, default=1456)
    ap.add_argument("--height", type=int, default=1088)
    ap.add_argument("--max-match-dt-ms", type=float, default=80.0)
    args = ap.parse_args()

    rclpy.init()
    node = H264ToFramePacketBridge(
        udp_port=args.port,
        jpeg_quality=args.quality,
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