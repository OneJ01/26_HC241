#!/usr/bin/env python3
# h264_meta_pub.py
# /dev/shm/h264_meta/latest_meta.json -> ROS2 /camera/frame_meta publish

import json
import time
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


META_PATH = Path("/dev/shm/h264_meta/latest_meta.json")
TOPIC = "/camera/frame_meta"


class H264MetaPublisher(Node):
    def __init__(self, rate_hz: float):
        super().__init__("h264_frame_meta_pub")

        self.pub = self.create_publisher(String, TOPIC, 10)
        self.period = 1.0 / max(rate_hz, 0.1)

        self.last_seq = None
        self.last_frame_t_cam_ns = None
        self.last_mtime_ns = 0

        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(f"watch meta: {META_PATH}")
        self.get_logger().info(f"publish -> {TOPIC} @ {rate_hz:.2f} Hz")

    def tick(self):
        try:
            st = META_PATH.stat()
        except Exception:
            return

        # 파일 갱신 없으면 스킵
        if st.st_mtime_ns == self.last_mtime_ns:
            return
        self.last_mtime_ns = st.st_mtime_ns

        try:
            raw = META_PATH.read_text()
            meta = json.loads(raw)
        except Exception as e:
            self.get_logger().warn(f"failed to read meta: {type(e).__name__}: {e}")
            return

        seq = meta.get("seq")
        try:
            seq = int(seq)
        except Exception:
            return
        # 필수 필드 확인
        frame_t_cam_ns = meta.get("frame_t_cam_ns", meta.get("t_cam_ns"))
        pts_ns = meta.get("pts_ns")

        if frame_t_cam_ns is None or pts_ns is None:
            self.get_logger().warn(f"invalid meta: seq={seq}, missing frame_t_cam_ns or pts_ns")
            return

        # 필드 이름 통일
        meta["frame_t_cam_ns"] = int(frame_t_cam_ns)
        meta["t_cam_ns"] = int(frame_t_cam_ns)
        meta["pts_ns"] = int(pts_ns)
        meta["seq"] = int(seq)

        # ------------------------------------------------------------
        # 중복 meta 방지 + sender 재시작(seq reset) 안전 처리
        # ------------------------------------------------------------
        cur_seq = int(seq)
        cur_t_cam_ns = int(frame_t_cam_ns)

        if self.last_seq is not None:
            try:
                last_seq = int(self.last_seq)
                last_t_cam_ns = (
                    int(self.last_frame_t_cam_ns)
                    if self.last_frame_t_cam_ns is not None
                    else None
                )
            except Exception:
                last_seq = None
                last_t_cam_ns = None

            if last_seq is not None and last_t_cam_ns is not None:
                # 완전히 같은 프레임이면 중복 발행 방지
                if cur_seq == last_seq and cur_t_cam_ns == last_t_cam_ns:
                    return

                # seq가 작아졌지만 촬영 시간이 더 최신이면 sender 재시작으로 판단하고 허용
                if cur_seq < last_seq and cur_t_cam_ns > last_t_cam_ns:
                    self.get_logger().warn(
                        f"seq reset detected: last_seq={last_seq}, cur_seq={cur_seq}; "
                        f"accept new camera session"
                    )

                # seq도 작고 시간도 과거/동일이면 stale meta로 판단하고 버림
                elif cur_seq < last_seq and cur_t_cam_ns <= last_t_cam_ns:
                    return

                # seq가 같은데 시간도 최신이 아니면 중복/역행으로 판단
                elif cur_seq == last_seq and cur_t_cam_ns <= last_t_cam_ns:
                    return

            elif last_seq is not None:
                # 시간 기준이 없는 예외 상황에서는 기존처럼 seq 기준으로만 방어
                if cur_seq <= last_seq:
                    return
        meta["meta_pub_time_ns"] = time.time_ns()

        msg = String()
        msg.data = json.dumps(meta, ensure_ascii=False, separators=(",", ":"))
        self.pub.publish(msg)

        self.last_seq = int(seq)
        self.last_frame_t_cam_ns = int(frame_t_cam_ns)

        if seq % 30 == 0:
            self.get_logger().info(
                f"pub seq={seq} pts_ns={meta['pts_ns']} frame_t_cam_ns={meta['frame_t_cam_ns']}"
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = H264MetaPublisher(rate_hz=args.rate)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()