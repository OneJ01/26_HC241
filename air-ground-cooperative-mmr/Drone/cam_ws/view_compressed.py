#!/usr/bin/env python3
#real file
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2
import threading
import time

TOPIC = "/camera/image_compressed"

class Viewer(Node):
    def __init__(self):
        super().__init__("compressed_viewer")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(CompressedImage, TOPIC, self.cb, qos)

        self.lock = threading.Lock()
        self.last_bytes = None
        self.last_stamp = None
        self.last_show_t = None

        cv2.namedWindow("camera/image_compressed", cv2.WINDOW_NORMAL)

        self.get_logger().info(f"subscribing: {TOPIC} (BestEffort, depth=1)")
        self.get_logger().info("press 'q' to quit")

    def cb(self, msg: CompressedImage):
        # 콜백에서는 "저장만" (디코드/GUI 금지)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.last_bytes = bytes(msg.data)
            self.last_stamp = t

def main():
    rclpy.init()
    node = Viewer()

    try:
        while rclpy.ok():
            # ROS 이벤트 조금 처리 (구독 콜백 호출)
            rclpy.spin_once(node, timeout_sec=0.01)

            # 최신 프레임 가져오기
            with node.lock:
                jpg = node.last_bytes
                t = node.last_stamp
                node.last_bytes = None  # 소비했으면 비워서 중복 디코드 방지

            # GUI 이벤트는 "항상" 처리돼야 창이 안 멈춤
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break

            if jpg is None or t is None:
                continue

            # 디코드
            arr = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # FPS 표시(대략)
            fps = 0.0
            if node.last_show_t is not None:
                dt = t - node.last_show_t
                if dt > 1e-6:
                    fps = 1.0 / dt
            node.last_show_t = t

            cv2.putText(frame, f"t={t:.6f}  fps~{fps:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            cv2.imshow("camera/image_compressed", frame)

    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
