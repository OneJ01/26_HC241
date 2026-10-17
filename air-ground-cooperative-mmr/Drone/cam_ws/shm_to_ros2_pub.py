import argparse, json, os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

FRAME_PATH = "/dev/shm/cam_frame.jpg"
META_PATH  = "/dev/shm/cam_meta.json"

IMG_TOPIC  = "/camera/image_raw/compressed"
META_TOPIC = "/camera/metadata"

class ShmCompressedPub(Node):
    def __init__(self, rate_hz: float):
        super().__init__("shm_cam_pub")
        self.pub_img  = self.create_publisher(CompressedImage, IMG_TOPIC, 10)
        self.pub_meta = self.create_publisher(String, META_TOPIC, 10)

        self.period = 1.0 / max(0.1, rate_hz)
        self.timer = self.create_timer(self.period, self.tick)

        self.last_mtime = 0.0
        self.get_logger().info(f"publishing {IMG_TOPIC} @ {rate_hz:.2f} Hz")
        self.get_logger().info(f"frame: {FRAME_PATH}")
        self.get_logger().info(f"meta : {META_PATH}")

    def tick(self):
        # 프레임 파일이 아직 없으면 패스
        if not os.path.exists(FRAME_PATH):
            return

        # 파일 갱신이 없으면(같은 프레임이면) 불필요한 publish 줄이기
        try:
            mtime = os.path.getmtime(FRAME_PATH)
        except Exception:
            return
        if mtime <= self.last_mtime:
            return
        self.last_mtime = mtime

        # JPG 바이트 그대로 읽어서 CompressedImage로 publish
        try:
            with open(FRAME_PATH, "rb") as f:
                jpg = f.read()
        except Exception as e:
            self.get_logger().warn(f"read frame failed: {e}")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imx296"
        msg.format = "jpeg"
        msg.data = jpg
        self.pub_img.publish(msg)

        # 메타 publish (있으면)
        if os.path.exists(META_PATH):
            try:
                with open(META_PATH, "r") as f:
                    meta_txt = f.read()
                # JSON이 아니어도 그냥 String으로 보냄
                self.pub_meta.publish(String(data=meta_txt))
            except Exception as e:
                self.get_logger().warn(f"read meta failed: {e}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = ShmCompressedPub(args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
