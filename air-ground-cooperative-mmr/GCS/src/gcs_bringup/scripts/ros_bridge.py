# ros_bridge.py
# ROS2 node / executor thread / camera subscriber / click publisher 담당

import json
import threading
import time
import numpy as np

class RosBridge:
    def __init__(
        self,
        frame_buffer,
        log_fn=None,
        h264_receiver=None,
        telemetry_callback=None,
        altitude_callback=None,
        target_debug_callback=None,
        target_point_callback=None,
        impact_debug_callback=None,
        impact_point_callback=None,
        mmr_callback=None,
        mmr_engagement_callback=None,
        mission_status_callback=None,
        mission_path_callback=None,
        ros_domain_id: int | None = None,
        camera_topic: str = "/camera/frame_packet",
        frame_meta_topic: str = "/camera/frame_meta",
        click_topic: str = "/ui/click_event",
        telemetry_topic: str = "/drone/telemetry",
        altitude_topic: str = "/compute/drone_altitude",
        target_debug_topic: str = "/compute/target_debug",
        target_point_topic: str = "/compute/target_point",
        impact_debug_topic: str = "/compute/impact_debug",
        impact_point_topic: str = "/compute/impact_point",
        mmr_topic: str = "/mmr/status",
        mmr_engagement_topic: str = "/mmr/engagement_status",
        mission_upload_request_topic: str = "/mission/upload_request",
        mission_status_topic: str = "/mission/status",
        mission_path_topic: str = "/mission/path",
        fire_ready_topic: str = "/mmr/fire_ready",
        tilt_status_topic: str = "/imu/tilt_status",
        fire_solution_status_topic: str = "/mmr/fire_solution/status",
        fire_ready_callback=None,
        tilt_status_callback=None,
        fire_solution_status_callback=None,
        bgr_image_topic: str = "/camera/image_jpeg_display",
    ):
        self.frame_buffer = frame_buffer
        self.log_fn = log_fn or print
        self.h264_receiver = h264_receiver
        self.camera_topic = camera_topic
        self.frame_meta_topic = frame_meta_topic
        self.click_topic = click_topic
        self.telemetry_topic = telemetry_topic
        self.telemetry_callback = telemetry_callback
        self.altitude_topic = altitude_topic
        self.altitude_callback = altitude_callback
        self.target_debug_topic = target_debug_topic
        self.target_point_topic = target_point_topic
        self.target_debug_callback = target_debug_callback
        self.target_point_callback = target_point_callback
        self.impact_debug_topic = impact_debug_topic
        self.impact_point_topic = impact_point_topic
        self.impact_debug_callback = impact_debug_callback
        self.impact_point_callback = impact_point_callback
        self.mmr_topic = mmr_topic
        self.mmr_callback = mmr_callback
        self.mmr_engagement_topic = mmr_engagement_topic
        self.mmr_engagement_callback = mmr_engagement_callback
        self.mission_upload_request_topic = mission_upload_request_topic
        self.mission_status_topic = mission_status_topic
        self.mission_path_topic = mission_path_topic
        self.mission_status_callback = mission_status_callback
        self.mission_path_callback = mission_path_callback
        self.pub_mission_upload = None
        self.bgr_image_topic = bgr_image_topic

        self.fire_ready_topic = fire_ready_topic
        self.fire_ready_callback = fire_ready_callback
        self.tilt_status_topic = tilt_status_topic
        self.tilt_status_callback = tilt_status_callback
        self.fire_solution_status_topic = fire_solution_status_topic
        self.fire_solution_status_callback = fire_solution_status_callback

        self.rclpy = None
        self.node = None
        self.executor = None
        self.thread = None
        self.running = False
        self.RosString = None
        self.pub_click = None

        self._init_ros()
        self._setup_click_publisher()
        self._setup_mission_upload_publisher()
        self._setup_camera_subscriber()
        self._setup_bgr_image_subscriber()
        self._setup_frame_meta_subscriber()
        self._setup_telemetry_subscriber()
        self._setup_altitude_subscriber()
        self._setup_target_debug_subscriber()
        self._setup_target_point_subscriber()
        self._setup_impact_debug_subscriber()
        self._setup_impact_point_subscriber()
        self._setup_mmr_subscriber()
        self._setup_mmr_engagement_subscriber()
        self._setup_mission_status_subscriber()
        self._setup_mission_path_subscriber()
        self._setup_fire_ready_subscriber()
        self._setup_tilt_status_subscriber()
        self._setup_fire_solution_status_subscriber()
        self._start_spin_thread()

    def _log(self, msg: str):
        try:
            self.log_fn(str(msg))
        except Exception:
            print(str(msg), flush=True)

    def _init_ros(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"[ROS/ERR] ROS2 import 실패: {e}")
            return

        self.rclpy = rclpy
        self.RosString = String

        if not rclpy.ok():
            try:
                rclpy.init(args=None)
            except Exception:
                pass

        self.node = Node("gcs_ui")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        print("[CHECK] gcs_ui node created from RosBridge", flush=True)
        self._log("[시스템] ROS Bridge 초기화 완료")

    def _setup_click_publisher(self):
        if self.node is None or self.RosString is None:
            return

        self.pub_click = self.node.create_publisher(
            self.RosString,
            self.click_topic,
            10
        )
        self._log(f"[시스템] 클릭 이벤트 발행 준비: {self.click_topic}")

    def _setup_mission_upload_publisher(self):
        if self.node is None or self.RosString is None:
            return

        self.pub_mission_upload = self.node.create_publisher(
            self.RosString,
            self.mission_upload_request_topic,
            10
        )
        self._log(f"[시스템] 미션 업로드 요청 발행 준비: {self.mission_upload_request_topic}")

    def publish_mission_upload_request(self, payload):
        """
        PC GCS UI → 라파 mission_manager.py

        payload 예시:
        {
            "type": "mission_upload_request",
            "mission_id": "A",
            "filename": "mission_A.waypoints",
            "sha256": "...",
            "waypoints_text": "QGC WPL 110\\n...",
            "point_count": 7,
            "stamp": ...
        }
        """
        try:
            if self.RosString is None:
                self._log("[MISSION/ERR] mission upload message type 준비 안 됨")
                return False

            if self.pub_mission_upload is None:
                self._log("[MISSION/ERR] mission upload publisher 준비 안 됨")
                return False

            msg = self.RosString()

            if isinstance(payload, dict):
                msg.data = json.dumps(payload, ensure_ascii=False)
            else:
                # 구버전 호환. 가능하면 이제 사용하지 않음.
                msg.data = json.dumps({
                    "type": "mission_upload_request",
                    "mission_id": str(payload),
                }, ensure_ascii=False)

            self.pub_mission_upload.publish(msg)
            return True

        except Exception as e:
            self._log(f"[MISSION/ERR] upload request 발행 실패: {type(e).__name__}: {e}")
            return False

    def _setup_mission_status_subscriber(self):
        if self.node is None or self.RosString is None:
            return

        if self.mission_status_callback is None:
            self._log("[시스템] 미션 상태 구독 비활성화: callback 없음")
            return

        def _mission_status_cb(msg):
            try:
                data = json.loads(msg.data)
                self.mission_status_callback(data)
            except Exception as e:
                self._log(f"[MISSION/ERR] 미션 상태 처리 실패: {type(e).__name__}: {e}")

        self.node.create_subscription(
            self.RosString,
            self.mission_status_topic,
            _mission_status_cb,
            10
        )
        self._log(f"[시스템] 미션 상태 구독 활성화: {self.mission_status_topic}")

    def _setup_mission_path_subscriber(self):
        if self.node is None or self.RosString is None:
            return

        if self.mission_path_callback is None:
            self._log("[시스템] 미션 경로 구독 비활성화: callback 없음")
            return

        def _mission_path_cb(msg):
            try:
                data = json.loads(msg.data)
                self.mission_path_callback(data)
            except Exception as e:
                self._log(f"[MISSION/ERR] 미션 경로 처리 실패: {type(e).__name__}: {e}")

        self.node.create_subscription(
            self.RosString,
            self.mission_path_topic,
            _mission_path_cb,
            10
        )
        self._log(f"[시스템] 미션 경로 구독 활성화: {self.mission_path_topic}")

    def _setup_camera_subscriber(self):
        if self.node is None:
            return

        try:
            from cam_msgs.msg import FramePacket
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )

            def _pkt_cb(msg: FramePacket):
                try:
                    jpg_bytes = bytes(msg.jpg)
                    meta = {
                        "seq": int(msg.seq),
                        "width": int(msg.width),
                        "height": int(msg.height),
                        "t_cam_ns": int(msg.t_cam_ns),
                        "FrameWallClock": int(msg.frame_wallclock_ns),
                        "SensorTimestamp": int(msg.sensor_timestamp_ns),
                        "ExposureTime_us": int(msg.exposure_us),
                        "AnalogueGain": float(msg.analogue_gain),
                        "DigitalGain": float(msg.digital_gain),
                        "FrameDuration": int(msg.frame_duration_ns),
                        "host_time_ns": int(msg.host_time_ns),
                    }
                    self.frame_buffer.put(jpg_bytes, meta)
                except Exception as e:
                    self._log(f"[ROS/ERR] 카메라 프레임 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(FramePacket, self.camera_topic, _pkt_cb, qos)
            self._log(f"[시스템] 카메라 프레임 구독 활성화: {self.camera_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 카메라 구독 비활성화: cam_msgs 없음 또는 import 실패: {e}")

    def _setup_frame_meta_subscriber(self):
        """
        H.264 timestamp-primary 구조용 meta subscriber.

        /camera/frame_meta:
            Pi의 h264_meta_pub.py가 발행하는 JSON String
            - seq
            - pts_ns
            - frame_t_cam_ns
            - width
            - height
            등을 포함한다.

        이 meta는 H264Receiver.add_meta(meta)로 전달되어,
        decoded H.264 frame의 PTS와 매칭된다.
        """
        if self.node is None:
            return

        if self.h264_receiver is None:
            self._log("[시스템] H.264 frame_meta 구독 비활성화: h264_receiver 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _frame_meta_cb(msg):
                try:
                    data = json.loads(msg.data)

                    # 필수 필드 확인
                    pts_ns = data.get("pts_ns")
                    frame_t_cam_ns = data.get("frame_t_cam_ns", data.get("t_cam_ns"))

                    if pts_ns is None or frame_t_cam_ns is None:
                        self._log("[ROS/ERR] frame_meta 필수 필드 누락: pts_ns/frame_t_cam_ns")
                        return

                    self.h264_receiver.add_meta(data)

                except Exception as e:
                    self._log(f"[ROS/ERR] frame_meta 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.frame_meta_topic,
                _frame_meta_cb,
                10
            )

            self._log(f"[시스템] H.264 frame_meta 구독 활성화: {self.frame_meta_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] H.264 frame_meta 구독 비활성화: {e}")

    def _setup_bgr_image_subscriber(self):
        """
        표시용 JPEG CompressedImage를 구독한다.
        bridge는 표시용만 downscale JPEG로 보내고,
        여기서 다시 원본 크기로 복원하여 UI 클릭 좌표계를 유지한다.
        """
        if self.node is None:
            return

        try:
            import cv2
            from sensor_msgs.msg import CompressedImage
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )

            def _parse_orig_size(fmt: str):
                orig_w = None
                orig_h = None
                try:
                    tokens = str(fmt).replace(";", " ").replace(",", " ").split()
                    for tok in tokens:
                        if tok.startswith("orig_width="):
                            orig_w = int(tok.split("=", 1)[1])
                        elif tok.startswith("orig_height="):
                            orig_h = int(tok.split("=", 1)[1])
                except Exception:
                    pass
                return orig_w, orig_h

            def _img_cb(msg: CompressedImage):
                try:
                    arr = np.frombuffer(msg.data, dtype=np.uint8)
                    if arr.size <= 0:
                        return

                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if bgr is None:
                        return

                    orig_w, orig_h = _parse_orig_size(msg.format)

                    # 클릭 좌표/캘리브레이션 기준 유지를 위해 UI buffer에는 원본 크기로 넣는다.
                    if (
                        orig_w is not None and orig_h is not None
                        and orig_w > 0 and orig_h > 0
                        and (bgr.shape[1] != orig_w or bgr.shape[0] != orig_h)
                    ):
                        bgr = cv2.resize(
                            bgr,
                            (int(orig_w), int(orig_h)),
                            interpolation=cv2.INTER_LINEAR
                        )

                    h, w = bgr.shape[:2]

                    try:
                        seq = int(msg.header.frame_id)
                    except Exception:
                        seq = -1

                    t_cam_ns = (
                        int(msg.header.stamp.sec) * 1_000_000_000
                        + int(msg.header.stamp.nanosec)
                    )

                    meta = {
                        "_frame_format": "bgr",
                        "_frame_source": "image_jpeg_display",
                        "seq": seq,
                        "width": int(w),
                        "height": int(h),
                        "t_cam_ns": int(t_cam_ns),
                        "frame_t_cam_ns": int(t_cam_ns),
                        "sync_ok": True,
                    }

                    self.frame_buffer.put_bgr_frame(bgr, meta)

                except Exception as e:
                    self._log(f"[ROS/ERR] JPEG 표시용 이미지 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(CompressedImage, self.bgr_image_topic, _img_cb, qos)
            self._log(f"[시스템] JPEG 표시용 이미지 구독 활성화: {self.bgr_image_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] JPEG 표시용 이미지 구독 비활성화: {e}")


    def _setup_telemetry_subscriber(self):
        if self.node is None:
            return

        if self.telemetry_callback is None:
            self._log("[시스템] 드론 텔레메트리 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _telemetry_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self.telemetry_callback(data)
                except Exception as e:
                    self._log(f"[ROS/ERR] 드론 텔레메트리 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.telemetry_topic,
                _telemetry_cb,
                10
            )
            self._log(f"[시스템] 드론 텔레메트리 구독 활성화: {self.telemetry_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 드론 텔레메트리 구독 비활성화: {e}")
            
    def _setup_altitude_subscriber(self):
        if self.node is None:
            return

        if self.altitude_callback is None:
            self._log("[시스템] 고도 정보 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _altitude_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self.altitude_callback(data)
                except Exception as e:
                    self._log(f"[ROS/ERR] 고도 정보 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.altitude_topic,
                _altitude_cb,
                10
            )
            self._log(f"[시스템] 고도 정보 구독 활성화: {self.altitude_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 고도 정보 구독 비활성화: {e}")

    def _setup_target_debug_subscriber(self):
        if self.node is None:
            return

        if self.target_debug_callback is None:
            self._log("[시스템] 역투영 디버그 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _target_debug_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self.target_debug_callback(data)
                except Exception as e:
                    self._log(f"[ROS/ERR] 역투영 디버그 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.target_debug_topic,
                _target_debug_cb,
                10
            )
            self._log(f"[시스템] 역투영 디버그 구독 활성화: {self.target_debug_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 역투영 디버그 구독 비활성화: {e}")

    def _setup_target_point_subscriber(self):
        if self.node is None:
            return

        if self.target_point_callback is None:
            self._log("[시스템] 표적 좌표 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _target_point_cb(msg):
                try:
                    data = json.loads(msg.data)

                    status = data.get("status")
                    if status is not None and status != "HIT":
                        self._log(f"[시스템] 표적 좌표 메시지 무시: status={status}")
                        return

                    self.target_point_callback(data)

                except Exception as e:
                    self._log(f"[ROS/ERR] 표적 좌표 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.target_point_topic,
                _target_point_cb,
                10
            )
            self._log(f"[시스템] 표적 좌표 구독 활성화: {self.target_point_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 표적 좌표 구독 비활성화: {e}")


    def _setup_impact_debug_subscriber(self):
        if self.node is None:
            return

        if self.impact_debug_callback is None:
            self._log("[시스템] 탄착 역투영 디버그 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _impact_debug_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self.impact_debug_callback(data)
                except Exception as e:
                    self._log(f"[ROS/ERR] 탄착 역투영 디버그 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.impact_debug_topic,
                _impact_debug_cb,
                10
            )
            self._log(f"[시스템] 탄착 역투영 디버그 구독 활성화: {self.impact_debug_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 탄착 역투영 디버그 구독 비활성화: {e}")

    def _setup_impact_point_subscriber(self):
        if self.node is None:
            return

        if self.impact_point_callback is None:
            self._log("[시스템] 탄착 좌표 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _impact_point_cb(msg):
                try:
                    data = json.loads(msg.data)
                    status = data.get("status")
                    if status is not None and status != "HIT":
                        self._log(f"[시스템] 탄착 좌표 메시지 무시: status={status}")
                        return
                    self.impact_point_callback(data)
                except Exception as e:
                    self._log(f"[ROS/ERR] 탄착 좌표 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.impact_point_topic,
                _impact_point_cb,
                10
            )
            self._log(f"[시스템] 탄착 좌표 구독 활성화: {self.impact_point_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 탄착 좌표 구독 비활성화: {e}")

    def _setup_mmr_subscriber(self):
        if self.node is None:
            return

        if self.mmr_callback is None:
            self._log("[시스템] MMR 상태 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _mmr_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self.mmr_callback(data)
                except Exception as e:
                    self._log(f"[ROS/ERR] MMR 상태 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.mmr_topic,
                _mmr_cb,
                10
            )
            self._log(f"[시스템] MMR 상태 구독 활성화: {self.mmr_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] MMR 상태 구독 비활성화: {e}")

    def _setup_mmr_engagement_subscriber(self):
        """MMR 전체 정렬·조준·발사 상태 스냅샷 구독."""
        if self.node is None:
            return

        if self.mmr_engagement_callback is None:
            self._log("[시스템] MMR 시퀀스 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _mmr_engagement_cb(msg):
                try:
                    data = json.loads(msg.data)
                    if not isinstance(data, dict):
                        raise ValueError("JSON root must be an object")
                    self.mmr_engagement_callback(data)
                except Exception as e:
                    self._log(
                        f"[ROS/ERR] MMR 시퀀스 처리 실패: "
                        f"{type(e).__name__}: {e}"
                    )

            # 현재 상태 스냅샷이므로 최신 1개만 유지한다.
            self.node.create_subscription(
                self.RosString,
                self.mmr_engagement_topic,
                _mmr_engagement_cb,
                1
            )
            self._log(
                f"[시스템] MMR 시퀀스 구독 활성화: "
                f"{self.mmr_engagement_topic}"
            )

        except Exception as e:
            self._log(f"[ROS/ERR] MMR 시퀀스 구독 비활성화: {e}")

    def _start_spin_thread(self):
        if self.executor is None:
            return

        self.running = True
        self.thread = threading.Thread(
            target=self._spin_thread_main,
            daemon=True
        )
        self.thread.start()

    def _spin_thread_main(self):
        try:
            while self.running and self.executor is not None:
                self.executor.spin_once(timeout_sec=0.01)
        except Exception as e:
            self._log(f"[ROS/SPIN/ERR] {type(e).__name__}: {e}")

    def publish_click(self, evt: dict):
        if self.pub_click is None or self.RosString is None:
            self._log("[ROS/ERR] 클릭 이벤트 발행 실패: publisher 준비 안 됨")
            return

        try:
            msg = self.RosString()
            msg.data = json.dumps(evt, separators=(",", ":"))
            self.pub_click.publish(msg)
        except Exception as e:
            self._log(f"[ROS/ERR] 클릭 이벤트 발행 실패: {type(e).__name__}: {e}")

    def _setup_fire_ready_subscriber(self):
        if self.node is None:
            return

        if self.fire_ready_callback is None:
            self._log("[시스템] 발사 준비 신호 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _cb(msg):
                try:
                    self.fire_ready_callback(json.loads(msg.data))
                except Exception as e:
                    self._log(f"[ROS/ERR] 발사 준비 신호 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.fire_ready_topic,
                _cb,
                10
            )
            self._log(f"[시스템] 발사 준비 신호 구독 활성화: {self.fire_ready_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 발사 준비 신호 구독 비활성화: {e}")

    def _setup_tilt_status_subscriber(self):
        if self.node is None:
            return

        if self.tilt_status_callback is None:
            self._log("[시스템] IMU 기울기 상태 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _cb(msg):
                try:
                    self.tilt_status_callback(json.loads(msg.data))
                except Exception as e:
                    self._log(f"[ROS/ERR] IMU 기울기 상태 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.tilt_status_topic,
                _cb,
                10
            )
            self._log(f"[시스템] IMU 기울기 상태 구독 활성화: {self.tilt_status_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] IMU 기울기 상태 구독 비활성화: {e}")

    def _setup_fire_solution_status_subscriber(self):
        if self.node is None:
            return

        if self.fire_solution_status_callback is None:
            self._log("[시스템] 사격각 계산 상태 구독 비활성화: callback 없음")
            return

        try:
            if self.RosString is None:
                from std_msgs.msg import String
                self.RosString = String

            def _cb(msg):
                try:
                    # fire_solution/status는 JSON 또는 plain string 둘 다 허용
                    raw = msg.data.strip()
                    try:
                        self.fire_solution_status_callback(json.loads(raw))
                    except json.JSONDecodeError:
                        self.fire_solution_status_callback({"status": raw})
                except Exception as e:
                    self._log(f"[ROS/ERR] 사격각 계산 상태 처리 실패: {type(e).__name__}: {e}")

            self.node.create_subscription(
                self.RosString,
                self.fire_solution_status_topic,
                _cb,
                10
            )
            self._log(f"[시스템] 사격각 계산 상태 구독 활성화: {self.fire_solution_status_topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 사격각 계산 상태 구독 비활성화: {e}")

    def shutdown(self):
        self.running = False

        try:
            if self.thread is not None and self.thread.is_alive():
                self.thread.join(timeout=0.5)
        except Exception:
            pass

        try:
            if self.executor is not None and self.node is not None:
                self.executor.remove_node(self.node)
        except Exception:
            pass

        try:
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass

        try:
            if self.rclpy is not None and self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass