# camera_frame_buffer.py
# 최신 카메라 프레임 1개만 저장하는 thread-safe buffer
#
# 기존 JPEG FramePacket 구조와 H.264 decoded frame 구조를 동시에 지원한다.
#
# 지원 형식:
# 1) JPEG bytes
#    - RosBridge의 기존 /camera/frame_packet 콜백에서 사용
#    - put(jpg_bytes, meta)
#
# 2) BGR frame
#    - H.264 receiver가 GStreamer로 디코딩한 OpenCV BGR ndarray 저장
#    - put_bgr_frame(bgr_frame, meta)
#
# UI는 take_latest()로 payload, meta를 가져가고,
# meta["_frame_format"] 값을 보고 jpeg/bgr 처리 방식을 결정한다.

import threading
import time


class CameraFrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()

        # 최신 프레임 payload
        # - JPEG 모드: bytes
        # - BGR 모드 : numpy ndarray
        self._latest_payload = None
        self._latest_meta = None

        # "jpeg" 또는 "bgr"
        self._latest_format = None

        # 수신 FPS 표시용
        self.rx_count = 0
        self.rx_t0 = time.time()
        self.rx_fps = 0.0
        self.rx_last_ns = 0

    # ------------------------------------------------------------
    # 기존 JPEG 경로 유지
    # ------------------------------------------------------------
    def put(self, jpg_bytes: bytes, meta: dict):
        """
        기존 ROS2 FramePacket callback thread에서 호출.
        JPEG bytes를 저장한다.

        기존 코드 호환용 메서드이므로 이름을 유지한다.
        """
        if not jpg_bytes:
            return

        meta2 = dict(meta or {})
        meta2["_frame_format"] = "jpeg"

        with self._lock:
            self._latest_payload = jpg_bytes
            self._latest_meta = meta2
            self._latest_format = "jpeg"

        self._update_rx_stats()

    # ------------------------------------------------------------
    # 신규 H.264 decoded frame 경로
    # ------------------------------------------------------------
    def put_bgr_frame(self, bgr_frame, meta: dict):
        """
        H.264 receiver thread에서 호출.
        GStreamer appsink에서 디코딩된 OpenCV BGR frame을 저장한다.

        bgr_frame:
            numpy ndarray, shape = (height, width, 3), dtype=uint8
        meta:
            frame_t_cam_ns, pts_ns, seq, width, height 등을 포함한 dict
        """
        if bgr_frame is None:
            return

        meta2 = dict(meta or {})
        meta2["_frame_format"] = "bgr"

        # receiver 쪽 버퍼 재사용 가능성 방지를 위해 copy 저장
        try:
            payload = bgr_frame.copy()
        except Exception:
            payload = bgr_frame

        with self._lock:
            self._latest_payload = payload
            self._latest_meta = meta2
            self._latest_format = "bgr"

        self._update_rx_stats()

    def take_latest(self):
        """
        UI thread에서 호출.
        최신 프레임 1개를 가져오고 버퍼를 비운다.

        return:
            payload, meta

        payload:
            - meta["_frame_format"] == "jpeg"이면 JPEG bytes
            - meta["_frame_format"] == "bgr"이면 OpenCV BGR ndarray
        """
        with self._lock:
            if self._latest_payload is None:
                return None, None

            payload = self._latest_payload
            meta = self._latest_meta or {}
            fmt = self._latest_format

            self._latest_payload = None
            self._latest_meta = None
            self._latest_format = None

        if isinstance(meta, dict) and fmt is not None:
            meta.setdefault("_frame_format", fmt)

        return payload, meta

    def _update_rx_stats(self):
        self.rx_count += 1
        self.rx_last_ns = time.time_ns()

        now = time.time()
        if now - self.rx_t0 >= 1.0:
            self.rx_fps = self.rx_count / (now - self.rx_t0)
            self.rx_count = 0
            self.rx_t0 = now

    def get_rx_fps(self) -> float:
        return float(self.rx_fps)

    def get_rx_last_ns(self) -> int:
        return int(self.rx_last_ns)