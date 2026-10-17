# h264_receiver.py
# PC side H.264 UDP receiver
#
# 역할:
# - Pi에서 오는 RTP/H.264 UDP stream 수신
# - GStreamer appsink로 decoded BGR frame 획득
# - decoded frame의 PTS를 읽음
# - /camera/frame_meta에서 온 meta와 pts_ns 기준 매칭
# - frame_buffer.put_bgr_frame(bgr, meta)로 UI에 전달
#
# 아직 ROS2 구독은 여기서 하지 않는다.
# RosBridge가 /camera/frame_meta를 구독한 뒤 receiver.add_meta(meta)를 호출하는 구조로 연결할 예정.

import time
import threading
from collections import deque

import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst


class H264Receiver:
    def __init__(
        self,
        frame_buffer,
        udp_port: int = 5600,
        width: int = 1456,
        height: int = 1088,
        max_match_dt_ms: float = 80.0,
        log_fn=None,
    ):
        self.frame_buffer = frame_buffer
        self.udp_port = int(udp_port)
        self.width = int(width)
        self.height = int(height)
        self.max_match_dt_ns = int(float(max_match_dt_ms) * 1e6)
        self.log_fn = log_fn or print

        self._meta_lock = threading.Lock()
        self._meta_history = deque(maxlen=300)  # 약 20초@15fps 정도
        # ------------------------------------------------------------
        # PTS offset 보정
        # ------------------------------------------------------------
        # Pi sender가 넣은 meta.pts_ns와
        # PC GStreamer decoder가 내보내는 decoded_pts_ns는
        # 같은 증가율을 가지지만 시작 기준이 다를 수 있다.
        #
        # 따라서:
        #   adjusted_pts_ns = decoded_pts_ns + self._pts_offset_ns
        # 로 보정한 뒤 frame_meta.pts_ns와 매칭한다.
        self._pts_offset_ns = None
        self._offset_samples = 0
        self._offset_alpha = 0.10

        self._running = False
        self._thread = None

        self._pipeline = None
        self._appsink = None

        self._rx_count = 0
        self._sync_ok_count = 0
        self._sync_fail_count = 0
        self._t0 = time.time()
        self._last_log_t = time.time()

        Gst.init(None)

    def _log(self, msg: str):
        try:
            self.log_fn(str(msg))
        except Exception:
            print(str(msg), flush=True)

    # ------------------------------------------------------------
    # RosBridge가 /camera/frame_meta를 받으면 이 함수 호출 예정
    # ------------------------------------------------------------
    def add_meta(self, meta: dict):
        """
        /camera/frame_meta JSON dict를 저장한다.

        필수 필드:
        - pts_ns
        - frame_t_cam_ns 또는 t_cam_ns

        권장 필드:
        - seq
        - width
        - height
        """
        if not isinstance(meta, dict):
            return

        try:
            pts_ns = int(meta.get("pts_ns"))
        except Exception:
            return

        frame_t_cam_ns = meta.get("frame_t_cam_ns", meta.get("t_cam_ns"))
        try:
            frame_t_cam_ns = int(frame_t_cam_ns)
        except Exception:
            return

        meta2 = dict(meta)
        meta2["pts_ns"] = int(pts_ns)
        meta2["frame_t_cam_ns"] = int(frame_t_cam_ns)
        meta2["t_cam_ns"] = int(frame_t_cam_ns)

        # PC에서 meta를 받은 시각. offset 초기 추정/디버깅용.
        meta2["_meta_rx_time_ns"] = time.time_ns()

        try:
            meta2["seq"] = int(meta2.get("seq", -1))
        except Exception:
            meta2["seq"] = -1

        with self._meta_lock:
            self._meta_history.append(meta2)

    def _find_nearest_meta(self, decoded_pts_ns: int):
        """
        decoded frame PTS와 가장 가까운 frame_meta를 찾는다.

        중요:
        - Pi sender의 meta.pts_ns와 PC decoder의 decoded_pts_ns는
          GStreamer RTP/decoder 과정에서 시작 기준이 달라질 수 있다.
        - 따라서 decoded_pts_ns를 그대로 쓰지 않고,
          decoded_pts_ns + pts_offset_ns 로 보정해서 매칭한다.

        return:
            matched_meta, dt_ns, adjusted_pts_ns

        매칭 실패 시:
            None, None 또는 best_dt, adjusted_pts_ns
        """
        if decoded_pts_ns is None:
            return None, None, None

        decoded_pts_ns = int(decoded_pts_ns)

        with self._meta_lock:
            if not self._meta_history:
                return None, None, None

            metas = list(self._meta_history)

        # ------------------------------------------------------------
        # 1. offset 초기화
        # ------------------------------------------------------------
        # 최초에는 가장 최근 meta를 기준으로 offset을 잡는다.
        # H.264 receiver와 frame_meta가 거의 같은 실시간 흐름으로 들어오므로
        # 초기 offset 추정값으로 사용 가능하다.
        if self._pts_offset_ns is None:
            latest_meta = metas[-1]
            try:
                latest_meta_pts = int(latest_meta.get("pts_ns"))
                self._pts_offset_ns = int(latest_meta_pts - decoded_pts_ns)
                self._offset_samples = 1
                self._log(
                    f"[H264_RX/SYNC] init pts_offset_ns={self._pts_offset_ns} "
                    f"({self._pts_offset_ns / 1e6:.1f} ms)"
                )
            except Exception:
                return None, None, None

        adjusted_pts_ns = int(decoded_pts_ns + self._pts_offset_ns)

        # ------------------------------------------------------------
        # 2. adjusted_pts_ns 기준 nearest meta 검색
        # ------------------------------------------------------------
        best = None
        best_dt = None

        for m in metas:
            try:
                pts = int(m.get("pts_ns"))
            except Exception:
                continue

            dt = abs(adjusted_pts_ns - pts)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = m

        if best is None or best_dt is None:
            return None, None, adjusted_pts_ns

        # ------------------------------------------------------------
        # 3. 허용 오차 밖이면 실패 처리
        # ------------------------------------------------------------
        if best_dt > self.max_match_dt_ns:
            return None, best_dt, adjusted_pts_ns

        # ------------------------------------------------------------
        # 4. offset 미세 보정
        # ------------------------------------------------------------
        # 매칭이 성공한 경우에만 offset을 조금씩 보정한다.
        # 갑자기 큰 점프가 들어오면 반영하지 않는다.
        try:
            observed_offset = int(best.get("pts_ns")) - decoded_pts_ns
            if self._pts_offset_ns is None:
                self._pts_offset_ns = observed_offset
                self._offset_samples = 1
            else:
                jump_ns = abs(observed_offset - int(self._pts_offset_ns))

                # 15fps 기준 프레임 간격 약 66ms.
                # 500ms 이상 튀면 잘못 매칭된 것으로 보고 offset 보정하지 않음.
                if jump_ns < 500_000_000:
                    alpha = float(self._offset_alpha)
                    self._pts_offset_ns = int(
                        (1.0 - alpha) * int(self._pts_offset_ns)
                        + alpha * int(observed_offset)
                    )
                    self._offset_samples += 1
        except Exception:
            pass

        return dict(best), best_dt, adjusted_pts_ns

    # ------------------------------------------------------------
    # GStreamer pipeline
    # ------------------------------------------------------------
    def _build_pipeline(self):
        pipeline_str = f"""
            udpsrc port={self.udp_port}
                caps=application/x-rtp,media=video,encoding-name=H264,payload=96
            ! rtpjitterbuffer latency=30 drop-on-latency=false
            ! rtph264depay request-keyframe=true wait-for-keyframe=true
            ! h264parse
            ! avdec_h264
            ! videoconvert
            ! video/x-raw,format=BGR
            ! queue leaky=downstream max-size-buffers=2
            ! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true
        """

        self._log("[H264_RX] GStreamer pipeline:")
        self._log(pipeline_str)

        pipeline = Gst.parse_launch(pipeline_str)
        appsink = pipeline.get_by_name("sink")

        if appsink is None:
            raise RuntimeError("appsink name=sink not found")

        return pipeline, appsink

    def start(self):
        if self._running:
            return

        self._pipeline, self._appsink = self._build_pipeline()

        self._running = True
        self._thread = threading.Thread(
            target=self._thread_main,
            daemon=True,
        )
        self._thread.start()

        self._log(f"[H264_RX] started: udp port={self.udp_port}")

    def stop(self):
        self._running = False

        try:
            if self._thread is not None and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if self._pipeline is not None:
                self._pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass

        self._pipeline = None
        self._appsink = None
        self._log("[H264_RX] stopped")

    def _check_bus_messages(self):
        if self._pipeline is None:
            return

        bus = self._pipeline.get_bus()
        while True:
            msg = bus.pop()
            if msg is None:
                break

            t = msg.type
            if t == Gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                self._log(f"[H264_RX/ERR] {err} debug={debug}")
            elif t == Gst.MessageType.EOS:
                self._log("[H264_RX] EOS")

    def _thread_main(self):
        try:
            self._pipeline.set_state(Gst.State.PLAYING)

            while self._running:
                self._check_bus_messages()

                sample = self._appsink.emit("try-pull-sample", 100_000_000)
                if sample is None:
                    continue

                self._handle_sample(sample)

        except Exception as e:
            self._log(f"[H264_RX/ERR] {type(e).__name__}: {e}")

        finally:
            try:
                if self._pipeline is not None:
                    self._pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass

    def _handle_sample(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()

        if buf is None or caps is None:
            return

        # decoded frame PTS
        decoded_pts_ns = None
        try:
            if buf.pts != Gst.CLOCK_TIME_NONE:
                decoded_pts_ns = int(buf.pts)
        except Exception:
            decoded_pts_ns = None

        # width / height 확인
        try:
            s = caps.get_structure(0)
            width = int(s.get_value("width"))
            height = int(s.get_value("height"))
        except Exception:
            width = self.width
            height = self.height

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return

        try:
            # videoconvert + video/x-raw,format=BGR 이므로 픽셀당 3바이트.
            # 단, GStreamer buffer는 row padding이 붙을 수 있어서
            # 실제 한 줄 크기(rowstride)가 width * 3과 다를 수 있다.
            #
            # 하단부만 계단식/층처럼 깨지는 현상은 보통 이 rowstride 무시 때문에 발생한다.
            expected_row_bytes = int(width) * 3
            total_size = int(mapinfo.size)

            if height <= 0 or width <= 0:
                return

            # 실제 rowstride 추정
            if total_size >= expected_row_bytes * int(height):
                rowstride = total_size // int(height)
            else:
                rowstride = expected_row_bytes

            if rowstride < expected_row_bytes:
                rowstride = expected_row_bytes

            raw = np.frombuffer(mapinfo.data, dtype=np.uint8)

            # buffer 전체를 (height, rowstride)로 보고,
            # 각 row에서 실제 영상 영역 width*3만 잘라낸다.
            rows = raw[: rowstride * int(height)].reshape((int(height), int(rowstride)))
            active = rows[:, :expected_row_bytes]

            bgr = active.reshape((int(height), int(width), 3)).copy()

        finally:
            buf.unmap(mapinfo)

        matched_meta, dt_ns, adjusted_pts_ns = self._find_nearest_meta(decoded_pts_ns)
        
        if matched_meta is not None:
            meta = matched_meta
            meta["sync_ok"] = True
            meta["decoded_pts_ns"] = int(decoded_pts_ns)
            meta["adjusted_pts_ns"] = int(adjusted_pts_ns) if adjusted_pts_ns is not None else -1
            meta["pts_offset_ns"] = int(self._pts_offset_ns) if self._pts_offset_ns is not None else None
            meta["pts_offset_ms"] = (
                float(self._pts_offset_ns / 1e6)
                if self._pts_offset_ns is not None else None
            )
            meta["pts_match_dt_ms"] = float(dt_ns / 1e6)
            meta["width"] = int(meta.get("width", width) or width)
            meta["height"] = int(meta.get("height", height) or height)
            self._sync_ok_count += 1
        else:
            # meta 매칭 실패 시에도 화면은 표시한다.
            # 단, 클릭 계산은 sync_ok False면 막아야 한다.
            meta = {
                "seq": -1,
                "width": int(width),
                "height": int(height),
                "decoded_pts_ns": int(decoded_pts_ns) if decoded_pts_ns is not None else -1,
                "adjusted_pts_ns": int(adjusted_pts_ns) if adjusted_pts_ns is not None else -1,
                "pts_offset_ns": int(self._pts_offset_ns) if self._pts_offset_ns is not None else None,
                "pts_offset_ms": (
                    float(self._pts_offset_ns / 1e6)
                    if self._pts_offset_ns is not None else None
                ),
                "pts_ns": int(adjusted_pts_ns) if adjusted_pts_ns is not None else -1,
                "frame_t_cam_ns": None,
                "t_cam_ns": None,
                "sync_ok": False,
                "pts_match_dt_ms": None if dt_ns is None else float(dt_ns / 1e6),
            }
            self._sync_fail_count += 1

        meta["_source"] = "h264_receiver"
        meta["_frame_format"] = "bgr"
        meta["rx_time_ns"] = time.time_ns()

        self.frame_buffer.put_bgr_frame(bgr, meta)

        self._rx_count += 1
        self._print_stats_periodically()

    def _print_stats_periodically(self):
        now = time.time()
        if now - self._last_log_t < 1.0:
            return

        elapsed = now - self._t0
        rx_fps = self._rx_count / elapsed if elapsed > 0 else 0.0

        total_sync = self._sync_ok_count + self._sync_fail_count
        sync_ratio = (self._sync_ok_count / total_sync * 100.0) if total_sync > 0 else 0.0

        offset_txt = (
            "None"
            if self._pts_offset_ns is None
            else f"{self._pts_offset_ns / 1e6:.1f}ms"
        )

        self._log(
            f"[H264_RX] rx_fps={rx_fps:.1f} "
            f"sync_ok={self._sync_ok_count} "
            f"sync_fail={self._sync_fail_count} "
            f"sync={sync_ratio:.1f}% "
            f"offset={offset_txt}"
        )

        self._rx_count = 0
        self._sync_ok_count = 0
        self._sync_fail_count = 0
        self._t0 = now
        self._last_log_t = now


# ------------------------------------------------------------
# 단독 수신 테스트용
# UI 통합 전, receiver가 import/실행되는지만 확인 가능
# 단, 이 테스트는 화면을 띄우지 않는다.
# 실제 영상 확인은 이미 gst-launch autovideosink로 완료했으므로
# 여기서는 에러 없이 rx 로그가 뜨는지만 본다.
# ------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    from camera_frame_buffer import CameraFrameBuffer

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5600)
    args = ap.parse_args()

    fb = CameraFrameBuffer()
    rx = H264Receiver(
        frame_buffer=fb,
        udp_port=args.port,
        log_fn=print,
    )

    rx.start()

    print("[H264_RX_TEST] running. Press Ctrl+C to stop.")

    try:
        while True:
            payload, meta = fb.take_latest()
            if payload is not None:
                print(
                    "[H264_RX_TEST] frame "
                    f"fmt={meta.get('_frame_format')} "
                    f"seq={meta.get('seq')} "
                    f"sync_ok={meta.get('sync_ok')} "
                    f"decoded_pts_ns={meta.get('decoded_pts_ns')} "
                    f"frame_t_cam_ns={meta.get('frame_t_cam_ns')}"
                )
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()