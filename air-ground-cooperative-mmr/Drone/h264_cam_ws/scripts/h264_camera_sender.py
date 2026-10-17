#!/usr/bin/env python3
# h264_camera_sender.py
# Pi Camera → H.264 UDP stream + timestamp meta JSON
#
# 목적:
# - H.264 영상은 PC로 UDP 송출
# - 각 프레임의 촬영 시각(frame_t_cam_ns)과 H.264 PTS(pts_ns)를 /dev/shm/h264_meta/latest_meta.json에 저장
# - 이후 Docker ROS2 publisher가 이 meta를 읽어 /camera/frame_meta로 발행

import os
import json
import time
import signal
import argparse
from pathlib import Path

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

from picamera2 import Picamera2


# =========================
# 기본 설정
# =========================
DEFAULT_SIZE = (1456, 1088)   # 기존 캘리브레이션 해상도 유지
DEFAULT_FPS = 15              # 안정성 우선
DEFAULT_BITRATE_KBPS = 5000   # 5 Mbps 시작값

META_DIR = Path("/dev/shm/h264_meta")
LATEST_META = META_DIR / "latest_meta.json"
LATEST_META_TMP = META_DIR / "latest_meta.json.tmp"

running = True


def handle_sig(sig, frame):
    global running
    running = False


def atomic_write_text(tmp_path: Path, final_path: Path, text: str):
    tmp_path.write_text(text)
    os.replace(tmp_path, final_path)


def build_pipeline(host: str, port: int, width: int, height: int, fps: int, bitrate_kbps: int):
    """
    appsrc로 raw frame을 넣고 x264enc로 H.264 인코딩 후 RTP/UDP 송출.
    - tune=zerolatency: 실시간 지연 최소화
    - bframes=0: 표시 순서/디코딩 순서 꼬임 방지
    - key-int-max=fps: 약 1초마다 I-frame
    """
    pipeline_str = f"""
        appsrc name=src is-live=true format=time do-timestamp=false
            caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1
        ! queue leaky=downstream max-size-buffers=2
        ! videoconvert
        ! video/x-raw,format=I420
        ! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps}
                  key-int-max=5 bframes=0 byte-stream=true
        ! h264parse config-interval=1
        ! rtph264pay pt=96 config-interval=1
        ! udpsink host={host} port={port} sync=false async=false
    """
    return pipeline_str


def main():
    global running

    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="PC IP address, e.g. 192.168.0.100")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--fps", type=int, default=DEFAULT_FPS)
    ap.add_argument("--width", type=int, default=DEFAULT_SIZE[0])
    ap.add_argument("--height", type=int, default=DEFAULT_SIZE[1])
    ap.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE_KBPS, help="kbps")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    width = int(args.width)
    height = int(args.height)
    fps = int(args.fps)
    frame_duration_ns = int(1_000_000_000 / max(fps, 1))

    META_DIR.mkdir(parents=True, exist_ok=True)

    Gst.init(None)

    pipeline_str = build_pipeline(
        host=args.host,
        port=args.port,
        width=width,
        height=height,
        fps=fps,
        bitrate_kbps=int(args.bitrate),
    )

    print("[H264_SENDER] GStreamer pipeline:")
    print(pipeline_str)

    pipeline = Gst.parse_launch(pipeline_str)
    appsrc = pipeline.get_by_name("src")
    if appsrc is None:
        raise RuntimeError("appsrc name=src not found")

    # Picamera2 설정
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": fps},
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(0.5)

    pipeline.set_state(Gst.State.PLAYING)

    print("[H264_SENDER] started")
    print(f"  stream -> udp://{args.host}:{args.port}")
    print(f"  size={width}x{height}, fps={fps}, bitrate={args.bitrate}kbps")
    print(f"  meta -> {LATEST_META}")

    seq = 0
    stream_t0_ns = None
    last_print = time.time()
    sent_count = 0
    fps_t0 = time.time()

    try:
        while running:
            req = picam2.capture_request()
            md = req.get_metadata()

            frame = req.make_array("main")
            req.release()

            # 기존 코드에서 RGB888 프레임을 OpenCV BGR처럼 쓰고 있었으므로
            # GStreamer caps도 BGR로 둔다.
            if not frame.flags["C_CONTIGUOUS"]:
                frame = frame.copy()

            frame_wc = md.get("FrameWallClock", None)
            sensor_ts = md.get("SensorTimestamp", None)
            exposure = md.get("ExposureTime", None)
            again = md.get("AnalogueGain", None)
            dgain = md.get("DigitalGain", None)
            frame_dur = md.get("FrameDuration", None)
            # Picamera2/libcamera FrameDuration은 보통 us 단위로 들어오는 경우가 많음
            # 15fps 기준 66666이면 66,666us = 66,666,000ns
            if frame_dur is not None:
                frame_duration_meta_ns = int(frame_dur)
                if frame_duration_meta_ns < 1_000_000:
                    frame_duration_meta_ns *= 1000
            else:
                frame_duration_meta_ns = int(frame_duration_ns)

            # frame_t_cam_ns: 계산 기준 시간
            if frame_wc is not None:
                frame_t_cam_ns = int(frame_wc)
            else:
                frame_t_cam_ns = time.time_ns()

            if stream_t0_ns is None:
                stream_t0_ns = frame_t_cam_ns

            # H.264 PTS: stream 시작 기준 상대 시간
            pts_ns = int(frame_t_cam_ns - stream_t0_ns)
            if pts_ns < 0:
                pts_ns = seq * frame_duration_ns

            data = frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)

            # 핵심: H.264 프레임 PTS와 meta.pts_ns를 동일하게 맞춤
            buf.pts = pts_ns
            buf.dts = pts_ns
            buf.duration = frame_duration_ns

            ret = appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                print(f"[H264_SENDER] push-buffer failed: {ret}")
                time.sleep(0.01)
                continue

            meta = {
                "seq": int(seq),
                "stream_t0_ns": int(stream_t0_ns),
                "frame_t_cam_ns": int(frame_t_cam_ns),
                "t_cam_ns": int(frame_t_cam_ns),
                "pts_ns": int(pts_ns),
                "width": int(width),
                "height": int(height),
                "fps": int(fps),
                "sensor_timestamp_ns": int(sensor_ts) if sensor_ts is not None else 0,
                "FrameWallClock": int(frame_wc) if frame_wc is not None else int(frame_t_cam_ns),
                "SensorTimestamp": int(sensor_ts) if sensor_ts is not None else 0,
                "ExposureTime_us": int(exposure) if exposure is not None else 0,
                "AnalogueGain": float(again) if again is not None else 0.0,
                "DigitalGain": float(dgain) if dgain is not None else 0.0,
                "FrameDuration": int(frame_dur) if frame_dur is not None else int(frame_duration_ns),
                "frame_duration_ns": int(frame_duration_meta_ns),
                "host_time_ns": int(time.time_ns()),
            }

            atomic_write_text(
                LATEST_META_TMP,
                LATEST_META,
                json.dumps(meta, separators=(",", ":")),
            )

            seq += 1
            sent_count += 1

            now = time.time()
            if now - last_print >= 1.0:
                elapsed = now - fps_t0
                out_fps = sent_count / elapsed if elapsed > 0 else 0.0
                print(
                    f"[H264_SENDER] seq={seq} out_fps={out_fps:.1f} "
                    f"pts_ns={pts_ns} frame_t_cam_ns={frame_t_cam_ns}"
                )
                last_print = now
                sent_count = 0
                fps_t0 = now

    except Exception as e:
        print(f"[H264_SENDER] ERROR: {type(e).__name__}: {e}")

    finally:
        try:
            appsrc.emit("end-of-stream")
        except Exception:
            pass
        try:
            pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        try:
            picam2.stop()
        except Exception:
            pass
        print("[H264_SENDER] stopped")


if __name__ == "__main__":
    main()