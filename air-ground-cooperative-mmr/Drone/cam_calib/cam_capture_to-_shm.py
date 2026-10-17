#!/usr/bin/env python3
# real file
import os
import time
import json
import signal
from pathlib import Path

import cv2
from picamera2 import Picamera2

# =========================
# 설정값
# =========================
FPS = 30
SIZE = (1456, 1088)          # IMX296 기본 모드에 맞춤
JPEG_QUALITY = 70            # 0~100 (높을수록 용량↑ CPU↑)
WARMUP_SEC = 0.4             # AE 안정화
WRITE_DIR = Path("/dev/shm") # RAM 디스크 (빠름)
FRAME_PATH = WRITE_DIR / "cam_frame.jpg"
META_PATH  = WRITE_DIR / "cam_meta.json"

# 원자적 갱신을 위한 임시 파일
FRAME_TMP = WRITE_DIR / "cam_frame.jpg.tmp"
META_TMP  = WRITE_DIR / "cam_meta.json.tmp"

running = True


def handle_sig(sig, frame):
    global running
    running = False


def atomic_write_bytes(tmp_path: Path, final_path: Path, data: bytes):
    tmp_path.write_bytes(data)
    os.replace(tmp_path, final_path)  # 원자적 rename


def atomic_write_text(tmp_path: Path, final_path: Path, text: str):
    tmp_path.write_text(text)
    os.replace(tmp_path, final_path)


def main():
    global running

    # SIGINT/SIGTERM 핸들링 (Ctrl+C, kill)
    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    WRITE_DIR.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": SIZE, "format": "BGR888"},
        controls={"FrameRate": FPS},
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(WARMUP_SEC)

    print("[cam_capture_to_shm] started")
    print(f"  FPS={FPS}, SIZE={SIZE}, JPEG_QUALITY={JPEG_QUALITY}")
    print(f"  frame -> {FRAME_PATH}")
    print(f"  meta  -> {META_PATH}")

    seq = 0
    last_print = time.time()

    # OpenCV JPEG 인코딩 옵션
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)]

    while running:
        try:
            req = picam2.capture_request()
            md = req.get_metadata()

            # 프레임 가져오기 (BGR numpy array)
            bgr = req.make_array("main")
            req.release()

            # 그대로 JPEG 인코딩 (OpenCV는 BGR 기준)
            ok, jpg = cv2.imencode(".jpg", bgr, encode_param)
            if not ok:
                # 인코딩 실패 시 스킵
                continue

            # 핵심 메타 추출
            sensor_ts = md.get("SensorTimestamp", None)  # ns
            frame_wc  = md.get("FrameWallClock", None)   # ns
            exposure  = md.get("ExposureTime", None)     # us
            again     = md.get("AnalogueGain", None)
            dgain     = md.get("DigitalGain", None)
            frame_dur = md.get("FrameDuration", None)

            out_md = {
                "seq": seq,
		"width": SIZE[0],
		"height": SIZE[1],
		"ExposureTime": int(md.get("ExposureTime", 0)),
                "t_cam_ns": frame_wc,          # 최우선 t_cam = FrameWallClock (ns)
                "FrameWallClock": frame_wc,
                "SensorTimestamp": sensor_ts,
                "ExposureTime_us": exposure,
                "AnalogueGain": again,
                "DigitalGain": dgain,
                "FrameDuration": frame_dur,
                "host_time_ns": time.time_ns(),  # 참고용(벽시계)
            }

            # 원자적 파일 갱신 (읽는 쪽에서 중간 깨짐 방지)
            atomic_write_bytes(FRAME_TMP, FRAME_PATH, jpg.tobytes())
            atomic_write_text(META_TMP, META_PATH, json.dumps(out_md))

            seq += 1

            # 1초에 한 번 상태 출력
            now = time.time()
            if now - last_print > 1.0:
                last_print = now
                print(f"[cam_capture_to_shm] seq={seq} fwc={frame_wc} sts={sensor_ts} exp_us={exposure}")

        except Exception as e:
            # 예외가 나도 루프를 살려서 계속 진행
            print(f"[cam_capture_to_shm] ERROR: {e}")
            time.sleep(0.1)

    picam2.stop()
    print("[cam_capture_to_shm] stopped")


if __name__ == "__main__":
    main()
