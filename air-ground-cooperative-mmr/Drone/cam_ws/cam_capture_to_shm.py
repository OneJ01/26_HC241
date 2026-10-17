#!/usr/bin/env python3
# real file (MODIFIED: seq-per-file + latest_seq pointer)

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
JPEG_QUALITY = 60            # 0~100 (높을수록 용량↑ CPU↑)
WARMUP_SEC = 0.4             # AE 안정화

# ✅ /dev/shm 아래에 프레임/메타를 "seq별 파일"로 저장
WRITE_DIR  = Path("/dev/shm")  # RAM 디스크 (빠름)
FRAMES_DIR = WRITE_DIR / "frames"
META_DIR   = WRITE_DIR / "meta"

# ✅ 최신 seq 포인터 (consumer는 이것만 믿고 seq를 얻는다)
LATEST_SEQ_PATH = WRITE_DIR / "latest_seq.txt"
LATEST_SEQ_TMP  = WRITE_DIR / "latest_seq.txt.tmp"

# ✅ /dev/shm 용량 보호: 최근 N개만 유지
KEEP_LAST_N = 120  # 예: 30fps 기준 약 4초 분량 (환경에 맞게 조절)

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


def prune_old(seq_now: int):
    """
    ✅ 오래된 파일 삭제 (rolling)
    - seq_now - KEEP_LAST_N 이전 파일 삭제
    """
    if KEEP_LAST_N <= 0:
        return
    old_seq = seq_now - KEEP_LAST_N
    if old_seq < 0:
        return

    jpg_path = FRAMES_DIR / f"{old_seq:06d}.jpg"
    json_path = META_DIR / f"{old_seq:06d}.json"

    try:
        if jpg_path.exists():
            jpg_path.unlink()
    except Exception:
        pass
    try:
        if json_path.exists():
            json_path.unlink()
    except Exception:
        pass


def main():
    global running

    # SIGINT/SIGTERM 핸들링 (Ctrl+C, kill)
    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    # ✅ 폴더 생성
    WRITE_DIR.mkdir(parents=True, exist_ok=True)
    FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    META_DIR.mkdir(parents=True, exist_ok=True)

    picam2 = Picamera2()

    cfg = picam2.create_video_configuration(
        main={"size": SIZE, "format": "RGB888"},
        controls={"FrameRate": FPS},
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(WARMUP_SEC)

    print("[cam_capture_to_shm] started (seq-per-file)")
    print(f"  FPS={FPS}, SIZE={SIZE}, JPEG_QUALITY={JPEG_QUALITY}")
    print(f"  frames dir -> {FRAMES_DIR}")
    print(f"  meta dir   -> {META_DIR}")
    print(f"  latest seq -> {LATEST_SEQ_PATH}")
    print(f"  KEEP_LAST_N={KEEP_LAST_N}")

    seq = 0
    last_print = time.time()

    # OpenCV JPEG 인코딩 옵션
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(JPEG_QUALITY)]

    while running:
        try:
            req = picam2.capture_request()
            md = req.get_metadata()

            # 프레임 가져오기
            frame = req.make_array("main")
            req.release()

            # color_raw.jpg가 정상으로 확인됨 → frame 자체가 OpenCV 기준 BGR 순서
            bgr = frame

            # OpenCV는 BGR 기준으로 JPEG 인코딩
            ok, jpg = cv2.imencode(".jpg", bgr, encode_param)
            if not ok:
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
                "t_cam_ns": frame_wc,          # 최우선 t_cam = FrameWallClock (ns)
                "FrameWallClock": frame_wc,
                "SensorTimestamp": sensor_ts,
                "ExposureTime_us": int(exposure) if exposure is not None else 0,
                "AnalogueGain": again,
                "DigitalGain": dgain,
                "FrameDuration": frame_dur,
                "host_time_ns": time.time_ns(),  # 참고용(벽시계)
            }

            # ✅ seq별 파일명 (tmp → final 원자 교체)
            frame_final = FRAMES_DIR / f"{seq:06d}.jpg"
            meta_final  = META_DIR   / f"{seq:06d}.json"
            frame_tmp   = FRAMES_DIR / f"{seq:06d}.jpg.tmp"
            meta_tmp    = META_DIR   / f"{seq:06d}.json.tmp"

            # ✅ (1) 프레임 저장
            atomic_write_bytes(frame_tmp, frame_final, jpg.tobytes())

            # ✅ (2) 메타 저장
            atomic_write_text(meta_tmp, meta_final, json.dumps(out_md, separators=(",", ":")))

            # ✅ (3) 마지막에 latest_seq 포인터 갱신 (이게 "커밋" 역할)
            atomic_write_text(LATEST_SEQ_TMP, LATEST_SEQ_PATH, str(seq))

            # ✅ rolling 삭제 (다음 seq로 넘어가기 전에 처리)
            prune_old(seq)

            seq += 1

            # 1초에 한 번 상태 출력
            now = time.time()
            if now - last_print > 1.0:
                last_print = now

                has_fwc = "FrameWallClock" in md
                has_sts = "SensorTimestamp" in md
                has_exp = "ExposureTime" in md

                print(
                    f"[cam_capture_to_shm] seq={seq} "
                    f"has(FWC={has_fwc}, STS={has_sts}, EXP={has_exp}) "
                    f"FWC={frame_wc} STS={sensor_ts} EXP(us)={exposure}"
                )

        except Exception as e:
            print(f"[cam_capture_to_shm] ERROR: {e}")
            time.sleep(0.1)

    picam2.stop()
    print("[cam_capture_to_shm] stopped")


if __name__ == "__main__":
    main()