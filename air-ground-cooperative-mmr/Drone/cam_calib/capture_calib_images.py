#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
from picamera2 import Picamera2

# =========================
# 설정
# =========================
SIZE = (1456, 1088)          # 원본 고정 (캘리브 기준)
FPS = 30
WARMUP_SEC = 0.5

OUT_DIR = Path("calib_imgs")  # 상대경로: ~/drone_gcs/calib_imgs
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main():
    picam2 = Picamera2()
    cfg = picam2.create_video_configuration(
        main={"size": SIZE, "format": "BGR888"},   # ✅ OpenCV 저장용(BGR)
        controls={"FrameRate": FPS},
    )
    picam2.configure(cfg)
    picam2.start()
    time.sleep(WARMUP_SEC)

    print("[capture_calib_images] started")
    print(f"  SIZE={SIZE}, FPS={FPS}")
    print(f"  SAVE DIR: {OUT_DIR.resolve()}")
    print("  Keys: [s]=save  [q]=quit")

    idx = 0
    while True:
        # 최신 프레임 가져오기
        frame = picam2.capture_array("main")  # BGR numpy

        # 화면 표시(저장은 원본 그대로, 표시는 resize 가능)
        disp = cv2.resize(frame, (960, 720), interpolation=cv2.INTER_AREA)
        cv2.putText(disp, f"saved: {idx}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow("calib capture (press s to save, q to quit)", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            # ✅ 원본 해상도 그대로 저장 (BGR->JPG, 뒤집기/색보정 금지)
            out_path = OUT_DIR / f"img_{idx:04d}.jpg"
            ok = cv2.imwrite(str(out_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            if ok:
                print(f"[SAVE] {out_path}")
                idx += 1
            else:
                print("[WARN] save failed")

    picam2.stop()
    cv2.destroyAllWindows()
    print("[capture_calib_images] stopped")


if __name__ == "__main__":
    main()