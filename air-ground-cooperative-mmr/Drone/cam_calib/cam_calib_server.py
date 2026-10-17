#!/usr/bin/env python3
import os, time
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify, request
from picamera2 import Picamera2

# =========================
# 캘브용 해상도 (우리 기존값)
# =========================
WIDTH  = 1456
HEIGHT = 1088
FPS = 30

# 저장 폴더 (라파)
SAVE_DIR = Path.home() / "calib_captures"
SAVE_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__)

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

_latest_bgr = None

def gen_mjpeg():
    global _latest_bgr
    frame_interval = 1.0 / max(1, FPS)

    while True:
        t0 = time.time()

        rgb = picam2.capture_array()              # (H,W,3) RGB
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        _latest_bgr = bgr

        ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")

        dt = time.time() - t0
        if dt < frame_interval:
            time.sleep(frame_interval - dt)

@app.get("/stream.mjpg")
def stream():
    return Response(gen_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.get("/capture")
def capture():
    """
    PC에서: http://<pi_ip>:8000/capture 호출하면
    라파에 저장하고, 저장된 파일명을 JSON으로 반환
    """
    global _latest_bgr
    if _latest_bgr is None:
        return jsonify({"ok": False, "err": "no_frame_yet"}), 503

    ts = time.strftime("%Y%m%d_%H%M%S")
    fname = f"calib_{WIDTH}x{HEIGHT}_{ts}.jpg"
    fpath = SAVE_DIR / fname
    cv2.imwrite(str(fpath), _latest_bgr)

    return jsonify({"ok": True, "file": fname, "path": str(fpath)})

@app.get("/download/<name>")
def download(name):
    fpath = SAVE_DIR / name
    if not fpath.exists():
        return jsonify({"ok": False, "err": "not_found"}), 404

    with open(fpath, "rb") as f:
        data = f.read()
    return Response(data, mimetype="image/jpeg")

if __name__ == "__main__":
    # 0.0.0.0: 외부(PC) 접속 허용
    app.run(host="0.0.0.0", port=8000, threaded=True)