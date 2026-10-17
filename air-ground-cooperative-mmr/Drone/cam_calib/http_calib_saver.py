#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import time
import json
import shutil

FRAME = Path("/dev/shm/cam_frame.jpg")
META  = Path("/dev/shm/cam_meta.json")

OUT = Path.home() / "cam_calib" / "calib_imgs"
OUT.mkdir(parents=True, exist_ok=True)

class H(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path != "/save":
            self.send_response(404); self.end_headers()
            self.wfile.write(b"not found")
            return

        if not FRAME.exists():
            self.send_response(503); self.end_headers()
            self.wfile.write(b"no /dev/shm/cam_frame.jpg")
            return

        ts = time.time_ns()
        jpg_out  = OUT / f"img_{ts}.jpg"
        meta_out = OUT / f"img_{ts}.json"

        shutil.copyfile(FRAME, jpg_out)
        if META.exists():
            shutil.copyfile(META, meta_out)
        else:
            meta_out.write_text(json.dumps({"ts": ts}, indent=2))

        self.send_response(200); self.end_headers()
        self.wfile.write(str(jpg_out).encode())

def main():
    srv = HTTPServer(("0.0.0.0", 8080), H)
    print(f"[http_calib_saver] listening :8080  save dir={OUT}")
    srv.serve_forever()

if __name__ == "__main__":
    main()