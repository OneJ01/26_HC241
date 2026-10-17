import time, csv
from picamera2 import Picamera2

N = 300
FPS = 30

picam2 = Picamera2()
cfg = picam2.create_video_configuration(
    main={"size": (1456, 1088), "format": "RGB888"},
    controls={"FrameRate": FPS}
)
picam2.configure(cfg)
picam2.start()

time.sleep(0.5)

out = "/tmp/meta_log.csv"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["i", "Sequence", "SensorTimestamp(ns)", "FrameWallClock(ns)", "ExposureTime(us)", "AnalogueGain"])
    for i in range(N):
        req = picam2.capture_request()
        md = req.get_metadata()

        seq = md.get("Sequence", -1)
        st  = md.get("SensorTimestamp", -1)
        fwc = md.get("FrameWallClock", -1)  # 없으면 -1로 남음
        exp = md.get("ExposureTime", -1)
        gain = md.get("AnalogueGain", -1)

        w.writerow([i, seq, st, fwc, exp, gain])
        req.release()

picam2.stop()
print("saved:", out)
