#!/usr/bin/env python3
import time
import cv2
from picamera2 import Picamera2

SIZE = (1456, 1088)

picam2 = Picamera2()
cfg = picam2.create_video_configuration(
    main={"size": SIZE, "format": "RGB888"}
)
picam2.configure(cfg)
picam2.start()
time.sleep(1.0)

req = picam2.capture_request()
frame = req.make_array("main")
req.release()
picam2.stop()

# 1번: frame 그대로 저장
cv2.imwrite("/home/pi/color_raw.jpg", frame)

# 2번: RGB->BGR 변환 후 저장
bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
cv2.imwrite("/home/pi/color_rgb2bgr.jpg", bgr)

print("saved:")
print("/home/pi/color_raw.jpg")
print("/home/pi/color_rgb2bgr.jpg")
