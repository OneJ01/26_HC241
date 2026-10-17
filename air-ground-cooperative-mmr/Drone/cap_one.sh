#!/bin/bash
set -e
mkdir -p ~/cam_calib/img

# 프리뷰/카메라 점유 싹 정리
sudo pkill -f rpicam-vid 2>/dev/null || true
sudo pkill -f rpicam 2>/dev/null || true
sudo pkill -f libcamera 2>/dev/null || true
sleep 0.2

# 원본 캡쳐
rpicam-still -n --width 1456 --height 1088 -o ~/cam_calib/img/calib_$(date +%Y%m%d_%H%M%S).jpg
