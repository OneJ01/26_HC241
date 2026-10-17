#!/usr/bin/env bash
set -e

PC_IP=${1:-192.168.0.100}
BITRATE=${2:-8000}
FPS=${3:-15}
FC_DEV=${4:-/dev/ttyACM0}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

SESSION="h264_pi"

echo "[START] PC_IP=${PC_IP}"
echo "[START] BITRATE=${BITRATE} kbps"
echo "[START] FPS=${FPS}"
echo "[START] FC_DEV=${FC_DEV}"
echo "[START] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# 기존 tmux 세션이 있으면 종료
tmux kill-session -t ${SESSION} 2>/dev/null || true

# 1번 창: H.264 카메라 sender
# Picamera2/libcamera는 Pi OS에서 직접 실행
tmux new-session -d -s ${SESSION} -n sender \
"cd ~/h264_cam_ws/scripts && \
python3 h264_camera_sender.py --host ${PC_IP} --port 5600 --fps ${FPS} --bitrate ${BITRATE}; \
exec bash"

# 2번 창: H.264 metadata ROS2 publisher
# /dev/shm/h264_meta/latest_meta.json 읽어서 /camera/frame_meta 발행
tmux new-window -t ${SESSION} -n meta \
"docker run -it --rm --net=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -v /dev/shm:/dev/shm \
  -v /home/pi/h264_cam_ws:/root/h264_cam_ws \
  ros:humble-ros-base \
  bash -lc 'cd /root/h264_cam_ws/scripts && python3 h264_meta_pub.py --rate 30'; \
exec bash"

# 3번 창: FC telemetry publisher
# FC가 /dev/ttyACM0으로 잡힌다는 전제
tmux new-window -t ${SESSION} -n telemetry \
"docker run -it --rm --net=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -v /dev/shm:/dev/shm \
  -v /home/pi/cam_ws:/root/cam_ws \
  --device=${FC_DEV}:${FC_DEV} \
  ros-humble-mavsdk \
  bash -lc 'cd /root/cam_ws && python3 heartbeat_check.py'; \
exec bash"

echo "[OK] tmux session '${SESSION}' started."
echo "[INFO] Move window: Ctrl+B then N"
echo "[INFO] Detach     : Ctrl+B then D"
echo "[INFO] Attach     : tmux attach -t ${SESSION}"
echo "[INFO] Stop       : tmux kill-session -t ${SESSION}"

tmux attach -t ${SESSION}
