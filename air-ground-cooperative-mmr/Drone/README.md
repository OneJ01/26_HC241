# Drone Raspberry Pi Source Code

2026 한이음 드림업 프로젝트
정찰 드론 Raspberry Pi 소스코드 및 실행환경 자료

## 1. 주요 구성

| 경로 | 설명 |
|---|---|
| cam_ws/scripts/mission_manager.py | FC 통신, 텔레메트리 및 자동비행 미션 관리 |
| cam_ws/src/cam_msgs/ | ROS2 커스텀 메시지 정의 |
| cam_ws/*.py | 카메라, FC 및 ROS2 관련 보조 코드 |
| h264_cam_ws/scripts/ | H.264 영상 송신 및 메타데이터 발행 |
| ros_mavsdk_docker/Dockerfile | ROS2 Humble 및 MAVSDK 실행환경 |
| cam_calib/ | 카메라 캘리브레이션 관련 코드 |
| rtk_inject/ | RTK 보정 데이터 관련 코드 |
| tools/ | 카메라 메타데이터 기록 테스트 코드 |
| start_h264_pi.sh | 기존 tmux 기반 실행 스크립트 |
| cap_one.sh | 캘리브레이션 이미지 촬영 스크립트 |

## 2. 실행환경

- Raspberry Pi OS
- ROS2 Humble
- MAVSDK Python
- Picamera2 / libcamera
- GStreamer
- Docker
- ROS_DOMAIN_ID=42

## 3. 참고사항

- FC 기본 연결 주소: serial:///dev/ttyACM0:115200
- H.264 영상 송신 기본 포트: UDP 5600
- start_h264_pi.sh는 기존 실행 스크립트이며 최종 Mission Manager 실행 방식과 차이가 있음.
- 원본 비행 미션 파일에는 실제 시험 좌표가 포함되어 있어 공개용 자료에서 제외함.
- Python 가상환경, ROS2 빌드 결과물, 로그 및 시험 기록은 제외함.
- 실제 구동 시 장비 환경에 맞게 IP, 시리얼 포트 및 Docker 설정을 확인해야 함.
