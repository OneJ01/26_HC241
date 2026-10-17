# GCS (Ground Control Station) — PC Workspace

드론에서 송신한 H.264 영상과 프레임 메타데이터를 수신해 표시하고, 운용자가 영상에서 선택한 지점을
드론 위치·자세·카메라 캘리브레이션·DEM을 이용해 지상 좌표로 역투영한 뒤 MMR로 전달하는 ROS 2 워크스페이스입니다.

## 1. 구성

```text
GCS/
├── missions/                      # Mission Planner(QGC WPL 110) 형식 미션 파일
└── src/
    ├── cam_msgs/                  # 커스텀 메시지 FramePacket.msg (Drone/cam_ws/src/cam_msgs 와 동일)
    └── gcs_bringup/
        ├── launch/
        │   ├── h264_gcs.launch.py     # 최종 실행 launch (H.264 수신 → 좌표 계산 → UI)
        │   ├── gcs_refactor.launch.py # 이전 버전 launch
        │   └── gcs.launch.py          # 초기 버전 launch
        └── scripts/
            ├── gcs_ui.py                     # PyQt6 운용 GUI (카메라/지도/HUD, MMR 상태 표시)
            ├── ros_bridge.py                 # GUI ↔ ROS 2 토픽 연결
            ├── h264_receiver.py              # GStreamer 기반 RTP/H.264 UDP 수신
            ├── h264_to_bgr_bridge.py         # H.264 디코딩 프레임 + 메타 매칭 → 표시용 토픽 발행
            ├── h264_to_framepacket_bridge.py # H.264 → FramePacket(JPEG) 변환 브리지
            ├── camera_frame_buffer.py        # 최신 프레임 1장 보관용 thread-safe 버퍼
            ├── compute_logger.py             # 픽셀 역투영·DEM 교차로 표적/탄착 좌표 계산
            ├── mission_manager.py            # FC 연결·텔레메트리·미션 업로드 관리
            ├── mission_manager_dummy.py      # FC 없이 미션 경로 표시를 시험하는 더미 노드
            ├── calib_result.json / .npz      # 카메라 캘리브레이션 결과
            └── assets/                       # GUI 로고 이미지
```

## 2. 주요 토픽

| 토픽 | 방향 | 설명 |
|---|---|---|
| `/camera/frame_meta` | 수신 | 드론 카메라 프레임 메타데이터 |
| `/drone/telemetry` | 수신 | 드론 위치·자세 텔레메트리 |
| `/compute/target_point` | 발행 | 영상 클릭 지점의 역투영 좌표 |
| `/target/point` | 발행 | MMR로 전달하는 표적 좌표 |
| `/compute/impact_point` | 발행 | 탄착 지점 계산 결과 |
| `/mmr/status`, `/mmr/engagement_status`, `/mmr/fire_ready` | 수신 | MMR 상태 표시 |
| `/mission/upload_request`, `/mission/status`, `/mission/path` | 송·수신 | 자동비행 미션 관리 |

세부 메시지 구조는 각 스크립트 소스코드를 기준으로 합니다.

## 3. 개발 환경

- Ubuntu 22.04, ROS 2 Humble
- Python 3: PyQt6, OpenCV(`cv2`), NumPy, pyproj, rasterio, MAVSDK
- GStreamer + PyGObject(`gi`) — H.264 수신용
- `ROS_DOMAIN_ID=42` (드론·MMR과 동일하게 설정)

## 4. 빌드 및 실행

```bash
# 코드에서 워크스페이스 경로를 ~/h264_gcs_ws 로 참조하므로 해당 경로에 배치
cp -r GCS ~/h264_gcs_ws
cd ~/h264_gcs_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch gcs_bringup h264_gcs.launch.py
# 선택 인자: domain_id:=42 port:=5600 quality:=80 rate:=30 max_match_dt_ms:=100
```

## 5. 실행 전 환경별 수정 사항

| 항목 | 위치 | 내용 |
|---|---|---|
| Python 실행 경로 | `launch/h264_gcs.launch.py`의 `PYTHON` | 개발 PC의 가상환경 경로로 지정되어 있으므로 실행 환경의 Python 경로로 변경 |
| DEM 파일 | `compute_logger.py`, `gcs_ui.py`의 DEM 경로 | `~/gcs_ws/data/dem_tin_1m_pot.tif` 를 참조. DEM 원본은 저장소에 포함하지 않음 (`MMR/data/README.md` 참고) |
| 미션 파일 경로 | `gcs_ui.py`, `mission_manager.py` | `~/h264_gcs_ws/missions/` 참조 (`mission_manager.py`는 `GCS_WS`, `MISSION_DIR` 환경변수로 변경 가능) |

## 6. 공개 버전 범위

다음 항목은 공개본에서 제외했습니다.

- colcon `build/`, `install/`, `log/` 산출물
- `__pycache__/` 등 Python 캐시
- 개발 중 생성된 백업 파일(`*.bak_*`, `*.before_*`) 및 `_pre_mmr_patch_backup/`
