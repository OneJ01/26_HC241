# 공중-지상 협력형 박격포 모바일 로봇 (Air-Ground Cooperative Mobile Mortar Robot, MMR)

정찰 드론이 촬영한 영상에서 표적 지점을 선택하면, GCS가 드론의 RTK GNSS 위치·자세와 카메라 정보를 이용해
해당 지점의 지상 좌표를 산출하고, 지상의 로커-보기(rocker-bogie)형 6륜 모바일 로봇(MMR)이
사격 가능 위치로 이동한 뒤 틸트 축을 제어하여 표적을 타격하는 공중-지상 협력 시스템입니다.

> 공모전 제출용 공개 저장소입니다. 빌드 산출물, 개발 중 백업 파일, 제조사 SDK, DEM 원본 데이터 및 계정 정보는 포함하지 않습니다.

## 1. 시스템 구성

```text
Drone (Raspberry Pi)
  · Global shutter camera, RTK GNSS, FC / auto mission
        │  H.264 video (UDP) + frame meta / telemetry (ROS 2)
        ▼
GCS (Ubuntu PC)
  · Video display, pixel back-projection + DEM → target coordinate
        │  /target/point            ▲  /mmr/* status
        ▼                           │
MMR (Ubuntu PC, ROS 2)
  · RTK GNSS / IMU, fire solution, 6-wheel BLDC drive
        │  UART text command (M0±N, HOME0, FIRE ...)
        ▼
NUCLEO-F767ZI firmware
  · Tilt-axis stepper, firing solenoid
```

## 2. 저장소 구조

```text
.
├── README.md
├── .gitignore            # 세 파트 공통 제외 규칙
├── Drone/                # 정찰 드론 Raspberry Pi 코드
├── GCS/                  # 지상통제 PC ROS 2 워크스페이스
└── MMR/                  # 지상 모바일 로봇 ROS 2 워크스페이스
    └── firmware/
        └── stepnsol_1/   # NUCLEO-F767ZI 틸트축·발사 제어 펌웨어
```

| 파트 | 실행 환경 | 주요 내용 | 문서 |
|---|---|---|---|
| **Drone** | Raspberry Pi, ROS 2 Humble (Docker), MAVSDK | 카메라 캡처·H.264 송신, 프레임 메타 발행, FC 통신·자동비행 미션, RTK 보정 주입, 카메라 캘리브레이션 | [Drone/README.md](Drone/README.md) |
| **GCS** | Ubuntu 22.04, ROS 2 Humble, PyQt6 | H.264 수신·디코딩, 픽셀 역투영·DEM 교차 좌표 계산, 운용 GUI, 미션 관리 | [GCS/README.md](GCS/README.md) |
| **MMR** | Ubuntu 22.04, ROS 2 Humble | 듀얼 안테나 RTK GNSS, IMU 자세 감시, 6륜 BLDC 구동, 탄도·사격제원 계산, 조준 제어, 운용 GUI | [MMR/README.md](MMR/README.md) |
| **MMR firmware** | STM32 NUCLEO-F767ZI | 틸트축 스텝모터 제어, 원점 복귀, 발사 솔레노이드, UART 명령 처리 | [MMR/firmware/README.md](MMR/firmware/README.md) |

## 3. 공통 사항

- 세 파트는 동일한 `ROS_DOMAIN_ID=42`에서 통신합니다.
- `cam_msgs/FramePacket.msg`는 Drone과 GCS 워크스페이스에 각각 동일하게 포함되어 있습니다.
- 각 파트는 독립된 워크스페이스이므로 해당 디렉터리에서 개별적으로 빌드합니다.
- 시리얼 포트, IP, DEM 경로 등 장비별 설정은 각 파트 README의 안내에 따라 실행 환경에 맞게 수정해야 합니다.

## 4. 공개 버전에서 제외한 항목

- ROS 2 `build/`, `install/`, `log/` 및 STM32CubeIDE `Debug/` 빌드 산출물
- Python 캐시, 개발 중 생성된 백업 파일(`*.bak_*`, `*.before_*` 등)
- NTRexLAB MW-AHRS 제조사 드라이버/SDK ([MMR/vendor/README.md](MMR/vendor/README.md))
- DEM GeoTIFF 원본 데이터 ([MMR/data/README.md](MMR/data/README.md))
- NTRIP 계정 등 인증 정보
