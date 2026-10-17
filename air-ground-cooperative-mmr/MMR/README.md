# Air-Ground Cooperative Mobile Mortar Robot (MMR)

ROS 2 기반 **공중-지상 협력형 모바일 로봇(MMR)** 프로젝트의 공개 소스 저장소입니다.
정찰/GCS에서 전달된 표적 정보를 지상 플랫폼이 수신하고, GNSS·IMU·구동부·조준 제어·운용 GUI를 하나의 ROS 2 시스템으로 통합하는 것을 목표로 제작했습니다.

> 이 저장소는 공모전·포트폴리오 공개용으로 정리한 버전입니다. 제조사 제공 IMU 드라이버/SDK, 빌드 산출물, 로컬 개발 잔재 및 일부 외부 데이터는 포함하지 않습니다.

## 1. 주요 구성

| 패키지 | 역할 |
|---|---|
| `gnss_driver` | u-blox ZED-F9P 기반 듀얼 안테나 GNSS 위치·헤딩 수신 및 NTRIP 연동 |
| `mmr_status` | GNSS/자세 상태 통합, 프로젝트 전용 IMU tilt monitor, 통합 launch |
| `mmr_drive_node` | 6륜 BLDC 기반 모바일 플랫폼 구동 및 상태 관리 |
| `fire_solution` | 표적 정보 기반 계산, 자세/환경 보정 및 조준 제어 로직 |
| `mmr_gui` | PyQt6 기반 MMR 운용·상태 표시 GUI |

## 2. 시스템 구조

```text
Recon Drone / GCS
        │
        │ target information
        ▼
     MMR ROS 2
        │
        ├── GNSS ────────────── position / heading
        ├── MW-AHRS X1 ─────── IMU data
        │        └── tilt_monitor_node
        ├── MMR status ─────── integrated robot state
        ├── Drive control ──── 6-wheel BLDC platform
        ├── Solution / Aim ─── target processing & coordination
        └── GUI ────────────── operator monitoring / control
```

ROS 2 Humble을 기준으로 개발했으며, 각 기능을 독립 패키지로 분리한 뒤 `mmr_status`의 통합 launch에서 함께 실행하도록 구성했습니다.

## 3. Repository 구조

```text
MMR/
├── README.md
├── data/
│   └── README.md
├── vendor/
│   └── README.md
├── firmware/
│   └── stepnsol_1/        # NUCLEO-F767ZI 틸트축 스텝모터·발사 제어 펌웨어 (STM32CubeIDE)
└── src/
    ├── fire_solution/
    ├── gnss_driver/
    ├── mmr_drive_node/
    ├── mmr_gui/
    └── mmr_status/
```

`.gitignore`는 저장소 루트에서 MMR·GCS·Drone 전체에 공통 적용됩니다.
펌웨어 세부 내용은 [`firmware/README.md`](firmware/README.md)를 참고해 주세요.

`build/`, `install/`, `log/`은 ROS 2 빌드 산출물이므로 버전 관리에서 제외합니다.

## 4. IMU 드라이버

MW-AHRS X1 연동에는 NTRexLAB에서 제공한 ROS/Linux 드라이버 패키지를 사용했습니다.
제조사 제공 소스, 헤더 및 정적 라이브러리는 이 저장소에서 재배포하지 않습니다.

- Manufacturer repository: https://github.com/ntrexlab/2th_NtrexAHRS_lib_ROS
- Vendor package expected by this project: `stella_ahrs`
- Vendor executable expected by the integrated launch: `stella_ahrs_node`
- Main IMU topic: `/imu/data`

프로젝트에서 추가한 자세 안정성 판단 로직은 `mmr_status/mmr_status/tilt_monitor_node.py`에 별도로 포함되어 있습니다.
세부 설치 조건은 [`vendor/README.md`](vendor/README.md)를 참고해 주세요.

## 5. Terrain / DEM 데이터

개발 환경에서는 지형 고도 참조를 위해 GeoTIFF DEM 데이터를 사용했습니다.
원본 DEM 파일은 공개 저장소에 포함하지 않으며, 기본 코드에서는 다음 경로를 참조합니다.

```text
~/mmr_ws/data/dem_tin_1m_pot.tif
```

필요한 경우 호환되는 GeoTIFF를 해당 경로에 배치하거나 launch/config에서 경로를 변경할 수 있습니다.

## 6. 개발 환경

- Ubuntu Linux
- ROS 2 Humble
- Python 3
- PyQt6
- NumPy
- pyproj
- rasterio
- pyserial
- pyubx2

하드웨어 구성에 따라 ROS 2 표준 패키지와 시스템 의존 패키지가 추가로 필요할 수 있습니다.

## 7. Build

저장소를 ROS 2 워크스페이스로 준비한 뒤 빌드합니다.

```bash
cd ~/mmr_ws
colcon build --symlink-install
source install/setup.bash
```

IMU 기능까지 함께 사용할 경우 NTRexLAB의 `stella_ahrs` 패키지를 별도로 설치한 뒤 같은 ROS 2 환경에서 빌드/소싱해야 합니다.

## 8. 실행

통합 launch는 `mmr_status` 패키지에 포함되어 있습니다.

```bash
ros2 launch mmr_status mmr_launch.py
```

제조사 IMU 패키지를 설치하지 않은 개발 환경에서는 IMU 기능을 비활성화하여 소프트웨어 구성을 확인할 수 있습니다.

```bash
ros2 launch mmr_status mmr_launch.py use_imu:=false
```

실제 시리얼 장치 경로와 하드웨어 파라미터는 실행 환경에 맞게 설정해야 합니다.

## 9. 주요 ROS 2 인터페이스

대표적인 인터페이스는 다음과 같습니다.

```text
/gnss/fix_hp
/gnss/heading
/imu/data
/imu/tilt_status
/mmr/status
/mmr/drive/status
/target/point
/mmr/fire_solution
/mmr/engagement_status
```

세부 메시지 구조와 상태 필드는 각 패키지의 소스코드를 기준으로 합니다.

## 10. 공개 버전 범위

이 저장소에는 프로젝트에서 구현한 ROS 2 통합 및 응용 코드가 중심적으로 포함되어 있습니다.
다음 항목은 공개본에서 제외했습니다.

- NTRexLAB MW-AHRS 제조사 드라이버/SDK 및 binary library
- ROS 2 `build/`, `install/`, `log/` 산출물
- 로컬 rosbag 및 개발환경 캐시
- 개발 중 사용한 보조/리플레이 스크립트
- 외부 DEM GeoTIFF 원본 데이터
- 내부 디버깅용 단계별 운용 문서

외부 의존성의 권리와 배포 조건은 각 원 저작자/제공자의 정책을 따릅니다.

## 11. Project scope

본 저장소는 공중 정찰 플랫폼, GCS, 지상 모바일 로봇을 연계하는 통합 시스템의 MMR 측 소프트웨어 구현을 정리한 자료입니다.
주요 개발 범위는 센서 데이터 수집, 위치·자세 상태 관리, 모바일 플랫폼 구동, 상위 시스템과의 표적 정보 연계, 제어 로직 및 운용 GUI 통합입니다.
