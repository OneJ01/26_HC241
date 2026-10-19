# 공중-지상 협력형 박격포 모바일 로봇

**Air-Ground Cooperative Mobile Mortar Robot (MMR)**  
2026년 한이음 드림업 프로젝트 · **26_HC241**

정찰 드론, 지상통제 PC(GCS), 지상 모바일 로봇(MMR)을 연결하는 공중-지상 협력 시스템입니다. 드론 영상과 위치·자세 정보를 GCS에서 확인하고, 지상 플랫폼의 상태와 제어 기능을 통합된 소프트웨어로 관리합니다.


---

## **💡1. 프로젝트 개요**

### 1-1. 프로젝트 소개

- **프로젝트명:** 공중-지상 협력형 박격포 모바일 로봇
- **프로젝트 번호:** 26_HC241
- **프로젝트 정의:** 정찰 드론의 영상 기반 표적 정보와 지상 박격포 모바일 로봇을 연계하여, 정찰·좌표 산출·이동·조준을 통합하는 공중-지상 협력 시스템
- **주요 구성:** 정찰 드론 / GCS PC / 로커-보기 방식의 6륜 MMR / 공압식 박격포 모듈 및 임베디드 제어부

사용자가 지정한 경로를 따라 정찰 드론이 자동 비행하며 현장을 관측하고, 운용자가 영상에서 지정한 지점을 실제 지상 좌표로 변환합니다. 지상 모바일 로봇은 로커-보기 구조를 통해 험지와 야외 환경에서의 이동성을 확보하고, 전달받은 정보를 바탕으로 공압식 박격포 모듈의 포신을 제어하도록 구성했습니다. 정찰 드론과 지상 로봇의 정보를 GCS에서 연계하며, 운용자의 최종 발사 승인을 포함하는 협력 운용 체계 구현을 목표로 합니다.

#### 작품 사진

| MMR | 정찰 드론 |
|---|---|
| ![MMR 전체 사진](docs/images/image(20261019-102641).png) | ![정찰 드론 전체 사진](docs/images/image(20261019-102651).png) |

### 1-2. 개발 배경 및 필요성

정찰·관측 정보를 지상 장비의 운용으로 연결하려면 표적 정보의 전달, 좌표 확인, 장비 이동과 조준 등 여러 과정이 필요합니다. 사람이 현장에서 직접 관측하고 수작업으로 정보를 전달·조정하는 방식은 운용 인력의 위험 지역 노출과 작업 부담을 높이고, 정보 전달 지연이나 조작 오차를 유발할 수 있습니다.

본 프로젝트는 정찰 드론과 박격포 모바일 로봇을 연계하여 이러한 과정을 원격화·자동화하고자 합니다. 공중 영상 기반의 표적 확인과 좌표 산출, 지상 플랫폼의 이동 및 조준 기능을 하나의 운용 환경으로 통합하여 운용자의 노출을 줄이고 대응 과정의 효율을 높이는 것을 목표로 합니다.

### 1-3. 프로젝트 특장점

- **지형을 고려한 좌표 변환:** 드론 영상에서 선택한 지점에 영상 역투영과 DEM 지형 데이터를 적용하여 지형 고도를 반영한 지상 좌표를 산출합니다.
- **영상·비행 정보의 시간 정합:** 영상 프레임과 드론의 비행 데이터를 타임스탬프 기준으로 대응시켜 통신 지연에 따른 좌표 산출 오차를 줄이도록 설계했습니다.
- **공중-지상 협력 운용:** 정찰 정보 전달과 지상 플랫폼의 제어를 연계하여 운용자의 현장 노출과 수작업에 필요한 시간을 줄이는 것을 지향합니다.
- **로커-보기 기반 이동 플랫폼:** 6륜 로커-보기 구조를 적용하여 야외 지형에서의 접지력과 이동 안정성을 확보하도록 구성했습니다.
- **통합 UI와 운용자 승인:** ROS 2 기반 통신과 운용 UI로 각 장비의 정보를 통합하고, 최종 발사 승인에 운용자가 개입하도록 설계했습니다.

### 1-4. 주요 기능

| 구분 | 주요 기능 |
|---|---|
| 공중 정찰 | 지정 경로 기반 드론 자동 비행, 실시간 영상 수집·전송 및 영상에서의 표적 지점 확인 |
| 지상 좌표 산출 | 영상에서 지정한 지점을 지형 고도를 반영한 지상 좌표로 변환 |
| 지상 플랫폼 이동 | 6륜 로커-보기 구조와 조이스틱 기반 원격 주행을 이용한 야외 이동 |
| 포신 제어 | 산출된 좌표 정보를 활용한 포신 제어 및 오차 보정 기능 |
| 정보 공유·상태 표시 | 정찰 드론과 MMR의 정보를 GCS에서 연계하고 영상·위치·자세·장비 상태 표시 |
| 통합 운용 | 표적 확인·좌표 산출·지상 플랫폼 제어를 연결하고 운용자의 최종 승인 절차 반영 |

![GCS 운용 화면](docs/images/image(20261019-102719).png)


### 1-5. 기대 효과 및 활용 분야

- **운용 인력의 노출 감소:** 원격 정찰과 지상 플랫폼 운용을 통해 위험 지역에서 인력이 직접 수행해야 하는 작업을 줄일 수 있습니다.
- **정보 전달 및 운용 효율 향상:** 영상·좌표·장비 상태를 연결하여 수작업 전달과 반복 확인에 필요한 시간 및 조작 부담을 줄이는 효과를 기대합니다.
- **좌표 산출·조준 과정의 오차 감소:** 지형 정보와 시간 정합을 활용하고 수작업 조정 과정을 줄여 대응 과정의 일관성을 높이는 것을 목표로 합니다.
- **활용 분야:** 공중-지상 협력형 정찰 체계, 위험 지역 원격 대응 플랫폼, 재난 현장 정찰 및 복구 로봇 등


### 1-6. 기술 스택

| 분야 | 적용 기술 및 환경 |
|---|---|
| 운영체제 | Ubuntu 22.04, Raspberry Pi OS |
| 미들웨어 | ROS 2 Humble |
| 개발 언어 | Python, C, C++ |
| 사용자 인터페이스 | PyQt6 |
| 영상 처리·전송 | Picamera2 / libcamera, OpenCV, GStreamer, H.264 |
| 데이터 처리 | NumPy, pyproj, rasterio |
| 비행 제어 연동 | MAVSDK, 기존 FC 펌웨어 |
| 센서 | u-blox ZED-F9P GNSS, NTRexLAB MW-AHRS X1 |
| 임베디드 개발 | STM32 NUCLEO-F767ZI, STM32CubeIDE |
| 개발·관리 | GitHub, Docker, colcon |

### 1-7. 기성 기술 적용 및 자체 개발 범위

| 구분 | 기성품·외부 기술 적용 | 팀 개발 내용 |
|---|---|---|
| 정찰 드론 | KOA FC의 기존 비행 펌웨어, Raspberry Pi 및 카메라 관련 라이브러리 | 영상·메타데이터 처리, GCS 연동 소프트웨어 |
| GCS PC | ROS 2, PyQt6, OpenCV, GStreamer 등 | 자체 제작 UI, 좌표 변환 및 플랫폼 정보 연계 |
| MMR | GNSS 장비 및 기본 제공 도구, NTRexLAB AHRS 제조사 드라이버 | 조이스틱 기반 원격 주행, 상태 관리, 포신 제어 알고리즘 및 시스템 통합 |


---

## **💡2. 팀원 소개**

팀원 5명과 멘토로 구성됩니다.


| 구분 | 담당 분야 | 역할 |
|---|---|---|
| 멘티1 | 정찰 드론 | 드론 구성·운용, 영상 수집·전송 및 GCS 연동 |
| 멘티2 | GCS PC | 운용 UI 개발, 영상·지도·상태 정보 표시 및 플랫폼 연동 |
| 멘티3 | MMR 기구부 | 로커-보기 플랫폼의 기구 구성·조립 및 장치 배치 |
| 멘티4 | MMR 전장부 | 전원·배선 구성, 센서 및 제어 장치 연결·점검 |
| 멘티5 | MMR 소프트웨어 | 센서 데이터 연동, 원격 주행 및 상태 관리 소프트웨어 |
| 멘토 | 전체 총괄 | 프로젝트 멘토 · 기술 자문 |

---

## **💡3. 시스템 구성도**

### 3-1. 전체 시스템 구성

| 구성 요소 | 역할 |
|---|---|
| 정찰 드론 | 카메라와 비행 제어 장치를 이용한 공중 관측 및 데이터 수집 |
| GCS PC | 수신 영상과 상태 정보 표시, 플랫폼 간 정보 연계 |
| 지상 MMR | 지상 이동, 센서 데이터 수집, 운용 상태 제공 |
| 임베디드 제어부 | MMR 하위 장치와 상위 소프트웨어 간 연동 |



### 3-2. 하드웨어 구성

- **정찰 드론:** 비행 제어 장치(FC), Raspberry Pi, 글로벌 셔터 카메라, GNSS
- **GCS PC:** Ubuntu 기반 운용 PC
- **MMR:** 로커-보기 방식의 6륜 플랫폼, BLDC 구동부, GNSS, AHRS, 온보드 PC, STM32 제어 보드

#### MMR 시스템 구성도

![MMR 하드웨어 블록 다이어그램](docs/images/image(20261019-102905).png)

#### 정찰 드론 시스템 구성도

![정찰 드론 하드웨어 블록 다이어그램](docs/images/image(20261019-102931).png)

### 3-3. 소프트웨어 구성 및 저장소 안내

실제 소스코드는 저장소의 `air-ground-cooperative-mmr/` 디렉터리에 있습니다.

| 경로 | 내용 |
|---|---|
| [Drone/](air-ground-cooperative-mmr/Drone/) | Raspberry Pi 기반 드론 소프트웨어, 영상 처리 및 관련 도구 |
| [GCS/](air-ground-cooperative-mmr/GCS/) | GCS UI와 영상·상태 정보 처리 소프트웨어 |
| [MMR/](air-ground-cooperative-mmr/MMR/) | 지상 플랫폼의 ROS 2 패키지 및 관련 자료 |
| [MMR/vendor/](air-ground-cooperative-mmr/MMR/vendor/) | 외부 제조사 드라이버 관련 안내 |



### 3-4. 정보 처리 흐름

1. 정찰 드론에서 영상과 비행 상태 정보를 수집합니다.
2. GCS에서 수신 영상을 표시하고 관련 상태 정보를 함께 확인합니다.
3. 지상 플랫폼의 센서·주행 상태를 운용 화면에 반영합니다.
4. 운용자가 화면을 통해 각 플랫폼의 상태를 확인합니다.



---

## **💡4. 작품 소개영상**

[![공중-지상 협력형 박격포 모바일 로봇 소개영상](https://img.youtube.com/vi/6zkmAgEBdJQ/hqdefault.jpg)](https://www.youtube.com/watch?v=6zkmAgEBdJQ)

**위 썸네일을 클릭하시면 소개영상으로 이동합니다.**  
[YouTube에서 소개영상 보기](https://www.youtube.com/watch?v=6zkmAgEBdJQ)

---

## **💡5. 핵심 소스코드**

정찰 드론·GCS PC·MMR에서 각각 하나의 대표 소스 파일을 선정했습니다. 아래 코드는 각 파일에서 발췌한 일부이며, 전체 구현은 연결된 원본 파일에서 확인할 수 있습니다.

| 파트 | 대표 소스 파일 | 담당 기능 |
|---|---|---|
| 정찰 드론 | [h264_camera_sender.py](air-ground-cooperative-mmr/Drone/h264_cam_ws/scripts/h264_camera_sender.py) | 카메라 영상의 H.264 인코딩 및 RTP/UDP 전송 |
| GCS PC | [camera_frame_buffer.py](air-ground-cooperative-mmr/GCS/src/gcs_bringup/scripts/camera_frame_buffer.py) | 영상 수신 처리와 UI 사이의 최신 프레임 관리 |
| MMR | [mmr_status_node.py](air-ground-cooperative-mmr/MMR/src/mmr_status/mmr_status/mmr_status_node.py) | GNSS 위치·헤딩·수신 상태를 GUI 표시용 상태로 통합 |

### 5-1. 정찰 드론 — 카메라 영상 인코딩 및 전송

**소스코드:** [h264_camera_sender.py](air-ground-cooperative-mmr/Drone/h264_cam_ws/scripts/h264_camera_sender.py)

Raspberry Pi에서 수집한 카메라 영상을 H.264로 인코딩하여 전송하는 코드입니다. 별도의 프레임 메타데이터도 기록합니다. 아래는 `build_pipeline()` 함수 내부의 발췌로, 원시 영상 입력, 영상 형식 변환, H.264 인코딩, RTP 패킷화, UDP 전송으로 이어지는 영상 처리 구성을 보여 줍니다.

```python
    pipeline_str = f"""
        appsrc name=src is-live=true format=time do-timestamp=false
            caps=video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1
        ! queue leaky=downstream max-size-buffers=2
        ! videoconvert
        ! video/x-raw,format=I420
        ! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps}
                  key-int-max=5 bframes=0 byte-stream=true
        ! h264parse config-interval=1
        ! rtph264pay pt=96 config-interval=1
        ! udpsink host={host} port={port} sync=false async=false
    """
    return pipeline_str
```

### 5-2. GCS PC — 화면 표시용 최신 프레임 관리

**소스코드:** [camera_frame_buffer.py](air-ground-cooperative-mmr/GCS/src/gcs_bringup/scripts/camera_frame_buffer.py)

GCS의 영상 수신 처리와 UI 표시 처리 사이에서 최신 프레임 한 장을 보관하는 클래스입니다. JPEG 데이터와 디코딩된 BGR 프레임을 지원하며, 아래 메서드는 잠금 구간에서 프레임을 가져온 뒤 보관 공간을 비웁니다. 반환되는 메타데이터에 영상 형식을 포함해 UI가 표시 형식을 구분할 수 있도록 합니다.

```python
    def take_latest(self):
        """
        UI thread에서 호출.
        최신 프레임 1개를 가져오고 버퍼를 비운다.

        return:
            payload, meta

        payload:
            - meta["_frame_format"] == "jpeg"이면 JPEG bytes
            - meta["_frame_format"] == "bgr"이면 OpenCV BGR ndarray
        """
        with self._lock:
            if self._latest_payload is None:
                return None, None

            payload = self._latest_payload
            meta = self._latest_meta or {}
            fmt = self._latest_format

            self._latest_payload = None
            self._latest_meta = None
            self._latest_format = None

        if isinstance(meta, dict) and fmt is not None:
            meta.setdefault("_frame_format", fmt)

        return payload, meta
```

### 5-3. MMR — 센서 정보의 운용 화면용 상태 통합

**소스코드:** [mmr_status_node.py](air-ground-cooperative-mmr/MMR/src/mmr_status/mmr_status/mmr_status_node.py)

GNSS 드라이버에서 전달받은 위치·헤딩·수신 품질 정보를 GUI에서 사용할 통합 상태로 정리하는 ROS 2 노드입니다. 위치 정보와 수신 품질 정보를 구분해 처리하고, 헤딩 정보의 갱신 여부도 상태에 반영합니다.

아래는 수신 품질을 화면 표시용 문자열로 변환하는 함수입니다. `NO_FIX`, `GNSS`, `RTK_FLOAT`, `RTK_FIX` 등의 상태를 구분하며, **이 프로젝트의 GNSS 드라이버가 사용하는 상태 표현에 맞춘 매핑**입니다. ROS 표준 상태값 자체가 모든 드라이버에서 동일한 RTK 의미를 갖는다는 뜻은 아닙니다.

```python
def _fix_quality(msg: NavSatFix) -> str:
    """
    NavSatFix status/covariance 기반 fix 품질 문자열 변환.

    현재 gnss_driver에서:
      - NO_FIX
      - GNSS
      - RTK_FLOAT
      - RTK_FIX
    형태로 표현하기 위한 단순 매핑.
    """
    s = msg.status.status
    c = msg.position_covariance_type

    if s == NavSatStatus.STATUS_NO_FIX:
        return "NO_FIX"

    if s == NavSatStatus.STATUS_SBAS_FIX:
        return "RTK_FLOAT"

    if s == NavSatStatus.STATUS_GBAS_FIX:
        return "RTK_FIX" if c == NavSatFix.COVARIANCE_TYPE_KNOWN else "RTK_FLOAT"

    if s == NavSatStatus.STATUS_FIX:
        return "GNSS"

    return "UNKNOWN"

```

### 5-4. 외부 제공 소프트웨어 구분

AHRS 연동에는 [NTRexLAB 제조사 패키지](https://github.com/ntrexlab/2th_NtrexAHRS_lib_ROS)를 사용했습니다. 제조사 드라이버 자체는 팀의 자체 개발 코드로 분류하지 않으며, 공개 저장소에는 해당 드라이버와 SDK를 재배포하지 않습니다.

팀의 개발 내용은 영상·상태 정보 처리, UI, 플랫폼 간 연동 및 응용 제어 소프트웨어를 중심으로 정리했습니다.

---

### 참고 자료

- [제조사 드라이버 구분 안내](air-ground-cooperative-mmr/MMR/vendor/README.md)

