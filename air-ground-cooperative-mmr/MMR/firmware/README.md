# MMR Firmware — NUCLEO-F767ZI (`stepnsol_1`)

MMR 포탑의 **틸트(고각) 축 스텝모터**와 **발사 솔레노이드**를 제어하는 STM32 펌웨어입니다.
상위 ROS 2 노드(`MMR/src/fire_solution/fire_solution/turret_control_node.py`)와 UART 텍스트 명령으로 통신합니다.

## 1. 개발 환경

| 항목 | 내용 |
|---|---|
| 보드 | STM32 NUCLEO-F767ZI |
| IDE | STM32CubeIDE (CubeMX 설정 파일: `stepnsol_1.ioc`) |
| 라이브러리 | STM32F7 HAL, CMSIS (`Drivers/`, ST 제공 라이선스 파일 포함) |
| 클록 | HSI 16 MHz, PLL 미사용 |
| 스텝 드라이버 | CS-D508 (STEP/DIR/ENA 입력, ALM 출력을 In-Position 신호로 사용) |

## 2. 주요 하드웨어 설정 (`Core/Src/main.c`, `Core/Inc/main.h` 기준)

| 기능 | 설정 |
|---|---|
| STEP 펄스 | TIM2 PWM, PSC=15, ARR=3999 → 250 Hz / CH1=Motor0, CH2=Motor1 |
| Motor0 DIR / ENA | PD0 / PD1 (ENA LOW = enable) |
| Motor1 DIR / ENA | PD2 / PD3 |
| 홈 센서 (HOME45) | PC0 |
| In-Position (ALM) | PC8, 내부 풀업 |
| 발사 솔레노이드 | PB0, 100 ms 펄스 |
| 통신 | USART3 115200 bps, 인터럽트 수신 (ST-Link VCP) |
| 소프트웨어 리밋 | Motor0 위치 0 ~ `ELEV_MAX_PULSE`(950) |

## 3. UART 명령 프로토콜

명령은 개행(`\n`)으로 끝나는 한 줄 텍스트입니다.

| 명령 | 동작 | 응답 |
|---|---|---|
| `M0+N` / `M0-N` | Motor0(틸트)를 N 스텝 이동 | `ACK M0` / `NAK BUSY` / `NAK LIMIT_MIN` / `NAK LIMIT_MAX` |
| `M1+N` / `M1-N` | Motor1을 N 스텝 이동 | `ACK M1` / `NAK BUSY` |
| `HOME0` | Motor0 원점 복귀 시퀀스 (백오프 → 탐색) | `ACK HOME0` → `DONE HOME0` 또는 `ERR HOME0` |
| `STOP` | 모든 모터 정지 | `ACK STOP` |
| `FIRE` | 발사 솔레노이드 구동 | `ACK FIRE` |
| `STATUS` | 위치·홈·동작 상태 보고 | `POS=… HOMED=… HOMING=… …` |
| `SYNC` / `FLUSH` | 수신 버퍼 초기화 | `ACK SYNC` |
| (형식 오류) | — | `NAK FORMAT` |

부팅 완료 시 `BOOT OK`를 송신합니다.

## 4. 빌드 및 업로드

1. STM32CubeIDE → **File → Import → Existing Projects into Workspace** 에서 `stepnsol_1` 폴더 선택
2. **Project → Build Project** (출력은 `Debug/`에 생성되며 Git에서 제외됨)
3. NUCLEO 보드를 USB로 연결한 뒤 **Run → Debug As → STM32 C/C++ Application**

디버그 실행 구성(`*.launch`)은 개인 PC 경로를 포함하므로 저장소에서 제외했습니다. 처음 실행할 때 CubeIDE가 자동으로 다시 생성합니다.
