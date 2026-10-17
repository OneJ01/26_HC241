# MMR 운용 GUI
# MMR 위치·헤딩, GNSS 단독 테스트, IMU 자세, 발사 상태, 표적 좌표 처리 흐름을 표시
# 실행: ros2 run mmr_gui main_gui

import sys, time, json, math, os
from collections import deque
import numpy as np

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPointF, QObject, pyqtSlot
from PyQt6.QtGui import (
    QShortcut, QKeySequence, QFont, QAction,
    QPainter, QPen, QBrush, QPolygonF, QColor, QImage, QPixmap
)
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QStackedWidget, QTextEdit, QFrame, QSplitter, QSizePolicy, QPushButton
)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebChannel import QWebChannel

from pyproj import Transformer

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DEM_PATH = os.path.expanduser("~/mmr_ws/data/dem_tin_1m_pot.tif")
DEFAULT_DIAGNOSTIC_EVENT_PATH = "/tmp/mmr_local_diagnostic_event.json"
DEFAULT_IMPACT_THRESHOLD_M = 15.0
DEFAULT_IMPACT_HISTORY_MAX_COUNT = 20

try:
    import rasterio
    _RASTERIO_OK = True
except ImportError:
    _RASTERIO_OK = False


# -----------------------------------------------------------------------------
# GUI 한글 상태·오류 설명
# -----------------------------------------------------------------------------
# ROS 토픽에 실리는 영문 상태 코드는 다른 노드와의 호환성을 위해 변경하지 않고,
# 운용자가 보는 GUI에서만 원인과 확인 항목을 한글로 풀어 표시한다.
_ENGAGEMENT_STATE_KO = {
    "STARTUP_WAIT_HOME": "터렛 홈 완료 대기",
    "IDLE": "표적 수신 대기",
    "TARGET_CHECK": "표적·탄도해 확인",
    "REPOSITION_REQUIRED": "사거리 확보를 위한 재배치 필요",
    "WAIT_ALIGN": "A 홀드 대기 · 차체 정렬",
    "BODY_ALIGNING": "차체 방위각 정렬 중",
    "ALIGN_COMPLETE_WAIT_RELEASE": "차체 정렬 완료 · A 버튼 해제 대기",
    "FINAL_RECHECK": "정렬 상태 최종 재확인",
    "READY_FOR_AIM": "A 홀드 대기 · 포신 조준",
    "AIM_CONFIRMING": "포신 조준 승인 홀드 중",
    "TURRET_AIMING": "포신 목표 각도 이동 중",
    "FINAL_AIM_VERIFY": "포신 조준 결과 최종 확인",
    "AIM_READY": "발사 준비 완료 · Y 홀드 대기",
    "FIRE_CONFIRMING": "발사 승인 Y 홀드 중",
    "FIRING": "발사 시퀀스 진행 중",
    "FIRED": "발사 시퀀스 완료",
    "FAULT": "시퀀스 고장 정지",
    "ESTOP": "비상정지 작동",
}

# 수동 주행 중 GUI에서 숨길 차체 정렬 관련 알림 코드.
# 실제 AimCoordinator 판정과 안전 인터록은 그대로 유지하고 표시만 억제한다.
_DRIVE_MOVING_HIDDEN_ALIGNMENT_REASONS = {
    "YAW_OUT_OF_TOLERANCE",
    "YAW_OUT_AFTER_RELEASE",
    "ALIGN_DRIFTED",
    "ALIGN_BUTTON_RELEASED",
    "WAITING_FOR_WHEELS_TO_STOP",
    "DRIVE_NOT_STOPPED",
    "MANUAL_DRIVE_OVERRIDE",
}

# 주행 때문에 IMU stationary 판정만 해제된 경우 GUI에서 숨길 알림 코드.
# 실제 roll/pitch 제한 초과는 이 집합과 무관하게 계속 표시한다.
_DRIVE_MOVING_HIDDEN_IMU_REASONS = {
    "IMU_NOT_STABLE",
    "WAITING_FOR_IMU_STABLE",
    "TILT_NOT_OK",
}

_TURRET_STATE_KO = {
    "STARTUP": "터렛 초기화 중",
    "STARTUP_HOMING": "시작 홈 복귀 중",
    "HOME_REQUIRED": "홈 복귀 필요",
    "HOMING": "홈 복귀 중",
    "HOME_READY": "홈 위치 정상",
    "TURRET_AIMING": "포신 이동 중",
    "AIM_READY": "포신 조준 완료",
    "FIRING": "발사 준비·실행 중",
    "FIRED": "발사 완료",
    "FAULT": "터렛 고장",
    "ESTOP": "비상정지",
}

_REASON_KO = {
    # AimCoordinator 상태 원인
    "WAITING_FOR_TURRET_HOME": "터렛 홈 완료를 기다리는 중입니다. HOME 센서와 STM 연결을 확인하십시오.",
    "TARGET_AVAILABLE": "표적 좌표가 확인되어 처리 준비가 되었습니다.",
    "TARGET_RECEIVED": "새 표적 좌표를 수신했습니다.",
    "NEW_TARGET": "기존 표적과 다른 새 표적이 들어와 이전 조준 잠금을 해제했습니다.",
    "NEXT_TARGET": "발사 완료 후 다음 표적을 수신했습니다.",
    "NEXT_TARGET_BUFFERED_DURING_POST_FIRE": "발사 후 홈 복귀가 끝날 때까지 새 표적을 다음 교전용으로 보관합니다.",
    "FIRE_COMPLETE_HOME_READY": "발사 완료 후 포신 홈 복귀가 끝나 다음 표적 대기 상태로 전환했습니다.",
    "FIRE_COMPLETE_TARGET_RETAINED": "발사 완료와 포신 홈 복귀가 끝났습니다. 마지막 표적 좌표를 유지하며 새 A 입력을 기다립니다.",
    "TARGET_RETAINED_WAIT_A": "마지막 표적 좌표가 유지 중입니다. 같은 좌표로 다시 교전하려면 A 버튼을 새로 누르십시오.",
    "REENGAGEMENT_REQUESTED": "보관 중인 표적 좌표로 새로운 교전 시퀀스를 시작합니다.",
    "FIRE_COMPLETE_WAITING_AUTO_HOME": "발사 완료 표시 후 포신 자동 홈 복귀 시작을 기다리는 중입니다.",
    "POST_FIRE_AUTO_HOMING": "발사 완료 후 포신이 자동으로 홈 위치로 복귀하는 중입니다.",
    "POST_FIRE_HOME_REQUIRED": "발사 후 자동 홈 복귀에 실패하여 홈 복귀가 다시 필요합니다.",
    "NO_SOLUTION": "유효한 탄도해가 없습니다. 표적 좌표와 탄도해 노드 상태를 확인하십시오.",
    "SOLUTION_STALE": "탄도해 갱신이 끊겼습니다. /mmr/fire_solution 발행 상태를 확인하십시오.",
    "SOLUTION_READY": "탄도해가 유효하며 정렬 단계로 진행할 수 있습니다.",
    "OUT_OF_RANGE": "표적이 현재 발사 가능 거리 밖입니다. MMR 위치를 이동해야 합니다.",
    "INVALID_FIRE_SOLUTION": "탄도해의 방위각 또는 고각 값이 유효하지 않습니다.",
    "NO_IMU": "IMU 데이터가 없습니다. 센서 포트와 /imu/tilt_status를 확인하십시오.",
    "IMU_STALE": "IMU 데이터 갱신이 끊겼습니다. 센서 통신 상태를 확인하십시오.",
    "IMU_NOT_STABLE": "차체가 움직이거나 진동 중입니다. 정지 후 IMU 안정화를 기다리십시오.",
    "INVALID_IMU_ANGLES": "IMU 롤·피치 값이 유효하지 않습니다.",
    "WAITING_FOR_IMU_STABLE": "IMU가 안정 판정을 내릴 때까지 기다리는 중입니다.",
    "ELEVATION_OUT_OF_RANGE": "요구 포신 고각이 기구 한계를 벗어났습니다. 거리 또는 탄도 파라미터를 확인하십시오.",
    "YAW_OUT_OF_TOLERANCE": "차체 방위각 오차가 허용 범위를 벗어났습니다. A 홀드로 재정렬하십시오.",
    "YAW_OUT_AFTER_RELEASE": "A 버튼 해제 후 방위각이 다시 허용 범위를 벗어났습니다.",
    "DRIVE_NOT_STOPPED": "구동 바퀴가 완전히 정지하지 않았습니다. 모터 피드백과 정지 상태를 확인하십시오.",
    "WAITING_FOR_WHEELS_TO_STOP": "차체 정렬 후 바퀴 정지를 기다리는 중입니다.",
    "MANUAL_DRIVE_OVERRIDE": "조이스틱 수동 주행 입력이 감지되어 자동 정렬을 중단했습니다.",
    "MOVE_WITH_JOYSTICK": "표적이 사거리 밖입니다. 조이스틱으로 MMR을 재배치하십시오.",
    "POSITION_NOW_VALID": "현재 위치에서 표적이 발사 가능 범위에 들어왔습니다.",
    "ALIGN_BUTTON_HELD": "A 버튼 홀드를 감지하여 차체 정렬을 시작합니다.",
    "ALIGN_SETTLING": "방위각 허용 범위 진입 후 안정 시간을 확인 중입니다.",
    "ALIGN_FINAL_CORRECTION_REQUIRED": "정지 후 IMU 보정 재검증에서 잔여 방위 오차가 확인되어 미세 조향을 다시 수행합니다.",
    "ALIGN_COMPLETE_RELEASE_BUTTON": "차체 정렬이 완료되었습니다. 다음 단계 진행을 위해 A 버튼을 해제하십시오.",
    "ALIGN_BUTTON_RELEASED": "A 버튼이 조기에 해제되어 정렬을 취소했습니다.",
    "ALIGN_DRIFTED": "정렬 중 방위각 오차가 다시 커졌습니다.",
    "HOLD_A_TO_AIM": "A 버튼을 계속 눌러 포신 조준을 승인하십시오.",
    "AIM_HOLD_STARTED": "포신 조준 승인 A 홀드를 시작했습니다.",
    "HOLD_TO_CONFIRM_AIM": "A 홀드 시간 충족을 기다리는 중입니다.",
    "AIM_HOLD_CANCELLED": "A 버튼이 해제되어 포신 조준 승인이 취소되었습니다.",
    "TURRET_AIM_REQUESTED": "터렛에 목표 고각 이동 명령을 전송했습니다.",
    "TURRET_AIM_COMPLETE": "터렛이 현재 잠금 표적·탄도해 버전·목표 고각과 일치하는 조준 완료를 보고했습니다.",
    "AIM_REQUEST_REJECTED": "터렛이 포신 조준 요청을 거부했습니다. 뒤에 표시되는 세부 원인을 확인한 뒤 A 홀드로 다시 시도하십시오.",
    "WAITING_FOR_TURRET_AIM_ACCEPT": "포신 조준 요청을 터렛 노드가 접수해 상태를 갱신하기를 기다리는 중입니다.",
    "TURRET_AIM_RESULT_MISMATCH": "터렛의 조준 완료 정보가 현재 잠금 표적·탄도해 버전·목표 고각과 일치하지 않습니다. 이전 조준 결과가 남았는지 확인하고 다시 조준하십시오.",
    "TURRET_READY_LOST_REAIM": "최종 조준 확인 중 터렛의 조준 완료 또는 발사 준비 신호가 해제되었습니다. 포신 상태를 확인하고 A 홀드로 다시 조준하십시오.",
    "TURRET_NOT_AIM_READY_REAIM": "최종 조준 확인 중 터렛 상태가 AIM_READY가 아니어서 재조준 단계로 복귀했습니다.",
    "TARGET_CHANGED_DURING_AIM": "포신 이동 중 표적이 변경되어 안전상 재조준이 필요합니다.",
    "TARGET_CHANGED": "조준 잠금 이후 표적이 변경되었습니다. 새 표적으로 다시 조준하십시오.",
    "LOCKED_ELEVATION_MISSING": "잠금된 포신 고각 정보가 없어 시퀀스를 중단했습니다.",
    "ELEVATION_CHANGED_REAIM": "현재 탄도 고각이 잠금된 조준 고각과 달라 재조준이 필요합니다.",
    "WAITING_FOR_TURRET_READY": "터렛의 HOME·조준 완료·발사 준비 신호를 기다리는 중입니다.",
    "HOLD_Y_TO_FIRE": "발사 준비가 완료되었습니다. Y 버튼을 홀드해 발사를 승인하십시오.",
    "FIRE_HOLD_STARTED": "발사 승인 Y 홀드를 시작했습니다.",
    "HOLD_TO_CONFIRM_FIRE": "Y 홀드 시간 충족을 기다리는 중입니다.",
    "FIRE_HOLD_CANCELLED": "Y 버튼이 해제되어 발사 승인이 취소되었습니다.",
    "FIRE_REQUESTED": "터렛에 발사 확인 명령을 전송했습니다.",
    "FIRE_REQUEST_REJECTED": "터렛이 발사 확인 요청을 거부했습니다. 뒤에 표시되는 세부 원인을 확인하십시오.",
    "WAITING_FOR_TURRET_FIRE_ACCEPT": "발사 확인 요청을 터렛 노드가 접수해 상태를 갱신하기를 기다리는 중입니다.",
    "HOME_REQUIRED_DURING_FIRE": "발사 요청 처리 중 터렛 홈 기준이 무효화되었습니다. 홈 복귀 후 다시 조준해야 합니다.",
    "TURRET_RETURNED_HOME_READY_DURING_FIRE": "발사 요청 처리 중 터렛이 홈 준비 상태로 돌아가 조준 잠금이 해제되었습니다. A 홀드부터 다시 진행하십시오.",
    "FIRE_COMPLETE": "STM이 발사 시퀀스 완료를 보고했습니다.",
    "TURRET_STATUS_STALE": "터렛 상태 갱신이 끊겼습니다. /mmr/turret/status와 STM UART를 확인하십시오.",
    "TURRET_STATUS_STALE_DURING_ATOMIC_ACTION": "포신 이동 또는 발사 중 터렛 상태 수신이 끊겼습니다.",
    "TURRET_NOT_FIRE_READY": "터렛이 발사 준비 상태가 아닙니다. HOME·INP·M0RUN·조준 완료를 확인하십시오.",
    "TURRET_HOME_INVALID": "터렛 홈 기준이 유효하지 않습니다. HOME 복귀를 다시 수행하십시오.",
    "TURRET_ESTOP": "터렛 비상정지가 작동 중입니다. 원인 제거 후 RESET 하십시오.",
    "TURRET_FAULT": "터렛 노드가 고장 상태를 보고했습니다. 하단 터렛 오류를 확인하십시오.",
    "FAULT_CLEARED_HOME_VALID": "터렛 홈 상태가 정상으로 회복되어 고장 상태를 해제했습니다.",
    # 탄도해·표적 입력 오류
    "TARGET_POINT_MISSING": "표적 좌표가 없습니다. GCS 또는 지도에서 표적을 다시 지정하십시오.",
    "TARGET_POINT_STALE": "표적 좌표 갱신 시간이 초과되었습니다. 표적 송신 상태를 확인하십시오.",
    "TARGET_POINT_INVALID": "표적 좌표의 위도·경도·고도 값이 유효하지 않습니다.",
    "TARGET_POINT_PARSE_ERROR": "표적 좌표 메시지 형식이 올바르지 않습니다.",
    "TARGET_DEM_INTERSECTION_FAILED": "표적 지점의 DEM 고도를 구하지 못했습니다. DEM 파일 범위와 좌표를 확인하십시오.",
    "TARGET_PROJECTION_FAILED": "표적 좌표 투영 계산에 실패했습니다. 좌표계 설정을 확인하십시오.",
    "TARGET_SYNC_FAILED": "표적 좌표와 계산 입력 동기화에 실패했습니다.",
    "TARGET_SYNC_ERROR_TOO_LARGE": "표적 데이터 간 시간 또는 위치 오차가 허용 범위를 초과했습니다.",
    "MMR_FIX_MISSING": "MMR GNSS 위치가 수신되지 않았습니다.",
    "MMR_FIX_STALE": "MMR GNSS 위치 갱신이 끊겼습니다.",
    "MMR_FIX_INVALID": "MMR GNSS 위치가 무효 상태입니다. FIX 품질을 확인하십시오.",
    "HEADING_MISSING": "MMR 헤딩이 수신되지 않았습니다.",
    "HEADING_STALE": "MMR 헤딩 갱신이 끊겼습니다. 듀얼 GNSS 베이스라인 상태를 확인하십시오.",
    "HEADING_INVALID": "MMR 헤딩 값이 유효하지 않습니다.",
    "INTERNAL_INPUT_MISSING": "탄도 계산에 필요한 입력값이 누락되었습니다.",
    "TARGET_NOT_HIT": "DEM 계산선이 표적 지형과 교차하지 않았습니다. 표적 좌표와 DEM 범위를 확인하십시오.",
    # 터렛·STM 오류
    "ESTOP_LATCHED": "비상정지가 유지 중입니다. 위험 요인 제거 후 RESET 하십시오.",
    "ESTOP_DURING_TURRET_MOTION": "포신 이동 중 비상정지가 입력되어 즉시 정지했습니다.",
    "ESTOP_DURING_HOME_PREP": "홈 복귀 준비 중 비상정지가 입력되었습니다.",
    "ESTOP_DURING_UART_WAIT": "STM 응답 대기 중 비상정지가 입력되었습니다.",
    "ESTOP_DURING_FIRE_SEQUENCE": "발사 시퀀스 중 비상정지가 입력되어 발사를 중단했습니다.",
    "TILT_NOT_OK": "차체 롤·피치가 허용 범위를 벗어나 발사를 제한했습니다.",
    "TILT_STATUS_NEVER_RECEIVED": "IMU 기울기 상태를 한 번도 수신하지 못했습니다. tilt_monitor 노드와 센서 포트를 확인하십시오.",
    "YAW_UART_SEND_FAILED": "방위축 UART 명령 전송에 실패했습니다.",
    "YAW_AXIS_DISABLED_WARNING": "터렛 방위축이 비활성화되어 차체 회전 정렬이 필요합니다.",
    "SOLUTION_NOT_IN_RANGE": "표적이 발사 가능 사거리 밖입니다. MMR을 재배치하십시오.",
    "INVALID_ANGLE": "계산된 조준 각도가 유효하지 않습니다.",
    "NOT_ARMED": "발사 계통 준비 상태가 아닙니다.",
    "IMU_ROLL_LIMIT_EXCEEDED": "차체 롤 각도가 허용 한계를 초과했습니다.",
    "IMU_PITCH_LIMIT_EXCEEDED": "차체 피치 각도가 허용 한계를 초과했습니다.",
    "TURRET_BUSY": "터렛이 다른 명령을 수행 중이라 새 명령을 거부했습니다.",
    "AIM_NOT_READY": "포신 조준 완료 조건이 충족되지 않아 발사 명령을 거부했습니다.",
    "AIM_REQUEST_PARSE_ERROR": "포신 조준 요청 JSON 형식이 잘못되었습니다.",
    "INVALID_ELEVATION": "요청된 포신 고각 값이 숫자가 아니거나 유효하지 않습니다.",
    "ELEVATION_LIMIT_EXCEEDED": "요청 포신 고각이 소프트 리밋을 벗어났습니다.",
    "ELEV_UART_SEND_FAILED": "STM으로 포신 이동 명령을 전송하지 못했습니다. UART 포트와 케이블을 확인하십시오.",
    "SERIAL_NOT_OPEN": "STM 시리얼 포트가 열리지 않았습니다. 포트명·권한·케이블을 확인하십시오.",
    "UART_RESPONSE_TIMEOUT": "STM 응답 시간이 초과되었습니다. STM 전원·포트·보드 정지를 확인하십시오.",
    "UART_READ_FAILED": "STM UART 수신 중 오류가 발생했습니다. USB 케이블·포트 권한·STM 전원을 확인하십시오.",
    "UART_SYNC": "STM 통신 동기화를 진행 중입니다.",
    "HOME_PREP_IDLE_TIMEOUT": "STOP 전송 후에도 틸트 모터가 정지 상태가 되지 않았습니다. M0RUN·기구 걸림·드라이버 상태를 확인하십시오.",
    "POST_REJECT_STATUS_FAILED": "STM이 명령을 거부한 뒤 상태 재확인에도 실패했습니다. STM 통신과 펌웨어 상태를 확인하십시오.",
    "ELEV_MOTOR_WAIT_TIMEOUT": "포신이 제한 시간 안에 목표 위치에 도달하지 못했습니다. INP 신호·모터 구동·기구 걸림을 확인하십시오.",
    "FIRE_SEQUENCE_STARTED": "발사 시퀀스가 시작되었습니다.",
    "FIRE_REQUEST_ACCEPTED": "발사 요청을 접수했습니다.",
    "FIRE_COMPLETE": "발사 시퀀스가 완료되었습니다.",
    "FIRE_ABORTED": "발사 시퀀스가 중단되었습니다.",
    "COMMAND_REJECTED": "현재 터렛 상태에서 허용되지 않는 명령이라 거부했습니다.",
    "UNKNOWN_FIRE_COMMAND": "알 수 없는 발사 명령을 수신했습니다.",
    "UNKNOWN_HOME_COMMAND": "알 수 없는 홈 복귀 명령을 수신했습니다.",
    "UNKNOWN_WORKER_COMMAND": "터렛 작업 스레드가 알 수 없는 명령을 수신했습니다.",
    "HOME_PREP_STOP_SEND_FAILED": "재홈 전 STOP 명령 전송에 실패했습니다.",
    "HOME0_NOT_ATTEMPTED": "HOME0 명령이 실행되지 못했습니다.",
    "MOTION_ABORTED": "포신 동작 오류로 현재 시퀀스를 중단했습니다.",
    "HOME_FAILED": "터렛 홈 복귀에 실패했습니다.",
    "HOME_REQUIRED_AFTER_ESTOP": "비상정지 해제 후 홈 기준이 무효화되어 다시 홈 복귀해야 합니다.",
    "ESTOP_RESET_PENDING": "비상정지 해제 요청을 접수했으며 STOP 처리 완료 후 자동 홈 복귀를 시작합니다.",
    "STOP_UART_FAILED": "STM STOP 명령 전송에 실패하여 안전상 발사를 차단했습니다.",
    "FIRE_UART_FAILED": "STM FIRE 명령 전송에 실패했습니다. UART 연결과 STM 상태를 확인하십시오.",
    "FIRE_BLOCKED": "발사 직전 STM 준비 조건이 충족되지 않아 발사를 차단했습니다.",
    "FIRE_BLOCKED_AFTER_STOP": "STOP 후 재확인에서 STM 준비 조건이 맞지 않아 발사를 차단했습니다.",
    "INP_MISSING_IN_STATUS": "STM STATUS 응답에 INP 값이 없습니다. 펌웨어와 통신 형식을 확인하십시오.",
    "INP_NOT_READY": "스텝 드라이버 INP가 미도달 상태입니다. 포신이 목표 위치에 도달했는지 확인하십시오.",
    "M0RUN_NOT_IDLE": "스텝모터가 아직 구동 중입니다. 기구 걸림 또는 이동 완료 지연을 확인하십시오.",
    "STATUS_SEND_FAILED": "STM STATUS 명령 전송에 실패했습니다.",
    "STM_REJECTED": "STM이 명령을 오류로 거부했습니다.",
    "HOME0_ACK_FAILED": "STM에서 HOME0 수신 확인 응답을 받지 못했습니다.",
    "HOME0_DONE_FAILED": "STM에서 HOME0 완료 응답을 받지 못했습니다.",
    "HOME0_STATUS_FAILED": "HOME0 후 STM 상태 조회에 실패했습니다.",
    "HOME0_INVALID_STATUS": "HOME0 후 POS·HOMED·HOME45 상태가 정상 홈 조건과 맞지 않습니다.",
    "HOME0_SEND_FAILED": "STM으로 HOME0 명령을 전송하지 못했습니다.",
    "HOME_BEFORE_AIM_FAILED": "낮은 각도로 재접근하기 위한 조준 전 재홈에 실패했습니다.",
    "FIRE_STATUS_FAILED": "발사 전 STM 상태 조회에 실패했습니다.",
    "WORKER_EXCEPTION": "터렛 작업 스레드에서 예외가 발생했습니다.",
}

_COMMAND_KO = {
    "AIM_REQUEST_ACCEPTED": "포신 조준 요청 접수",
    "AIM_AFTER_HOME": "재홈 후 포신 조준",
    "AIM_COMPLETE": "포신 조준 완료",
    "AIM_COMPLETE_NO_MOVE": "현재 각도가 목표각과 같아 이동 없이 조준 완료",
    "HOME_REQUEST_ACCEPTED": "홈 복귀 요청 접수",
    "HOME_QUEUED": "홈 복귀 대기열 등록",
    "HOME_STARTED": "홈 복귀 시작",
    "HOME_COMPLETE": "홈 복귀 완료",
    "HOME_ABORTED": "홈 복귀 중단",
    "HOME_BEFORE_AIM": "조준 전 재홈 실행",
    "STOP": "모터 정지 명령",
    "FIRE_READY_VERIFY": "발사 직전 준비 상태 확인",
    "FIRE_REQUEST_ACCEPTED": "발사 요청 접수",
    "FIRE_COMPLETE": "발사 시퀀스 완료",
    "FIRE_ABORTED": "발사 시퀀스 중단",
    "FIRE_BLOCKED": "발사 차단",
    "MOTION_ABORTED": "포신 동작 중단",
    "ESTOP_REQUESTED": "비상정지 요청",
    "ESTOP_RESET": "비상정지 해제",
    "ESTOP_RESET_QUEUED": "비상정지 해제 대기열 등록",
    "AUTO_HOME_AFTER_ESTOP_RESET": "비상정지 해제 후 자동 홈 복귀",
    "AUTO_HOME_AFTER_ESTOP_RESET_QUEUED": "비상정지 해제 후 자동 홈 복귀 대기열 등록",
    "ESTOP_STOP_SENT": "비상정지 STOP 전송",
    "STARTUP_HOME_DISABLED": "시작 시 자동 홈 비활성화",
}

_STATUS_KO = {
    "BUFFER_UPDATED": "새 표적 버퍼 갱신",
    "BUFFER_PENDING": "표적 처리 대기",
    "FIRED": "발사 완료",
    "STOP_UART_FAILED": "STOP 통신 실패",
    "FIRE_UART_FAILED": "FIRE 통신 실패",
    "TARGET_POINT_MISSING": "표적 좌표 없음",
    "TARGET_POINT_STALE": "표적 좌표 갱신 끊김",
    "TARGET_POINT_INVALID": "표적 좌표 값이 유효하지 않음",
    "TARGET_POINT_PARSE_ERROR": "표적 좌표 메시지 형식 오류",
    "TARGET_DEM_INTERSECTION_FAILED": "표적 지형 고도 계산 실패",
    "TARGET_PROJECTION_FAILED": "표적 좌표 투영 계산 실패",
    "TARGET_SYNC_FAILED": "표적 좌표 동기화 실패",
    "TARGET_SYNC_ERROR_TOO_LARGE": "표적 좌표 동기화 오차 과다",
    "MMR_FIX_MISSING": "MMR GNSS 위치 없음",
    "MMR_FIX_STALE": "MMR GNSS 위치 갱신 끊김",
    "MMR_FIX_INVALID": "MMR GNSS 위치 무효",
    "HEADING_MISSING": "MMR 헤딩 없음",
    "HEADING_STALE": "MMR 헤딩 갱신 끊김",
    "HEADING_INVALID": "MMR 헤딩 값 무효",
    "INTERNAL_INPUT_MISSING": "탄도 계산 필수 입력 누락",
    "TARGET_NOT_HIT": "DEM 역투영 결과가 표적 지형과 교차하지 않음",
    "CANCELLED": "계산 취소",
    "CLEARED": "상태 초기화",
}


def _ko_engagement_state(value) -> str:
    code = str(value or "UNKNOWN").upper()
    return _ENGAGEMENT_STATE_KO.get(code, f"미등록 상태({code})")


def _ko_turret_state(value) -> str:
    code = str(value or "UNKNOWN").upper()
    return _TURRET_STATE_KO.get(code, f"미등록 터렛 상태({code})")


def _ko_command(value) -> str:
    code = str(value or "").strip()
    if not code:
        return "명령 기록 없음"
    return _COMMAND_KO.get(code.upper(), f"{code}")


def _ko_status(value) -> str:
    code = str(value or "").strip()
    if not code:
        return "상태 정보 없음"
    return _STATUS_KO.get(code.upper(), _REASON_KO.get(code.upper(), code))


def _ko_reason(value) -> str:
    """영문 상태/복합 오류 코드를 야외 운용용 한글 원인 설명으로 변환한다."""
    raw = str(value or "").strip()
    if not raw:
        return "특이사항 없음"

    upper = raw.upper()
    if upper in _REASON_KO:
        return _REASON_KO[upper]

    # STM 복합 오류는 ':'로 원인이 연쇄 연결된다.
    # 예: HOME_BEFORE_AIM_FAILED:HOME0_ACK_FAILED:STM_REJECTED:ERR HOME0
    parts = [part.strip() for part in raw.split(":") if part.strip()]
    translated = []
    for part in parts:
        key = part.upper()
        if key in _REASON_KO:
            text = _REASON_KO[key]
        elif key.startswith("ERR HOME0"):
            text = "STM이 HOME0 명령을 거부했습니다. 홈 센서·모터 정지 상태·펌웨어 재홈 처리를 확인하십시오."
        elif key.startswith("ERR "):
            text = f"STM 오류 응답({part})"
        elif key.startswith("LAST_STATUS="):
            text = f"마지막 STM 상태: {part.split('=', 1)[1]}"
        elif key.startswith("POS=") or key.startswith("HOMED=") or key.startswith("HOME45=") or key.startswith("M0RUN=") or key.startswith("INP="):
            text = f"STM 상태값 {part}"
        elif key.startswith("UART_READ_FAILED"):
            text = "STM UART 수신 중 오류가 발생했습니다."
        elif key.startswith("STM_REJECTED"):
            text = _REASON_KO["STM_REJECTED"]
        elif key.startswith("HOME0_ACK_FAILED"):
            text = _REASON_KO["HOME0_ACK_FAILED"]
        elif key.startswith("HOME0_DONE_FAILED"):
            text = _REASON_KO["HOME0_DONE_FAILED"]
        elif key.startswith("HOME0_STATUS_FAILED"):
            text = _REASON_KO["HOME0_STATUS_FAILED"]
        elif key.startswith("HOME0_INVALID_STATUS"):
            text = _REASON_KO["HOME0_INVALID_STATUS"]
        elif key.startswith("FIRE_STATUS_FAILED"):
            text = _REASON_KO["FIRE_STATUS_FAILED"]
        elif key.startswith("STATUS_SEND_FAILED"):
            text = _REASON_KO["STATUS_SEND_FAILED"]
        elif key.startswith("INP_MISSING_IN_STATUS"):
            text = _REASON_KO["INP_MISSING_IN_STATUS"]
        elif key.startswith("INP_NOT_READY"):
            text = _REASON_KO["INP_NOT_READY"]
        elif key.startswith("M0RUN_NOT_IDLE"):
            text = _REASON_KO["M0RUN_NOT_IDLE"]
        elif "TIMEOUT" in key:
            text = f"응답 또는 동작 대기 시간이 초과되었습니다({part})."
        else:
            text = f"미등록 세부 코드({part})"
        if text not in translated:
            translated.append(text)

    if translated:
        return " → ".join(translated)
    return f"미등록 오류 코드({raw})"


# ---------- MapBridge ----------
class MapBridge(QObject):
    map_clicked = pyqtSignal(float, float)
    map_right_clicked = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def onMapClick(self, lat: float, lon: float):
        self.map_clicked.emit(lat, lon)

    @pyqtSlot(float, float)
    def onMapRightClick(self, lat: float, lon: float):
        self.map_right_clicked.emit(lat, lon)


# ---------- HoldProgressRing ----------
class HoldProgressRing(QWidget):
    """ALIGN/AIM 및 FIRE Hold 진행률을 원형 게이지로 표시한다."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._progress = 0.0
        self._mode = "IDLE"
        self._label = "준비"
        self.setFixedSize(62, 62)
        self.setToolTip("정렬·조준·발사 홀드 진행률")

    def set_status(self, progress: float, mode: str, label: str):
        try:
            value = float(progress)
        except Exception:
            value = 0.0
        next_progress = max(0.0, min(1.0, value))
        next_mode = str(mode or "IDLE").upper()
        next_label = str(label or "")

        # 20Hz 상태 토픽이 같은 내용을 반복해도 불필요한 repaint를 하지
        # 않는다.  서로 다른 토픽이 같은 위젯을 번갈아 갱신할 때 보이던
        # 깜빡임도 함께 줄어든다.
        if (
            abs(next_progress - self._progress) < 0.002
            and next_mode == self._mode
            and next_label == self._label
        ):
            return

        self._progress = next_progress
        self._mode = next_mode
        self._label = next_label
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        margin = 7
        rect = self.rect().adjusted(margin, margin, -margin, -margin)

        painter.setPen(QPen(QColor("#263238"), 6, Qt.PenStyle.SolidLine,
                            Qt.PenCapStyle.RoundCap))
        painter.drawArc(rect, 90 * 16, -360 * 16)

        color = {
            "ALIGN": QColor("#29b6f6"),
            "AIM": QColor("#00e5ff"),
            "FIRE": QColor("#ff5252"),
            "DONE": QColor("#00e676"),
            "FAULT": QColor("#ff1744"),
        }.get(self._mode, QColor("#607d8b"))

        painter.setPen(QPen(color, 6, Qt.PenStyle.SolidLine,
                            Qt.PenCapStyle.RoundCap))
        span = -int(360 * 16 * self._progress)
        if span:
            painter.drawArc(rect, 90 * 16, span)

        painter.setPen(QColor("#eceff1"))
        painter.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._label)


# ---------- MapView ----------
class MapView(QFrame):
    dem_clicked = pyqtSignal(float, float, object)
    test_target_right_clicked = pyqtSignal(float, float, object)

    def __init__(self, dem_path: str = DEFAULT_DEM_PATH):
        super().__init__()
        self.setStyleSheet("""
            QFrame { background-color: #101317; border: 2px solid #444; border-radius: 8px; }
        """)
        self.dem_path = dem_path
        self.dem_ds = None
        self.dem_band = None
        self.to_crs = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(8)

        self.title = QLabel("DEM 지도")
        self.title.setStyleSheet("color:#cfd8dc;")
        self.title.setFont(QFont("Arial", 10, QFont.Weight.Bold))

        self.web = QWebEngineView()
        self.web.setStyleSheet("border:2px solid #333;border-radius:8px;")

        lay.addWidget(self.title)
        lay.addWidget(self.web, 1)

        self.bridge = MapBridge()
        self.bridge.map_clicked.connect(self._on_map_click)
        self.bridge.map_right_clicked.connect(self._on_map_right_click)

        self.channel = QWebChannel()
        self.channel.registerObject("bridge", self.bridge)
        self.web.page().setWebChannel(self.channel)

        self._html_ready = False
        self._load_dem()
        self._load_html()
        self.web.loadFinished.connect(self._on_load_finished)

    def _on_load_finished(self, ok: bool):
        self._html_ready = ok
        if not ok:
            self.title.setText("지도 화면 로드 실패")

    def _run_js(self, js: str):
        """HTML 로드 완료 후에만 JS 실행. 미완료 시 무시(ReferenceError 방지)."""
        if self._html_ready:
            self.web.page().runJavaScript(js)

    def update_mmr(self, lat: float, lon: float, heading_deg: float):
        self._run_js(f"updateMmr({lat}, {lon}, {heading_deg});")
        self.title.setText(f"지도  ●MMR {lat:.7f}, {lon:.7f}  헤딩={heading_deg:.1f}°")

    def update_drone_target(
        self,
        lat: float,
        lon: float,
        info: str = "",
        radius_m: float = DEFAULT_IMPACT_THRESHOLD_M,
    ):
        """표적 마커와 유효 타격 반경을 지도에 표시."""
        info_json = json.dumps(str(info), ensure_ascii=False)
        self._run_js(
            f"updateDroneTarget({float(lat)}, {float(lon)}, {info_json}, "
            f"{float(radius_m)});"
        )

    def clear_drone_target(self):
        """표적 마커와 유효 타격 반경 제거."""
        self._run_js("clearDroneTarget();")

    def show_impact_result(
        self,
        impact_id: int,
        lat: float,
        lon: float,
        target_lat: float,
        target_lon: float,
        error_m: float,
        result: str,
        info_html: str,
    ):
        """탄착 마커, 표적-탄착 편차선, 거리 라벨을 지도에 표시."""
        result_json = json.dumps(str(result), ensure_ascii=False)
        info_json = json.dumps(str(info_html), ensure_ascii=False)
        self._run_js(
            "showImpactResult("
            f"{int(impact_id)}, {float(lat)}, {float(lon)}, "
            f"{float(target_lat)}, {float(target_lon)}, {float(error_m)}, "
            f"{result_json}, {info_json});"
        )

    def clear_impact_results(self):
        """지도에서 탄착 마커와 편차선을 모두 제거."""
        self._run_js("clearImpactResults();")

    def _load_dem(self):
        if not _RASTERIO_OK:
            self.title.setText("rasterio 없음 — DEM 비활성")
            return
        if not os.path.exists(self.dem_path):
            self.title.setText(f"DEM 파일 없음: {self.dem_path}")
            return
        try:
            self.dem_ds = rasterio.open(self.dem_path)
            self.dem_band = self.dem_ds.read(1)
            if str(self.dem_ds.crs) != "EPSG:4326":
                self.to_crs = Transformer.from_crs(
                    "EPSG:4326", self.dem_ds.crs, always_xy=True)
        except Exception as e:
            self.title.setText(f"DEM 로드 실패: {e}")

    def _load_html(self):
        html = """
<!DOCTYPE html><html><head>
  <meta charset="utf-8"/>
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
  <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
  <style>html,body,#map{width:100%;height:100%;margin:0;padding:0;background:#000;}</style>
</head><body>
<div id="map"></div>
<script>
  var map = L.map('map', {
    minZoom: 3,
    maxZoom: 22,
    zoomSnap: 0.25,
    zoomDelta: 0.5,
    wheelPxPerZoomLevel: 120
  }).setView([37.341, 126.732], 18);

  L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxNativeZoom: 19,
    maxZoom: 22,
    attribution: 'Tiles © Esri'
  }).addTo(map);

  L.tileLayer('https://services.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}', {
    maxNativeZoom: 19,
    maxZoom: 22,
    attribution: 'Labels © Esri'
  }).addTo(map);

  var clickMarker=null, mmrMarker=null, mmrHeadingLine=null, droneTargetMarker=null;
  var targetRadiusCircle=null;
  var impactLayers=[];

  function makeVehicleIcon(headingDeg, fillColor, strokeColor, centerColor) {
    var svg='<svg xmlns="http://www.w3.org/2000/svg" width="36" height="36" viewBox="-18 -18 36 36">'+
      '<g transform="rotate('+headingDeg+')">'+
      '<polygon points="0,-13 8,9 0,4 -8,9" fill="'+fillColor+'" stroke="'+strokeColor+'" stroke-width="1.5" opacity="0.93"/>'+
      '<circle cx="0" cy="0" r="3" fill="'+centerColor+'" opacity="0.9"/>'+
      '</g></svg>';
    return L.divIcon({html:svg,className:'',iconSize:[36,36],iconAnchor:[18,18]});
  }

  window.updateMmr = function(lat, lon, headingDeg) {
    var ll=[lat,lon];
    if (!mmrMarker) {
      mmrMarker=L.marker(ll,{icon:makeVehicleIcon(headingDeg,'#00e5ff','#004d60','#ffffff'),zIndexOffset:1000}).addTo(map);
      mmrMarker.bindPopup('');
      map.setView(ll, map.getZoom());
    } else {
      mmrMarker.setLatLng(ll);
      mmrMarker.setIcon(makeVehicleIcon(headingDeg,'#00e5ff','#004d60','#ffffff'));
    }
    mmrMarker.getPopup().setContent('<b style="color:#00e5ff">MMR</b><br>위도:'+lat.toFixed(7)+'<br>경도:'+lon.toFixed(7)+'<br>헤딩:'+headingDeg.toFixed(1)+'°');
    if (mmrHeadingLine) { map.removeLayer(mmrHeadingLine); mmrHeadingLine=null; }
  };

  window.showResult = function(lat, lon, text) {
    if (clickMarker) clickMarker.setLatLng([lat,lon]).bindPopup(text).openPopup();
    else { clickMarker=L.marker([lat,lon]).addTo(map); clickMarker.bindPopup(text).openPopup(); }
  };

  function makeDroneTargetIcon() {
    var svg='<svg xmlns="http://www.w3.org/2000/svg" width="28" height="28" viewBox="-14 -14 28 28">'+
      '<circle cx="0" cy="0" r="9" fill="none" stroke="#ff1744" stroke-width="2" opacity="0.9"/>'+
      '<line x1="-13" y1="0" x2="13" y2="0" stroke="#ff1744" stroke-width="2" opacity="0.9"/>'+
      '<line x1="0" y1="-13" x2="0" y2="13" stroke="#ff1744" stroke-width="2" opacity="0.9"/>'+
      '<circle cx="0" cy="0" r="2.5" fill="#ff1744" opacity="1.0"/>'+
      '</svg>';
    return L.divIcon({html:svg, className:'', iconSize:[28,28], iconAnchor:[14,14]});
  }

  function makeImpactIcon(impactId, result) {
    var good = String(result || '').toUpperCase() === 'SUCCESS';
    var color = good ? '#00e676' : '#ff5252';
    var text = String(impactId || '');
    var html = '<div style="width:28px;height:28px;border-radius:50%;' +
      'background:rgba(10,15,20,0.92);border:3px solid ' + color + ';' +
      'color:#fff;font:bold 12px Arial;line-height:22px;text-align:center;' +
      'box-shadow:0 0 5px rgba(0,0,0,0.8);">' + text + '</div>';
    return L.divIcon({html:html,className:'',iconSize:[28,28],iconAnchor:[14,14]});
  }

  window.updateDroneTarget = function(lat, lon, info, radiusM) {
    var ll = [lat, lon];
    var radius = Number(radiusM);
    if (!isFinite(radius) || radius <= 0) radius = 15.0;
    if (!droneTargetMarker) {
      droneTargetMarker = L.marker(ll, {icon: makeDroneTargetIcon(), zIndexOffset: 900}).addTo(map);
      droneTargetMarker.bindPopup('');
    } else {
      droneTargetMarker.setLatLng(ll);
    }
    var popupHtml = '<b style="color:#ff1744">표적 좌표</b><br>lat:'+lat.toFixed(7)+'<br>lon:'+lon.toFixed(7);
    if (info) popupHtml += '<br><span style="color:#aaa">'+info+'</span>';
    popupHtml += '<br><span style="color:#00e676">유효 타격 반경: '+radius.toFixed(1)+' m</span>';
    droneTargetMarker.getPopup().setContent(popupHtml);

    if (!targetRadiusCircle) {
      targetRadiusCircle = L.circle(ll, {
        radius: radius,
        color: '#00e676',
        weight: 2.5,
        opacity: 0.95,
        fillColor: '#00e676',
        fillOpacity: 0.10,
        dashArray: '7, 5',
        interactive: false
      }).addTo(map);
    } else {
      targetRadiusCircle.setLatLng(ll);
      targetRadiusCircle.setRadius(radius);
    }
  };

  window.clearDroneTarget = function() {
    if (droneTargetMarker) { map.removeLayer(droneTargetMarker); droneTargetMarker = null; }
    if (targetRadiusCircle) { map.removeLayer(targetRadiusCircle); targetRadiusCircle = null; }
  };

  window.clearImpactResults = function() {
    for (var i = 0; i < impactLayers.length; i++) {
      var item = impactLayers[i];
      if (item.marker) map.removeLayer(item.marker);
      if (item.line) map.removeLayer(item.line);
      if (item.label) map.removeLayer(item.label);
    }
    impactLayers = [];
  };

  window.showImpactResult = function(impactId, lat, lon, targetLat, targetLon, errorM, result, text) {
    var good = String(result || '').toUpperCase() === 'SUCCESS';
    var lineColor = good ? '#00e676' : '#ff5252';

    var marker = L.marker([lat, lon], {
      icon: makeImpactIcon(impactId, result),
      zIndexOffset: 1500 + Number(impactId || 0)
    }).addTo(map);
    marker.bindPopup(text);

    var line = L.polyline([[targetLat, targetLon], [lat, lon]], {
      color: lineColor,
      weight: 3,
      opacity: 0.95,
      dashArray: '8, 6',
      interactive: false
    }).addTo(map);

    var midLat = (Number(targetLat) + Number(lat)) / 2.0;
    var midLon = (Number(targetLon) + Number(lon)) / 2.0;
    var labelHtml = '<div style="white-space:nowrap;padding:2px 6px;border-radius:5px;' +
      'background:rgba(10,15,20,0.88);border:1px solid ' + lineColor + ';' +
      'color:#fff;font:bold 12px Arial;">#' + impactId + ' ' +
      Number(errorM).toFixed(1) + ' m</div>';
    var label = L.marker([midLat, midLon], {
      icon: L.divIcon({html:labelHtml,className:'',iconSize:[100,24],iconAnchor:[50,12]}),
      interactive: false,
      zIndexOffset: 1600 + Number(impactId || 0)
    }).addTo(map);

    impactLayers.push({marker:marker,line:line,label:label});
  };

  var _rangeSector = null;

  function makeSectorLatLngs(lat, lon, rInner, rOuter, headingDeg, halfAngleDeg, steps) {
    steps = steps || 48;
    var pts = [];
    var latRad = lat * Math.PI / 180.0;
    var mPerLat = 111320.0;
    var mPerLon = 111320.0 * Math.cos(latRad);

    var startAngle = headingDeg - halfAngleDeg;
    var endAngle   = headingDeg + halfAngleDeg;

    for (var i = 0; i <= steps; i++) {
      var a = (startAngle + (endAngle - startAngle) * i / steps) * Math.PI / 180.0;
      pts.push([lat + rOuter * Math.cos(a) / mPerLat,
                lon + rOuter * Math.sin(a) / mPerLon]);
    }

    for (var j = steps; j >= 0; j--) {
      var b = (startAngle + (endAngle - startAngle) * j / steps) * Math.PI / 180.0;
      pts.push([lat + rInner * Math.cos(b) / mPerLat,
                lon + rInner * Math.sin(b) / mPerLon]);
    }
    return pts;
  }

  window.drawRangeRings = function(data, mmrLat, mmrLng, mmrHdg) {
    if (_rangeSector) { map.removeLayer(_rangeSector); _rangeSector = null; }
    if (!data || mmrLat === undefined || mmrLng === undefined) return;

    var rMax      = data.range_max;
    var rMin      = data.range_min;
    var halfAngle = data.pan_half_angle_deg !== undefined ? data.pan_half_angle_deg : 45.0;
    var hdg       = (mmrHdg !== undefined && mmrHdg !== null) ? mmrHdg : 0.0;

    var pts = makeSectorLatLngs(mmrLat, mmrLng, rMin, rMax, hdg, halfAngle, 60);

    _rangeSector = L.polygon(pts, {
      color:       '#00e676',
      weight:      1.5,
      fillColor:   '#00e676',
      fillOpacity: 0.10,
      opacity:     0.7,
      dashArray:   null,
    }).addTo(map);
  };

  new QWebChannel(qt.webChannelTransport, function(channel) { window.bridge=channel.objects.bridge; });
  map.on('click', function(e) {
    if (clickMarker) map.removeLayer(clickMarker);
    clickMarker=L.marker([e.latlng.lat,e.latlng.lng]).addTo(map);
    if (window.bridge) window.bridge.onMapClick(e.latlng.lat,e.latlng.lng);
  });

  map.on('contextmenu', function(e) {
    if (window.bridge) window.bridge.onMapRightClick(e.latlng.lat, e.latlng.lng);
  });
</script></body></html>"""
        self.web.setHtml(html)

    def _lookup_dem_z(self, lat: float, lon: float):
        """지도 좌표의 DEM 고도값을 읽는다. 실패 시 None 반환."""
        if self.dem_ds is None or self.dem_band is None:
            return None
        try:
            if self.to_crs is not None:
                x, y = self.to_crs.transform(lon, lat)
            else:
                x, y = lon, lat
            row, col = self.dem_ds.index(x, y)
        except Exception:
            return None

        if not (0 <= row < self.dem_band.shape[0] and 0 <= col < self.dem_band.shape[1]):
            return None

        try:
            z = float(self.dem_band[row, col])
        except Exception:
            return None

        nodata = self.dem_ds.nodata
        if (nodata is not None and z == nodata) or math.isnan(z):
            return None

        return z

    def _on_map_right_click(self, lat: float, lon: float):
        """우클릭 좌표를 GUI 테스트 표적 후보로 전달한다."""
        z = self._lookup_dem_z(lat, lon)
        self.test_target_right_clicked.emit(lat, lon, z)

    def _on_map_click(self, lat: float, lon: float):
        if self.dem_ds is None or self.dem_band is None:
            self.dem_clicked.emit(lat, lon, None)
            self._run_js(f"showResult({lat},{lon},'DEM not loaded');")
            return
        try:
            if self.to_crs is not None:
                x, y = self.to_crs.transform(lon, lat)
            else:
                x, y = lon, lat
            row, col = self.dem_ds.index(x, y)
        except Exception:
            self.dem_clicked.emit(lat, lon, None)
            self._run_js(f"showResult({lat},{lon},'좌표 변환 실패');")
            return
        if not (0 <= row < self.dem_band.shape[0] and 0 <= col < self.dem_band.shape[1]):
            self.dem_clicked.emit(lat, lon, None)
            self._run_js(f"showResult({lat},{lon},'DEM 범위 밖');")
            return
        z = float(self.dem_band[row, col])
        nodata = self.dem_ds.nodata
        if (nodata is not None and z == nodata) or (math.isnan(z)):
            self.dem_clicked.emit(lat, lon, None)
            self._run_js(f"showResult({lat},{lon},'NoData');")
            return
        self.title.setText(f"지도 - 위도={lat:.7f}, 경도={lon:.7f}, 고도={z:.3f}m")
        self.dem_clicked.emit(lat, lon, z)
        self._run_js(f"showResult({lat},{lon},'고도: {z:.3f} m');")


# ---------- AttitudeIndicator ----------
class AttitudeIndicator(QFrame):
    def __init__(self):
        super().__init__()
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.setMinimumSize(220, 210)
        self.setStyleSheet("QFrame { background-color: #101317; border: 2px solid #444; border-radius: 8px; }")

    def set_attitude(self, roll_deg, pitch_deg):
        self.roll_deg = 0.0 if roll_deg is None else float(roll_deg)
        self.pitch_deg = 0.0 if pitch_deg is None else float(pitch_deg)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = self.width()
        h = self.height()
        cx = w / 2
        cy = h / 2
        r = min(w, h) * 0.42

        painter.save()
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#0b0f14"))
        painter.drawEllipse(QPointF(cx, cy), r, r)
        painter.restore()

        painter.save()
        painter.setClipRect(self.rect())
        painter.translate(cx, cy)
        painter.rotate(-self.roll_deg)
        pitch_px = r / 30.0
        po = self.pitch_deg * pitch_px
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#2d6cdf"))
        painter.drawRect(int(-2 * r), int(-2 * r + po), int(4 * r), int(2 * r))
        painter.setBrush(QColor("#7a4b2a"))
        painter.drawRect(int(-2 * r), int(po), int(4 * r), int(2 * r))
        painter.setPen(QPen(QColor("white"), 2))
        painter.drawLine(QPointF(-2 * r, po), QPointF(2 * r, po))
        painter.restore()

        painter.setPen(QPen(QColor("#cfcfcf"), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), r, r)
        painter.setPen(QPen(QColor("#ffd54a"), 3))
        wing = r * 0.42
        gap = r * 0.10
        painter.drawLine(QPointF(cx - wing, cy), QPointF(cx - gap, cy))
        painter.drawLine(QPointF(cx + gap, cy), QPointF(cx + wing, cy))
        painter.drawLine(QPointF(cx, cy - 8), QPointF(cx, cy + 8))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawLine(QPointF(cx, cy - r - 6), QPointF(cx, cy - r + 10))
        painter.setPen(QColor("#dcdcdc"))
        painter.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        painter.drawText(12, 20, f"롤   {self.roll_deg:+.1f}°")
        painter.drawText(12, 38, f"피치 {self.pitch_deg:+.1f}°")


# ---------- MainWindow ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MMR 운용 GUI")
        self.resize(1280, 800)

        # ── 내부 상태를 UI 생성보다 먼저 초기화 ─────────────────────
        # _build_hud_panel()은 생성 과정에서 버튼 스타일 갱신 함수를 즉시
        # 호출한다. 따라서 GUI 캐시와 입력 상태를 위젯 생성 전에 모두
        # 준비해 초기화 순서에 따른 AttributeError를 원천 차단한다.
        self._last_status_text = None

        # 표적 좌표 입력 모드
        self._gui_target_test_mode: bool = False
        self._test_target_seq: int = 0

        # ROS2 연결 및 publisher 기본값
        self._ros_enabled = False
        self._ros_node = None
        self._ros_executor = None
        self._ros_timer = None
        self._pub_fire_arm = None
        self._pub_fire_confirm = None
        self._pub_operator_input = None
        self._pub_turret_home_request = None
        self._pub_emergency_stop = None
        self._pub_target_point = None

        # 최신 ROS 데이터 캐시
        self._latest_mmr_data = None
        self._latest_tilt_status_data = None
        self._latest_fire_ready_data = None
        self._latest_fire_sol_status_data = None
        self._latest_fire_solution_data = None
        self._latest_target_point_data = None
        self._pending_impact_points = deque()
        self._latest_engagement_data = None
        self._latest_turret_status_data = None
        self._latest_drive_status_data = None
        self._latest_range_info_data = None
        self._latest_mmr_pose = None
        self._prev_tilt_ok = None
        # 이동·진동 때문에 stationary 판정만 false가 된 상태인지 구분한다.
        # 안전 토픽은 그대로 유지하고 GUI 경고 로그만 선택적으로 억제한다.
        self._tilt_motion_only_invalid_ui: bool = False
        self._tilt_motion_log_suppressed_ui: bool = False

        # AimCoordinator GUI 입력 및 상태 캐시
        self._gui_align_aim_pressed: bool = False
        self._gui_fire_pressed: bool = False
        self._last_engagement_state: str = ""
        self._last_engagement_reason: str = ""
        self._awaiting_reengagement_ui: bool = False
        self._last_turret_state: str = ""
        self._last_turret_error: str = ""
        self._last_turret_estop: bool = False
        self._estop_latched_ui: bool = False

        # 구동 상태 표시 캐시. 수동 주행 중에는 차체 정렬 미완료 경고를
        # 숨기되 자동 정렬 중 상태 표시는 그대로 유지한다.
        self._drive_moving_ui: bool = False
        self._drive_any_moving_ui: bool = False
        self._drive_auto_align_ui: bool = False
        self._drive_mode_ui: str = "STOPPED"
        self._drive_status_stamp: float = 0.0

        # 상태 표시 안정화
        self._engagement_status_stamp: float = 0.0
        self._engagement_authority_sec: float = 1.0
        self._last_fire_status_key: str = ""
        self._last_operator_style_key: str = ""
        self._fired_banner_until: float = 0.0
        self._fired_banner_duration_sec: float = 1.5
        self._gui_armed: bool = False

        # 중복 표시·로그 방지 캐시
        self._last_fire_sol_status_logged: str = ""
        self._last_buffer_pending_key: str = ""
        self._last_solution_label_key: str = ""
        self._last_target_point_label_key: str = ""
        self._last_fire_ready_event_key: str = ""
        self._local_diag_event_path: str = DEFAULT_DIAGNOSTIC_EVENT_PATH
        self._last_local_diag_event_seq: int = 0

        # GCS 표적/탄착 표시 상태. 표적은 발사 완료 후에도 유지하며,
        # 새 target_id 수신 또는 명시적 표적 해제 때만 탄착 이력을 초기화한다.
        self._impact_threshold_m: float = DEFAULT_IMPACT_THRESHOLD_M
        self._impact_history_max_count: int = DEFAULT_IMPACT_HISTORY_MAX_COUNT
        self._require_target_sync_ok_ui: bool = True
        self._require_projection_ok_ui: bool = True
        self._require_dem_intersection_ok_ui: bool = True
        self._reject_stale_impact_ui: bool = True
        self._retained_target_data: dict | None = None
        self._retained_target_key: str = ""
        self._target_received_ns: int = 0
        self._impact_history: list[dict] = []
        self._impact_counter: int = 0
        self._impact_reset_ns: int = 0
        self._impact_seen_keys: set[tuple] = set()

        # GNSS 직접 구독 캐시
        self._direct_fix_lat: float | None = None
        self._direct_fix_lon: float | None = None
        self._direct_fix_qual: str = "NO_FIX"
        self._direct_fix_source: str = "NONE"
        self._direct_heading_deg: float = 0.0
        self._direct_fix_stamp: float = 0.0
        self._direct_hdg_stamp: float = 0.0
        self._mmr_status_stamp: float = 0.0
        self._gnss_warn_logged: bool = False

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(0)
        self.setCentralWidget(central)

        self.vsplit = QSplitter(Qt.Orientation.Vertical)
        self.vsplit.setHandleWidth(6)
        root.addWidget(self.vsplit)

        # ── 지도 ──────────────────────────────────────────────────────
        self.map_view = MapView()
        self.map_view.dem_clicked.connect(self._on_dem_click)
        self.map_view.test_target_right_clicked.connect(self._on_test_target_right_click)
        self.vsplit.addWidget(self.map_view)

        # ── 하단 패널 ─────────────────────────────────────────────────
        self.bottom_panel = QWidget()
        self.bottom_panel.setMinimumHeight(205)
        self.bottom_panel.setMaximumHeight(260)
        bottom = QHBoxLayout(self.bottom_panel)
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.setSpacing(5)

        self.hud = self._build_hud_panel()
        bottom.addWidget(self.hud, 7)

        self.log = self._build_log_panel()
        self.log.setMinimumWidth(250)
        self.log.setFont(QFont("Consolas", 9))
        bottom.addWidget(self.log, 2)

        self.att_indicator = AttitudeIndicator()
        self.att_indicator.setMinimumWidth(190)
        bottom.addWidget(self.att_indicator, 0)

        self.vsplit.addWidget(self.bottom_panel)

        self._build_menubar()
        QTimer.singleShot(0, self.set_vertical_split)

        # 내부 상태는 위젯 생성 전에 초기화되어 있다.

        self._setup_ros()
        self._initialize_local_diagnostic_cursor()

        self._mmr_timer = QTimer(self)
        self._mmr_timer.timeout.connect(self._flush_mmr_latest)
        self._mmr_timer.start(100)

        self._tilt_timer = QTimer(self)
        self._tilt_timer.timeout.connect(self._flush_tilt_status_latest)
        self._tilt_timer.start(200)

        self._fire_ready_timer = QTimer(self)
        self._fire_ready_timer.timeout.connect(self._flush_fire_ready_latest)
        self._fire_ready_timer.start(100)

        self._range_info_timer = QTimer(self)
        self._range_info_timer.timeout.connect(self._flush_range_info_latest)
        self._range_info_timer.start(500)

        self._fire_sol_status_timer = QTimer(self)
        self._fire_sol_status_timer.timeout.connect(self._flush_fire_sol_status_latest)
        self._fire_sol_status_timer.start(200)

        self._fire_solution_timer = QTimer(self)
        self._fire_solution_timer.timeout.connect(self._flush_fire_solution_latest)
        self._fire_solution_timer.start(150)

        self._target_point_timer = QTimer(self)
        self._target_point_timer.timeout.connect(self._flush_target_point_latest)
        self._target_point_timer.start(150)

        self._impact_point_timer = QTimer(self)
        self._impact_point_timer.timeout.connect(self._flush_impact_points)
        self._impact_point_timer.start(100)

        # AimCoordinator 상태 및 GUI Hold 입력 heartbeat
        self._engagement_timer = QTimer(self)
        self._engagement_timer.timeout.connect(self._flush_engagement_latest)
        self._engagement_timer.start(50)

        # turret_control_node의 STM/INP 상태 표시용
        self._turret_status_timer = QTimer(self)
        self._turret_status_timer.timeout.connect(self._flush_turret_status_latest)
        self._turret_status_timer.start(100)

        # 구동 상태를 받아 수동 주행 중 불필요한 정렬 경고 표시를 억제한다.
        self._drive_status_timer = QTimer(self)
        self._drive_status_timer.timeout.connect(self._flush_drive_status_latest)
        self._drive_status_timer.start(100)

        # coordinator의 operator_input timeout(기본 0.5초)보다 충분히 빠르게
        # Hold 상태를 20Hz로 재발행한다.
        self._operator_input_timer = QTimer(self)
        self._operator_input_timer.timeout.connect(self._publish_operator_input_heartbeat)
        self._operator_input_timer.start(50)

        # GNSS 직접 구독 폴백 타이머
        self._gnss_direct_timer = QTimer(self)
        self._gnss_direct_timer.timeout.connect(self._flush_gnss_direct)
        self._gnss_direct_timer.start(200)

        # ROS topic을 사용하지 않는 로컬 전용 진단 이벤트.
        self._local_diag_timer = QTimer(self)
        self._local_diag_timer.timeout.connect(self._flush_local_diagnostic_event)
        self._local_diag_timer.start(150)

    # ================================================================
    # ROS2 초기화
    # ================================================================
    def _setup_ros(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                QoSProfile,
                QoSReliabilityPolicy,
                QoSHistoryPolicy,
            )
            from std_msgs.msg import String, Float64
            from sensor_msgs.msg import NavSatFix
        except Exception as e:
            self._log(f"[시스템 오류] ROS2 초기화 실패: {e}")
            return

        if not rclpy.ok():
            try:
                rclpy.init(args=None)
            except Exception:
                pass

        self._ros_node = Node("mmr_gui")
        self._ros_node.declare_parameter(
            "impact_threshold_m", DEFAULT_IMPACT_THRESHOLD_M
        )
        self._ros_node.declare_parameter(
            "impact_history_max_count", DEFAULT_IMPACT_HISTORY_MAX_COUNT
        )
        self._ros_node.declare_parameter("require_target_sync_ok", True)
        self._ros_node.declare_parameter("require_projection_ok", True)
        self._ros_node.declare_parameter("require_dem_intersection_ok", True)
        self._ros_node.declare_parameter("reject_stale_impact", True)
        self._ros_node.declare_parameter("diagnostic_local_event_path", DEFAULT_DIAGNOSTIC_EVENT_PATH)

        self._impact_threshold_m = max(
            0.1,
            float(self._ros_node.get_parameter("impact_threshold_m").value),
        )
        self._impact_history_max_count = max(
            1,
            int(self._ros_node.get_parameter("impact_history_max_count").value),
        )
        self._require_target_sync_ok_ui = bool(
            self._ros_node.get_parameter("require_target_sync_ok").value
        )
        self._require_projection_ok_ui = bool(
            self._ros_node.get_parameter("require_projection_ok").value
        )
        self._require_dem_intersection_ok_ui = bool(
            self._ros_node.get_parameter("require_dem_intersection_ok").value
        )
        self._reject_stale_impact_ui = bool(
            self._ros_node.get_parameter("reject_stale_impact").value
        )
        self._local_diag_event_path = str(
            self._ros_node.get_parameter("diagnostic_local_event_path").value
            or DEFAULT_DIAGNOSTIC_EVENT_PATH
        )

        self._ros_executor = SingleThreadedExecutor()
        self._ros_executor.add_node(self._ros_node)

        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(
            lambda: self._ros_executor.spin_once(timeout_sec=0.0))
        self._ros_timer.start(15)

        self._ros_enabled = True

        # GUI 표시용 토픽은 과거 메시지보다 최신 상태 1개가 중요하므로
        # 구독 큐를 depth=1로 제한한다.
        qos_latest = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 탄착점은 개별 이벤트이므로 최신값 하나로 덮어쓰지 않고 짧은 큐를 유지한다.
        qos_events = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=max(10, self._impact_history_max_count),
        )

        # /mmr/status
        def _mmr_cb(msg):
            try:
                import time as _t
                self._latest_mmr_data = json.loads(msg.data)
                self._mmr_status_stamp = _t.monotonic()
            except Exception as e:
                self._log(f"[수신 오류] MMR 상태 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/mmr/status", _mmr_cb, qos_latest)

        # /imu/tilt_status
        def _tilt_cb(msg):
            try:
                self._latest_tilt_status_data = json.loads(msg.data)
            except Exception as e:
                self._log(f"[수신 오류] IMU 기울기 상태 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/imu/tilt_status", _tilt_cb, qos_latest)

        # /mmr/fire_ready
        def _fire_ready_cb(msg):
            try:
                self._latest_fire_ready_data = json.loads(msg.data)
            except Exception as e:
                self._log(f"[수신 오류] 발사 준비 상태 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/mmr/fire_ready", _fire_ready_cb, qos_latest)

        # /mmr/fire_solution/status
        def _fire_sol_cb(msg):
            try:
                raw = msg.data.strip()
                try:
                    self._latest_fire_sol_status_data = json.loads(raw)
                except json.JSONDecodeError:
                    self._latest_fire_sol_status_data = {"status": raw}
            except Exception as e:
                self._log(f"[수신 오류] 탄도해 상태 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/mmr/fire_solution/status", _fire_sol_cb, qos_latest)

        # /mmr/fire_solution — 실제 탄도해, 표적 좌표, revision 표시용
        def _fire_solution_cb(msg):
            try:
                self._latest_fire_solution_data = json.loads(msg.data)
            except Exception as e:
                self._log(f"[수신 오류] 탄도해 데이터 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/mmr/fire_solution", _fire_solution_cb, qos_latest)

        # /target/point — GCS 또는 GUI에서 들어온 원본 표적 좌표 즉시 표시용
        def _target_point_cb(msg):
            try:
                raw = msg.data.strip()
                self._latest_target_point_data = json.loads(raw)
            except Exception as e:
                self._log(f"[수신 오류] 표적 좌표 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/target/point", _target_point_cb, qos_latest)

        # /compute/impact_point — GCS 우클릭으로 계산된 실제 탄착 좌표 이벤트
        def _impact_point_cb(msg):
            try:
                data = json.loads(msg.data.strip())
                if not isinstance(data, dict):
                    raise ValueError("impact point JSON root must be object")
                self._pending_impact_points.append(data)
                while len(self._pending_impact_points) > max(
                    50, self._impact_history_max_count * 4
                ):
                    self._pending_impact_points.popleft()
            except Exception as e:
                self._log(f"[수신 오류] 탄착 좌표 처리 실패: {e}")

        self._ros_node.create_subscription(
            String, "/compute/impact_point", _impact_point_cb, qos_events
        )

        # /mmr/engagement_status — 정렬·조준·발사 상태 머신 및 Hold 진행률
        def _engagement_cb(msg):
            try:
                self._latest_engagement_data = json.loads(msg.data)
                self._engagement_status_stamp = time.monotonic()
            except Exception as e:
                self._log(f"[수신 오류] 발사 시퀀스 상태 처리 실패: {e}")

        self._ros_node.create_subscription(
            String, "/mmr/engagement_status", _engagement_cb, qos_latest)

        # /mmr/turret/status — STM 스텝모터/INP/Home 상태 표시용
        def _turret_status_cb(msg):
            try:
                self._latest_turret_status_data = json.loads(msg.data)
            except Exception as e:
                self._log(f"[수신 오류] 터렛 상태 처리 실패: {e}")

        self._ros_node.create_subscription(
            String, "/mmr/turret/status", _turret_status_cb, qos_latest)

        # /mmr/drive/status — 수동 주행 중 정렬 경고 표시 억제용
        def _drive_status_cb(msg):
            try:
                self._latest_drive_status_data = json.loads(msg.data)
                self._drive_status_stamp = time.monotonic()
            except Exception as e:
                self._log(f"[수신 오류] 구동 상태 처리 실패: {e}")

        self._ros_node.create_subscription(
            String, "/mmr/drive/status", _drive_status_cb, qos_latest)

        self._log("[시스템] ROS2 노드 시작 완료")
        self._log("[시스템] 상태 수신 및 제어 명령 준비 완료")

        # /mmr/range_info
        def _range_info_cb(msg):
            try:
                self._latest_range_info_data = json.loads(msg.data)
            except Exception as e:
                self._log(f"[수신 오류] 발사 가능 범위 처리 실패: {e}")

        self._ros_node.create_subscription(String, "/mmr/range_info", _range_info_cb, qos_latest)

        # GUI 시작 시 기본값으로 부채꼴 초기화 (기능 비활성화)
        # /mmr/range_info가 실제로 수신되기 전까지는 사거리 부채꼴을 그리지 않는다.
        # /mmr/status만 발행했을 때 기본 부채꼴이 뜨는 혼동과 불필요한 redraw를 막기 위함.
        # import math as _math

        # def _r(deg):
        #     return (15.0 ** 2) * _math.sin(2 * _math.radians(deg)) / 9.8044

        # self._latest_range_info_data = {
        #     "elev_min_deg": 45.0,
        #     "elev_max_deg": 61.0,
        #     "range_at_elev_min": round(_r(45.0), 2),
        #     "range_at_elev_max": round(_r(61.0), 2),
        #     "range_max": round(_r(45.0), 2),
        #     "range_min": round(_r(61.0), 2),
        #     "pan_half_angle_deg": 45.0,
        #     "note": "default_gui_init",
        # }

        # 구형 ARM/FIRE publisher는 기존 기능 호환을 위해 유지한다.
        # 새 GUI 버튼은 안전 상태 머신을 우회하지 않고 /mmr/operator_input만 발행한다.
        self._pub_fire_arm = self._ros_node.create_publisher(String, "/mmr/fire_arm", 10)
        self._pub_fire_confirm = self._ros_node.create_publisher(String, "/mmr/fire_confirm", 10)
        self._pub_operator_input = self._ros_node.create_publisher(
            String, "/mmr/operator_input", 10)
        self._pub_turret_home_request = self._ros_node.create_publisher(
            String, "/mmr/turret/home_request", 10)
        self._pub_emergency_stop = self._ros_node.create_publisher(
            String, "/mmr/emergency_stop", 10)

        # 표적 좌표 입력 publisher
        # 토글이 ON일 때 지도 우클릭 좌표를 표적 좌표로 발행한다.
        self._pub_target_point = self._ros_node.create_publisher(String, "/target/point", 10)

        self._log("[시스템] A 홀드 정렬·조준 및 Y 홀드 발사 입력 준비 완료")
        self._log("[시스템] 발사 가능 범위 및 표적 좌표 입력 준비 완료")
        self._log(
            f"[시스템] 탄착 좌표 구독 준비 완료 — 판정 반경={self._impact_threshold_m:.1f}m"
        )

        # GUI 표시용 GNSS 폴백도 최신값 1개만 유지한다.
        qos_gnss = qos_latest

        def _fix_quality_from_msg(msg):
            from sensor_msgs.msg import NavSatStatus

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

        def _pvt_cb(msg):
            """
            /gnss/fix_pvt 콜백.
            GUI fallback에서 fix 품질 표시용으로만 사용한다.
            """
            self._direct_fix_qual = _fix_quality_from_msg(msg)

        def _hp_cb(msg):
            """
            /gnss/fix_hp 콜백.
            GUI fallback에서 실제 위치 표시용으로 사용한다.
            """
            import time as _t

            self._direct_fix_lat = msg.latitude
            self._direct_fix_lon = msg.longitude
            self._direct_fix_stamp = _t.monotonic()
            self._direct_fix_source = "/gnss/fix_hp"

        def _hdg_cb(msg):
            """
            /gnss/heading 콜백.
            GUI fallback에서 heading 표시용으로 사용한다.
            """
            import time as _t

            self._direct_heading_deg = float(msg.data) % 360.0
            self._direct_hdg_stamp = _t.monotonic()

        self._ros_node.create_subscription(
            NavSatFix,
            "/gnss/fix_pvt",
            _pvt_cb,
            qos_gnss,
        )

        self._ros_node.create_subscription(
            NavSatFix,
            "/gnss/fix_hp",
            _hp_cb,
            qos_gnss,
        )

        self._ros_node.create_subscription(
            Float64,
            "/gnss/heading",
            _hdg_cb,
            qos_gnss,
        )

        self._log("[시스템] GNSS 위치·헤딩 상태 수신 준비 완료")

    # ================================================================
    # flush / apply
    # ================================================================
    def _flush_mmr_latest(self):
        if self._latest_mmr_data is None:
            return
        data = self._latest_mmr_data
        self._latest_mmr_data = None
        self._apply_mmr(data)

    def _apply_mmr(self, data: dict):
        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        hdg = data.get("heading_deg")

        if lat is None or lon is None:
            return

        lat = float(lat)
        lon = float(lon)
        hdg = float(hdg) if hdg is not None else 0.0

        fix = data.get("fix_quality", "—")
        fix_source = data.get("fix_source", "")
        heading_valid = data.get("heading_valid", True)

        self._latest_mmr_pose = (lat, lon, hdg)
        self.map_view.update_mmr(lat, lon, hdg)

        # 발표/캡처 화면에서는 내부 토픽 출처를 노출하지 않고 좌표만 표시한다.
        self.lbl_mmr_pos.setText(f"{lat:.7f},{lon:.7f}")

        hdg_label = f"{hdg:.1f}° [{fix}]"
        if heading_valid is False:
            hdg_label += "  ⚠️헤딩 갱신 끊김"
        self.lbl_mmr_hdg.setText(hdg_label)

        self._redraw_range_sector()

    def _flush_tilt_status_latest(self):
        if self._latest_tilt_status_data is None:
            return
        data = self._latest_tilt_status_data
        self._latest_tilt_status_data = None
        self._apply_tilt_status(data)

    def _apply_tilt_status(self, data: dict):
        tilt_ok = data.get("tilt_ok")
        roll_deg = data.get("roll_deg")
        pitch_deg = data.get("pitch_deg")
        stationary = data.get("stationary")
        roll_ok = data.get("roll_ok")
        pitch_ok = data.get("pitch_ok")
        state = str(data.get("state", "") or "").upper()

        self.att_indicator.set_attitude(roll_deg, pitch_deg)

        roll_s = f"{float(roll_deg):+.1f}°" if roll_deg is not None else "—"
        pitch_s = f"{float(pitch_deg):+.1f}°" if pitch_deg is not None else "—"

        # tilt_monitor는 require_stationary_for_tilt_ok=true일 때 주행·진동 중
        # tilt_ok=false를 발행한다. 이 상태는 실제 기울기 한계 초과가 아니므로
        # GUI 로그에서는 숨기되 /imu/tilt_status와 안전 인터록은 변경하지 않는다.
        motion_only_invalid = (
            tilt_ok is False
            and stationary is False
            and roll_ok is not False
            and pitch_ok is not False
            and state in {"", "IMU_MOVING_OR_ACCEL"}
        )
        self._tilt_motion_only_invalid_ui = bool(motion_only_invalid)

        if motion_only_invalid:
            self._tilt_motion_log_suppressed_ui = True
            return

        # 주행이 끝난 직후에는 현재 실제 상태를 한 번 다시 표시한다.
        if self._tilt_motion_log_suppressed_ui:
            self._tilt_motion_log_suppressed_ui = False
            self._prev_tilt_ok = None

        if tilt_ok != self._prev_tilt_ok:
            self._prev_tilt_ok = tilt_ok
            if tilt_ok is True:
                self._log(f"[IMU] 차체 수평 정상 — 롤={roll_s}, 피치={pitch_s}")
            elif tilt_ok is False:
                self._log(f"[IMU 경고] 차체 기울기 허용 범위 초과 — 롤={roll_s}, 피치={pitch_s}. 차체를 안정시키십시오.")

    def _flush_fire_ready_latest(self):
        if self._latest_fire_ready_data is None:
            return
        data = self._latest_fire_ready_data
        self._latest_fire_ready_data = None
        self._apply_fire_ready(data)

    def _apply_fire_ready(self, data: dict):
        fire_ready = data.get("fire_ready")
        reason = data.get("reason", "")
        target_key = data.get("target_key", "")
        if isinstance(target_key, str) and target_key.startswith("id:"):
            target_key = target_key[3:]
        uart_ok = data.get("uart_ok")
        tilt_ok = data.get("tilt_ok")
        status = data.get("status", "")
        armed = data.get("armed")

        if armed is True and not self._gui_armed:
            self._gui_armed = True
            self._update_arm_ui(True)
        elif armed is False and self._gui_armed:
            self._gui_armed = False
            self._update_arm_ui(False)

        event_key = json.dumps(
            {
                "fire_ready": fire_ready,
                "status": status,
                "reason": reason,
                "target_key": target_key,
                "uart_ok": uart_ok,
                "tilt_ok": tilt_ok,
                "armed": armed,
            },
            sort_keys=True,
            ensure_ascii=False,
            default=str,
        )
        repeated_event = event_key == self._last_fire_ready_event_key
        self._last_fire_ready_event_key = event_key

        if fire_ready is True:
            solver_elev = data.get("solver_elevation_deg")
            solver_yaw = data.get("solver_yaw_deg")

            cmd_elev = data.get("cmd_elevation_deg")
            cmd_yaw = data.get("cmd_yaw_deg")

            imu_roll = data.get("imu_roll_deg")
            imu_pitch = data.get("imu_pitch_deg")
            imu_applied = data.get("imu_correction_applied")

            elev_clamped = data.get("elev_clamped")
            clamped_warn = " ⚠️기구 한계로 각도 제한" if elev_clamped else ""

            def _fmt_deg(v):
                try:
                    return f"{float(v):.2f}°"
                except Exception:
                    return "—"

            def _fmt_signed_deg(v):
                try:
                    return f"{float(v):+.2f}°"
                except Exception:
                    return "—"

            if not repeated_event:
                self._log(
                    f"[포신 조준] 표적={target_key} | "
                    f"고각 계산값 {_fmt_deg(solver_elev)} → 명령값 {_fmt_deg(cmd_elev)}{clamped_warn} | "
                    f"방위각 계산값 {_fmt_deg(solver_yaw)} → 명령값 {_fmt_deg(cmd_yaw)} | "
                    f"IMU 보정={'적용' if imu_applied else '미적용'} "
                    f"피치={_fmt_signed_deg(imu_pitch)}, 롤={_fmt_signed_deg(imu_roll)}"
                )

            self._update_fire_ready_ui(True)

        elif status == "BUFFER_UPDATED":
            lat = data.get("buffer_lat", "?")
            lon = data.get("buffer_lon", "?")
            arm_hint = "발사 계통 준비 후 처리" if not data.get("armed") else "발사 계통 준비됨"

            self._last_buffer_pending_key = ""
            self._last_fire_sol_status_logged = ""

            if not repeated_event:
                self._log(
                    f"[표적 좌표] 새 표적 수신 — 표적={target_key} "
                    f"위도={lat}, 경도={lon} | {arm_hint}"
                )

            try:
                self.map_view.update_drone_target(
                    float(lat),
                    float(lon),
                    f"{target_key} | {arm_hint}",
                )
            except Exception:
                pass

        elif status == "BUFFER_PENDING":
            lat = data.get("buffer_lat", "?")
            lon = data.get("buffer_lon", "?")
            age = data.get("buffer_age_sec", "?")

            if target_key != self._last_buffer_pending_key:
                self._last_buffer_pending_key = target_key
                self._log(
                    f"[표적 좌표] 처리 대기 — 표적={target_key} "
                    f"위도={lat}, 경도={lon} | 대기={age}초 | 발사 계통 준비 버튼을 눌러 처리"
                )

        elif status == "FIRED":
            if not repeated_event:
                self._log(
                    f"[발사 시퀀스] 완료 — 표적={target_key} | "
                    "표적 좌표와 탄착 판정 이력은 유지합니다."
                )
            self._update_fire_ready_ui(False)

        elif status == "STOP_UART_FAILED":
            if not repeated_event:
                self._log(f"[발사 차단] STM 정지 명령 전송 실패로 발사 명령을 보내지 않았습니다 — 표적={target_key}")
            self._update_fire_ready_ui(False)

        elif status == "FIRE_UART_FAILED":
            if not repeated_event:
                self._log(f"[발사 실패] STM 발사 명령 전송 실패 — 표적={target_key}. UART 포트와 STM 상태를 확인하십시오.")
            self._update_fire_ready_ui(False)

        else:
            reason_ko = _ko_reason(reason)

            # 정상 주행으로 stationary 판정만 해제된 동안에는 차체수평 불량
            # 로그를 남기지 않는다. 실제 roll/pitch 제한 초과는 그대로 기록한다.
            hide_motion_imu_notice = (
                self._drive_motion_recent_for_ui()
                and bool(getattr(self, "_tilt_motion_only_invalid_ui", False))
                and str(reason or "").upper() in _DRIVE_MOVING_HIDDEN_IMU_REASONS
            )
            if reason and not repeated_event and not hide_motion_imu_notice:
                self._log(
                    f"[발사 거부] {reason_ko} — "
                    f"표적={target_key}, UART={'정상' if uart_ok else '오류'}, 차체수평={'정상' if tilt_ok else '불량'}"
                )
            self._update_fire_ready_ui(False)

    def _update_arm_ui(self, armed: bool):
        """ARM 상태 버튼 색상 업데이트."""
        if not hasattr(self, "btn_arm"):
            return

        if armed:
            self.btn_arm.setText("🔴 발사 계통 준비됨")
            self.btn_arm.setStyleSheet(
                "QPushButton { background-color: #b71c1c; color: white; "
                "font-weight: bold; border-radius: 6px; padding: 4px 12px; }"
                "QPushButton:hover { background-color: #d32f2f; }"
            )
        else:
            self.btn_arm.setText("⬜ 발사 계통 준비")
            self.btn_arm.setStyleSheet(
                "QPushButton { background-color: #263238; color: #b0bec5; "
                "font-weight: bold; border-radius: 6px; padding: 4px 12px; }"
                "QPushButton:hover { background-color: #37474f; }"
            )

    @staticmethod
    def _set_label_if_changed(label: QLabel, text: str, *, style: str | None = None, tooltip: str | None = None) -> None:
        """Avoid repainting unchanged high-rate status labels."""
        if label.text() != text:
            label.setText(text)
        if style is not None and label.styleSheet() != style:
            label.setStyleSheet(style)
        if tooltip is not None and label.toolTip() != tooltip:
            label.setToolTip(tooltip)

    def _set_fire_status(self, text: str, color: str, key: str) -> None:
        """Change the compact fire badge only when its visual state changed."""
        if not hasattr(self, "lbl_fire_status"):
            return
        visual_key = f"{key}|{text}|{color}"
        if visual_key == self._last_fire_status_key:
            return
        self._last_fire_status_key = visual_key
        self.lbl_fire_status.setText(text)
        self.lbl_fire_status.setStyleSheet(f"color:{color};font-weight:bold;")

    def _engagement_status_is_authoritative(self) -> bool:
        return (
            self._engagement_status_stamp > 0.0
            and time.monotonic() - self._engagement_status_stamp
            <= self._engagement_authority_sec
        )

    def _update_fire_ready_ui(self, ready: bool):
        """Legacy /mmr/fire_ready fallback.

        When the coordinator status is arriving normally it is the single
        authority for this badge.  This prevents READY/WAIT states from two
        topics alternating every 50~100 ms.
        """
        if self._engagement_status_is_authoritative():
            return
        if ready:
            self._set_fire_status("🟢 발사 준비", "#00e676", "legacy-ready")
        else:
            self._set_fire_status("⬛ 대기", "#546e7a", "legacy-idle")

    def _update_test_target_ui(self, enabled: bool):
        """GUI 단독 표적 좌표 입력 토글 버튼 표시 업데이트."""
        if not hasattr(self, "btn_test_target"):
            return

        if enabled:
            self.btn_test_target.setText("표적 좌표 입력 켜짐")
            self.btn_test_target.setStyleSheet(
                "QPushButton { background-color: #ef6c00; color: white; "
                "font-weight: bold; border-radius: 6px; padding: 4px 10px; }"
                "QPushButton:hover { background-color: #fb8c00; }"
            )
        else:
            self.btn_test_target.setText("표적 좌표 입력 꺼짐")
            self.btn_test_target.setStyleSheet(
                "QPushButton { background-color: #263238; color: #b0bec5; "
                "font-weight: bold; border-radius: 6px; padding: 4px 10px; }"
                "QPushButton:hover { background-color: #37474f; }"
            )

    def _on_test_target_toggle(self, checked: bool):
        """지도 우클릭 기반 표적 좌표 입력 모드 토글."""
        self._gui_target_test_mode = bool(checked)
        self._update_test_target_ui(self._gui_target_test_mode)

        if self._gui_target_test_mode:
            self._log("[표적 좌표] 수동 입력 모드 켜짐 — 지도에서 마우스 오른쪽 버튼으로 표적을 지정하십시오.")
        else:
            self._log("[표적 좌표] 수동 입력 모드 꺼짐")

    def _on_test_target_right_click(self, lat: float, lon: float, z):
        """표적 좌표 입력 모드 ON 상태에서 지도 우클릭 좌표를 발행한다."""
        if not self._gui_target_test_mode:
            return

        if not self._ros_enabled or self._pub_target_point is None:
            self._log("[표적 좌표] ROS2 미연결 — 좌표 입력 실패")
            return

        from std_msgs.msg import String as RosString    

        self._test_target_seq += 1
        target_key = f"T{self._test_target_seq:03d}"

        z_ok = z is not None
        z_m = float(z) if z_ok else 0.0
        z_source = "DEM" if z_ok else "DEFAULT_0M"

        payload = {
            "status": "HIT",
            "target_key": target_key,
            "target_id": target_key,
            "source": "target_point",
            "lat_deg": float(lat),
            "lon_deg": float(lon),
            "z_m": z_m,
            "sync_ok": True,
            "projection_ok": True,
            # z_m이 포함된 표적으로 발행한다. DEM이 없으면 z_m=0.0 fallback을 사용한다.
            "dem_intersection_ok": True,
            "gui_dem_available": z_ok,
            "z_source": z_source,
            "stamp": time.time(),
        }

        msg = RosString()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_target_point.publish(msg)

        self.lbl_target.setText(f"{lat:.6f}, {lon:.6f}")
        self.lbl_demz.setText(f"{z_m:.3f}" if z_ok else "0.000*")

        self.map_view.update_drone_target(
            float(lat),
            float(lon),
            f"{target_key} | z={z_m:.2f}m",
        )

        fallback_note = "" if z_ok else " | DEM 고도 없음: 고도 0.0m 임시 적용"
        self._log(
            f"[표적 좌표] 입력 완료 — 표적={target_key} "
            f"위도={lat:.7f}, 경도={lon:.7f}, 고도={z_m:.2f}m{fallback_note}"
        )

    def _on_arm_btn_clicked(self):
        if not self._ros_enabled:
            self._log("[발사 계통 준비] ROS2가 연결되지 않아 명령을 보낼 수 없습니다.")
            return

        from std_msgs.msg import String as RosString

        msg = RosString()

        if self._gui_armed:
            msg.data = "DISARM"
            self._log("[GUI] 발사 계통 준비 해제 명령 전송")
        else:
            msg.data = "ARM"
            self._log("[GUI] 발사 계통 준비 명령 전송 — 표적 수신 후 포신 제어를 시작합니다.")

        self._pub_fire_arm.publish(msg)

    def _on_fire_btn_clicked(self):
        if not self._ros_enabled:
            self._log("[발사 명령] ROS2가 연결되지 않아 명령을 보낼 수 없습니다.")
            return

        if not self._gui_armed:
            self._log("[발사 명령 거부] 발사 계통 준비 상태가 아닙니다. 준비 버튼을 먼저 누르십시오.")
            return

        from std_msgs.msg import String as RosString

        msg = RosString()
        msg.data = "FIRE"
        self._pub_fire_confirm.publish(msg)
        self._log("[GUI] 발사 확인 명령 전송")

    def _on_turret_home_clicked(self):
        """GUI HOME 버튼: 기존 조작 키/hold 로직을 건드리지 않고 HOME 요청만 발행한다."""
        if not self._ros_enabled or not hasattr(self, "_pub_turret_home_request"):
            self._log("[터렛 홈 복귀] ROS2가 연결되지 않아 요청을 보낼 수 없습니다.")
            return

        from std_msgs.msg import String as RosString

        msg = RosString()
        msg.data = "HOME"
        self._pub_turret_home_request.publish(msg)
        self._log("[GUI] 터렛 홈 복귀 요청 전송")

    def _publish_emergency_stop(self, command: str, source: str = "gui"):
        """공통 /mmr/emergency_stop 토픽으로 ESTOP 또는 RESET을 발행한다."""
        if not self._ros_enabled or not hasattr(self, "_pub_emergency_stop"):
            self._log("[비상정지] ROS2가 연결되지 않아 명령을 보낼 수 없습니다.")
            return

        cmd = str(command).strip().upper()
        if cmd not in {"ESTOP", "RESET"}:
            self._log(f"[비상정지 오류] 지원하지 않는 명령입니다: {command}")
            return

        from std_msgs.msg import String as RosString

        # E-Stop/Reset 시 GUI Hold 입력이 남아 있지 않도록 먼저 해제한다.
        self._release_operator_inputs()

        msg = RosString()
        msg.data = cmd
        self._pub_emergency_stop.publish(msg)

        if cmd == "ESTOP":
            self._estop_latched_ui = True
            self._set_fire_status("🔴 E-STOP", "#ff5252", "estop-command")
            self.hold_ring.set_status(0.0, "ESTOP", "STOP")
        elif cmd == "RESET":
            self._estop_latched_ui = False
            self._set_fire_status("🟠 홈 복귀", "#ffab40", "estop-reset-home")
            self.hold_ring.set_status(0.0, "AIM", "홈\n복귀")

        self._log(f"[GUI] 비상정지 명령 전송: {'작동' if cmd == 'ESTOP' else '해제'} (입력={source})")

    def _on_estop_clicked(self):
        self._publish_emergency_stop("ESTOP", source="gui_button")

    def _on_reset_clicked(self):
        self._publish_emergency_stop("RESET", source="gui_button")

    def _set_operator_button(self, button: str, pressed: bool):
        """GUI 버튼 press/release를 coordinator 입력으로 변환한다."""
        pressed = bool(pressed)
        if pressed and getattr(self, "_estop_latched_ui", False):
            self._log("[버튼 입력 차단] 비상정지가 작동 중이므로 A/Y 홀드 입력을 무시합니다.")
            self._release_operator_inputs()
            return

        if button == "align_aim":
            self._gui_align_aim_pressed = pressed
            if pressed:
                self._gui_fire_pressed = False
        elif button == "fire":
            self._gui_fire_pressed = pressed
            if pressed:
                self._gui_align_aim_pressed = False
        else:
            return

        self._publish_operator_input()

        action = "누름" if pressed else "해제"
        button_ko = "A 정렬·조준" if button == "align_aim" else "Y 발사"
        self._log(f"[GUI 홀드] {button_ko} 버튼 {action}")
        self._update_operator_button_styles()

    def _publish_operator_input_heartbeat(self):
        """버튼을 누르는 동안 20Hz heartbeat를 보내 Hold가 끊기지 않게 한다."""
        if self._gui_align_aim_pressed or self._gui_fire_pressed:
            self._publish_operator_input()

    def _publish_operator_input(self):
        if not self._ros_enabled or not hasattr(self, "_pub_operator_input"):
            return

        from std_msgs.msg import String as RosString

        payload = {
            "align_aim_pressed": bool(self._gui_align_aim_pressed),
            "fire_pressed": bool(self._gui_fire_pressed),
            "source": "mmr_gui",
            "stamp": time.time(),
        }
        msg = RosString()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_operator_input.publish(msg)

    def _release_operator_inputs(self):
        """GUI 종료·포커스 변경 시 남아 있는 Hold 입력을 안전하게 해제한다."""
        was_pressed = self._gui_align_aim_pressed or self._gui_fire_pressed
        self._gui_align_aim_pressed = False
        self._gui_fire_pressed = False
        if was_pressed:
            self._publish_operator_input()
        self._update_operator_button_styles()

    def _flush_turret_status_latest(self):
        if self._latest_turret_status_data is None:
            return
        data = self._latest_turret_status_data
        self._latest_turret_status_data = None
        self._apply_turret_status(data)

    def _apply_turret_status(self, data: dict):
        """터렛·STM 상태를 현장 운용용 한글 설명으로 표시한다.

        ROS 토픽의 원본 상태 코드와 기존 A/Y 홀드 제어 흐름은 변경하지 않는다.
        """
        if not isinstance(data, dict):
            return

        state = str(data.get("state", "—"))
        state_ko = _ko_turret_state(state)
        home_valid = data.get("home_valid")
        busy = data.get("busy")
        aim_complete = data.get("aim_complete")
        fire_ready = data.get("fire_ready")
        uart_enabled = data.get("uart_enabled")
        uart_open = data.get("uart_open")
        current_pulse = data.get("current_tilt_pulse")
        target_pulse = data.get("target_tilt_pulse")
        current_deg = data.get("current_elevation_deg")
        target_deg = data.get("target_elevation_deg")
        last_cmd = data.get("last_command", "")
        last_error = data.get("last_error", "")
        estop_latched = bool(data.get("estop_latched", False))
        self._estop_latched_ui = estop_latched

        stm_status = (
            data.get("stm_last_status")
            if isinstance(data.get("stm_last_status"), dict)
            else {}
        )
        inp = data.get("stm_in_position", stm_status.get("INP"))
        m0run = data.get("stm_m0run", stm_status.get("M0RUN"))

        def _flag(value):
            if value is True:
                return "예"
            if value is False:
                return "아니오"
            if value is None:
                return "—"
            return str(value)

        def _bit(value):
            if value is True or value == 1 or str(value) == "1":
                return "정상"
            if value is False or value == 0 or str(value) == "0":
                return "미충족"
            return "—"

        def _run(value):
            if value is True or value == 1 or str(value) == "1":
                return "구동 중"
            if value is False or value == 0 or str(value) == "0":
                return "정지"
            return "—"

        def _fmt_deg(value):
            try:
                if value is None:
                    return "—"
                return f"{float(value):.2f}°"
            except Exception:
                return "—"

        def _fmt_pulse(value):
            return "—" if value is None else str(value)

        if hasattr(self, "lbl_turret_state"):
            home_s = "정상" if home_valid is True else "무효" if home_valid is False else "—"
            uart_s = "정상" if (uart_enabled is False or uart_open is True) else "연결 오류"
            estop_s = "작동" if estop_latched else "해제"
            turret_state_text = (
                f"터렛: {state_ko} | 홈={home_s} | 작업중={_flag(busy)} | "
                f"STM 통신={uart_s} | 비상정지={estop_s}"
            )
            turret_state_tip = (
                f"원본 상태 코드: {state}\n"
                f"HOME={home_valid}, BUSY={busy}, UART_OPEN={uart_open}, ESTOP={estop_latched}"
            )

            state_upper = state.upper()
            if estop_latched or state_upper in {"FAULT", "ESTOP"}:
                color = "#ff5252"
            elif state_upper in {"HOME_REQUIRED", "HOMING", "STARTUP_HOMING", "TURRET_AIMING"}:
                color = "#ffab40"
            elif state_upper in {"HOME_READY", "AIM_READY", "FIRED"}:
                color = "#00e676"
            else:
                color = "#eceff1"
            self._set_label_if_changed(
                self.lbl_turret_state,
                turret_state_text,
                style=f"color:{color};font-weight:bold;",
                tooltip=turret_state_tip,
            )

        if hasattr(self, "lbl_turret_motion"):
            turret_motion_text = (
                f"위치 도달(INP)={_bit(inp)} | 모터={_run(m0run)} | "
                f"펄스 {_fmt_pulse(current_pulse)}→{_fmt_pulse(target_pulse)} | "
                f"고각 {_fmt_deg(current_deg)}→{_fmt_deg(target_deg)} | "
                f"조준완료={_flag(aim_complete)} | 발사준비={_flag(fire_ready)}"
            )
            self._set_label_if_changed(self.lbl_turret_motion, turret_motion_text)

        if hasattr(self, "lbl_turret_error"):
            cmd_ko = _ko_command(last_cmd)
            error_ko = _ko_reason(last_error) if last_error else "오류 없음"
            full_display = f"최근 명령: {cmd_ko} | 원인: {error_ko}"
            display = full_display if len(full_display) <= 120 else full_display[:117] + "..."
            turret_error_tip = (
                f"한글 설명\n{full_display}\n\n"
                f"원본 명령 코드: {last_cmd or '없음'}\n"
                f"원본 오류 코드: {last_error or '없음'}"
            )
            self._set_label_if_changed(
                self.lbl_turret_error,
                display,
                style="color:#ff8a80;" if last_error else "color:#90a4ae;",
                tooltip=turret_error_tip,
            )

        if estop_latched and hasattr(self, "lbl_fire_status"):
            self._set_fire_status("🔴 비상정지", "#ff5252", "estop")
            if hasattr(self, "hold_ring"):
                self.hold_ring.set_status(0.0, "ESTOP", "정지")

        log_key = f"{state}:{last_error}:{estop_latched}"
        prev_key = f"{self._last_turret_state}:{self._last_turret_error}:{self._last_turret_estop}"
        if log_key != prev_key:
            if last_error:
                self._log(
                    f"[터렛 오류] {state_ko} — {_ko_reason(last_error)} "
                    f"(원본 코드: {last_error})"
                )
            elif state != self._last_turret_state:
                self._log(f"[터렛 상태] {state_ko}")
            if estop_latched != self._last_turret_estop:
                self._log(
                    "[터렛 비상정지] 비상정지가 작동했습니다."
                    if estop_latched
                    else "[터렛 비상정지] 비상정지가 해제되었습니다. 홈 상태를 다시 확인하십시오."
                )
            self._last_turret_state = state
            self._last_turret_error = str(last_error or "")
            self._last_turret_estop = bool(estop_latched)

    def _drive_motion_recent_for_ui(self) -> bool:
        """최근 구동 상태가 실제 이동 중임을 나타내는지 반환한다."""
        age = time.monotonic() - float(getattr(self, "_drive_status_stamp", 0.0) or 0.0)
        return (
            0.0 <= age <= 1.0
            and bool(getattr(self, "_drive_any_moving_ui", False))
        )

    def _flush_drive_status_latest(self):
        """수동 주행 여부만 GUI 캐시에 반영한다.

        all_stopped=false이면서 auto_align_active=false인 경우에만 일반 이동으로
        판단한다. 자동 정렬 회전 중에는 기존 정렬 상태 표시를 유지한다.
        """
        if self._latest_drive_status_data is None:
            return

        data = self._latest_drive_status_data
        self._latest_drive_status_data = None

        mode = str(data.get("mode", "UNKNOWN") or "UNKNOWN").upper()
        auto_align_active = bool(data.get("auto_align_active", False))
        all_stopped = bool(data.get("all_stopped", True))
        manual_override = bool(data.get("manual_override_active", False))

        # 피드백 기반 all_stopped가 false이거나 최근 수동 명령이 활성화된 동안을
        # 수동 주행으로 본다. 자동 정렬은 별도 표시가 필요하므로 제외한다.
        any_moving = not all_stopped or manual_override
        moving = any_moving and not auto_align_active

        self._drive_mode_ui = mode
        self._drive_auto_align_ui = auto_align_active
        self._drive_any_moving_ui = bool(any_moving)
        self._drive_moving_ui = bool(moving)

    def _flush_engagement_latest(self):
        if self._latest_engagement_data is None:
            return
        data = self._latest_engagement_data
        self._latest_engagement_data = None
        self._apply_engagement_status(data)

    def _apply_engagement_status(self, data: dict):
        """정렬·조준·발사 상태를 한글 원인 설명으로 표시한다."""
        state = str(data.get("state", "UNKNOWN")).upper()
        reason = str(data.get("reason", ""))
        state_ko = _ko_engagement_state(state)
        reason_ko = _ko_reason(reason)

        # 비상정지 해제 뒤 coordinator가 FAULT 상태를 유지하는 동안은
        # 실제 장치 고장과 구분해 RESET·자동 홈 복귀가 필요한 복구 상태로 표시한다.
        # ROS 상태 코드와 안전 인터록은 그대로 유지하고 GUI 문구만 바꾼다.
        reason_upper = reason.upper()
        estop_recovery_needed = (
            state == "FAULT"
            and "ESTOP" in reason_upper
        )
        if estop_recovery_needed:
            state_ko = "시퀀스 복구 필요"

        # 수동 주행 중에는 정렬 미완료가 정상 상태이므로 GUI 경고만 숨긴다.
        # 코디네이터 상태와 안전 인터록은 수정하지 않는다.
        hide_alignment_notice = (
            bool(getattr(self, "_drive_moving_ui", False))
            and not bool(getattr(self, "_drive_auto_align_ui", False))
            and reason in _DRIVE_MOVING_HIDDEN_ALIGNMENT_REASONS
        )
        hide_motion_imu_notice = (
            self._drive_motion_recent_for_ui()
            and bool(getattr(self, "_tilt_motion_only_invalid_ui", False))
            and reason in _DRIVE_MOVING_HIDDEN_IMU_REASONS
        )
        hide_expected_moving_notice = hide_alignment_notice or hide_motion_imu_notice
        if hide_expected_moving_notice:
            state_ko = "MMR 이동 중"
            reason_ko = (
                "주행 중에는 정지 상태 기반 차체 정렬·IMU 안정 경고를 표시하지 않습니다. "
                "정지 후 자동으로 다시 확인합니다."
            )

        now_mono = time.monotonic()
        previous_state = self._last_engagement_state
        if state == "FIRED" and previous_state != "FIRED":
            self._fired_banner_until = now_mono + self._fired_banner_duration_sec
        fired_banner_active = state == "FIRED" and now_mono < self._fired_banner_until
        post_fire_wait_display = state == "FIRED" and not fired_banner_active
        aim_progress = float(data.get("aim_hold_progress", 0.0) or 0.0)
        fire_progress = float(data.get("fire_hold_progress", 0.0) or 0.0)
        revision = data.get("solution_revision")
        locked_revision = data.get("locked_solution_revision")
        target_key = data.get("active_target_key") or "—"
        awaiting_reengagement = bool(data.get("awaiting_reengagement", False))
        self._awaiting_reengagement_ui = awaiting_reengagement
        # FAULT 복구 표시가 표적 유지 상태에 덮이지 않도록 한다.
        # 발사 완료 후 표적을 유지한 상태에서 E-Stop이 발생해도
        # RESET·홈 복귀가 끝날 때까지는 복구 필요 상태를 우선 표시한다.
        if awaiting_reengagement and state != "FAULT":
            state_ko = "표적 유지 · 재교전 대기"

        self._update_heading_alignment_ui(data)

        if getattr(self, "_estop_latched_ui", False):
            if hasattr(self, "hold_ring"):
                self.hold_ring.set_status(0.0, "ESTOP", "정지")
            if hasattr(self, "lbl_fire_status"):
                self._set_fire_status("🔴 비상정지", "#ff5252", "estop")
            if hasattr(self, "lbl_engagement_state"):
                self.lbl_engagement_state.setText("비상정지 작동")
                estop_text = "비상정지 신호가 유지 중입니다. 위험 요인을 제거한 뒤 정지 해제를 수행하십시오."
                self.lbl_engagement_reason.setText(estop_text)
                self.lbl_engagement_reason.setToolTip(
                    f"{estop_text}\n원본 토픽: /mmr/emergency_stop = ESTOP"
                )
            self._update_operator_button_styles(state)
            return

        if hasattr(self, "lbl_engagement_state"):
            display_state_ko = (
                "대기 · 포신 홈 복귀 중"
                if post_fire_wait_display
                else state_ko
            )
            self._set_label_if_changed(self.lbl_engagement_state, display_state_ko)
            full_reason_text = (
                f"원인: {reason_ko} | 표적={target_key} | "
                f"탄도해 버전={revision} | 잠금 버전={locked_revision}"
            )
            reason_text = full_reason_text
            if len(reason_text) > 120:
                reason_text = reason_text[:117] + "..."
            engagement_tip = (
                f"{full_reason_text}\n원본 상태 코드: {state}\n원본 원인 코드: {reason or '없음'}"
            )
            self._set_label_if_changed(
                self.lbl_engagement_reason,
                reason_text,
                tooltip=engagement_tip,
            )

        if hide_alignment_notice:
            self.hold_ring.set_status(0.0, "IDLE", "이동")
        elif state == "IDLE" and awaiting_reengagement:
            self.hold_ring.set_status(0.0, "AIM", "A\n재교전")
        elif state == "AIM_CONFIRMING":
            self.hold_ring.set_status(aim_progress, "AIM", f"조준\n{aim_progress*100:.0f}%")
        elif state in {"TURRET_AIMING", "FINAL_AIM_VERIFY"}:
            self.hold_ring.set_status(1.0, "AIM", "조준\n확인")
        elif state == "FIRE_CONFIRMING":
            self.hold_ring.set_status(fire_progress, "FIRE", f"발사\n{fire_progress*100:.0f}%")
        elif state == "AIM_READY":
            self.hold_ring.set_status(0.0, "FIRE", "Y\n홀드")
        elif state == "FIRED" and fired_banner_active:
            self.hold_ring.set_status(1.0, "DONE", "발사\n완료")
        elif state == "FIRED":
            self.hold_ring.set_status(0.0, "IDLE", "대기")
        elif estop_recovery_needed:
            self.hold_ring.set_status(0.0, "FAULT", "복구")
        elif state == "FAULT":
            self.hold_ring.set_status(0.0, "FAULT", "고장")
        elif state in {"WAIT_ALIGN", "BODY_ALIGNING", "ALIGN_COMPLETE_WAIT_RELEASE"}:
            self.hold_ring.set_status(0.0, "ALIGN", "정렬")
        elif state == "READY_FOR_AIM":
            self.hold_ring.set_status(0.0, "AIM", "A\n홀드")
        else:
            self.hold_ring.set_status(0.0, "IDLE", "준비")

        if state == "IDLE" and awaiting_reengagement:
            self._set_fire_status("🟦 표적 유지", "#40c4ff", "target-retained")
        elif state in {"AIM_READY", "FIRE_CONFIRMING"}:
            self._set_fire_status("🟢 발사 승인 대기", "#00e676", "fire-ready")
        elif state == "FIRING":
            self._set_fire_status("🟠 발사 진행 중", "#ffab40", "firing")
        elif state == "FIRED" and fired_banner_active:
            self._set_fire_status("🔵 발사 완료", "#40c4ff", "fired-banner")
        elif state == "FIRED":
            self._set_fire_status("⬛ 대기", "#546e7a", "post-fire-wait")
        elif estop_recovery_needed:
            self._set_fire_status(
                "🟠 시퀀스 복구 필요",
                "#ffab40",
                "estop-recovery",
            )
        elif state == "FAULT":
            self._set_fire_status("🔴 시퀀스 고장", "#ff5252", "fault")
        else:
            self._set_fire_status("⬛ 대기", "#546e7a", f"idle-{state}")

        if state != self._last_engagement_state or reason != self._last_engagement_reason:
            # 정상 이동 중 정렬·IMU 불안정은 예상된 상태이므로 로그를 남기지 않는다.
            if not hide_expected_moving_notice:
                self._log(
                    f"[발사 시퀀스] {state_ko} — {reason_ko} "
                    f"(상태 코드={state}, 원인 코드={reason or '없음'})"
                )
            self._last_engagement_state = state
            self._last_engagement_reason = reason

        self._update_operator_button_styles(state)

    def _update_operator_button_styles(self, state: str | None = None):
        if not hasattr(self, "btn_align_aim") or not hasattr(self, "btn_fire"):
            return

        state = str(state or getattr(self, "_last_engagement_state", "") or "").upper()
        style_key = (
            f"{state}|{bool(getattr(self, '_gui_align_aim_pressed', False))}|"
            f"{bool(getattr(self, '_gui_fire_pressed', False))}"
        )
        # HUD 생성 도중 이 메서드가 상태 캐시 초기화보다 먼저 호출될 수 있다.
        # getattr 폴백으로 최초 생성 시 AttributeError를 방지한다.
        if style_key == getattr(self, "_last_operator_style_key", ""):
            return
        self._last_operator_style_key = style_key

        if state in {"WAIT_ALIGN", "BODY_ALIGNING", "ALIGN_COMPLETE_WAIT_RELEASE"}:
            align_text = "A 홀드 · 차체 정렬"
        elif state in {"READY_FOR_AIM", "AIM_CONFIRMING"}:
            align_text = "A 홀드 · 포신 조준"
        elif state == "IDLE" and bool(getattr(self, "_awaiting_reengagement_ui", False)):
            align_text = "A 누름 · 동일 표적 재교전"
        else:
            align_text = "A 홀드 · 정렬/조준"

        self.btn_align_aim.setText(align_text)
        self.btn_fire.setText("Y 홀드 · 발사 승인")

        align_pressed = bool(getattr(self, "_gui_align_aim_pressed", False))
        fire_pressed = bool(getattr(self, "_gui_fire_pressed", False))
        align_bg = "#0277bd" if align_pressed else "#0d47a1"
        fire_bg = "#c62828" if fire_pressed else "#7f0000"

        self.btn_align_aim.setStyleSheet(
            f"QPushButton {{ background-color:{align_bg}; color:white; "
            "font-weight:bold; border-radius:6px; padding:4px 10px; }"
            "QPushButton:hover { background-color:#1565c0; }"
            "QPushButton:pressed { background-color:#29b6f6; }"
        )
        self.btn_fire.setStyleSheet(
            f"QPushButton {{ background-color:{fire_bg}; color:white; "
            "font-weight:bold; border-radius:6px; padding:4px 10px; }"
            "QPushButton:hover { background-color:#b71c1c; }"
            "QPushButton:pressed { background-color:#ff5252; }"
        )

    def _flush_range_info_latest(self):
        if self._latest_range_info_data is None:
            return

        # 캐시를 지우지 않고 유지 — _redraw_range_sector가 재사용
        self._apply_range_info(self._latest_range_info_data)

    def _apply_range_info(self, data: dict):
        """
        /mmr/range_info 수신 → Leaflet drawRangeRings() 호출.
        MMR 헤딩 기준 ±pan_half_angle_deg 범위의 부채꼴로 발사 가능 구역 표시.
        MMR 위치 미수신 시 생략.
        """
        if self._latest_mmr_pose is None:
            return

        lat, lon, hdg = self._latest_mmr_pose

        import json as _json

        js = f"drawRangeRings({_json.dumps(data)}, {lat}, {lon}, {hdg});"
        self.map_view._run_js(js)

    def _redraw_range_sector(self):
        """MMR 위치/헤딩 변경 시 부채꼴 재갱신."""
        if self._latest_range_info_data is not None:
            self._apply_range_info(self._latest_range_info_data)

    def _flush_fire_solution_latest(self):
        if self._latest_fire_solution_data is None:
            return
        data = self._latest_fire_solution_data
        self._latest_fire_solution_data = None
        self._apply_fire_solution(data)

    def _flush_target_point_latest(self):
        if self._latest_target_point_data is None:
            return
        data = self._latest_target_point_data
        self._latest_target_point_data = None
        self._apply_target_point(data)

    def _flush_impact_points(self):
        """Process queued impact events in arrival order without dropping bursts."""
        # DDS 토픽 간 도착 순서는 보장되지 않으므로 표적이 먼저 확정될 때까지
        # 탄착 이벤트를 큐에 유지한다.
        if not isinstance(self._retained_target_data, dict):
            return
        processed = 0
        while self._pending_impact_points and processed < 20:
            data = self._pending_impact_points.popleft()
            self._apply_impact_point(data)
            processed += 1

    @staticmethod
    def _finite_float(value):
        try:
            value = float(value)
            return value if math.isfinite(value) else None
        except Exception:
            return None

    @staticmethod
    def _target_key_from_data(data: dict) -> str:
        value = None
        for name in ("target_key", "target_id", "seq"):
            candidate = data.get(name)
            if candidate not in {None, ""}:
                value = candidate
                break
        if value is None:
            return "target:anonymous"
        text = str(value)
        return text[3:] if text.startswith("id:") else text

    def _validate_target_for_gui(self, data: dict) -> tuple[bool, str]:
        if not isinstance(data, dict):
            return False, "JSON 객체가 아닙니다"
        status = str(data.get("status", "")).upper()
        if status != "HIT":
            return False, f"status={status or '누락'}"
        lat = self._finite_float(data.get("lat_deg"))
        lon = self._finite_float(data.get("lon_deg"))
        z = self._finite_float(data.get("z_m"))
        if lat is None or lon is None or z is None:
            return False, "위도·경도·고도 값이 유효하지 않습니다"
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return False, "위도·경도 범위를 벗어났습니다"
        if self._require_target_sync_ok_ui and data.get("sync_ok") is not True:
            return False, "sync_ok가 true가 아닙니다"
        if self._require_projection_ok_ui and data.get("projection_ok") is not True:
            return False, "projection_ok가 true가 아닙니다"
        if (
            self._require_dem_intersection_ok_ui
            and data.get("dem_intersection_ok") is not True
        ):
            return False, "dem_intersection_ok가 true가 아닙니다"
        return True, "OK"

    def _clear_impact_history(
        self,
        *,
        log_message: bool = False,
        reset_before_ns: int = 0,
        clear_pending: bool = False,
    ):
        # GCS와 MMR PC의 로컬 시계가 다를 수 있으므로 MMR의 time.time_ns()를
        # GCS t_click_ns와 직접 비교하지 않는다. GCS가 제공한 source timestamp만 사용한다.
        try:
            self._impact_reset_ns = max(0, int(reset_before_ns or 0))
        except Exception:
            self._impact_reset_ns = 0
        self._impact_history.clear()
        self._impact_counter = 0
        self._impact_seen_keys.clear()
        if clear_pending:
            self._pending_impact_points.clear()
        try:
            self.map_view.clear_impact_results()
        except Exception:
            pass
        if hasattr(self, "lbl_impact_coord"):
            self.lbl_impact_coord.setText("—")
            self.lbl_impact_result.setText("탄착 대기")
            self.lbl_impact_result.setStyleSheet(
                "color:#ffd54f; border:none; background:transparent;"
            )
        if log_message:
            self._log("[탄착 초기화] 탄착 지점과 판정 이력을 삭제했습니다. 표적은 유지됩니다.")

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2) -> float:
        radius_m = 6371008.8
        p1 = math.radians(float(lat1))
        p2 = math.radians(float(lat2))
        dp = p2 - p1
        dl = math.radians(float(lon2) - float(lon1))
        a = (
            math.sin(dp / 2.0) ** 2
            + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
        )
        return 2.0 * radius_m * math.asin(min(1.0, math.sqrt(a)))

    def _target_impact_distance_m(self, impact: dict) -> tuple[float | None, str]:
        target = self._retained_target_data
        if not isinstance(target, dict):
            return None, "NO_TARGET"

        target_frame = str(target.get("world_frame") or "").strip()
        impact_frame = str(impact.get("world_frame") or "").strip()
        frame_compatible = not (
            target_frame and impact_frame and target_frame != impact_frame
        )
        if frame_compatible:
            tx = self._finite_float(target.get("x_m"))
            ty = self._finite_float(target.get("y_m"))
            ix = self._finite_float(impact.get("x_m"))
            iy = self._finite_float(impact.get("y_m"))
            if all(value is not None for value in (tx, ty, ix, iy)):
                return math.hypot(ix - tx, iy - ty), "PROJECTED_XY"

        try:
            return self._haversine_m(
                target.get("lat_deg"),
                target.get("lon_deg"),
                impact.get("lat_deg"),
                impact.get("lon_deg"),
            ), "HAVERSINE"
        except Exception:
            return None, "DISTANCE_FAILED"

    @staticmethod
    def _impact_event_key(data: dict) -> tuple:
        return (
            str(data.get("seq", "")),
            str(data.get("t_click_ns", "")),
            str(data.get("lat_deg", "")),
            str(data.get("lon_deg", "")),
        )

    def _draw_impact_entry(self, entry: dict):
        target = self._retained_target_data
        data = entry.get("data") if isinstance(entry, dict) else None
        if not isinstance(target, dict) or not isinstance(data, dict):
            return
        try:
            impact_id = int(entry["impact_id"])
            lat = float(data["lat_deg"])
            lon = float(data["lon_deg"])
            target_lat = float(target["lat_deg"])
            target_lon = float(target["lon_deg"])
            error_m = float(entry["error_m"])
            result = str(entry["result"])
        except Exception:
            return

        result_ko = "타격 성공" if result == "SUCCESS" else "재타격 요청"
        text = (
            f"<b style='color:#ffb74d'>탄착 지점 #{impact_id}</b>"
            f"<br>lat: {lat:.7f}<br>lon: {lon:.7f}"
            f"<br>표적 편차: {error_m:.2f} m"
            f"<br><b>판정: {result_ko}</b>"
        )
        z = self._finite_float(data.get("z_m"))
        pose_dt = self._finite_float(data.get("pose_dt_ms"))
        if z is not None:
            text += f"<br>DEM 고도: {z:.2f} m"
        if pose_dt is not None:
            text += f"<br>Pose Δt: {pose_dt:.1f} ms"
        text += f"<br>거리 계산: {entry.get('distance_mode', 'UNKNOWN')}"

        self.map_view.show_impact_result(
            impact_id,
            lat,
            lon,
            target_lat,
            target_lon,
            error_m,
            result,
            text,
        )

    def _redraw_impact_history(self):
        self.map_view.clear_impact_results()
        for entry in self._impact_history:
            self._draw_impact_entry(entry)

    def _apply_impact_point(self, data: dict):
        if not isinstance(self._retained_target_data, dict):
            self._log("[탄착 좌표 무시] 비교할 유효 표적이 없습니다.")
            return
        if not isinstance(data, dict):
            self._log("[탄착 좌표 무시] JSON 객체 형식이 아닙니다.")
            return

        status = str(data.get("status", "")).upper()
        if status not in {"HIT", "HIT_OK", "HIT_WARN"}:
            self._log(
                f"[탄착 좌표 무시] 좌표 계산 상태가 유효하지 않습니다: "
                f"{status or '누락'}"
            )
            return

        lat = self._finite_float(data.get("lat_deg"))
        lon = self._finite_float(data.get("lon_deg"))
        if lat is None or lon is None:
            self._log("[탄착 좌표 무시] 위도·경도 값이 유효하지 않습니다.")
            return

        try:
            click_ns = int(data.get("t_click_ns") or 0)
        except Exception:
            click_ns = 0
        try:
            target_publish_ns = int(
                self._retained_target_data.get("t_publish_ns") or 0
            )
        except Exception:
            target_publish_ns = 0

        if self._reject_stale_impact_ui:
            if click_ns and click_ns < self._impact_reset_ns:
                self._log("[탄착 좌표 무시] 탄착 초기화 이전 요청의 지연 응답입니다.")
                return
            if click_ns and target_publish_ns and click_ns < target_publish_ns:
                self._log(
                    "[탄착 좌표 무시] 현재 표적이 발행되기 전에 생성된 이전 탄착 메시지입니다."
                )
                return

        event_key = self._impact_event_key(data)
        if event_key in self._impact_seen_keys:
            return
        self._impact_seen_keys.add(event_key)

        normalized = dict(data)
        for key in (
            "lat_deg", "lon_deg", "z_m", "range_m", "x_m", "y_m",
            "bearing_deg", "pose_dt_ms",
        ):
            value = self._finite_float(normalized.get(key))
            if value is not None:
                normalized[key] = value

        error_m, distance_mode = self._target_impact_distance_m(normalized)
        if error_m is None or not math.isfinite(error_m):
            if hasattr(self, "lbl_impact_result"):
                self.lbl_impact_result.setText("거리 계산 실패")
                self.lbl_impact_result.setStyleSheet(
                    "color:#ff5252; border:none; background:transparent;"
                )
            self._log("[탄착 판정 실패] 표적-탄착 수평거리 계산이 불가능합니다.")
            return

        self._impact_counter += 1
        result = "SUCCESS" if error_m <= self._impact_threshold_m else "REATTACK"
        entry = {
            "impact_id": self._impact_counter,
            "target_key": self._retained_target_key,
            "data": normalized,
            "error_m": float(error_m),
            "distance_mode": distance_mode,
            "result": result,
        }
        self._impact_history.append(entry)
        if len(self._impact_history) > self._impact_history_max_count:
            self._impact_history = self._impact_history[-self._impact_history_max_count:]
            self._redraw_impact_history()
        else:
            self._draw_impact_entry(entry)

        result_ko = "타격 성공" if result == "SUCCESS" else "재타격 요청"
        color = "#00e676" if result == "SUCCESS" else "#ff5252"
        if hasattr(self, "lbl_impact_coord"):
            self.lbl_impact_coord.setText(
                f"#{self._impact_counter} {lat:.6f}, {lon:.6f}"
            )
            self.lbl_impact_result.setText(f"{error_m:.1f}m / {result_ko}")
            self.lbl_impact_result.setStyleSheet(
                f"color:{color}; border:none; background:transparent;"
            )

        self._log(
            f"[탄착 판정] 표적={self._retained_target_key} | "
            f"#{self._impact_counter} 편차={error_m:.2f}m | "
            f"기준={self._impact_threshold_m:.1f}m → {result_ko} "
            f"({distance_mode})"
        )

    @staticmethod
    def _fmt_float(value, digits=2, suffix=""):
        try:
            value = float(value)
            if not math.isfinite(value):
                return "—"
            return f"{value:.{digits}f}{suffix}"
        except Exception:
            return "—"

    @staticmethod
    def _strip_target_prefix(value):
        text = str(value or "—")
        return text[3:] if text.startswith("id:") else text

    def _apply_target_point(self, data: dict):
        """유효한 /target/point를 보관하고 지도·HUD에 계속 표시한다."""
        if not isinstance(data, dict):
            return

        active = data.get("active", True)
        command = str(data.get("command", data.get("status", ""))).upper()
        clear_requested = (
            bool(data.get("clear_target", False))
            or active is False
            or command in {"CLEAR", "CLEARED", "CANCEL", "CANCELLED", "CLEAR_TARGET"}
        )
        if clear_requested:
            self._retained_target_data = None
            self._retained_target_key = ""
            self._target_received_ns = 0
            try:
                clear_publish_ns = int(data.get("t_publish_ns") or 0)
            except Exception:
                clear_publish_ns = 0
            self._clear_impact_history(
                log_message=False,
                reset_before_ns=clear_publish_ns,
                clear_pending=True,
            )
            if hasattr(self, "lbl_active_target"):
                self.lbl_active_target.setText("—")
                self.lbl_target_coord.setText("—")
                self.lbl_solution_aim.setText("해 계산 대기")
                self.lbl_solution_rev.setText("표적 해제")
                self.lbl_solution_rev.setStyleSheet("color:#90a4ae;")
            try:
                self.map_view.clear_drone_target()
            except Exception:
                pass
            self._log("[표적 좌표] 명시적 표적 해제 수신")
            return

        valid, reason = self._validate_target_for_gui(data)
        if not valid:
            self._log(f"[표적 좌표 거부] {reason}")
            if hasattr(self, "lbl_solution_rev"):
                self.lbl_solution_rev.setText("좌표 검증 실패")
                self.lbl_solution_rev.setStyleSheet("color:#ff5252;")
            return

        target_key = self._target_key_from_data(data)
        previous_key = self._retained_target_key
        previous_target = self._retained_target_data

        # 일부 GCS 구현은 재시작 후 target_id를 다시 사용할 수 있다.
        # ID가 같더라도 seq 또는 좌표가 달라지면 새 표적으로 취급하여
        # 이전 탄착 이력이 새 표적에 섞이지 않도록 한다.
        target_payload_changed = False
        if isinstance(previous_target, dict):
            previous_seq = previous_target.get("seq")
            current_seq = data.get("seq")
            if (
                previous_seq not in {None, ""}
                and current_seq not in {None, ""}
                and str(previous_seq) != str(current_seq)
            ):
                target_payload_changed = True
            else:
                previous_lat = self._finite_float(previous_target.get("lat_deg"))
                previous_lon = self._finite_float(previous_target.get("lon_deg"))
                previous_z = self._finite_float(previous_target.get("z_m"))
                current_lat = self._finite_float(data.get("lat_deg"))
                current_lon = self._finite_float(data.get("lon_deg"))
                current_z = self._finite_float(data.get("z_m"))
                if all(
                    value is not None
                    for value in (
                        previous_lat, previous_lon, previous_z,
                        current_lat, current_lon, current_z,
                    )
                ):
                    horizontal_shift_m = self._haversine_m(
                        previous_lat, previous_lon, current_lat, current_lon
                    )
                    target_payload_changed = (
                        horizontal_shift_m > 0.05
                        or abs(current_z - previous_z) > 0.05
                    )

        is_new_target = bool(
            previous_key
            and (target_key != previous_key or target_payload_changed)
        )
        if is_new_target:
            try:
                target_publish_ns = int(data.get("t_publish_ns") or 0)
            except Exception:
                target_publish_ns = 0
            self._clear_impact_history(
                log_message=False,
                reset_before_ns=target_publish_ns,
                clear_pending=False,
            )
            self._log(
                f"[표적 변경] {previous_key} → {target_key} | "
                "이전 탄착 이력을 초기화했습니다."
            )

        self._retained_target_data = dict(data)
        self._retained_target_key = target_key
        self._target_received_ns = time.time_ns()

        lat = float(data["lat_deg"])
        lon = float(data["lon_deg"])
        z = float(data["z_m"])

        if hasattr(self, "lbl_active_target"):
            self.lbl_active_target.setText(str(target_key))
            self.lbl_target_coord.setText(
                f"{lat:.6f}, {lon:.6f} | z={z:.2f}m"
            )
            self.lbl_solution_aim.setText("계산 대기")
            self.lbl_solution_rev.setText("좌표 보관")
            self.lbl_solution_rev.setStyleSheet("color:#90a4ae;")

        info = (
            f"{target_key} | z={z:.2f}m | "
            f"유효 반경={self._impact_threshold_m:.1f}m"
        )
        try:
            self.map_view.update_drone_target(
                lat,
                lon,
                info,
                self._impact_threshold_m,
            )
        except Exception:
            pass

        log_key = f"{target_key}:{lat}:{lon}:{z}"
        if log_key != self._last_target_point_label_key:
            self._last_target_point_label_key = log_key
            self._log(
                f"[표적 좌표] 수신·보관 — 표적={target_key} "
                f"위도={lat:.7f}, 경도={lon:.7f}, 고도={z:.2f}m | "
                "sync/projection/DEM 검증 정상"
            )

    def _apply_fire_solution(self, data: dict):
        """/mmr/fire_solution 탄도해를 GUI target panel에 표시."""
        if not isinstance(data, dict):
            return

        target_key = (
            data.get("target_key")
            or data.get("target_id")
            or data.get("active_target_key")
            or "target"
        )
        target_key = self._strip_target_prefix(target_key)
        lat = data.get("target_lat_deg")
        lon = data.get("target_lon_deg")
        z = data.get("target_z_m")
        dist = data.get("horizontal_dist_m")
        elev = data.get("elevation_deg")
        yaw = data.get("turret_yaw_deg")
        revision = data.get("solution_revision")
        in_range = bool(data.get("in_range", False))
        recalculated = bool(data.get("recalculated", False))
        trigger = data.get("recalculate_trigger", "")

        if hasattr(self, "lbl_active_target"):
            self.lbl_active_target.setText(str(target_key))
            self.lbl_target_coord.setText(
                f"{self._fmt_float(lat, 6)}, {self._fmt_float(lon, 6)} | z={self._fmt_float(z, 2, 'm')}"
            )
            self.lbl_solution_aim.setText(
                f"거리={self._fmt_float(dist, 1, 'm')} | 고각={self._fmt_float(elev, 1, '°')} | 방위={self._fmt_float(yaw, 1, '°')}"
            )
            range_text = "사거리 내" if in_range else "사거리 밖"
            calc_text = "재계산" if recalculated else "기존 해"
            self.lbl_solution_rev.setText(f"버전={revision} | {range_text} | {calc_text}")
            self.lbl_solution_rev.setStyleSheet(
                "color:#00e676;font-weight:bold;" if in_range else "color:#ff5252;font-weight:bold;"
            )

        try:
            self.map_view.update_drone_target(
                float(lat),
                float(lon),
                f"{target_key} | 거리={self._fmt_float(dist, 1, 'm')} | 고각={self._fmt_float(elev, 1, '°')}",
                self._impact_threshold_m,
            )
        except Exception:
            pass

        log_key = f"{target_key}:{revision}:{in_range}:{recalculated}:{trigger}"
        if recalculated and log_key != self._last_solution_label_key:
            self._last_solution_label_key = log_key
            self._log(
                f"[탄도해] 표적={target_key}, 버전={revision}, "
                f"거리={self._fmt_float(dist, 2, 'm')}, 고각={self._fmt_float(elev, 2, '°')}, "
                f"방위각={self._fmt_float(yaw, 2, '°')}, 재계산 원인={_ko_status(trigger)}"
            )

    def _update_heading_alignment_ui(self, data: dict):
        """coordinator가 실제 사용한 현재/목표 헤딩과 오차를 표시."""
        if not hasattr(self, "lbl_align_heading"):
            return

        cur = data.get("current_heading_deg")
        tgt = data.get("target_heading_deg")
        err = data.get("heading_error_deg")
        tol = data.get("heading_tolerance_deg")
        within = data.get("heading_within_tolerance")

        cur_s = self._fmt_float(cur, 1, "°")
        tgt_s = self._fmt_float(tgt, 1, "°")
        err_s = self._fmt_float(err, 2, "°")
        tol_s = self._fmt_float(tol, 1, "°")

        self.lbl_align_heading.setText(f"현재 {cur_s} → 목표 {tgt_s}")
        self.lbl_heading_error.setText(f"오차 {err_s} / 허용 {tol_s}")

        if within is True:
            self.lbl_heading_error.setStyleSheet("color:#00e676;font-weight:bold;")
        elif within is False:
            self.lbl_heading_error.setStyleSheet("color:#ffab40;font-weight:bold;")
        else:
            self.lbl_heading_error.setStyleSheet("color:#90a4ae;")

    def _flush_fire_sol_status_latest(self):
        if self._latest_fire_sol_status_data is None:
            return

        data = self._latest_fire_sol_status_data
        self._latest_fire_sol_status_data = None
        self._apply_fire_solution_status(data)

    # ── GNSS 직접 구독 폴백 ──────────────────────────────────────────
    _MMR_STATUS_TIMEOUT_SEC = 3.0
    _GNSS_DIRECT_TIMEOUT_SEC = 10.0

    def _flush_gnss_direct(self):
        import time as _t

        now = _t.monotonic()

        # /mmr/status 정상 수신 중이면 폴백 불필요
        if (
            self._mmr_status_stamp > 0
            and now - self._mmr_status_stamp < self._MMR_STATUS_TIMEOUT_SEC
        ):
            self._gnss_warn_logged = False
            return

        # GNSS 직접 위치 수신 여부 확인
        if self._direct_fix_lat is None or self._direct_fix_lon is None:
            if not self._gnss_warn_logged:
                self._gnss_warn_logged = True
                self._log("[GNSS] 위치 데이터 수신 대기 중입니다. GNSS 노드와 포트를 확인하십시오.")
            return

        # GNSS 직접 위치 타임아웃 체크
        fix_age = now - self._direct_fix_stamp
        if fix_age > self._GNSS_DIRECT_TIMEOUT_SEC:
            if not self._gnss_warn_logged:
                self._gnss_warn_logged = True
                self._log(
                    f"[GNSS 경고] 위치 데이터 수신이 {fix_age:.1f}초 동안 끊겼습니다 — "
                    "안테나·USB 포트·GNSS 노드 실행 상태를 확인하십시오."
                )
            self.lbl_mmr_pos.setText("GNSS 수신 끊김")
            self.lbl_mmr_hdg.setText("— [위치 고정 없음]")
            return

        # heading은 현재 수신 주기가 느릴 수 있으므로 12초까지 마지막 값을 유지
        self._gnss_warn_logged = False

        hdg_age = now - self._direct_hdg_stamp if self._direct_hdg_stamp > 0 else 999.0
        hdg_stale = hdg_age >= 12.0
        hdg = self._direct_heading_deg

        self._apply_gnss_direct(
            self._direct_fix_lat,
            self._direct_fix_lon,
            hdg,
            self._direct_fix_qual,
            self._direct_fix_source,
            hdg_stale=hdg_stale,
        )

    def _apply_gnss_direct(
        self,
        lat: float,
        lon: float,
        hdg: float,
        fix_qual: str,
        fix_source: str = "GNSS_DIRECT",
        hdg_stale: bool = False,
    ):
        """
        /gnss/fix_hp + /gnss/heading 직접 수신 데이터로 지도·HUD 갱신.
        fix 품질 표시는 /gnss/fix_pvt 기준을 사용한다.
        /mmr/status 폴백 경로 전용.
        """
        self._latest_mmr_pose = (lat, lon, hdg)
        self.map_view.update_mmr(lat, lon, hdg)

        # 발표/캡처 화면에서는 내부 토픽 출처를 노출하지 않고 좌표만 표시한다.
        self.lbl_mmr_pos.setText(f"{lat:.7f},{lon:.7f}")

        hdg_label = f"{hdg:.1f}° [{fix_qual}]"
        if hdg_stale:
            hdg_label += "  ⚠️헤딩 갱신 끊김"
        self.lbl_mmr_hdg.setText(hdg_label)

        self._redraw_range_sector()

    def _apply_fire_solution_status(self, data: dict):
        status = data.get("status", str(data))

        if status in ("TARGET_POINT_MISSING", "TARGET_POINT_STALE"):
            return

        if status == self._last_fire_sol_status_logged:
            return

        self._last_fire_sol_status_logged = status
        self._log(f"[탄도해 상태] {_ko_status(status)} (코드={status})")

    # ================================================================
    # UI 빌드
    # ================================================================
    def _build_menubar(self):
        menubar = self.menuBar()
        view_menu = menubar.addMenu("보기")
        view_menu.addAction("지도 중심을 MMR로", self._center_map_to_mmr)

    def _center_map_to_mmr(self):
        if self._latest_mmr_pose is not None:
            lat, lon, hdg = self._latest_mmr_pose
            self.map_view._run_js(
                f"map.setView([{lat}, {lon}], map.getZoom());"
            )

    def _initialize_local_diagnostic_cursor(self):
        path = str(self._local_diag_event_path or "").strip()
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            self._last_local_diag_event_seq = int(payload.get("seq", 0) or 0)
        except Exception:
            self._last_local_diag_event_seq = 0

    def _flush_local_diagnostic_event(self):
        path = str(self._local_diag_event_path or "").strip()
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            seq = int(payload.get("seq", 0))
            if seq <= 0 or seq == self._last_local_diag_event_seq:
                return
            self._last_local_diag_event_seq = seq
            message = str(payload.get("message", "")).strip()
            if message:
                self._log(message)
        except FileNotFoundError:
            return
        except Exception:
            # Local-only debug channel must never disturb the operator GUI.
            return

    def _build_log_panel(self) -> QTextEdit:
        te = QTextEdit()
        te.setReadOnly(True)
        te.setMinimumHeight(150)

        # 로그가 무제한 누적되면 장시간 실행 시 QTextEdit 자체가 느려질 수 있으므로
        # 최근 500줄만 유지한다. GUI 기능과 ROS 토픽 처리에는 영향 없음.
        te.document().setMaximumBlockCount(500)

        te.setStyleSheet("""
            QTextEdit { background-color: #0e0e0e; color: #cfcfcf;
                        border: 2px solid #444; border-radius: 8px; }
        """)
        return te

    def _build_hud_panel(self) -> QWidget:
        panel = QFrame()
        panel.setStyleSheet("""
            QFrame { background-color: #101317; border: 2px solid #444; border-radius: 8px; }
            QLabel { color: #dcdcdc; }
        """)

        outer = QVBoxLayout(panel)
        outer.setContentsMargins(6, 4, 6, 4)
        outer.setSpacing(2)

        def make_chip(title_text: str, value_text: str = "—", *,
                      min_w: int = 90, value_min_w: int = 50,
                      value_font_size: int = 10, title_color: str = "#8fa3b8",
                      value_color: str = "#dcdcdc"):
            """한 줄 요약 바용 compact chip.

            기존 QLabel 변수는 그대로 유지하고, 레이아웃만 압축한다.
            """
            frame = QFrame()
            frame.setMinimumHeight(24)
            frame.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
            frame.setStyleSheet("""
                QFrame { background-color:#11181d; border:1px solid #34434a; border-radius:6px; }
            """)
            lay = QHBoxLayout(frame)
            lay.setContentsMargins(5, 1, 5, 1)
            lay.setSpacing(4)

            title = QLabel(title_text)
            title.setFont(QFont("Arial", 9, QFont.Weight.DemiBold))
            title.setStyleSheet(f"color:{title_color}; border:none; background:transparent;")
            title.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

            value = QLabel(value_text)
            value.setMinimumWidth(value_min_w)
            value.setFont(QFont("Consolas", value_font_size, QFont.Weight.Bold))
            value.setStyleSheet(f"color:{value_color}; border:none; background:transparent;")
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            value.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

            lay.addWidget(title)
            lay.addWidget(value, 1)
            frame.setMinimumWidth(min_w)
            return frame, value

        def make_compound_chip(items: list[tuple[str, QLabel, int]], *,
                           min_w: int = 120, value_font_size: int = 9,
                           title_color: str = "#8fa3b8") -> QFrame:
            """여러 값을 한 chip에 묶어 한 줄 정보바 폭 낭비를 줄인다."""
            frame = QFrame()
            frame.setMinimumHeight(23)
            frame.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
            frame.setStyleSheet("""
                QFrame { background-color:#11181d; border:1px solid #34434a; border-radius:6px; }
            """)
            lay = QHBoxLayout(frame)
            lay.setContentsMargins(5, 1, 5, 1)
            lay.setSpacing(4)

            for idx, (title_text, value_label, value_min_w) in enumerate(items):
                if idx > 0:
                    sep = QLabel("|")
                    sep.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
                    sep.setStyleSheet("color:#455a64; border:none; background:transparent;")
                    lay.addWidget(sep)

                title = QLabel(title_text)
                title.setFont(QFont("Arial", 9, QFont.Weight.DemiBold))
                title.setStyleSheet(f"color:{title_color}; border:none; background:transparent;")
                title.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
                lay.addWidget(title)

                value_label.setMinimumWidth(value_min_w)
                value_label.setFont(QFont("Consolas", value_font_size, QFont.Weight.Bold))
                value_label.setStyleSheet("color:#dcdcdc; border:none; background:transparent;")
                value_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
                value_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
                lay.addWidget(value_label, 1)

            frame.setMinimumWidth(min_w)
            return frame

        # ── 1줄 요약: MMR 위치/헤딩 상태 ───────────────────────────────
        # 짧고 우선순위가 높은 차량 상태는 가장 위로 올린다.
        mmr_row = QHBoxLayout()
        mmr_row.setSpacing(5)

        self._mmr_pos_chip, self.lbl_mmr_pos = make_chip(
            "MMR", "—", min_w=205, value_min_w=140,
            value_font_size=10, title_color="#80deea", value_color="#00e5ff")
        self._mmr_hdg_chip, self.lbl_mmr_hdg = make_chip(
            "헤딩/위치품질", "—", min_w=245, value_min_w=170,
            value_font_size=10, title_color="#80deea", value_color="#00e5ff")
        self._align_chip, self.lbl_align_heading = make_chip(
            "정렬", "—", min_w=105, value_min_w=55,
            value_font_size=10, title_color="#80deea", value_color="#ffd54f")
        self._err_chip, self.lbl_heading_error = make_chip(
            "오차", "—", min_w=95, value_min_w=48,
            value_font_size=10, title_color="#80deea", value_color="#90a4ae")

        mmr_row.addWidget(self._mmr_pos_chip, 1)
        mmr_row.addWidget(self._mmr_hdg_chip, 2)
        mmr_row.addWidget(self._align_chip, 0)
        mmr_row.addWidget(self._err_chip, 0)
        mmr_row.addStretch(1)

        # ── 2줄 요약: 클릭/표적/탄도 상태 ─────────────────────────────
        # 실제 좌표 문자열이 길어 1줄 압축 시 잘리므로 2줄로 나눈다.
        # 1행은 짧은 식별 정보, 2행은 긴 좌표/계산 상태를 넓게 표시한다.
        summary_block = QVBoxLayout()
        summary_block.setSpacing(2)
        summary_row_1 = QHBoxLayout()
        summary_row_1.setSpacing(5)
        summary_row_2 = QHBoxLayout()
        summary_row_2.setSpacing(5)

        self.lbl_target = QLabel("—")
        self.lbl_demz = QLabel("—")
        click_chip = make_compound_chip(
            [("지도 클릭", self.lbl_target, 155), ("고도", self.lbl_demz, 42)],
            min_w=260, value_font_size=9, title_color="#8fa3b8")

        self.lbl_active_target = QLabel("—")
        target_id_chip = make_compound_chip(
            [("표적", self.lbl_active_target, 48)],
            min_w=88, value_font_size=9, title_color="#8fa3b8")

        self.lbl_solution_rev = QLabel("—")
        rev_chip = make_compound_chip(
            [("탄도해", self.lbl_solution_rev, 125)],
            min_w=160, value_font_size=9, title_color="#8fa3b8")

        self.lbl_target_coord = QLabel("—")
        coord_chip = make_compound_chip(
            [("표적 좌표", self.lbl_target_coord, 330)],
            min_w=405, value_font_size=9, title_color="#8fa3b8")

        self.lbl_solution_aim = QLabel("—")
        solution_chip = make_compound_chip(
            [("거리/고각/방위각", self.lbl_solution_aim, 230)],
            min_w=300, value_font_size=9, title_color="#8fa3b8")

        self.lbl_impact_coord = QLabel("—")
        self.lbl_impact_result = QLabel("탄착 대기")
        impact_chip = make_compound_chip(
            [
                ("탄착", self.lbl_impact_coord, 155),
                ("판정", self.lbl_impact_result, 110),
            ],
            min_w=340, value_font_size=9, title_color="#ffb74d")
        self.lbl_impact_result.setStyleSheet(
            "color:#ffd54f; border:none; background:transparent;"
        )

        # 1행은 짧은 식별 정보 위주로 구성한다.
        summary_row_1.addWidget(click_chip, 2)
        summary_row_1.addWidget(target_id_chip, 0)
        summary_row_1.addWidget(rev_chip, 1)
        summary_row_1.addStretch(1)

        # 2행은 긴 COORD와 D/E/Y 값을 넓게 배치해 실제 탄도값이 잘리지 않게 한다.
        summary_row_2.addWidget(coord_chip, 3)
        summary_row_2.addWidget(solution_chip, 2)
        summary_row_2.addWidget(impact_chip, 2)

        summary_block.addLayout(summary_row_1)
        summary_block.addLayout(summary_row_2)

        # ── 발사/터렛 제어 및 상태 패널 ───────────────────────────────
        fire_frame = QFrame()
        fire_frame.setStyleSheet("""
            QFrame { background-color: #1a0a0a; border: 1px solid #5a1010; border-radius: 6px; }
            QLabel { color: #ef9a9a; }
        """)

        fire_outer = QVBoxLayout(fire_frame)
        fire_outer.setContentsMargins(6, 4, 6, 4)
        fire_outer.setSpacing(2)

        fire_top = QHBoxLayout()
        fire_top.setSpacing(7)
        fire_bottom = QHBoxLayout()
        fire_bottom.setSpacing(5)

        # ALIGN/AIM Hold 버튼 — press/release를 /mmr/operator_input으로 발행
        self.btn_align_aim = QPushButton("A 홀드 · 정렬/조준")
        self.btn_align_aim.setFixedWidth(135)
        self.btn_align_aim.setFixedHeight(34)
        self.btn_align_aim.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        self.btn_align_aim.pressed.connect(
            lambda: self._set_operator_button("align_aim", True))
        self.btn_align_aim.released.connect(
            lambda: self._set_operator_button("align_aim", False))

        # FIRE Hold 버튼 — coordinator가 AIM_READY 상태인지 최종 판정
        self.btn_fire = QPushButton("Y 홀드 · 발사")
        self.btn_fire.setFixedWidth(110)
        self.btn_fire.setFixedHeight(34)
        self.btn_fire.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        self.btn_fire.pressed.connect(
            lambda: self._set_operator_button("fire", True))
        self.btn_fire.released.connect(
            lambda: self._set_operator_button("fire", False))

        # 터렛 HOME 버튼 — 키 매핑과 Hold 입력 로직은 건드리지 않고 토픽만 발행한다.
        self.btn_turret_home = QPushButton("홈 복귀")
        self.btn_turret_home.setFixedWidth(82)
        self.btn_turret_home.setFixedHeight(30)
        self.btn_turret_home.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        self.btn_turret_home.clicked.connect(self._on_turret_home_clicked)
        self.btn_turret_home.setStyleSheet(
            "QPushButton { background-color:#263238; color:#e0f7fa; "
            "font-weight:bold; border-radius:6px; padding:4px 10px; }"
            "QPushButton:hover { background-color:#37474f; }"
        )

        self.btn_estop = QPushButton("비상정지")
        self.btn_estop.setFixedWidth(82)
        self.btn_estop.setFixedHeight(30)
        self.btn_estop.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        self.btn_estop.clicked.connect(self._on_estop_clicked)
        self.btn_estop.setStyleSheet(
            "QPushButton { background-color:#b71c1c; color:white; "
            "font-weight:bold; border-radius:6px; padding:4px 10px; }"
            "QPushButton:hover { background-color:#d32f2f; }"
            "QPushButton:pressed { background-color:#ff5252; }"
        )

        self.btn_estop_reset = QPushButton("정지 해제")
        self.btn_estop_reset.setFixedWidth(82)
        self.btn_estop_reset.setFixedHeight(30)
        self.btn_estop_reset.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        self.btn_estop_reset.clicked.connect(self._on_reset_clicked)
        self.btn_estop_reset.setStyleSheet(
            "QPushButton { background-color:#263238; color:#ffcc80; "
            "font-weight:bold; border-radius:6px; padding:4px 10px; }"
            "QPushButton:hover { background-color:#37474f; }"
            "QPushButton:pressed { background-color:#ffb74d; color:#111; }"
        )

        self.hold_ring = HoldProgressRing()
        self.hold_ring.setFixedSize(40, 40)
        self._update_operator_button_styles()

        # 표적 좌표 입력 토글 버튼
        # ON 상태에서 지도 우클릭 시 표적 좌표를 발행한다.
        self.btn_test_target = QPushButton("표적 좌표 입력 꺼짐")
        self.btn_test_target.setCheckable(True)
        self.btn_test_target.setFixedWidth(140)
        self.btn_test_target.setFixedHeight(30)
        self.btn_test_target.setFont(QFont("Arial", 9, QFont.Weight.Bold))
        self.btn_test_target.clicked.connect(self._on_test_target_toggle)
        self._update_test_target_ui(False)

        # 발사 상태 표시 레이블
        self.lbl_fire_status = QLabel("⬛ 대기")
        self.lbl_fire_status.setMinimumWidth(90)
        self.lbl_fire_status.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        self.lbl_fire_status.setStyleSheet("color: #546e7a; font-weight: bold;")

        fire_top.addWidget(self.btn_align_aim)
        fire_top.addWidget(self.btn_fire)
        fire_top.addWidget(self.btn_turret_home)
        fire_top.addWidget(self.btn_estop)
        fire_top.addWidget(self.btn_estop_reset)
        fire_top.addWidget(self.hold_ring)
        fire_top.addStretch(1)
        fire_top.addWidget(self.btn_test_target)
        fire_top.addWidget(self.lbl_fire_status)

        # 상태 머신 표시
        status_frame = QFrame()
        status_frame.setStyleSheet("QFrame { background-color:#140808; border:1px solid #6a1b1b; border-radius:5px; }")
        status_lay = QVBoxLayout(status_frame)
        status_lay.setContentsMargins(6, 1, 6, 1)
        status_lay.setSpacing(0)
        self.lbl_engagement_state = QLabel("상태 머신 대기")
        self.lbl_engagement_state.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.lbl_engagement_state.setStyleSheet("color:#eceff1;font-weight:bold; border:none; background:transparent;")
        self.lbl_engagement_reason = QLabel("발사 시퀀스 상태 수신 대기")
        self.lbl_engagement_reason.setFont(QFont("Consolas", 9))
        self.lbl_engagement_reason.setStyleSheet("color:#90a4ae; border:none; background:transparent;")
        self.lbl_engagement_reason.setWordWrap(False)
        self.lbl_engagement_reason.setMinimumWidth(0)
        self.lbl_engagement_reason.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed
        )
        status_lay.addWidget(self.lbl_engagement_state)
        status_lay.addWidget(self.lbl_engagement_reason)

        # 터렛/스텝모터 상태 표시
        turret_frame = QFrame()
        turret_frame.setStyleSheet("QFrame { background-color:#140808; border:1px solid #6a1b1b; border-radius:5px; }")
        turret_lay = QVBoxLayout(turret_frame)
        turret_lay.setContentsMargins(6, 1, 6, 1)
        turret_lay.setSpacing(0)
        self.lbl_turret_state = QLabel("터렛 상태 수신 대기")
        self.lbl_turret_state.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        self.lbl_turret_state.setStyleSheet("color:#90a4ae;font-weight:bold; border:none; background:transparent;")
        self.lbl_turret_state.setMinimumWidth(0)
        self.lbl_turret_state.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed
        )
        self.lbl_turret_motion = QLabel("위치도달=— 모터구동=— | 펄스 —→— | 고각 —→—")
        self.lbl_turret_motion.setFont(QFont("Consolas", 9))
        self.lbl_turret_motion.setStyleSheet("color:#b0bec5; border:none; background:transparent;")
        self.lbl_turret_motion.setMinimumWidth(0)
        self.lbl_turret_motion.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed
        )
        self.lbl_turret_error = QLabel("최근 명령=없음 | 오류=없음")
        self.lbl_turret_error.setFont(QFont("Consolas", 9))
        self.lbl_turret_error.setStyleSheet("color:#90a4ae; border:none; background:transparent;")
        self.lbl_turret_error.setWordWrap(False)
        self.lbl_turret_error.setMinimumWidth(0)
        self.lbl_turret_error.setSizePolicy(
            QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed
        )
        turret_lay.addWidget(self.lbl_turret_state)
        turret_lay.addWidget(self.lbl_turret_motion)
        turret_lay.addWidget(self.lbl_turret_error)

        fire_bottom.addWidget(status_frame, 1)
        fire_bottom.addWidget(turret_frame, 3)

        fire_outer.addLayout(fire_top)
        fire_outer.addLayout(fire_bottom)

        outer.addLayout(mmr_row)
        outer.addLayout(summary_block)
        outer.addWidget(fire_frame)

        return panel

    def _on_dem_click(self, lat: float, lon: float, z):
        self.lbl_target.setText(f"{lat:.6f}, {lon:.6f}")

        if z is None:
            self.lbl_demz.setText("데이터 없음")
            self._log(f"[지도 클릭] 위도={lat:.7f}, 경도={lon:.7f}, DEM 고도=데이터 없음")
        else:
            self.lbl_demz.setText(f"{z:.3f}")
            self._log(f"[지도 클릭] 위도={lat:.7f}, 경도={lon:.7f}, 고도={z:.3f}m")

    def set_vertical_split(self):
        total_h = max(1, self.vsplit.height())
        bottom_h = 245
        self.vsplit.setSizes([max(1, total_h - bottom_h), bottom_h])

    def _log(self, msg: str):
        self.log.append(msg)
        self.log.moveCursor(self.log.textCursor().MoveOperation.End)

    def closeEvent(self, e):
        # 창이 닫힐 때 Hold 입력이 coordinator에 남지 않도록 즉시 해제한다.
        try:
            self._release_operator_inputs()
            if self._ros_executor is not None and self._ros_node is not None:
                self._ros_executor.remove_node(self._ros_node)
                self._ros_node.destroy_node()
        except Exception:
            pass
        super().closeEvent(e)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        QTimer.singleShot(0, self.set_vertical_split)


# ---------- 실행 ----------
def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()