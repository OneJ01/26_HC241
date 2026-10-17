# PyQt6 기반: QStackedWidget(카메라/맵/2분할) + 하단 HUD(2줄)/로그 + 영상 오버레이(Target)
# [수정] MMR 절대위치 + AHRS 헤딩 지도 마커 추가
# 실행:  python main_gui.py

import sys, time
import json
import math
import hashlib
import numpy as np
import os
import cv2
import threading
from camera_frame_buffer import CameraFrameBuffer
from ros_bridge import RosBridge
try:
    from h264_receiver import H264Receiver
except Exception as e:
    H264Receiver = None
    print(f"[UI] WARNING: H264Receiver import failed: {type(e).__name__}: {e}", flush=True)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPoint, QPointF, QByteArray, QObject, pyqtSlot
from PyQt6.QtGui import (
    QShortcut, QKeySequence, QFont, QAction, QGuiApplication,
    QPainter, QPainterPath, QPen, QBrush, QPolygonF, QColor, QImage, QPixmap
)
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QStackedWidget, QTextEdit, QFrame, QSplitter, QSizePolicy,
    QPushButton, QGridLayout
)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebChannel import QWebChannel
import rasterio
from pyproj import Transformer

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DEM_PATH = os.path.expanduser("~/gcs_ws/data/dem_tin_1m_pot.tif")

# 카메라 클릭 좌표 / Raw Ray 디버그 로그 출력 여부
# 평소에는 False, 클릭 좌표/보정/Ray 문제 확인할 때만 True
DEBUG_CAMERA_RAY_LOG = False

# MMR 전체 시퀀스 상태 표시명. 등록되지 않은 값은 원문을 그대로 표시한다.
MMR_STATE_LABELS = {
    "STARTUP_WAIT_HOME": "초기 포탑 정렬",
    "IDLE": "표적 대기",
    "TARGET_CHECK": "표적/탄도해 확인",
    "REPOSITION_REQUIRED": "재배치 필요",
    "WAIT_ALIGN": "차체 정렬 대기",
    "BODY_ALIGNING": "차체 정렬 중",
    "ALIGN_COMPLETE_WAIT_RELEASE": "차체 정렬 완료",
    "FINAL_RECHECK": "정렬 최종 확인",
    "READY_FOR_AIM": "포신 조준 대기",
    "AIM_CONFIRMING": "조준 승인 중",
    "TURRET_AIMING": "포신 조준 중",
    "FINAL_AIM_VERIFY": "조준 최종 확인",
    "AIM_READY": "조준 완료",
    "FIRE_CONFIRMING": "발사 승인 중",
    "FIRING": "발사 처리 중",
    "FIRED": "발사 완료",
    "FAULT": "시스템 오류",
}

MMR_REASON_LABELS = {
    "WAITING_FOR_TURRET_HOME": "포탑 홈 대기",
    "TURRET_STATUS_STALE": "포탑 상태 미수신",
    "TURRET_STATUS_STALE_DURING_ATOMIC_ACTION": "포탑 상태 복구 대기",
    "TURRET_HOME_INVALID": "포탑 홈 무효",
    "TURRET_ESTOP": "포탑 E-STOP",
    "HOME_READY": "홈 확인 완료",
    "NO_SOLUTION": "탄도해 수신 대기",
    "TARGET_RECEIVED": "새 표적 수신",
    "TARGET_AVAILABLE": "표적 검증 중",
    "NEW_TARGET": "새 표적 확인",
    "NEXT_TARGET": "다음 표적 확인",
    "SOLUTION_READY": "탄도해 정상",
    "SOLUTION_STALE": "탄도해 오래됨",
    "INVALID_FIRE_SOLUTION": "탄도해 오류",
    "OUT_OF_RANGE": "사거리 이탈",
    "ELEVATION_OUT_OF_RANGE": "포신 고각 범위 이탈",
    "MOVE_WITH_JOYSTICK": "MMR 이동 필요",
    "POSITION_NOW_VALID": "현재 위치 유효",
    "NO_IMU": "IMU 수신 대기",
    "IMU_STALE": "IMU 상태 미수신",
    "IMU_NOT_STABLE": "IMU 안정 대기",
    "INVALID_IMU_ANGLES": "IMU 각도 오류",
    "WAITING_FOR_IMU_STABLE": "IMU 안정 대기",
    "DRIVE_NOT_STOPPED": "구동부 정지 대기",
    "WAITING_FOR_WHEELS_TO_STOP": "바퀴 정지 대기",
    "MANUAL_DRIVE_OVERRIDE": "수동 주행 우선",
    "ALIGN_BUTTON_HELD": "A HOLD 입력",
    "ALIGN_BUTTON_RELEASED": "A 버튼 해제",
    "BODY_ROTATING": "목표 방향 회전 중",
    "ALIGN_SETTLING": "정렬 안정화 중",
    "ALIGN_COMPLETE_RELEASE_BUTTON": "A 버튼 해제",
    "ALIGN_DRIFTED": "정렬 이탈",
    "YAW_OUT_AFTER_RELEASE": "해제 후 헤딩 이탈",
    "YAW_OUT_OF_TOLERANCE": "헤딩 허용오차 이탈",
    "HOLD_A_TO_AIM": "A HOLD 포신 조준",
    "AIM_HOLD_STARTED": "조준 승인 시작",
    "HOLD_TO_CONFIRM_AIM": "A HOLD 유지",
    "AIM_HOLD_CANCELLED": "조준 승인 취소",
    "TURRET_AIM_REQUESTED": "포신 이동 명령",
    "TURRET_AIM_COMPLETE": "포신 이동 완료",
    "ELEVATION_CHANGED_REAIM": "고각 변경 · 재조준",
    "WAITING_FOR_TURRET_READY": "포탑 준비 대기",
    "LOCKED_ELEVATION_MISSING": "잠금 고각 누락",
    "TARGET_CHANGED_DURING_AIM": "조준 중 표적 변경",
    "TARGET_CHANGED": "표적 변경",
    "HOLD_Y_TO_FIRE": "Y HOLD 발사 승인",
    "FIRE_HOLD_STARTED": "발사 승인 시작",
    "HOLD_TO_CONFIRM_FIRE": "Y HOLD 유지",
    "FIRE_HOLD_CANCELLED": "발사 승인 취소",
    "FIRE_REQUESTED": "발사 명령 전달",
    "FIRE_COMPLETE": "발사 완료",
    "TURRET_NOT_FIRE_READY": "포탑 발사 준비 안 됨",
    "FAULT_CLEARED_HOME_VALID": "오류 복구 · 홈 정상",
}

# -------------------------
# WGS84 LLA -> ECEF
# -------------------------
def lla_to_ecef(lon_deg: float, lat_deg: float, alt_m: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    N = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (N + alt_m) * cos_lat * math.cos(lon)
    y = (N + alt_m) * cos_lat * math.sin(lon)
    z = (N * (1.0 - e2) + alt_m) * sin_lat
    return x, y, z


# ---------- OverlayPanel ----------
class OverlayPanel(QFrame):
    def __init__(self):
        super().__init__()
        self.setStyleSheet("""
            QFrame { background-color: rgba(0,0,0,110); border: 1px solid #555; border-radius: 8px; }
            QLabel { color: #eeeeee; }
        """)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(2)
        self.lbl_main = QLabel("TARGET (x= —, y= —)  |  RANGE — m  BEARING —°  |  CONF —%")
        self.lbl_main.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
        self.lbl_sub  = QLabel("lat, lon = —, —")
        self.lbl_sub.setFont(QFont("Consolas", 9))
        lay.addWidget(self.lbl_main)
        lay.addWidget(self.lbl_sub)

    def update_text(self, x=None, y=None, rng=None, brg=None, conf=None, lat=None, lon=None):
        def fmt(v, n=2):
            return ("—" if v is None else f"{v:.{n}f}")
        self.lbl_main.setText(
            f"TARGET (x= {fmt(x,2)}, y= {fmt(y,2)})  |  RANGE {fmt(rng,0)} m  BEARING {fmt(brg,0)}°  |  CONF {fmt(conf,0)}%"
        )
        if lat is None or lon is None:
            self.lbl_sub.setText("lat, lon = —, —")
        else:
            self.lbl_sub.setText(f"lat, lon = {lat:.6f}, {lon:.6f}")


# ---------- FramedLabel ----------
class FramedLabel(QWidget):
    clicked = pyqtSignal(QPoint)

    def __init__(self, title: str, bg="#111", fg="#ddd"):
        super().__init__()
        self.setAutoFillBackground(True)
        self.setStyleSheet(f"""
            QWidget {{ background-color: {bg}; color: {fg}; border: 2px solid #444; border-radius: 8px; }}
            QLabel {{ color: {fg}; }}
        """)
        self._root = QVBoxLayout(self)
        self._root.setContentsMargins(16, 16, 16, 16)
        self._root.setSpacing(0)
        self.title = QLabel(title)
        self.title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        self._root.addWidget(self.title, 1)

    def mousePressEvent(self, e):
        self.clicked.emit(e.position().toPoint())


# ---------- CameraView ----------
class _VideoLabel(QLabel):
    clicked = pyqtSignal(QPoint, str)

    def __init__(self, text: str = ""):
        super().__init__(text)

        # Reticle 표시 옵션
        self.reticle_enabled = True

        # 실제 영상이 QLabel 안에서 표시되는 영역
        # CameraView._render_pixmap()에서 갱신됨
        self._display_rect = None  # (x, y, w, h)

    def set_display_rect(self, x: int, y: int, w: int, h: int):
        """
        QLabel 전체 영역이 아니라, 실제 카메라 영상이 표시되는 영역을 저장한다.
        KeepAspectRatio 때문에 좌우/상하 여백이 생길 수 있으므로 이 값이 필요하다.
        """
        self._display_rect = (int(x), int(y), int(w), int(h))
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.LeftButton:
            click_type = "target"
        elif e.button() == Qt.MouseButton.RightButton:
            click_type = "impact"
        else:
            return
        self.clicked.emit(e.position().toPoint(), click_type)

    def paintEvent(self, event):
        """
        카메라 영상 위에 관측병 쌍안경 느낌의 미니멀 mil-reticle을 그린다.
        - 원형 외곽선 없음
        - 화면을 많이 가리지 않는 중앙 십자선
        - 중앙은 비워두고 주변에 짧은 눈금만 표시
        """
        super().paintEvent(event)

        if not self.reticle_enabled:
            return

        if self._display_rect is None:
            return

        x0, y0, rw, rh = self._display_rect
        if rw <= 10 or rh <= 10:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # ------------------------------------------------------------
        # 실제 영상 영역 기준 중심점
        # ------------------------------------------------------------
        cx = x0 + rw / 2.0
        cy = y0 + rh / 2.0

        base = min(rw, rh)

        # reticle 전체 길이: 너무 길면 화면을 가리므로 중앙 근처만 표시
        h_len = min(base * 0.23, 220.0)
        v_len = min(base * 0.20, 190.0)

        # 중앙은 표적을 가리지 않도록 비움
        center_gap = max(12.0, base * 0.015)

        # 눈금 간격과 길이
        tick_spacing = max(18.0, base * 0.025)
        minor_tick = max(4.0, base * 0.006)
        major_tick = max(7.0, base * 0.010)

        # ------------------------------------------------------------
        # 선 스타일
        # 흰색 계열 + 반투명: 실제 영상 방해 최소화
        # ------------------------------------------------------------
        line_color = QColor(235, 245, 245, 165)
        tick_color = QColor(235, 245, 245, 155)
        center_color = QColor(255, 80, 80, 150)

        pen = QPen(line_color)
        pen.setWidth(1)
        painter.setPen(pen)

        # ------------------------------------------------------------
        # 수평선: 중앙 gap을 남기고 좌우로 분리
        # ------------------------------------------------------------
        painter.drawLine(QPointF(cx - h_len, cy), QPointF(cx - center_gap, cy))
        painter.drawLine(QPointF(cx + center_gap, cy), QPointF(cx + h_len, cy))

        # ------------------------------------------------------------
        # 수직선: 중앙 gap을 남기고 상하로 분리
        # ------------------------------------------------------------
        painter.drawLine(QPointF(cx, cy - v_len), QPointF(cx, cy - center_gap))
        painter.drawLine(QPointF(cx, cy + center_gap), QPointF(cx, cy + v_len))

        # ------------------------------------------------------------
        # 수평선 위 눈금
        # ------------------------------------------------------------
        tick_pen = QPen(tick_color)
        tick_pen.setWidth(1)
        painter.setPen(tick_pen)

        max_h_ticks = int(h_len // tick_spacing)
        for i in range(1, max_h_ticks + 1):
            tlen = major_tick if i % 3 == 0 else minor_tick

            # 오른쪽 눈금
            tx = cx + i * tick_spacing
            if tx < cx + h_len:
                painter.drawLine(QPointF(tx, cy - tlen), QPointF(tx, cy + tlen))

            # 왼쪽 눈금
            tx = cx - i * tick_spacing
            if tx > cx - h_len:
                painter.drawLine(QPointF(tx, cy - tlen), QPointF(tx, cy + tlen))

        # ------------------------------------------------------------
        # 수직선 위 눈금
        # ------------------------------------------------------------
        max_v_ticks = int(v_len // tick_spacing)
        for i in range(1, max_v_ticks + 1):
            tlen = major_tick if i % 3 == 0 else minor_tick

            # 아래쪽 눈금
            ty = cy + i * tick_spacing
            if ty < cy + v_len:
                painter.drawLine(QPointF(cx - tlen, ty), QPointF(cx + tlen, ty))

            # 위쪽 눈금
            ty = cy - i * tick_spacing
            if ty > cy - v_len:
                painter.drawLine(QPointF(cx - tlen, ty), QPointF(cx + tlen, ty))

        # ------------------------------------------------------------
        # 중앙 기준점: 표적을 가리지 않게 아주 작게만 표시
        # ------------------------------------------------------------
        center_pen = QPen(center_color)
        center_pen.setWidth(1)
        painter.setPen(center_pen)

        dot_gap = 4.0
        dot_len = 3.0

        painter.drawLine(QPointF(cx - dot_gap - dot_len, cy), QPointF(cx - dot_gap, cy))
        painter.drawLine(QPointF(cx + dot_gap, cy), QPointF(cx + dot_gap + dot_len, cy))
        painter.drawLine(QPointF(cx, cy - dot_gap - dot_len), QPointF(cx, cy - dot_gap))
        painter.drawLine(QPointF(cx, cy + dot_gap), QPointF(cx, cy + dot_gap + dot_len))

        painter.end()


class CameraView(QWidget):
    clicked = pyqtSignal(QPoint, str)

    def __init__(self, title: str = "DRONE CAMERA"):
        super().__init__()
        self.setStyleSheet(
            "QWidget { background-color: #0c1a24; border: 2px solid #444; border-radius: 8px; }"
        )
        self._src_w = None
        self._src_h = None
        self._scale = 1.0
        self._off_x = 0.0
        self._off_y = 0.0

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(8)

        self.title = QLabel(title)
        self.title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title.setFont(QFont("Arial", 16, QFont.Weight.Bold))

        self.video = _VideoLabel("No camera frame")
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.video.setStyleSheet("QLabel { background-color: #081018; color: #bbb; border: 1px solid #222; }")
        self.video.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        self.video.clicked.connect(self._on_video_click)

        root.addWidget(self.title)
        root.addWidget(self.video, 1)

        self.overlay = OverlayPanel()
        self.overlay.setParent(self)
        self.overlay.resize(560, 60)
        self.overlay.show()

        self._last_pixmap: QPixmap | None = None

    def set_frame_jpeg(self, jpg_bytes: bytes, meta: dict | None = None):
        if not jpg_bytes:
            return
        t0 = time.perf_counter()
        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        dt = time.perf_counter() - t0
        if dt > 0.06:
            try:
                mw = self.window()
                if hasattr(mw, "_drop_decode_cnt"):
                    mw._drop_decode_cnt += 1
            except Exception:
                pass
            return
        h, w, _ = bgr.shape
        qimg = QImage(bgr.data, w, h, 3*w, QImage.Format.Format_BGR888).copy()
        qimg = qimg.mirrored(True, True)
        self._src_w = qimg.width()
        self._src_h = qimg.height()
        if isinstance(meta, dict):
            try:
                mw = int(meta.get("width", self._src_w))
                mh = int(meta.get("height", self._src_h))
                if mw > 0 and mh > 0:
                    self._src_w, self._src_h = mw, mh
            except Exception:
                pass
        self._last_pixmap = QPixmap.fromImage(qimg)
        self._render_pixmap()

    def set_frame_qimg(self, qimg: QImage, meta: dict | None = None):
        if qimg is None or qimg.isNull():
            return
        self._src_w = qimg.width()
        self._src_h = qimg.height()
        if isinstance(meta, dict):
            try:
                mw = int(meta.get("width", self._src_w))
                mh = int(meta.get("height", self._src_h))
                if mw > 0 and mh > 0:
                    self._src_w, self._src_h = mw, mh
            except Exception:
                pass
        self._last_pixmap = QPixmap.fromImage(qimg)
        self._render_pixmap()

    def _render_pixmap(self):
        if self._last_pixmap is None or self._src_w is None or self._src_h is None:
            return
        lw = self.video.width()
        lh = self.video.height()
        if lw <= 2 or lh <= 2:
            return
        sw = lw / float(self._src_w)
        sh = lh / float(self._src_h)
        self._scale = min(sw, sh)
        disp_w = int(self._src_w * self._scale)
        disp_h = int(self._src_h * self._scale)
        self._off_x = (lw - disp_w) / 2.0
        self._off_y = (lh - disp_h) / 2.0
        pm = self._last_pixmap.scaled(
            disp_w, disp_h,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation
        )
        self.video.setPixmap(pm)
        # 실제 영상 표시 영역을 _VideoLabel에 전달해서 reticle이 영상 중앙에 정확히 그려지게 함
        self.video.set_display_rect(int(self._off_x), int(self._off_y), int(disp_w), int(disp_h))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.overlay.move(20, self.height() - self.overlay.height() - 20)
        self._render_pixmap()

    def _on_video_click(self, p: QPoint, click_type: str):
        if self._src_w is None or self._src_h is None:
            return
        x = p.x() - self._off_x
        y = p.y() - self._off_y
        if x < 0 or y < 0:
            return
        u = int(x / self._scale)
        v = int(y / self._scale)
        u = (self._src_w - 1) - u
        v = (self._src_h - 1) - v
        if 0 <= u < self._src_w and 0 <= v < self._src_h:
            self.clicked.emit(QPoint(u, v), click_type)


# ---------- MapBridge ----------
class MapBridge(QObject):
    map_clicked = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def onMapClick(self, lat: float, lon: float):
        self.map_clicked.emit(lat, lon)


# ---------- MapView ----------
class MapView(QFrame):
    dem_clicked = pyqtSignal(float, float, object)

    def __init__(self, dem_path: str = DEFAULT_DEM_PATH):
        super().__init__()
        self.setStyleSheet("""
            QFrame { background-color: #101317; border: 2px solid #444; border-radius: 8px; }
        """)
        self.dem_path = dem_path
        self.dem_ds = None
        self.dem_band = None
        self.to_5179 = Transformer.from_crs("EPSG:4326", "EPSG:5179", always_xy=True)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(8)

        self.title = QLabel("DEM Map")
        self.title.setStyleSheet("color:#cfd8dc;")
        self.title.setFont(QFont("Arial", 10, QFont.Weight.Bold))

        self.web = QWebEngineView()
        self.web.setStyleSheet("border:2px solid #333;border-radius:8px;")

        lay.addWidget(self.title)
        lay.addWidget(self.web, 1)

        self.bridge = MapBridge()
        self.bridge.map_clicked.connect(self._on_map_click)

        self.channel = QWebChannel()
        self.channel.registerObject("bridge", self.bridge)
        self.web.page().setWebChannel(self.channel)

        self._load_dem()
        self._load_html()

    def update_drone_pose(self, lat: float, lon: float, yaw_deg: float):
        self.web.page().runJavaScript(f"updateDrone({lat}, {lon}, {yaw_deg});")

    # [추가] MMR 위치/헤딩 갱신
    def update_mmr(self, lat: float, lon: float, heading_deg: float):
        self.web.page().runJavaScript(f"updateMmr({lat}, {lon}, {heading_deg});")
        self.title.setText(
            f"Map  ●MMR {lat:.7f}, {lon:.7f}  hdg={heading_deg:.1f}°"
        )

    def update_mission_path(self, points, current_wp=None):
        """
        points 예시:
        [
            {"seq": 0, "lat": 37.3412, "lon": 126.7312, "alt": 9.0, "cmd": "TAKEOFF"},
            {"seq": 1, "lat": 37.3411, "lon": 126.7315, "alt": 9.0, "cmd": "WAYPOINT"}
        ]
        """
        try:
            points_json = json.dumps(points, ensure_ascii=False)
            current_js = "null" if current_wp is None else str(int(current_wp))
            self.web.page().runJavaScript(
                f"showMissionPath({points_json}, {current_js});"
            )
        except Exception as e:
            self.title.setText(f"Mission path draw failed: {type(e).__name__}")

    def _load_dem(self):
        if not os.path.exists(self.dem_path):
            self.title.setText(f"DEM not found: {self.dem_path}")
            return
        self.dem_ds = rasterio.open(self.dem_path)
        self.dem_band = self.dem_ds.read(1)

    def _load_html(self):
        html = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Leaflet Map</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
  <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
  <style>
    html, body, #map { width:100%; height:100%; margin:0; padding:0; background:#000; }
  </style>
</head>
<body>
<div id="map"></div>
<script>
  var map = L.map('map').setView([37.341, 126.732], 18);

L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {
    maxZoom: 22,
    maxNativeZoom: 19,
    attribution: 'Tiles © Esri',
    updateWhenIdle: true,
    updateWhenZooming: false,
    keepBuffer: 4
  }
).addTo(map);

  var clickMarker  = null;   // DEM click marker
  var targetMarker = null;   // final target marker
  var targetRadiusCircle = null; // target-centered 15 m circle
  var impactLayers = [];     // impact markers / lines / distance labels
  var droneMarker  = null;
  var droneHeading = null;
  var mmrMarker      = null;
  var mmrHeadingLine = null;
  var missionPolyline = null;
  var missionMarkers = [];

    function makeVehicleIcon(headingDeg, fillColor, strokeColor, centerColor) {
    var svg =
        '<svg xmlns="http://www.w3.org/2000/svg" width="36" height="36" viewBox="-18 -18 36 36">' +
        '<g transform="rotate(' + headingDeg + ')">' +
            '<polygon points="0,-13 8,9 0,4 -8,9"' +
            ' fill="' + fillColor + '" stroke="' + strokeColor + '" stroke-width="1.5" opacity="0.93"/>' +
            '<circle cx="0" cy="0" r="3" fill="' + centerColor + '" opacity="0.9"/>' +
            '<line x1="0" y1="-13" x2="0" y2="-19"' +
            ' stroke="' + fillColor + '" stroke-width="2" stroke-linecap="round"/>' +
        '</g>' +
        '</svg>';
    return L.divIcon({
        html: svg, className: '',
        iconSize: [36, 36], iconAnchor: [18, 18], popupAnchor: [0, -20]
    });
    }

    function makeMmrIcon(headingDeg) {
        var fillColor = '#00e5ff';
        var strokeColor = '#004d60';
        var darkStroke = '#002f38';
        var centerColor = '#ffffff';

        var svg =
            '<svg xmlns="http://www.w3.org/2000/svg" width="42" height="42" viewBox="-21 -21 42 42">' +
            '<g transform="rotate(' + headingDeg + ')">' +

                // heading 방향 포신
                '<line x1="0" y1="-5" x2="0" y2="-17"' +
                ' stroke="' + fillColor + '" stroke-width="3" stroke-linecap="round"/>' +

                // 포신 끝
                '<rect x="-2.2" y="-20" width="4.4" height="6"' +
                ' rx="1" ry="1" fill="' + fillColor + '"' +
                ' stroke="' + darkStroke + '" stroke-width="1"/>' +

                // 좌측 트랙
                '<rect x="-14" y="-3" width="5" height="17"' +
                ' rx="1.5" ry="1.5" fill="#008fa1"' +
                ' stroke="' + darkStroke + '" stroke-width="1.3"/>' +

                // 우측 트랙
                '<rect x="9" y="-3" width="5" height="17"' +
                ' rx="1.5" ry="1.5" fill="#008fa1"' +
                ' stroke="' + darkStroke + '" stroke-width="1.3"/>' +

                // 트랙 디테일
                '<line x1="-14" y1="1" x2="-9" y2="1" stroke="' + darkStroke + '" stroke-width="0.8"/>' +
                '<line x1="-14" y1="6" x2="-9" y2="6" stroke="' + darkStroke + '" stroke-width="0.8"/>' +
                '<line x1="-14" y1="11" x2="-9" y2="11" stroke="' + darkStroke + '" stroke-width="0.8"/>' +
                '<line x1="9" y1="1" x2="14" y2="1" stroke="' + darkStroke + '" stroke-width="0.8"/>' +
                '<line x1="9" y1="6" x2="14" y2="6" stroke="' + darkStroke + '" stroke-width="0.8"/>' +
                '<line x1="9" y1="11" x2="14" y2="11" stroke="' + darkStroke + '" stroke-width="0.8"/>' +

                // 차체
                '<path d="M -8 -6 L 8 -6 L 10 -2 L 10 14 L -10 14 L -10 -2 Z"' +
                ' fill="' + fillColor + '"' +
                ' stroke="' + darkStroke + '" stroke-width="1.6" opacity="0.95"/>' +

                // 상부 장갑선
                '<path d="M -5 -3 L 5 -3 L 7 1 L 7 11 L -7 11 L -7 1 Z"' +
                ' fill="none" stroke="#00b8c8" stroke-width="1" opacity="0.85"/>' +

                // 중심점
                '<circle cx="0" cy="4" r="3.2"' +
                ' fill="' + centerColor + '"' +
                ' stroke="' + darkStroke + '" stroke-width="1"/>' +

            '</g>' +
            '</svg>';

        return L.divIcon({
            html: svg,
            className: '',
            iconSize: [42, 42],
            iconAnchor: [21, 21],
            popupAnchor: [0, -22]
        });
    }


  function makeTargetIcon() {
    var svg =
      '<svg xmlns="http://www.w3.org/2000/svg" width="42" height="42" viewBox="-21 -21 42 42">' +
        '<circle cx="0" cy="0" r="13.5" fill="rgba(0,0,0,0.20)" stroke="#ff1744" stroke-width="2.6"/>' +
        '<circle cx="0" cy="0" r="5.2" fill="none" stroke="#ff8a80" stroke-width="1.4" opacity="0.85"/>' +
        '<line x1="-20" y1="0" x2="-7" y2="0" stroke="#ff1744" stroke-width="2.6" stroke-linecap="round"/>' +
        '<line x1="7" y1="0" x2="20" y2="0" stroke="#ff1744" stroke-width="2.6" stroke-linecap="round"/>' +
        '<line x1="0" y1="-20" x2="0" y2="-7" stroke="#ff1744" stroke-width="2.6" stroke-linecap="round"/>' +
        '<line x1="0" y1="7" x2="0" y2="20" stroke="#ff1744" stroke-width="2.6" stroke-linecap="round"/>' +
        '<circle cx="0" cy="0" r="2.8" fill="#ff1744" stroke="#fff2f0" stroke-width="1.1"/>' +
      '</svg>';

    return L.divIcon({
      html: svg,
      className: '',
      iconSize: [42, 42],
      iconAnchor: [21, 21],
      popupAnchor: [0, -20]
    });
  }


  function makeImpactIcon(impactId, result) {
    var good = String(result || '').toUpperCase() === 'SUCCESS';
    var main = good ? '#00e676' : '#ff9100';
    var edge = good ? '#004d2b' : '#7a3600';
    var svg =
      '<svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="-24 -24 48 48">' +
        '<g>' +
          '<line x1="0" y1="-21" x2="0" y2="-11" stroke="' + main + '" stroke-width="3"/>' +
          '<line x1="0" y1="11" x2="0" y2="21" stroke="' + main + '" stroke-width="3"/>' +
          '<line x1="-21" y1="0" x2="-11" y2="0" stroke="' + main + '" stroke-width="3"/>' +
          '<line x1="11" y1="0" x2="21" y2="0" stroke="' + main + '" stroke-width="3"/>' +
          '<line x1="-15" y1="-15" x2="-8" y2="-8" stroke="' + main + '" stroke-width="2.5"/>' +
          '<line x1="15" y1="-15" x2="8" y2="-8" stroke="' + main + '" stroke-width="2.5"/>' +
          '<line x1="-15" y1="15" x2="-8" y2="8" stroke="' + main + '" stroke-width="2.5"/>' +
          '<line x1="15" y1="15" x2="8" y2="8" stroke="' + main + '" stroke-width="2.5"/>' +
          '<circle cx="0" cy="0" r="10" fill="' + main + '" stroke="' + edge + '" stroke-width="2" opacity="0.95"/>' +
          '<text x="0" y="4.5" text-anchor="middle" font-family="Arial" font-size="11" font-weight="bold" fill="#101010">#' + impactId + '</text>' +
        '</g>' +
      '</svg>';
    return L.divIcon({
      html: svg,
      className: '',
      iconSize: [48, 48],
      iconAnchor: [24, 24],
      popupAnchor: [0, -24]
    });
  }


  window.updateMmr = function(lat, lon, headingDeg) {
    var ll = [lat, lon];
    if (!mmrMarker) {
      mmrMarker = L.marker(ll, {
        icon: makeMmrIcon(headingDeg),
        zIndexOffset: 2800
        }).addTo(map);
      mmrMarker.bindPopup('');
    } else {
      mmrMarker.setLatLng(ll);
      mmrMarker.setIcon(makeMmrIcon(headingDeg));
    }
    mmrMarker.getPopup().setContent(
      '<b style="color:#00e5ff">MMR</b><br>' +
      'lat: '     + lat.toFixed(7)        + '<br>' +
      'lon: '     + lon.toFixed(7)        + '<br>' +
      'heading: ' + headingDeg.toFixed(1) + '&deg;'
    );
    var lineLen = 0.00008;
    var rad = headingDeg * Math.PI / 180.0;
    var lat2 = lat + lineLen * Math.cos(rad);
    var lon2 = lon + lineLen * Math.sin(rad);
    var pts = [[lat, lon], [lat2, lon2]];
    if (!mmrHeadingLine) {
      mmrHeadingLine = L.polyline(pts, { color: '#00e5ff', weight: 2, opacity: 0.8 }).addTo(map);
    } else {
      mmrHeadingLine.setLatLngs(pts);
    }
  };

    window.updateDrone = function(lat, lon, yawDeg) {
    var ll = [lat, lon];
    if (!droneMarker) {
        droneMarker = L.marker(ll, {
        icon: makeVehicleIcon(yawDeg, '#ff9800', '#7a4b00', '#ffffff'),
        zIndexOffset: 3000
        }).addTo(map);
        droneMarker.bindPopup('');
    } else {
        droneMarker.setLatLng(ll);
        droneMarker.setIcon(makeVehicleIcon(yawDeg, '#ff9800', '#7a4b00', '#ffffff'));
    }

    droneMarker.getPopup().setContent(
        '<b style="color:#ff9800">DRONE</b><br>' +
        'lat: ' + lat.toFixed(7) + '<br>' +
        'lon: ' + lon.toFixed(7) + '<br>' +
        'yaw: ' + yawDeg.toFixed(1) + '&deg;'
    );

    if (droneHeading) {
        map.removeLayer(droneHeading);
        droneHeading = null;
    }
    };

window.showMissionPath = function(points, currentSeq) {
  if (missionPolyline) {
    map.removeLayer(missionPolyline);
    missionPolyline = null;
  }

  for (var i = 0; i < missionMarkers.length; i++) {
    map.removeLayer(missionMarkers[i]);
  }
  missionMarkers = [];

  if (!points || points.length === 0) {
    return;
  }

  function missionColorByCmd(cmd, isCurrent) {
    cmd = String(cmd || '').toUpperCase();

    if (isCurrent) {
      return {
        bg: '#ffea00',
        border: '#ffffff',
        text: '#111827',
        shadow: '0 0 8px rgba(255,234,0,0.75)'
      };
    }

    if (cmd.indexOf('TAKEOFF') >= 0) {
      return {
        bg: '#22c55e',
        border: '#bbf7d0',
        text: '#04130a',
        shadow: '0 0 7px rgba(34,197,94,0.60)'
      };
    }

    if (cmd.indexOf('LAND') >= 0) {
      return {
        bg: '#ef4444',
        border: '#fecaca',
        text: '#ffffff',
        shadow: '0 0 7px rgba(239,68,68,0.60)'
      };
    }

    if (cmd.indexOf('RTL') >= 0) {
      return {
        bg: '#f97316',
        border: '#fed7aa',
        text: '#111827',
        shadow: '0 0 7px rgba(249,115,22,0.60)'
      };
    }

    return {
      bg: '#00bcd4',
      border: '#cffafe',
      text: '#06151a',
      shadow: '0 0 8px rgba(0,229,255,0.55)'
    };
  }

  function missionLabelByCmd(seq, cmd) {
    cmd = String(cmd || '').toUpperCase();

    if (cmd.indexOf('TAKEOFF') >= 0) {
      return 'T';
    }

    if (cmd.indexOf('LAND') >= 0) {
      return 'L';
    }

    if (cmd.indexOf('RTL') >= 0) {
      return 'R';
    }

    return String(seq);
  }

  function makeMissionWpIcon(seq, cmd, isCurrent) {
    cmd = String(cmd || '').toUpperCase();

    var c = missionColorByCmd(cmd, isCurrent);
    var label = missionLabelByCmd(seq, cmd);

    var isActionCmd =
      cmd.indexOf('TAKEOFF') >= 0 ||
      cmd.indexOf('LAND') >= 0 ||
      cmd.indexOf('RTL') >= 0;

    // 일반 WAYPOINT는 조금 크게, TAKEOFF/LAND/RTL은 작게
    var size = isActionCmd ? 31 : 38;
    var fontSize = isActionCmd ? 17 : 19;
    var borderWidth = isCurrent ? 3 : 2;

    if (isCurrent) {
      size = isActionCmd ? 35 : 42;
      fontSize = isActionCmd ? 19 : 21;
    }

    var html =
      '<div style="' +
        'width:' + size + 'px;' +
        'height:' + size + 'px;' +
        'border-radius:50%;' +
        'background:' + c.bg + ';' +
        'border:' + borderWidth + 'px solid ' + c.border + ';' +
        'box-shadow:' + c.shadow + ';' +
        'display:flex;' +
        'align-items:center;' +
        'justify-content:center;' +
        'font-size:' + fontSize + 'px;' +
        'font-weight:900;' +
        'font-family:Arial, sans-serif;' +
        'color:' + c.text + ';' +
        'text-shadow:0 1px 1px rgba(0,0,0,0.35);' +
      '">' + label + '</div>';

    return L.divIcon({
      className: '',
      html: html,
      iconSize: [size, size],
      iconAnchor: [size / 2, size / 2],
      popupAnchor: [0, -size / 2]
    });
  }

  var latlngs = [];

  // 경로선 좌표 생성
  for (var i = 0; i < points.length; i++) {
    var p = points[i];
    var lat = Number(p.lat);
    var lon = Number(p.lon);

    if (!isFinite(lat) || !isFinite(lon)) {
      continue;
    }

    if (Math.abs(lat) < 1e-9 && Math.abs(lon) < 1e-9) {
      continue;
    }

    latlngs.push([lat, lon]);
  }

  // 미션 경로선
  if (latlngs.length >= 2) {
    missionPolyline = L.polyline(latlngs, {
      color: '#00e5ff',
      weight: 5,
      opacity: 0.95,
      dashArray: '8, 7'
    }).addTo(map);
  }

  // 마커 생성
  for (var i = 0; i < points.length; i++) {
    var p = points[i];
    var seq = (p.seq !== undefined && p.seq !== null) ? Number(p.seq) : i;
    var lat = Number(p.lat);
    var lon = Number(p.lon);
    var alt = (p.alt !== undefined && p.alt !== null) ? Number(p.alt) : null;
    var cmd = p.cmd || '-';

    if (!isFinite(lat) || !isFinite(lon)) {
      continue;
    }

    if (Math.abs(lat) < 1e-9 && Math.abs(lon) < 1e-9) {
      continue;
    }

    var isCurrent = (
      currentSeq !== null &&
      currentSeq !== undefined &&
      seq === Number(currentSeq)
    );

    var cmdUpper = String(cmd || '').toUpperCase();
    var zIndex = isCurrent ? 2400 : 1800;

    // TAKEOFF / LAND / RTL 같은 액션 명령은 일반 WAYPOINT보다 위에 표시
    if (
      cmdUpper.indexOf('TAKEOFF') >= 0 ||
      cmdUpper.indexOf('LAND') >= 0 ||
      cmdUpper.indexOf('RTL') >= 0
    ) {
      zIndex = isCurrent ? 2700 : 2300;
    }

    var marker = L.marker([lat, lon], {
      icon: makeMissionWpIcon(seq, cmd, isCurrent),
      zIndexOffset: zIndex
    }).addTo(map);

    var label = missionLabelByCmd(seq, cmd);
    var titleText = 'SEQ ' + seq + ' / ' + label;

    marker.bindPopup(
      '<b style="color:#00e5ff; font-size:15px;">' + titleText + '</b><br>' +
      '<b>cmd:</b> ' + cmd + '<br>' +
      '<b>alt:</b> ' + (alt === null ? '-' : alt.toFixed(1)) + ' m<br>' +
      '<b>lat:</b> ' + lat.toFixed(7) + '<br>' +
      '<b>lon:</b> ' + lon.toFixed(7)
    );

    missionMarkers.push(marker);
  }
};

  window.showResult = function(lat, lon, text) {
    if (clickMarker) {
      clickMarker.setLatLng([lat, lon]);
      clickMarker.bindPopup(text).openPopup();
    } else {
      clickMarker = L.marker([lat, lon]).addTo(map);
      clickMarker.bindPopup(text).openPopup();
    }
  };

  window.showTargetResult = function(lat, lon, text) {
    if (!targetMarker) {
      targetMarker = L.marker([lat, lon], {
        icon: makeTargetIcon(),
        zIndexOffset: 1200
      }).addTo(map);
      targetMarker.bindPopup('');
    } else {
      targetMarker.setLatLng([lat, lon]);
      targetMarker.setIcon(makeTargetIcon());
    }
    targetMarker.getPopup().setContent(text);
    targetMarker.openPopup();

    if (!targetRadiusCircle) {
      targetRadiusCircle = L.circle([lat, lon], {
        radius: 15.0,
        color: '#ff1744',
        weight: 2.5,
        opacity: 0.95,
        fillColor: '#ff1744',
        fillOpacity: 0.12,
        interactive: false
      }).addTo(map);
    } else {
      targetRadiusCircle.setLatLng([lat, lon]);
      targetRadiusCircle.setRadius(15.0);
    }
  };

  window.clearImpactResults = function() {
    for (var i = 0; i < impactLayers.length; i++) {
      var item = impactLayers[i];
      if (item.marker) { map.removeLayer(item.marker); }
      if (item.line) { map.removeLayer(item.line); }
      if (item.label) { map.removeLayer(item.label); }
    }
    impactLayers = [];
  };

  window.showImpactResult = function(impactId, lat, lon, targetLat, targetLon, errorM, result, text) {
    var good = String(result || '').toUpperCase() === 'SUCCESS';
    var lineColor = good ? '#00e676' : '#ff1744';

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
    var labelHtml = '<div style="white-space:nowrap; padding:2px 6px; border-radius:5px;' +
      'background:rgba(10,15,20,0.88); border:1px solid ' + lineColor + ';' +
      'color:#ffffff; font:bold 12px Arial;">#' + impactId + ' ' + Number(errorM).toFixed(1) + ' m</div>';
    var label = L.marker([midLat, midLon], {
      icon: L.divIcon({
        html: labelHtml,
        className: '',
        iconSize: [100, 24],
        iconAnchor: [50, 12]
      }),
      interactive: false,
      zIndexOffset: 1600 + Number(impactId || 0)
    }).addTo(map);

    impactLayers.push({marker: marker, line: line, label: label});
  };

  new QWebChannel(qt.webChannelTransport, function(channel) {
    window.bridge = channel.objects.bridge;
  });

  map.on('click', function(e) {
    var lat = e.latlng.lat;
    var lon = e.latlng.lng;
    if (clickMarker) { map.removeLayer(clickMarker); }
    clickMarker = L.marker([lat, lon]).addTo(map);
    if (window.bridge) { window.bridge.onMapClick(lat, lon); }
  });
</script>
</body>
</html>
"""
        self.web.setHtml(html)

    def _on_map_click(self, lat: float, lon: float):
        if self.dem_ds is None or self.dem_band is None:
            self.title.setText("DEM not loaded")
            self.dem_clicked.emit(lat, lon, None)
            self.web.page().runJavaScript(f"showResult({lat}, {lon}, 'DEM not loaded');")
            return
        x, y = self.to_5179.transform(lon, lat)
        try:
            row, col = self.dem_ds.index(x, y)
        except Exception:
            self.title.setText("좌표 변환 실패")
            self.dem_clicked.emit(lat, lon, None)
            self.web.page().runJavaScript(f"showResult({lat}, {lon}, '좌표 변환 실패');")
            return
        if not (0 <= row < self.dem_band.shape[0] and 0 <= col < self.dem_band.shape[1]):
            self.title.setText("DEM 범위 밖")
            self.dem_clicked.emit(lat, lon, None)
            self.web.page().runJavaScript(f"showResult({lat}, {lon}, '데이터 없음 (범위 밖)');")
            return
        z = float(self.dem_band[row, col])
        nodata = self.dem_ds.nodata
        no_data_hit = (nodata is not None and z == nodata) or np.isnan(z)
        if no_data_hit:
            self.title.setText(f"Map - lat={lat:.7f}, lon={lon:.7f}, NoData")
            self.dem_clicked.emit(lat, lon, None)
            self.web.page().runJavaScript(f"showResult({lat}, {lon}, '데이터 없음');")
            return
        self.title.setText(f"Map - lat={lat:.7f}, lon={lon:.7f}, z={z:.3f} m")
        self.dem_clicked.emit(lat, lon, z)
        self.web.page().runJavaScript(f"showResult({lat}, {lon}, '고도: {z:.3f} m');")


# ---------- AttitudeIndicator ----------
class AttitudeIndicator(QFrame):
    def __init__(self):
        super().__init__()
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.setObjectName("AttitudeIndicator")

        # 중요:
        # 가로 minimum을 크게 잡으면 부모 FLIGHT 패널 밖으로 밀릴 수 있음.
        # 세로 높이만 확보하고, 가로는 부모 패널이 주는 폭을 그대로 사용.
        self.setMinimumSize(1, 225)
        self.setMaximumHeight(240)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("""
            QFrame#AttitudeIndicator {
                background-color: transparent;
                border: none;
            }
        """)

    def set_attitude(self, roll_deg: float | None, pitch_deg: float | None):
        self.roll_deg  = 0.0 if roll_deg  is None else float(roll_deg)
        self.pitch_deg = 0.0 if pitch_deg is None else float(pitch_deg)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        if w <= 4 or h <= 4:
            painter.end()
            return

        # 전체 위젯 밖으로 절대 못 그리게 1차 clip
        painter.setClipRect(self.rect())

        # 자세계 배경 영역
        # 오른쪽 튀어나옴 방지를 위해 위젯 내부에서만 계산
        pad_l = 6
        pad_r = 6
        pad_t = 6
        pad_b = 6

        box_x = pad_l
        box_y = pad_t
        box_w = max(10, w - pad_l - pad_r - 1)
        box_h = max(10, h - pad_t - pad_b - 1)

        cx = box_x + box_w / 2.0
        cy = box_y + box_h * 0.52

        # 원은 원형 유지. 세로 기준으로 크게 잡되 상하/좌우 잘림 방지
        r = min(box_h * 0.43, box_w * 0.22)

        # 둥근 사각형 path
        bg_path = QPainterPath()
        bg_path.addRoundedRect(
            float(box_x),
            float(box_y),
            float(box_w),
            float(box_h),
            8.0,
            8.0
        )

        # ------------------------------------------------------------
        # 1. 배경 바탕
        # ------------------------------------------------------------
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#111820"))
        painter.drawPath(bg_path)

        # ------------------------------------------------------------
        # 2. Sky / Ground
        #    핵심: 직사각형 clipRect가 아니라 둥근 bg_path에 clip
        #    그래서 오른쪽/모서리로 절대 안 튀어나감
        # ------------------------------------------------------------
        painter.save()
        painter.setClipPath(bg_path)

        painter.translate(cx, cy)
        painter.rotate(-self.roll_deg)

        pitch_px_per_deg = r / 30.0
        pitch_offset = self.pitch_deg * pitch_px_per_deg

        painter.setPen(Qt.PenStyle.NoPen)

        painter.setBrush(QColor("#2f73e0"))  # sky
        painter.drawRect(
            int(-box_w * 1.7),
            int(-box_h * 2.0 + pitch_offset),
            int(box_w * 3.4),
            int(box_h * 2.0)
        )

        painter.setBrush(QColor("#7b4b2a"))  # ground
        painter.drawRect(
            int(-box_w * 1.7),
            int(pitch_offset),
            int(box_w * 3.4),
            int(box_h * 2.0)
        )

        # horizon
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawLine(
            QPointF(-box_w * 1.7, pitch_offset),
            QPointF( box_w * 1.7, pitch_offset)
        )

        # pitch ladder
        painter.setPen(QPen(QColor("#dbe4ee"), 1))
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = pitch_offset - deg * pitch_px_per_deg
            line_half = r * (0.20 if deg % 20 else 0.30)
            painter.drawLine(QPointF(-line_half, y), QPointF(line_half, y))

        painter.restore()

        # ------------------------------------------------------------
        # 3. 원형 자세계 외곽선 / tick
        # ------------------------------------------------------------
        painter.setPen(QPen(QColor("#eef4ff"), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), r, r)

        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawLine(QPointF(cx, cy - r - 10), QPointF(cx, cy - r + 12))
        painter.drawLine(QPointF(cx, cy + r - 12), QPointF(cx, cy + r + 10))

        # 중앙 aircraft symbol
        painter.setPen(QPen(QColor("#ffd54a"), 4))
        wing = r * 0.30
        gap = r * 0.08
        painter.drawLine(QPointF(cx - wing, cy), QPointF(cx - gap, cy))
        painter.drawLine(QPointF(cx + gap, cy), QPointF(cx + wing, cy))
        painter.drawLine(QPointF(cx, cy - 10), QPointF(cx, cy + 10))

        # ------------------------------------------------------------
        # 4. ROLL / PITCH 박스
        # ------------------------------------------------------------
        text_x = box_x + 18
        text_y = box_y + 16
        text_w = 138
        text_h = 56

        text_path = QPainterPath()
        text_path.addRoundedRect(
            float(text_x),
            float(text_y),
            float(text_w),
            float(text_h),
            7.0,
            7.0
        )

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(8, 12, 20, 198))
        painter.drawPath(text_path)

        painter.setPen(QColor("#eef4ff"))
        painter.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
        painter.drawText(text_x + 12, text_y + 22, f"ROLL  {self.roll_deg:+.1f}°")
        painter.drawText(text_x + 12, text_y + 43, f"PITCH {self.pitch_deg:+.1f}°")

        # ------------------------------------------------------------
        # 5. 테두리는 마지막에 다시 그림
        #    sky/ground가 border를 덮어버리는 문제 방지
        # ------------------------------------------------------------
        painter.setPen(QPen(QColor("#405a76"), 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPath(bg_path)

        painter.end()


# ---------- CircularGauge ----------
class CircularGauge(QFrame):
    def __init__(
        self,
        title: str,
        unit: str,
        min_value: float = 0.0,
        max_value: float = 20.0,
        accent_color: str = "#00e5ff",
    ):
        super().__init__()
        self.title = str(title)
        self.unit = str(unit)
        self.min_value = float(min_value)
        self.max_value = float(max_value)
        self.accent_color = str(accent_color)
        self.value = None

        self.setObjectName("CircularGauge")
        self.setMinimumSize(150, 120)
        self.setMaximumHeight(130)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("""
            QFrame#CircularGauge {
                background-color: transparent;
                border: none;
            }
        """)

    def set_value(self, value):
        try:
            if value is None:
                self.value = None
            else:
                self.value = float(value)
        except Exception:
            self.value = None

        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0

        if self.value is None:
            ratio = 0.0
            value_text = "—"
        else:
            denom = max(1e-6, self.max_value - self.min_value)
            ratio = (self.value - self.min_value) / denom
            ratio = max(0.0, min(1.0, ratio))
            value_text = f"{self.value:.1f}"

        painter.setPen(QColor("#9fb0c3"))
        painter.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        painter.drawText(
            self.rect().adjusted(0, 0, 0, 0),
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter,
            self.title
        )

        diam = int(min(w * 0.80, h * 0.70))
        diam = max(78, diam)
        gx = int(cx - diam / 2)
        gy = 31

        bg_pen = QPen(QColor("#33414f"), 9)
        bg_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(bg_pen)
        painter.drawArc(gx, gy, diam, diam, 225 * 16, -270 * 16)

        value_pen = QPen(QColor(self.accent_color), 9)
        value_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(value_pen)
        painter.drawArc(gx, gy, diam, diam, 225 * 16, int(-270 * ratio * 16))

        painter.setPen(QColor("#f5f7fa"))
        painter.setFont(QFont("Consolas", 22, QFont.Weight.Bold))
        painter.drawText(
            self.rect().adjusted(0, 22, 0, -18),
            Qt.AlignmentFlag.AlignCenter,
            value_text
        )

        painter.setPen(QColor("#b0bec5"))
        painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        painter.drawText(
            self.rect().adjusted(0, 0, 0, -2),
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter,
            self.unit
        )

        painter.end()


# ---------- MainWindow ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("공중지상협력형 박격포 GCS PC")

        self.K = None
        self.dist = None
        calib_path = os.path.join(BASE_DIR, "calib_result.json")
        try:
            with open(calib_path, "r") as f:
                cd = json.load(f)
            self.K    = np.array(cd["K"],    dtype=np.float64)
            self.dist = np.array(cd["dist"], dtype=np.float64).reshape(1, -1)
            print(f"[UI] Loaded calib: {calib_path}", flush=True)
        except Exception as e:
            print(f"[UI] WARNING: calib load failed: {e} (path={calib_path})", flush=True)

        self.resize(1400, 800)
        self.last_target = dict(x=None, y=None, rng=None, brg=None, conf=None, lat=None, lon=None)
        self._latest_target_data = None
        self._latest_impact_data = None
        self._impact_history = []
        self._impact_counter = 0
        self._impact_threshold_m = 15.0
        self._impact_reset_ns = 0
        self._last_status_text = None
        self._last_takeoff_reason = None
        self._in_air_cached = None
        self._rel_alt_cached = None
        self._height_agl_cached = None
        # ── Mission UI 상태 변수 ─────────────────────────────────────
        self.mission_state = "IDLE"
        self.selected_mission = None
        self.mission_buttons = {}
        self._latest_mission_path_data = None
        self._mission_path_points = []
        self._mission_current_wp = None
        self._mission_fc_current_raw = None
        self._mission_fc_current_ui = None
        self._mission_last_raw_preview = []
        self._selected_mission_payload = None
        self._selected_mission_file_path = None
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 6, 10, 10)
        root.setSpacing(4)
        self.setCentralWidget(central)

        self.vsplit = QSplitter(Qt.Orientation.Vertical)
        self.vsplit.setHandleWidth(6)
        root.addWidget(self.vsplit, 1)

        # ── QStackedWidget ──────────────────────────────────────────
        self.stack = QStackedWidget()
        self.stack.setStyleSheet("""
            QStackedWidget { background-color: #0d0d12; border: 2px solid #333; border-radius: 8px; }
        """)
        self.stack.setMaximumHeight(1250)
        self.vsplit.addWidget(self.stack)

        # 페이지 0: 카메라
        self.page_camera_full = QWidget()
        cam_lay = QVBoxLayout(self.page_camera_full)
        cam_lay.setContentsMargins(0, 0, 0, 0); cam_lay.setSpacing(0)
        self.cam_full = CameraView()
        self.cam_full.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.cam_full.clicked.connect(self._demo_click_update)
        cam_lay.addWidget(self.cam_full, 1)
        self.stack.addWidget(self.page_camera_full)

        # 페이지 1: 맵 전체
        self.page_map_full = QWidget()
        map_lay = QVBoxLayout(self.page_map_full); map_lay.setContentsMargins(0, 0, 0, 0)
        self.map_full = MapView()
        self.map_full.dem_clicked.connect(self._on_dem_click)
        map_lay.addWidget(self.map_full)
        self.stack.addWidget(self.page_map_full)

        # 페이지 2: 분할
        self.page_split = QWidget()
        split_lay = QVBoxLayout(self.page_split); split_lay.setContentsMargins(0, 0, 0, 0)
        self.split_map = MapView()
        self.split_map.dem_clicked.connect(self._on_dem_click)
        self.splitter = QSplitter(Qt.Orientation.Horizontal); self.splitter.setHandleWidth(6)
        self.split_cam = CameraView()
        for w in (self.split_cam, self.split_map):
            w.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.split_cam.clicked.connect(self._demo_click_update)
        self.splitter.addWidget(self.split_cam); self.splitter.addWidget(self.split_map)
        split_lay.addWidget(self.splitter, 1)
        self.stack.addWidget(self.page_split)

        QTimer.singleShot(0, self.set_split_half)
        QTimer.singleShot(0, self.set_vertical_split)

        # ── 하단 패널 ────────────────────────────────────────────────
        self.bottom_panel = QWidget()
        self.bottom_panel.setMinimumHeight(355)
        self.bottom_panel.setMaximumHeight(470)
        bottom = QHBoxLayout(self.bottom_panel)
        bottom.setContentsMargins(0, 0, 0, 0); bottom.setSpacing(8)
        self.hud = self._build_hud_panel()
        bottom.addWidget(self.hud, 3)
        # 우측 중간: 미션 패널 + 로그를 세로로 배치
        self.side_panel = QWidget()
        side_lay = QVBoxLayout(self.side_panel)
        side_lay.setContentsMargins(0, 0, 0, 0)
        side_lay.setSpacing(8)

        self.mission_panel = self._build_mission_panel()
        side_lay.addWidget(self.mission_panel, 0)

        self.log = self._build_log_panel()
        self._last_log_msg = None  # 연속 중복 로그 방지용
        side_lay.addWidget(self.log, 1)

        bottom.addWidget(self.side_panel, 1)

        # 비행 계기 통합 패널: 대형 자세계 + 하단 속도/고도
        self.gauge_panel = QFrame()
        self.gauge_panel.setObjectName("FlightInstrumentPanel")

        # 너무 큰 minimum width를 주면 전체 하단 레이아웃을 밀 수 있음.
        # 대신 적당한 폭을 확보하고 내부 위젯은 부모 폭에 맞춰 자동 확장.
        self.gauge_panel.setMinimumWidth(600)
        self.gauge_panel.setMaximumWidth(690)
        self.gauge_panel.setStyleSheet("""
            QFrame#FlightInstrumentPanel {
                background-color: #101317;
                border: 2px solid #444;
                border-radius: 8px;
            }
            QLabel#FlightPanelTitle {
                color: #d8e7f7;
                background-color: #16212d;
                border: 1px solid #35536a;
                border-radius: 7px;
                padding: 3px 12px;
            }
        """)

        gauge_outer = QVBoxLayout(self.gauge_panel)
        gauge_outer.setContentsMargins(10, 5, 10, 7)
        gauge_outer.setSpacing(5)

        flight_title = QLabel("비행 계기판")
        flight_title.setObjectName("FlightPanelTitle")
        flight_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        flight_title.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        flight_title.setFixedHeight(30)
        gauge_outer.addWidget(flight_title, 0)

        self.att_indicator = AttitudeIndicator()
        self.att_indicator.setMinimumHeight(225)
        self.att_indicator.setMaximumHeight(240)
        self.att_indicator.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        # addWidget으로 바로 넣어서 패널 내부 폭만 사용하게 함
        gauge_outer.addWidget(self.att_indicator, 0)

        bottom_gauge_row = QHBoxLayout()
        bottom_gauge_row.setContentsMargins(34, 0, 34, 0)
        bottom_gauge_row.setSpacing(0)

        self.spd_gauge = CircularGauge("속도", "m/s", 0.0, 15.0, "#00e5ff")
        self.agl_gauge = CircularGauge("상대고도", "m", 0.0, 30.0, "#ffd54f")

        self.spd_gauge.setMinimumSize(150, 120)
        self.spd_gauge.setMaximumSize(160, 130)

        self.agl_gauge.setMinimumSize(150, 120)
        self.agl_gauge.setMaximumSize(160, 130)

        bottom_gauge_row.addWidget(self.spd_gauge, 0, Qt.AlignmentFlag.AlignLeft)
        bottom_gauge_row.addStretch(1)
        bottom_gauge_row.addWidget(self.agl_gauge, 0, Qt.AlignmentFlag.AlignRight)

        gauge_outer.addLayout(bottom_gauge_row, 0)

        bottom.addWidget(self.gauge_panel, 1)
        self.vsplit.addWidget(self.bottom_panel)

        # ── 단축키 ───────────────────────────────────────────────────
        QShortcut(QKeySequence("F1"), self, activated=lambda: self.switch_mode(0))
        QShortcut(QKeySequence("F2"), self, activated=lambda: self.switch_mode(1))
        QShortcut(QKeySequence("F3"), self, activated=lambda: self.switch_mode(2))
        QShortcut(QKeySequence("C"),  self, activated=self.copy_target_to_clipboard)
        self._shortcut_clear_impacts = QShortcut(QKeySequence("R"), self)
        self._shortcut_clear_impacts.setContext(Qt.ShortcutContext.ApplicationShortcut)
        self._shortcut_clear_impacts.activated.connect(self.clear_impacts)

        self._build_menubar()
        self._t = 0
        self.switch_mode(0)

        # ── ROS2 상태 변수 ───────────────────────────────────────────
        self._frame_buffer     = CameraFrameBuffer()
        self._ros_bridge       = None

        # H.264/GStreamer는 UI 프로세스 안에서 실행하지 않는다.
        # QtWebEngine + GStreamer native 충돌 방지를 위해 별도 bridge 프로세스 사용.
        # 최종 구조:
        #   h264_to_bgr_bridge.py  →  /camera/image_bgr_display
        #   gcs_ui.py              →  /camera/image_bgr_display 구독 후 화면 표시
        self._h264_receiver = None
        print("[UI] H.264 direct receiver disabled: external BGR bridge mode", flush=True)

        self._display_seq      = -1
        self._display_frame_t_cam_ns = None
        self._display_pts_ns = None
        self._display_sync_ok = False
        self._display_meta     = {}
        self._latest_drone_pose = None   # (lat, lon, yaw)
        self._latest_mmr_pose   = None   # (lat, lon, hdg)
        self._latest_mmr_heading_deg = None
        self._latest_mmr_data = None
        self._latest_mmr_engagement_data = None
        self._last_mmr_pose_rx_monotonic = None
        self._last_mmr_engagement_rx_monotonic = None
        self._last_mmr_link_state = None
        self._last_mmr_engagement_signature = None
        self._mmr_pose_timeout_s = 1.5
        self._mmr_engagement_timeout_s = 1.0
        self._latest_drone_data = None
        self._latest_altitude_data = None
        self._latest_target_debug_data = None
        self._latest_target_point_data = None
        self._latest_impact_debug_data = None
        self._latest_impact_point_data = None
        self._latest_mission_status_data = None

        self._rx_cnt           = 0
        self._rx_t0            = time.time()
        self._rx_fps           = 0.0
        self._rx_last_ns       = 0
        self._disp_cnt         = 0
        self._disp_t0          = time.time()
        self._disp_fps         = 0.0
        self._drop_decode_cnt  = 0
        self._drop_decode_t0   = time.time()
        self._last_render_wall = time.time()

        self._ros_bridge = RosBridge(
            frame_buffer=self._frame_buffer,
            log_fn=self._log,
            h264_receiver=self._h264_receiver,
            telemetry_callback=self._on_telemetry_received,
            altitude_callback=self._on_altitude_received,
            target_debug_callback=self._on_target_debug_received,
            target_point_callback=self._on_target_point_received,
            impact_debug_callback=self._on_impact_debug_received,
            impact_point_callback=self._on_impact_point_received,
            mmr_callback=self._on_mmr_received,
            mmr_engagement_callback=self._on_mmr_engagement_received,
            mission_status_callback=self._on_mission_status_received,
            mission_path_callback=self._on_mission_path_received,
        )
        # 1차 검증에서는 카메라 병목만 확인하기 위해 임시 비활성화
        # ROS spin을 별도 thread로 빼면, UI를 직접 만지는 callback은 위험할 수 있음.
        #self._setup_ros_telemetry()
        #self._setup_ros_mmr()
        #self._setup_ros_altitude()
        #self._setup_ros_target()
        #self._setup_ros_target_debug()

        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render_latest)
        self._render_timer.start(66) # 카메라 갱신 타이머 66:15hz
        
        self._mmr_timer = QTimer(self)
        self._mmr_timer.timeout.connect(self._flush_mmr_latest)
        self._mmr_timer.start(100)   # 10Hz MMR 표시 주기
        
        self._drone_timer = QTimer(self)
        self._drone_timer.timeout.connect(self._flush_drone_latest)
        self._drone_timer.start(100)   # 10Hz 드론 표시 주기
        
        self._altitude_timer = QTimer(self)
        self._altitude_timer.timeout.connect(self._flush_altitude_latest)
        self._altitude_timer.start(100)   # 10Hz 고도 표시 주기
        
        self._target_debug_timer = QTimer(self)
        self._target_debug_timer.timeout.connect(self._flush_target_debug_latest)
        self._target_debug_timer.start(50)   # 20Hz 역투영 상태 표시

        self._target_point_timer = QTimer(self)
        self._target_point_timer.timeout.connect(self._flush_target_point_latest)
        self._target_point_timer.start(100)  # 10Hz 최종 표적 좌표 표시

        self._impact_debug_timer = QTimer(self)
        self._impact_debug_timer.timeout.connect(self._flush_impact_debug_latest)
        self._impact_debug_timer.start(50)

        self._impact_point_timer = QTimer(self)
        self._impact_point_timer.timeout.connect(self._flush_impact_point_latest)
        self._impact_point_timer.start(100)
        
        self._mission_timer = QTimer(self)
        self._mission_timer.timeout.connect(self._flush_mission_status_latest)
        self._mission_timer.start(100)

    # ================================================================
    # ROS2 초기화
    # ================================================================
    def _setup_ros_camera(self, topic: str = "/camera/frame_packet"):
        """
        ROS2 노드를 초기화한다.
        cam_msgs 가 없어도 노드는 뜨고, 카메라 구독만 비활성화된다.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
            self._RosString = String
        except Exception as e:
            self._log(f"[ROS/ERR] ROS2 초기화 실패: {e}")
            return

        if not rclpy.ok():
            try:
                rclpy.init(args=None)
            except Exception:
                pass

        self._ros_node  = Node("gcs_ui")
        print("[CHECK] gcs_ui node created", flush=True)
        self._pub_click = self._ros_node.create_publisher(
            self._RosString, "/ui/click_event", 10)

        from rclpy.executors import SingleThreadedExecutor
        self._ros_executor = SingleThreadedExecutor()
        self._ros_executor.add_node(self._ros_node)

        self._ros_enabled = True
        self._ros_thread = threading.Thread(
            target=self._ros_spin_thread_main,
            daemon=True
        )
        self._ros_thread.start()

        # cam_msgs 는 없으면 카메라만 비활성화
        try:
            from cam_msgs.msg import FramePacket
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )

            def _pkt_cb(msg: FramePacket):
                try:
                    jpg_bytes = bytes(msg.jpg)
                    meta = {
                        "seq"             : int(msg.seq),
                        "width"           : int(msg.width),
                        "height"          : int(msg.height),
                        "t_cam_ns"        : int(msg.t_cam_ns),
                        "FrameWallClock"  : int(msg.frame_wallclock_ns),
                        "SensorTimestamp" : int(msg.sensor_timestamp_ns),
                        "ExposureTime_us" : int(msg.exposure_us),
                        "AnalogueGain"    : float(msg.analogue_gain),
                        "DigitalGain"     : float(msg.digital_gain),
                        "FrameDuration"   : int(msg.frame_duration_ns),
                        "host_time_ns"    : int(msg.host_time_ns),
                    }
                    with self._frame_lock:
                        self._latest_jpg  = jpg_bytes
                        self._latest_meta = meta

                    self._rx_cnt     += 1
                    self._rx_last_ns  = time.time_ns()
                    now = time.time()
                    if now - self._rx_t0 >= 1.0:
                        self._rx_fps = self._rx_cnt / (now - self._rx_t0)
                        self._rx_cnt = 0
                        self._rx_t0  = now
                except Exception as e:
                    self._log(f"[ROS/ERR] 카메라 프레임 처리 실패: {type(e).__name__}: {e}")

            self._ros_node.create_subscription(FramePacket, topic, _pkt_cb, qos)
            self._log(f"[시스템] 카메라 프레임 구독 활성화: {topic}")

        except Exception as e:
            self._log(f"[ROS/ERR] 카메라 구독 비활성화: cam_msgs 없음 또는 import 실패: {e}")

    def _ros_spin_thread_main(self):
        try:
            while self._ros_enabled and self._ros_executor is not None:
                self._ros_executor.spin_once(timeout_sec=0.01)
        except Exception as e:
            print(f"[ROS/SPIN/ERR] {type(e).__name__}: {e}", flush=True)

    def _setup_ros_telemetry(self, topic: str = "/drone/telemetry"):
        if self._ros_node is None:
            self._log("[ROS/ERR] 드론 텔레메트리 구독 비활성화: ROS 노드 준비 안 됨")
            return
        try:
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"[ROS/ERR] 드론 텔레메트리 구독 비활성화: {e}")
            return

        def _telemetry_cb(msg):
            try:
                data = json.loads(msg.data)
                self._latest_drone_data = data
            except Exception as e:
                self._log(f"[ROS/ERR] 텔레메트리 처리 실패: {type(e).__name__}: {e}")

        self._ros_node.create_subscription(String, topic, _telemetry_cb, 10)
        self._log(f"[시스템] 드론 텔레메트리 구독 활성화: {topic}")

    def _setup_ros_altitude(self, topic: str = "/compute/drone_altitude"):
        if self._ros_node is None:
            self._log("[ROS/ERR] 고도 정보 구독 비활성화: ROS 노드 준비 안 됨")
            return
        try:
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"[ROS/ERR] 고도 정보 구독 비활성화: {e}")
            return

        def _altitude_cb(msg):
            try:
                data = json.loads(msg.data)
                self._apply_altitude_to_hud(data)
            except Exception as e:
                self._log(f"[ROS/ERR] 고도 정보 처리 실패: {type(e).__name__}: {e}")

        self._ros_node.create_subscription(String, topic, _altitude_cb, 10)
        self._log(f"[시스템] 고도 정보 구독 활성화: {topic}")

    def _setup_ros_target(self, topic: str = "/compute/target_point"):
        if self._ros_node is None:
            self._log("[ROS/ERR] 표적 좌표 구독 비활성화: ROS 노드 준비 안 됨")
            return
        try:
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"[ROS/ERR] 표적 좌표 구독 비활성화: {e}")
            return

        def _target_cb(msg):
            try:
                data = json.loads(msg.data)

                status = data.get("status")
                if status is not None and status != "HIT":
                    self._log(f"[시스템] 표적 좌표 메시지 무시: status={status}")
                    return

                self._apply_target_result(data)

                #lat = data.get("lat_deg")
                #lon = data.get("lon_deg")
                #if lat is not None and lon is not None:
                #    self._log(f"[TARGET MAP] lat={float(lat):.7f}, lon={float(lon):.7f}")

            except Exception as e:
                self._log(f"[ROS/ERR] 표적 좌표 처리 실패: {type(e).__name__}: {e}")

        self._ros_node.create_subscription(String, topic, _target_cb, 10)
        self._log(f"[시스템] 표적 좌표 구독 활성화: {topic}")
        
    def _setup_ros_target_debug(self, topic: str = "/compute/target_debug"):
        if self._ros_node is None:
            self._log("[ROS/ERR] 역투영 디버그 구독 비활성화: ROS 노드 준비 안 됨")
            return

        try:
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"[ROS/ERR] 역투영 디버그 구독 비활성화: {e}")
            return

        def _target_debug_cb(msg):
            try:
                data = json.loads(msg.data)
                self._apply_target_debug(data)
            except Exception as e:
                self._log(f"[ROS/ERR] 역투영 디버그 처리 실패: {type(e).__name__}: {e}")

        self._ros_node.create_subscription(String, topic, _target_debug_cb, 10)
        self._log(f"[시스템] 역투영 디버그 구독 활성화: {topic}")
        
    def _on_target_debug_received(self, data: dict):
        """
        RosBridge의 ROS thread에서 호출됨.
        UI를 직접 수정하지 않고 latest 변수에만 저장한다.
        """
        self._latest_target_debug_data = data


    def _on_target_point_received(self, data: dict):
        """
        RosBridge의 ROS thread에서 호출됨.
        UI를 직접 수정하지 않고 latest 변수에만 저장한다.
        """
        self._latest_target_point_data = data

    def _on_impact_debug_received(self, data: dict):
        self._latest_impact_debug_data = data

    def _on_impact_point_received(self, data: dict):
        self._latest_impact_point_data = data
        
    # [추가] MMR 구독
    def _setup_ros_mmr(self, topic: str = "/mmr/status"):
        if self._ros_node is None:
            self._log("[ROS/ERR] MMR 상태 구독 비활성화: ROS 노드 준비 안 됨")
            return
        try:
            from std_msgs.msg import String
        except Exception as e:
            self._log(f"[ROS/ERR] MMR 상태 구독 비활성화: {e}")
            return

        def _mmr_cb(msg):
            try:
                data = json.loads(msg.data)
                self._latest_mmr_data = data
            except Exception as e:
                self._log(f"[ROS/ERR] MMR 상태 처리 실패: {type(e).__name__}: {e}")

        self._ros_node.create_subscription(String, topic, _mmr_cb, 10)
        self._log(f"[시스템] MMR 상태 구독 활성화: {topic}")

    def _on_mmr_received(self, data: dict):
        """MMR 위치/헤딩 수신. ROS thread에서는 데이터만 저장한다."""
        if not isinstance(data, dict):
            return
        self._latest_mmr_data = data
        self._last_mmr_pose_rx_monotonic = time.monotonic()

    def _on_mmr_engagement_received(self, data: dict):
        """MMR 정렬·조준·발사 대표 상태 수신."""
        if not isinstance(data, dict):
            return
        self._latest_mmr_engagement_data = data
        self._last_mmr_engagement_rx_monotonic = time.monotonic()

    def _on_mission_status_received(self, data: dict):
        """
        RosBridge의 ROS thread에서 호출됨.
        UI 직접 수정 금지. latest 변수만 갱신.
        """
        self._latest_mission_status_data = data

    def _on_mission_path_received(self, data: dict):
        """
        RosBridge의 ROS thread에서 호출됨.
        UI 직접 수정 금지. latest 변수만 갱신.
        """
        self._latest_mission_path_data = data

    def _flush_mission_status_latest(self):
        data = self._latest_mission_status_data
        if data is not None:
            self._latest_mission_status_data = None
            self._apply_mission_status(data)

        path_data = self._latest_mission_path_data
        if path_data is not None:
            self._latest_mission_path_data = None
            self._apply_mission_path(path_data)

    def _apply_mission_status(self, data: dict):
        mission_id = data.get("mission_id")
        state = data.get("state")
        current_wp = data.get("current_wp")
        self._mission_current_wp = current_wp
        total_wp = data.get("total_wp")
        message = data.get("message")

        fc_current_raw = data.get("fc_current_raw")
        fc_current_ui = data.get("fc_current_ui")
        if fc_current_raw is not None:
            self._mission_fc_current_raw = fc_current_raw
        if fc_current_ui is not None:
            self._mission_fc_current_ui = fc_current_ui

        raw_preview = data.get("raw_items_preview")
        if isinstance(raw_preview, list) and raw_preview:
            self._mission_last_raw_preview = raw_preview
            self._log("[MISSION/RAW] FC read-back raw mission items:")
            for e in raw_preview[:12]:
                try:
                    self._log(
                        "[MISSION/RAW] "
                        f"seq={e.get('raw_seq')} frame={e.get('frame')} "
                        f"cmd={e.get('cmd')}({e.get('cmd_num')}) "
                        f"cur={e.get('current')} "
                        f"x={e.get('x_raw')} y={e.get('y_raw')} z={e.get('z')} "
                        f"lat={float(e.get('lat_raw', 0.0)):.7f} "
                        f"lon={float(e.get('lon_raw', 0.0)):.7f}"
                    )
                except Exception:
                    self._log(f"[MISSION/RAW] {e}")

        if mission_id is not None:
            self.selected_mission = str(mission_id)

        if state is not None:
            self._set_mission_state(str(state))

        if current_wp is not None:
            self._mission_current_wp = current_wp

        self._update_mission_seq_cmd_display(current_wp, total_wp)
        if current_wp is not None and self._mission_path_points:
            try:
                self.map_full.update_mission_path(self._mission_path_points, current_wp)
                self.split_map.update_mission_path(self._mission_path_points, current_wp)
            except Exception as e:
                self._log(f"[MISSION/ERR] 현재 WP 강조 실패: {type(e).__name__}: {e}")

        if message:
            self._log(f"[MISSION] {message}")

        self._refresh_mission_ui()

    def _apply_mission_path(self, data: dict):
        mission_id = data.get("mission_id")
        points = (
            data.get("points")
            or data.get("waypoints")
            or data.get("path")
            or []
        )
        current_wp = data.get("current_wp")
        if current_wp is not None:
            self._mission_current_wp = current_wp

        fc_current_raw = data.get("fc_current_raw")
        fc_current_ui = data.get("fc_current_ui")
        if fc_current_raw is not None:
            self._mission_fc_current_raw = fc_current_raw
        if fc_current_ui is not None:
            self._mission_fc_current_ui = fc_current_ui

        if mission_id is not None:
            self.selected_mission = str(mission_id)

        if not isinstance(points, list):
            self._log("[MISSION/ERR] 미션 경로 형식 오류: points가 list가 아님")
            return

        self._mission_path_points = points

        # 전체 지도/분할 지도 모두 갱신
        try:
            self.map_full.update_mission_path(points, current_wp)
            self.split_map.update_mission_path(points, current_wp)
        except Exception as e:
            self._log(f"[MISSION/ERR] 지도 경로 표시 실패: {type(e).__name__}: {e}")

        total_wp = len(points)

        if current_wp is not None:
            self._mission_current_wp = current_wp
        else:
            self._mission_current_wp = None

        self._update_mission_seq_cmd_display(current_wp, total_wp)

        self._log(f"[MISSION] Mission {self.selected_mission} 경로 표시: {total_wp} WP")
        self._refresh_mission_ui()

    def _update_visible_map_drone(self, lat: float, lon: float, yaw: float):
        idx = self.stack.currentIndex()
        try:
            if idx == 1:   # Map
                self.map_full.update_drone_pose(lat, lon, yaw)
            elif idx == 2: # Split
                self.split_map.update_drone_pose(lat, lon, yaw)
        except Exception as e:
            self._log(f"[ROS/ERR] 드론 위치 지도 반영 실패: {type(e).__name__}: {e}")


    def _update_visible_map_mmr(self, lat: float, lon: float, hdg: float):
        idx = self.stack.currentIndex()
        try:
            if idx == 1:   # Map
                self.map_full.update_mmr(lat, lon, hdg)
            elif idx == 2: # Split
                self.split_map.update_mmr(lat, lon, hdg)
        except Exception as e:
            self._log(f"[ROS/ERR] MMR 위치 지도 반영 실패: {type(e).__name__}: {e}")

    def _flush_mmr_latest(self):
        # 위치/헤딩과 전체 시퀀스는 서로 다른 토픽이므로 각각 최신값을 반영한다.
        if self._latest_mmr_data is not None:
            data = self._latest_mmr_data
            self._latest_mmr_data = None
            self._apply_mmr_to_map(data)

        if self._latest_mmr_engagement_data is not None:
            data = self._latest_mmr_engagement_data
            self._latest_mmr_engagement_data = None
            self._apply_mmr_engagement_to_hud(data)

        # 새 메시지가 없어도 타임아웃에 따라 LINK 상태를 갱신해야 한다.
        self._refresh_mmr_link_status()
        
    def _on_telemetry_received(self, data: dict):
        """
        RosBridge의 ROS thread에서 호출됨.
        UI를 직접 수정하지 말고 latest 변수에만 저장한다.
        실제 HUD 반영은 QTimer가 _flush_drone_latest()에서 처리한다.
        """
        self._latest_drone_data = data
        
    def _on_altitude_received(self, data: dict):
        """
        RosBridge의 ROS thread에서 호출됨.
        UI를 직접 수정하지 않고 latest 변수에만 저장한다.
        실제 HUD 반영은 QTimer에서 처리한다.
        """
        self._latest_altitude_data = data        

    def _flush_drone_latest(self):
        if self._latest_drone_data is None:
            return

        data = self._latest_drone_data
        self._latest_drone_data = None
        self._apply_telemetry_to_hud(data)
        
    def _flush_altitude_latest(self):
        if self._latest_altitude_data is None:
            return

        data = self._latest_altitude_data
        self._latest_altitude_data = None
        self._apply_altitude_to_hud(data)

    def _flush_target_debug_latest(self):
        if self._latest_target_debug_data is None:
            return

        data = self._latest_target_debug_data
        self._latest_target_debug_data = None
        self._apply_target_debug(data)


    def _flush_target_point_latest(self):
        if self._latest_target_point_data is None:
            return

        data = self._latest_target_point_data
        self._latest_target_point_data = None
        self._apply_target_result(data, reset_impacts=True)

    def _flush_impact_debug_latest(self):
        if self._latest_impact_debug_data is None:
            return
        data = self._latest_impact_debug_data
        self._latest_impact_debug_data = None
        self._apply_impact_debug(data)

    def _flush_impact_point_latest(self):
        if self._latest_impact_point_data is None:
            return
        data = self._latest_impact_point_data
        self._latest_impact_point_data = None
        self._apply_impact_result(data)

    # [추가] MMR 위치/헤딩 → 지도 + HUD
    def _apply_mmr_to_map(self, data: dict):
        lat = self._safe_float(data.get("lat_deg"))
        lon = self._safe_float(data.get("lon_deg"))
        hdg = self._safe_float(data.get("heading_deg"))

        if hdg is not None:
            self._latest_mmr_heading_deg = hdg % 360.0

        if lat is not None and lon is not None:
            map_hdg = self._latest_mmr_heading_deg or 0.0
            self._latest_mmr_pose = (lat, lon, map_hdg)
            self._update_visible_map_mmr(lat, lon, map_hdg)
            # 4칸 확장 시 폭을 확보하기 위해 위도/경도를 두 줄로 표시한다.
            self.lbl_mmr_pos.setText(f"{lat:.7f}\n{lon:.7f}")

    @staticmethod
    def _safe_float(value):
        if value is None or isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _clamp_progress(value):
        number = MainWindow._safe_float(value)
        if number is None:
            return None
        return max(0.0, min(1.0, number))

    @staticmethod
    def _fmt_deg(value, signed=False):
        number = MainWindow._safe_float(value)
        if number is None:
            return "—"
        return f"{number:+.1f}°" if signed else f"{number:.1f}°"

    def _apply_mmr_engagement_to_hud(self, data: dict):
        raw_state = str(data.get("state") or "UNKNOWN").strip() or "UNKNOWN"
        raw_reason = str(data.get("reason") or "—").strip() or "—"
        state_text = MMR_STATE_LABELS.get(raw_state, raw_state)
        reason_text = MMR_REASON_LABELS.get(raw_reason, raw_reason)

        self.lbl_mmr_state.setText(state_text)
        self.lbl_mmr_detail.setText(self._build_mmr_detail(data, raw_state, reason_text))
        self._set_mmr_state_colors(raw_state)

        signature = (raw_state, raw_reason)
        if signature != self._last_mmr_engagement_signature:
            self._last_mmr_engagement_signature = signature
            self._log(f"[MMR] {raw_state} / {raw_reason}")

    def _build_mmr_detail(self, data: dict, state: str, reason_text: str) -> str:
        if state == "BODY_ALIGNING":
            target = self._fmt_deg(data.get("target_heading_deg"))
            error = self._fmt_deg(data.get("heading_error_deg"), signed=True)
            return f"목표 {target}\n오차 {error}"

        if state == "ALIGN_COMPLETE_WAIT_RELEASE":
            return "정렬 완료\nA 버튼 해제"

        if state == "READY_FOR_AIM":
            return "A HOLD\n포신 조준"

        if state == "AIM_CONFIRMING":
            progress = self._clamp_progress(data.get("aim_hold_progress"))
            return f"A HOLD {progress * 100:.0f}%" if progress is not None else reason_text

        if state == "TURRET_AIMING":
            elev = self._fmt_deg(data.get("corrected_elevation_deg"))
            return f"포신 이동 중\n고각 {elev}"

        if state == "FINAL_AIM_VERIFY":
            elev = self._fmt_deg(data.get("locked_elevation_deg"))
            return f"조준 검증\n잠금 {elev}"

        if state == "AIM_READY":
            return "Y HOLD\n발사 승인"

        if state == "FIRE_CONFIRMING":
            progress = self._clamp_progress(data.get("fire_hold_progress"))
            return f"Y HOLD {progress * 100:.0f}%" if progress is not None else reason_text

        if state == "FIRING":
            return "포탑 응답\n대기 중"

        if state == "FIRED":
            return "발사 시퀀스\n완료"

        if state == "FAULT":
            return reason_text

        return reason_text

    def _set_mmr_state_colors(self, state: str):
        if state == "FAULT":
            color = "#ff5252"
        elif state == "REPOSITION_REQUIRED":
            color = "#ffd54f"
        elif state in ("ALIGN_COMPLETE_WAIT_RELEASE", "AIM_READY", "FIRED"):
            color = "#69f0ae"
        elif state in ("AIM_CONFIRMING", "FIRE_CONFIRMING"):
            color = "#ffd54f"
        else:
            color = "#00e5ff"
        self.lbl_mmr_state.setStyleSheet(f"color: {color};")
        self.lbl_mmr_detail.setStyleSheet(f"color: {color};")

    def _refresh_mmr_link_status(self):
        now = time.monotonic()
        pose_fresh = (
            self._last_mmr_pose_rx_monotonic is not None
            and now - self._last_mmr_pose_rx_monotonic <= self._mmr_pose_timeout_s
        )
        engagement_fresh = (
            self._last_mmr_engagement_rx_monotonic is not None
            and now - self._last_mmr_engagement_rx_monotonic
            <= self._mmr_engagement_timeout_s
        )

        if pose_fresh and engagement_fresh:
            link_state, link_color = "LINK OK", "#69f0ae"
        elif pose_fresh or engagement_fresh:
            link_state, link_color = "일부 수신", "#ffd54f"
        elif (self._last_mmr_pose_rx_monotonic is None
              and self._last_mmr_engagement_rx_monotonic is None):
            link_state, link_color = "LINK WAIT", "#b0bec5"
        else:
            link_state, link_color = "LINK LOST", "#ff5252"

        heading_text = (
            f"{self._latest_mmr_heading_deg:.1f}°"
            if pose_fresh and self._latest_mmr_heading_deg is not None
            else "—"
        )
        self.lbl_mmr_hdg.setText(f"{heading_text}\n{link_state}")
        self.lbl_mmr_hdg.setStyleSheet(f"color: {link_color};")

        if not pose_fresh:
            self.lbl_mmr_pos.setText("—\n—")

        if not engagement_fresh:
            never_received = self._last_mmr_engagement_rx_monotonic is None
            if never_received:
                self.lbl_mmr_state.setText("상태 수신 대기")
                lost_color = "#b0bec5"
            else:
                self.lbl_mmr_state.setText(
                    "상태 미수신" if pose_fresh else "MMR 통신 끊김"
                )
                lost_color = "#ffd54f" if pose_fresh else "#ff5252"
            self.lbl_mmr_detail.setText("동작 상태\n수신 대기")
            self.lbl_mmr_state.setStyleSheet(f"color: {lost_color};")
            self.lbl_mmr_detail.setStyleSheet(f"color: {lost_color};")

        if link_state != self._last_mmr_link_state:
            if self._last_mmr_link_state is not None:
                self._log(f"[MMR] {self._last_mmr_link_state} → {link_state}")
            self._last_mmr_link_state = link_state

    # ================================================================
    # 기존 메서드
    # ================================================================
    def set_vertical_split(self):
        total_h = max(1, self.vsplit.height())
        bottom_h = 390
        top_h = max(1, total_h - bottom_h)
        self.vsplit.setSizes([top_h, bottom_h])

    def _render_latest(self):
        idx = self.stack.currentIndex()

        # 카메라 화면이 보이는 모드가 아니면 디코딩하지 않음
        if idx not in (0, 2):
            return
        payload, meta = self._frame_buffer.take_latest()
        if payload is None:
            return

        fmt = "jpeg"
        if isinstance(meta, dict):
            fmt = meta.get("_frame_format", "jpeg")

        # ------------------------------------------------------------
        # 1) 기존 JPEG FramePacket 경로
        # ------------------------------------------------------------
        if fmt == "jpeg":
            t0 = time.perf_counter()

            arr = np.frombuffer(payload, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return

            dt = time.perf_counter() - t0
            if dt > 0.06:
                self._drop_decode_cnt += 1
                return

        # ------------------------------------------------------------
        # 2) 신규 H.264 decoded BGR 경로
        # ------------------------------------------------------------
        elif fmt == "bgr":
            bgr = payload
            if bgr is None:
                return

        else:
            self._log(f"[ROS/ERR] 알 수 없는 카메라 프레임 형식: {fmt}")
            return

        h, w, _ = bgr.shape
        qimg = QImage(bgr.data, w, h, 3 * w, QImage.Format.Format_BGR888).copy()
        qimg = qimg.mirrored(True, True)

        if idx == 0:
            self.cam_full.set_frame_qimg(qimg, meta)
        elif idx == 2:
            self.split_cam.set_frame_qimg(qimg, meta)
        else:
            return

        # 실제 화면 표시 성공 후 현재 display frame meta 갱신
        self._display_seq = int(meta.get("seq", -1)) if isinstance(meta, dict) else -1
        self._display_meta = meta if isinstance(meta, dict) else {}

        self._display_frame_t_cam_ns = None
        self._display_pts_ns = None
        self._display_sync_ok = False

        if isinstance(meta, dict):
            self._display_frame_t_cam_ns = meta.get("frame_t_cam_ns", meta.get("t_cam_ns"))
            self._display_pts_ns = meta.get("pts_ns", meta.get("adjusted_pts_ns"))
            self._display_sync_ok = bool(meta.get("sync_ok", False))

        self._disp_cnt += 1
        self._last_render_wall = time.time()

        now = time.time()
        if now - self._disp_t0 >= 1.0:
            self._disp_fps = self._disp_cnt / (now - self._disp_t0)
            self._disp_cnt = 0
            self._disp_t0 = now

            drop_dt = now - self._drop_decode_t0
            drop_rate = self._drop_decode_cnt / drop_dt if drop_dt > 0 else 0.0
            self._drop_decode_cnt = 0
            self._drop_decode_t0 = now

            rx_fps = self._frame_buffer.get_rx_fps()

            if isinstance(self._display_meta, dict):
                sync_ok = self._display_meta.get("sync_ok")
                seq = self._display_meta.get("seq", -1)
                match_dt = self._display_meta.get("pts_match_dt_ms")
                fmt = self._display_meta.get("_frame_format", "?")

                if match_dt is None:
                    sync_txt = f"sync={sync_ok}"
                else:
                    try:
                        sync_txt = f"sync={sync_ok}, dt={float(match_dt):.1f}ms"
                    except Exception:
                        sync_txt = f"sync={sync_ok}"
            else:
                seq = -1
                fmt = "?"
                sync_txt = "sync=?"

            # Sync 표시 축약
            sync_state = "—"
            sync_dt_txt = ""

            if isinstance(self._display_meta, dict):
                sync_ok = self._display_meta.get("sync_ok")
                match_dt = self._display_meta.get("pts_match_dt_ms")

                if sync_ok is True:
                    sync_state = "OK"
                elif sync_ok is False:
                    sync_state = "NG"

                if match_dt is not None:
                    try:
                        sync_dt_txt = f" {float(match_dt):.0f}ms"
                    except Exception:
                        sync_dt_txt = ""

            self.lbl_link.setText(
                f"RX {rx_fps:.1f} / DISP {self._disp_fps:.1f} / S {sync_state}{sync_dt_txt}"
            )
            
    def _apply_telemetry_to_hud(self, data: dict):
        def fnum(v, digits=1):
            if v is None: return "—"
            try: return f"{float(v):.{digits}f}"
            except: return "—"
        def fint(v):
            if v is None: return "—"
            try: return str(int(v))
            except: return "—"
        self._in_air_cached = data.get("in_air")
        self._rel_alt_cached = data.get("rel_alt_m")
        flight_mode = data.get("flight_mode")
        armed       = data.get("armed")
        armed_str   = "ARM" if armed is True else ("DISARM" if armed is False else "—")
        self.lbl_mode.setText(f"{flight_mode or '—'} / {armed_str}")
        self.lbl_batt.setText(f"{fnum(data.get('battery_v'), 2)}V")
        self.lbl_gps.setText(f"{data.get('gps_fix_type','—')} / {fint(data.get('num_satellites'))} / —")

        def ok_ng(v):
            if v is True:
                return "OK"
            if v is False:
                return "NG"
            return "—"

        health = data.get("health") or {}

        # MAVSDK FC health 상태
        # L = local position OK
        # G = global position OK
        # M = magnetometer calibration OK
        # A = FC armable
        l_ok = health.get("is_local_position_ok")
        g_ok = health.get("is_global_position_ok")
        m_ok = health.get("is_magnetometer_calibration_ok")
        a_ok = health.get("is_armable")

        self.lbl_ekf.setText(
            f"{ok_ng(l_ok)} / {ok_ng(g_ok)} / "
            f"{ok_ng(m_ok)} / {ok_ng(a_ok)}"
        )

        fc_health_values = (l_ok, g_ok, m_ok, a_ok)

        if any(value is False for value in fc_health_values):
            self.lbl_ekf.setStyleSheet("color: #ff5252;")
        elif all(value is True for value in fc_health_values):
            self.lbl_ekf.setStyleSheet("color: #00e676;")
        else:
            self.lbl_ekf.setStyleSheet("color: #ffd54f;")

#        gs      = data.get("ground_speed_m_s")
 #       rel_alt = data.get("rel_alt_m")
  #      self.lbl_spd_alt.setText(f"{fnum(gs, 2)} / {fnum(rel_alt, 2)}")

        gs = data.get("ground_speed_m_s")
        self.spd_gauge.set_value(gs)

        # 우측 하단 고도 원호는 사용자 확인용 상대고도(Relative Altitude)를 표시
        rel_alt_for_gauge = data.get("rel_alt_m")
        self.agl_gauge.set_value(rel_alt_for_gauge)

        roll  = data.get("roll_deg")
        pitch = data.get("pitch_deg")
        yaw   = data.get("yaw_deg")
        self.lbl_att.setText(f"{fnum(roll,1)} / {fnum(pitch,1)} / {fnum(yaw,1)}")
        self.att_indicator.set_attitude(roll, pitch)

        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        if lat is None or lon is None:
            self.lbl_drone_latlon.setText("—")
        else:
            self.lbl_drone_latlon.setText(f"{float(lat):.6f}, {float(lon):.6f}")

        status_text = data.get("status_text")
        if status_text and status_text != self._last_status_text:
            # ArduPilot/MAVLink 원문은 영어 그대로 유지
            self._log(f"[TEL/WARN] {status_text}")
            self._last_status_text = status_text

        if lat is not None and lon is not None and yaw is not None:
            lat_f = float(lat)
            lon_f = float(lon)
            yaw_f = float(yaw)
            self._latest_drone_pose = (lat_f, lon_f, yaw_f)
            self._update_visible_map_drone(lat_f, lon_f, yaw_f)

    def _apply_altitude_to_hud(self, data: dict):
        status = data.get("takeoff_status")
        reason = data.get("takeoff_reason")

        rel_alt = data.get("rel_alt_m")
        height_agl = data.get("height_agl_m")
        drone_alt = data.get("drone_alt_amsl_m")
        home_alt = data.get("home_ground_alt_m")
        gps_fix_type = data.get("gps_fix_type")
        gps_fix_ok = data.get("gps_fix_ok")

        # 원형 AGL 게이지:
        # height_agl_m은 계산용으로 캐시만 유지하고, 원호 게이지는 상대고도만 표시
        self._height_agl_cached = height_agl
        self.agl_gauge.set_value(rel_alt)

        def fmt(v, digits=2):
            if v is None:
                return "N/A"
            try:
                return f"{float(v):.{digits}f}"
            except Exception:
                return "N/A"

        # altitude topic 기준 rel_alt 캐시도 갱신
        self._rel_alt_cached = rel_alt

        # -----------------------------
        # 0. 고도 HUD 표시
        # -----------------------------
        # GPS Fix 전: Rel 값만 표시, AGL/AMSL은 N/A
        # GPS Fix 후 HOME SET 완료: Rel / AGL / AMSL 모두 표시 가능
        self.lbl_alt.setText(f"R {fmt(rel_alt)} m")

        # -----------------------------
        # 1. 기체가 이미 떠있는지 판단
        # -----------------------------
        airborne = False

        try:
            if self._in_air_cached is True:
                airborne = True
            elif rel_alt is not None and float(rel_alt) > 1.0:
                airborne = True
            elif height_agl is not None and float(height_agl) > 1.0:
                airborne = True
        except Exception:
            pass

        # -----------------------------
        # 2. HUD 표시
        # -----------------------------
        if airborne:
            self.lbl_takeoff.setText("AIRBORNE")
            self.lbl_takeoff.setStyleSheet("color: #4fc3f7;")

        else:
            if status == "READY":
                self.lbl_takeoff.setText("READY")
                self.lbl_takeoff.setStyleSheet("color: #00e676;")

            elif status == "NOT READY":
                self.lbl_takeoff.setText("NOT READY")
                self.lbl_takeoff.setStyleSheet("color: #ff5252;")

            else:
                self.lbl_takeoff.setText("CHECKING")
                self.lbl_takeoff.setStyleSheet("color: #aaaaaa;")


        # -----------------------------
        # 3. 로그
        # -----------------------------
        # 이미 이륙한 상태에서는 takeoff ready 검사 로그를 출력하지 않음
        # rel_alt too high 값이 계속 바뀌면서 로그창을 도배하는 문제 방지
        if (not airborne) and reason and reason != self._last_takeoff_reason:
            self._log(
                f"[이륙 확인] {reason} | "
                f"rel={fmt(rel_alt)}m, agl={fmt(height_agl)}m, "
                f"amsl={fmt(drone_alt)}m, home={fmt(home_alt)}m, "
                f"gps={gps_fix_type}, ok={gps_fix_ok}"
            )
            self._last_takeoff_reason = reason
    def _on_dem_click(self, lat: float, lon: float, z):
        # 맵 클릭 결과는 DEM 확인용으로만 표시
        # Drone LatLon은 텔레메트리 전용이므로 여기서 덮어쓰지 않음

        if z is None:
            self.lbl_demz.setText("NoData")
            self._log(f"[지도 클릭] lat={lat:.7f}, lon={lon:.7f} → DEM 데이터 없음")
        else:
            self.lbl_demz.setText(f"{z:.3f}")
            self._log(f"[지도 클릭] lat={lat:.7f}, lon={lon:.7f}, DEM고도={z:.3f}m")
            
            
    def _build_header_banner(self) -> QWidget:
        """메뉴바 우측 제작/팀명 표시"""
        banner = QWidget()
        banner.setObjectName("MenuHeaderBanner")
        banner.setFixedHeight(30)
        banner.setStyleSheet("""
            QWidget#MenuHeaderBanner {
                background: transparent;
                border: none;
            }
            QLabel {
                background: transparent;
                border: none;
                color: #f1f5f9;
            }
        """)

        lay = QHBoxLayout(banner)
        lay.setContentsMargins(0, 0, 12, 0)
        lay.setSpacing(8)

        team = QLabel("공중지상협력형박격포")
        team.setAlignment(Qt.AlignmentFlag.AlignCenter)
        team.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        team.setStyleSheet("color: #f1f5f9;")
        lay.addWidget(team, 0, Qt.AlignmentFlag.AlignVCenter)

        sep1 = QLabel("|")
        sep1.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sep1.setFont(QFont("Arial", 11))
        sep1.setStyleSheet("color: #94a3b8;")
        lay.addWidget(sep1, 0, Qt.AlignmentFlag.AlignVCenter)

        logo = QLabel()
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo_path = os.path.join(BASE_DIR, "assets", "tuk_logo.png")

# 로고 크기
        if os.path.exists(logo_path):
            pix = QPixmap(logo_path)
            if not pix.isNull():
                logo.setPixmap(
                    pix.scaledToHeight(
                        28,
                        Qt.TransformationMode.SmoothTransformation
                    )
                )
# 배너 글자 크기
        logo.setFixedHeight(28)
        logo.setMinimumWidth(38)
        lay.addWidget(logo, 0, Qt.AlignmentFlag.AlignVCenter)

        sep2 = QLabel("|")
        sep2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sep2.setFont(QFont("Arial", 11))
        sep2.setStyleSheet("color: #94a3b8;")
        lay.addWidget(sep2, 0, Qt.AlignmentFlag.AlignVCenter)

        school = QLabel("한국공학대학교")
        school.setAlignment(Qt.AlignmentFlag.AlignCenter)
        school.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        school.setStyleSheet("color: #f1f5f9;")
        lay.addWidget(school, 0, Qt.AlignmentFlag.AlignVCenter)

        return banner
    def _build_menubar(self):
        menubar = self.menuBar()
        menubar.setFixedHeight(34)
        view_menu = menubar.addMenu("View")
        view_menu.addAction("Camera (F1)", lambda: self.switch_mode(0))
        view_menu.addAction("Map (F2)",    lambda: self.switch_mode(1))
        view_menu.addAction("Split (F3)",  lambda: self.switch_mode(2))
        self.menu_header = self._build_header_banner()
        self.menu_header.setParent(menubar)
        self.menu_header.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.menu_header.show()
        QTimer.singleShot(0, self._position_menu_header)

    def _position_menu_header(self):
        """메뉴바 중앙에 제작 표시 배치"""
        if not hasattr(self, "menu_header"):
            return

        menubar = self.menuBar()
        w = self.menu_header.sizeHint().width()
        h = self.menu_header.sizeHint().height()

        x = max(0, (menubar.width() - w) // 2)
        y = max(0, (menubar.height() - h) // 2)

        self.menu_header.setGeometry(x, y, w, h)

    def _build_log_panel(self) -> QTextEdit:
        te = QTextEdit()
        te.setReadOnly(True)
        te.document().setMaximumBlockCount(500)  # 최근 500줄만 유지
        te.setMinimumHeight(150)
        te.setStyleSheet("""
            QTextEdit { background-color: #0e0e0e; color: #cfcfcf;
                        border: 2px solid #444; border-radius: 8px; }
        """)
        return te

    
    def _build_mission_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("MissionPanel")
        panel.setStyleSheet("""
            QFrame#MissionPanel {
                background-color: #111820;
                border: 2px solid #334155;
                border-radius: 8px;
            }
            QLabel { color: #dcdcdc; }
            QPushButton {
                background-color: #1e293b;
                color: #e5e7eb;
                border: 1px solid #475569;
                border-radius: 6px;
                padding: 5px 8px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #26364a; }
            QPushButton:disabled {
                background-color: #20242b;
                color: #6b7280;
                border: 1px solid #30363d;
            }
        """)

        outer = QVBoxLayout(panel)
        outer.setContentsMargins(10, 7, 10, 8)
        outer.setSpacing(6)

        title = QLabel("섹터 미션")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        title.setStyleSheet(
            "color: #d8e7f7; "
            "background-color: #16212d; "
            "border: 1px solid #35536a; "
            "border-radius: 6px; "
            "padding: 3px;"
        )
        outer.addWidget(title)

        mission_grid = QGridLayout()
        mission_grid.setSpacing(5)

        for idx, mission_id in enumerate(("A", "B", "C", "D")):
            btn = QPushButton(f"Mission {mission_id}")
            btn.clicked.connect(
                lambda checked=False, m=mission_id: self._select_mission(m)
            )
            self.mission_buttons[mission_id] = btn
            mission_grid.addWidget(btn, idx // 2, idx % 2)

        # 사전 테스트/임의 미션용 Custom 버튼
        self.btn_mission_custom = QPushButton("Custom")
        self.btn_mission_custom.clicked.connect(
            lambda checked=False: self._select_mission("CUSTOM")
        )
        self.mission_buttons["CUSTOM"] = self.btn_mission_custom
        mission_grid.addWidget(self.btn_mission_custom, 2, 0, 1, 2)

        outer.addLayout(mission_grid)

        cmd_row = QHBoxLayout()
        cmd_row.setSpacing(5)

        self.btn_mission_upload = QPushButton("FC 업로드")
        self.btn_mission_upload.clicked.connect(self._request_mission_upload)

        cmd_row.addWidget(self.btn_mission_upload)

        outer.addLayout(cmd_row)

        status_row = QHBoxLayout()
        status_row.setSpacing(8)

        self.lbl_mission_selected = QLabel("선택: —")
        self.lbl_mission_state = QLabel("상태: IDLE")
        self.lbl_mission_wp = QLabel("SEQ: — / —")
        self.lbl_mission_fc = QLabel("FC: —")
        self.lbl_mission_cmd = QLabel("CMD: —")

        for lbl in (
            self.lbl_mission_selected,
            self.lbl_mission_state,
            self.lbl_mission_wp,
            self.lbl_mission_fc,
            self.lbl_mission_cmd,
        ):
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
            lbl.setStyleSheet("color: #cbd5e1;")
            status_row.addWidget(lbl)

        outer.addLayout(status_row)

        self._refresh_mission_ui()
        return panel

    def _set_mission_state(self, state: str):
        self.mission_state = str(state)
        self._refresh_mission_ui()

    def _mission_state_upper(self):
        return str(getattr(self, "mission_state", "IDLE")).strip().upper()

    def _is_mission_locked(self):
        """
        미션 선택/업로드 차단 상태.
        PAUSED는 드론이 아직 ARMED 상태일 가능성이 높으므로 차단한다.
        REJECTED/ABORTED/COMPLETED/READY/ERROR는 다시 선택 가능하게 둔다.
        """
        return self._mission_state_upper() in {
            "SYNCING",
            "CONNECTING",
            "CLEARING",
            "UPLOADING",
            "VERIFYING",
            "RUNNING",
            "PAUSED",
            "BUSY",
        }

    def _refresh_mission_ui(self):
        if not hasattr(self, "lbl_mission_state"):
            return

        selected = self.selected_mission or "—"
        state = self._mission_state_upper()
        locked = self._is_mission_locked()

        if selected == "—":
            self.lbl_mission_selected.setText("선택: —")
        else:
            self.lbl_mission_selected.setText(f"선택: Mission {selected}")

        self.lbl_mission_state.setText(f"상태: {state}")

        # 상태별 색상 표시
        if state == "READY":
            state_color = "#22c55e"      # 초록
        elif state == "LOADED":
            state_color = "#38bdf8"      # 하늘색
        elif state == "RUNNING":
            state_color = "#38bdf8"      # 하늘색
        elif state == "PAUSED":
            state_color = "#facc15"      # 노랑
        elif state == "COMPLETED":
            state_color = "#a78bfa"      # 보라
        elif state in ("ABORTED", "REJECTED", "ERROR"):
            state_color = "#f87171"      # 빨강
        elif state in ("SYNCING", "CONNECTING", "CLEARING", "UPLOADING", "VERIFYING", "BUSY"):
            state_color = "#fb923c"      # 주황
        else:
            state_color = "#cbd5e1"      # 기본

        self.lbl_mission_state.setStyleSheet(
            f"color: {state_color}; font-weight: bold;"
        )

        for mission_id, btn in self.mission_buttons.items():
            btn.setEnabled(not locked)

            if self.selected_mission == mission_id:
                btn.setStyleSheet(
                    "background-color: #2563eb; "
                    "color: white; "
                    "border: 1px solid #60a5fa; "
                    "border-radius: 6px; "
                    "padding: 5px 8px; "
                    "font-weight: bold;"
                )
            else:
                btn.setStyleSheet("")

        upload_allowed_states = {
            "LOADED",
            "READY",
            "COMPLETED",
            "ABORTED",
            "REJECTED",
        }

        self.btn_mission_upload.setEnabled(
            (not locked)
            and self.selected_mission is not None
            and self._selected_mission_payload is not None
            and state in upload_allowed_states
        )

    def _mission_file_path(self, mission_id: str) -> str:
        mission_id = str(mission_id).strip().upper()
        return os.path.expanduser(
            f"~/h264_gcs_ws/missions/mission_{mission_id}.waypoints"
        )

    def _parse_qgc_waypoints_text(self, text: str):
        """
        Mission Planner / QGC WPL 110 형식 파싱.
        - seq 0 HOME row는 지도 표시에서 제외
        - TAKEOFF 좌표가 0,0이면 HOME 좌표로 보정해서 T 표시
        - LAND/RTL 좌표가 0,0이면 직전 유효 좌표로 보정
        """
        cmd_map = {
            16: "WAYPOINT",
            17: "LOITER_UNLIM",
            18: "LOITER_TURNS",
            19: "LOITER_TIME",
            20: "RTL",
            21: "LAND",
            22: "TAKEOFF",
        }

        lines = text.splitlines()
        if not lines:
            raise ValueError("empty waypoint file")

        if not lines[0].strip().startswith("QGC WPL"):
            raise ValueError("not QGC WPL format")

        points = []

        home_lat = None
        home_lon = None
        home_alt = None

        last_valid_lat = None
        last_valid_lon = None
        last_valid_alt = None

        for line in lines[1:]:
            line = line.strip()
            if not line:
                continue

            cols = line.split()
            if len(cols) < 11:
                continue

            try:
                seq = int(cols[0])
                current_flag = int(cols[1])
                cmd_num = int(float(cols[3]))
                lat = float(cols[8])
                lon = float(cols[9])
                alt = float(cols[10])
            except Exception:
                continue

            # Mission Planner의 seq 0은 HOME 기준 행.
            # 지도에는 표시하지 않고 TAKEOFF 보정용으로만 저장.
            if seq == 0 and current_flag == 1:
                if abs(lat) > 1e-9 or abs(lon) > 1e-9:
                    home_lat = lat
                    home_lon = lon
                    home_alt = alt
                continue

            cmd = cmd_map.get(cmd_num, f"CMD_{cmd_num}")

            # lat/lon 0,0 보정
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                cmd_upper = cmd.upper()

                if "TAKEOFF" in cmd_upper and home_lat is not None and home_lon is not None:
                    lat = home_lat
                    lon = home_lon
                    if abs(alt) < 1e-9 and home_alt is not None:
                        alt = home_alt

                elif last_valid_lat is not None and last_valid_lon is not None:
                    lat = last_valid_lat
                    lon = last_valid_lon
                    if abs(alt) < 1e-9 and last_valid_alt is not None:
                        alt = last_valid_alt

            # 그래도 0,0이면 표시 제외
            if abs(lat) < 1e-9 and abs(lon) < 1e-9:
                continue

            points.append({
                "seq": seq,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "cmd": cmd,
                "cmd_num": cmd_num,
            })

            last_valid_lat = lat
            last_valid_lon = lon
            last_valid_alt = alt

        if not points:
            raise ValueError("no valid mission points")

        return points

    def _load_local_mission_payload(self, mission_id: str):
        """
        PC 로컬 mission_X.waypoints 파일을 읽고,
        UI 미리보기용 points와 라파 전송용 payload를 만든다.
        """
        mission_id = str(mission_id).strip().upper()
        path = self._mission_file_path(mission_id)

        if not os.path.exists(path):
            raise FileNotFoundError(path)

        with open(path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read()

        points = self._parse_qgc_waypoints_text(text)
        sha256 = hashlib.sha256(text.encode("utf-8")).hexdigest()

        payload = {
            "type": "mission_upload_request",
            "mission_id": mission_id,
            "filename": f"mission_{mission_id}.waypoints",
            "sha256": sha256,
            "waypoints_text": text,
            "point_count": len(points),
            "stamp": time.time(),
        }

        return path, points, payload

    def _get_mission_seq_max(self, total_wp=None):
        """
        Mission Planner seq가 0부터 시작할 수도 있고 1부터 시작할 수도 있으므로,
        실제 path points 안의 seq 최대값을 우선 사용한다.
        """
        if self._mission_path_points:
            seqs = []
            for p in self._mission_path_points:
                try:
                    seqs.append(int(p.get("seq", len(seqs))))
                except Exception:
                    pass

            if seqs:
                return max(seqs)

        if total_wp is not None:
            try:
                return max(0, int(total_wp) - 1)
            except Exception:
                pass

        return None
    
    def _get_mission_cmd_by_seq(self, seq):
        if seq is None:
            return None

        try:
            seq_i = int(seq)
        except Exception:
            return None

        for p in self._mission_path_points:
            try:
                p_seq = int(p.get("seq", -1))
            except Exception:
                continue

            if p_seq == seq_i:
                return p.get("cmd")

        return None

    def _format_mission_cmd(self, cmd, seq=None):
        if cmd is None:
            return "—"

        cmd_txt = str(cmd).upper()

        # MAVLink 원문/축약 모두 대응
        if "WAYPOINT" in cmd_txt:
            if seq is None:
                return "WAYPOINT"
            try:
                return f"WAYPOINT {int(seq)}"
            except Exception:
                return "WAYPOINT"

        if "TAKEOFF" in cmd_txt:
            return "TAKEOFF"

        if "LOITER" in cmd_txt:
            return "LOITER"

        if "RTL" in cmd_txt:
            return "RTL"

        if "LAND" in cmd_txt:
            return "LAND"

        return cmd_txt

    def _update_mission_fc_display(self):
        if not hasattr(self, "lbl_mission_fc"):
            return

        raw = getattr(self, "_mission_fc_current_raw", None)
        ui = getattr(self, "_mission_fc_current_ui", None)

        if raw is None and ui is None:
            self.lbl_mission_fc.setText("FC: —")
            return

        if raw is not None and ui is not None:
            self.lbl_mission_fc.setText(f"FC: raw {int(raw)}→UI {int(ui)}")
        elif raw is not None:
            self.lbl_mission_fc.setText(f"FC: raw {int(raw)}")
        else:
            self.lbl_mission_fc.setText(f"FC: UI {int(ui)}")

    def _update_mission_seq_cmd_display(self, current_seq=None, total_wp=None):
        seq_max = self._get_mission_seq_max(total_wp)

        if current_seq is None:
            cur_txt = "—"
        else:
            try:
                cur_txt = str(int(current_seq))
            except Exception:
                cur_txt = str(current_seq)

        max_txt = "—" if seq_max is None else str(seq_max)
        self.lbl_mission_wp.setText(f"SEQ: {cur_txt} / {max_txt}")

        self._update_mission_fc_display()

        cmd = self._get_mission_cmd_by_seq(current_seq)
        cmd_txt = self._format_mission_cmd(cmd, current_seq)
        self.lbl_mission_cmd.setText(f"CMD: {cmd_txt}")

    def _select_mission(self, mission_id: str):
        if self._is_mission_locked():
            self._log(
                f"[MISSION] 현재 상태({self._mission_state_upper()})에서는 미션 변경 차단"
            )
            return

        mission_id = str(mission_id).strip().upper()

        self.selected_mission = mission_id
        self._mission_current_wp = None
        self._mission_fc_current_raw = None
        self._mission_fc_current_ui = None
        self._mission_last_raw_preview = []
        self._mission_path_points = []
        self._selected_mission_payload = None
        self._selected_mission_file_path = None

        try:
            path, points, payload = self._load_local_mission_payload(mission_id)
        except FileNotFoundError:
            self._update_mission_seq_cmd_display(None)
            self._set_mission_state("ERROR")
            self._log(
                f"[MISSION/ERR] PC 로컬 파일 없음: "
                f"~/h264_gcs_ws/missions/mission_{mission_id}.waypoints"
            )
            return
        except Exception as e:
            self._update_mission_seq_cmd_display(None)
            self._set_mission_state("ERROR")
            self._log(f"[MISSION/ERR] Mission {mission_id} 로컬 파일 파싱 실패: {type(e).__name__}: {e}")
            return

        self._mission_path_points = points
        self._selected_mission_payload = payload
        self._selected_mission_file_path = path

        try:
            self.map_full.update_mission_path(points, None)
            self.split_map.update_mission_path(points, None)
        except Exception as e:
            self._log(f"[MISSION/ERR] 로컬 미션 미리보기 실패: {type(e).__name__}: {e}")

        self._update_mission_seq_cmd_display(None, len(points))
        self._set_mission_state("LOADED")
        self._log(
            f"[MISSION] Mission {mission_id} 로컬 파일 로드 완료: "
            f"{os.path.basename(path)}, {len(points)} WP"
        )

    def _request_mission_upload(self):
        if self.selected_mission is None:
            self._log("[MISSION] 먼저 Mission A/B/C/D 또는 Custom을 선택")
            return

        if self._is_mission_locked():
            self._log(
                f"[MISSION] 현재 상태({self._mission_state_upper()})에서는 FC 업로드 차단"
            )
            return

        if self._selected_mission_payload is None:
            self._log("[MISSION/ERR] 업로드할 미션 payload가 없음. 미션을 다시 선택")
            return

        # 업로드 직전에 다시 파일을 읽는다.
        # Mission Planner에서 저장 후 UI에서 바로 업로드하는 상황 반영.
        try:
            path, points, payload = self._load_local_mission_payload(self.selected_mission)
        except Exception as e:
            self._set_mission_state("ERROR")
            self._log(f"[MISSION/ERR] 업로드 전 로컬 미션 파일 확인 실패: {type(e).__name__}: {e}")
            return

        self._mission_path_points = points
        self._selected_mission_payload = payload
        self._selected_mission_file_path = path
        self._mission_current_wp = None
        self._mission_fc_current_raw = None
        self._mission_fc_current_ui = None
        self._mission_last_raw_preview = []

        try:
            self.map_full.update_mission_path(points, None)
            self.split_map.update_mission_path(points, None)
        except Exception as e:
            self._log(f"[MISSION/ERR] 업로드 전 지도 미리보기 갱신 실패: {type(e).__name__}: {e}")

        self._update_mission_seq_cmd_display(None, len(points))

        self._set_mission_state("SYNCING")
        self._log(
            f"[MISSION] Mission {self.selected_mission} 파일 전송 요청: "
            f"{os.path.basename(path)}, sha256={payload['sha256'][:8]}..."
        )

        sent = False
        if self._ros_bridge is not None:
            sent = self._ros_bridge.publish_mission_upload_request(payload)

        if not sent:
            self._set_mission_state("ERROR")
            self._log("[MISSION/ERR] ROS 업로드 요청 발행 실패")
            return

    def _mission_upload_done_mock(self):
        if self.mission_state != "UPLOADING":
            return

        self._set_mission_state("READY")
        self._mission_current_wp = None
        self._update_mission_seq_cmd_display(None)
        self._log(f"[MISSION] Mission {self.selected_mission} 업로드 완료(더미)")


    def _build_hud_panel(self) -> QWidget:
        panel = QFrame()
        panel.setStyleSheet("""
            QFrame { background-color: #101317; border: 2px solid #444; border-radius: 8px; }
            QLabel { color: #dcdcdc; }
        """)
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(12, 5, 12, 5)
        outer.setSpacing(8)

        def make_item(name, val="—"):
            box = QVBoxLayout()
            box.setContentsMargins(0, 0, 0, 0)
            box.setSpacing(0)

            title = QLabel(name)
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title.setStyleSheet("color:#8fa3b8;")
            title.setFont(QFont("Arial", 13, QFont.Weight.Bold))
            title.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

            value = QLabel(val)
            value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value.setFont(QFont("Consolas", 15, QFont.Weight.Bold))
            value.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

            box.addWidget(title, 1)
            box.addWidget(value, 2)

            return box, value

        # 1행
        row1 = QHBoxLayout()
        row1.setSpacing(16)
        self._b1, self.lbl_mode     = make_item("비행모드 / ARM")
        self._b2, self.lbl_batt     = make_item("배터리(전압)")
        self._b3, self.lbl_gps      = make_item("GPS Fix / 위성 수")
        self._b4, self.lbl_link     = make_item("영상 RX/DISP/Sync")
        self._b10, self.lbl_takeoff = make_item("이륙 상태")
        for b in (self._b1, self._b2, self._b3, self._b4, self._b10):
            row1.addLayout(b)

        # 2행
        row2 = QHBoxLayout()
        row2.setSpacing(16)
        self._b5, self.lbl_ekf          = make_item("FC L/G/M/A", "— / — / — / —")
        self._b6, self.lbl_demz         = make_item("지면고도 DEM")
        self._b7, self.lbl_alt          = make_item("상대고도 Rel")
        self._b8, self.lbl_att          = make_item("자세 Roll/Pitch/Yaw")
        self._b9, self.lbl_drone_latlon = make_item("드론 위경도")

        for b in (self._b5, self._b6, self._b7, self._b8, self._b9):
            row2.addLayout(b)

        # 3행: MMR / Target 분리
        row3 = QHBoxLayout()
        row3.setSpacing(12)

        # -------------------------
        # MMR Frame (파란 느낌)
        # -------------------------
        mmr_frame = QFrame()
        mmr_frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        mmr_frame.setStyleSheet("""
            QFrame { background-color: #0a1a20; border: 1px solid #1a5060; border-radius: 6px; }
            QLabel { color: #b2ebf2; }
        """)
        mmr_lay = QHBoxLayout(mmr_frame)
        mmr_lay.setContentsMargins(10, 3, 10, 3)
        mmr_lay.setSpacing(16)

        self._bm1, self.lbl_mmr_pos = make_item("MMR 위경도", "—\n—")
        self._bm2, self.lbl_mmr_hdg = make_item("MMR 헤딩 / 연결", "—\nLINK WAIT")
        self._bm3, self.lbl_mmr_state = make_item("MMR 동작 상태", "상태 수신 대기")
        self._bm4, self.lbl_mmr_detail = make_item("진행 / 세부 상태", "—")

        self.lbl_mmr_pos.setStyleSheet("color: #00e5ff;")
        self.lbl_mmr_hdg.setStyleSheet("color: #b0bec5;")
        self.lbl_mmr_state.setStyleSheet("color: #b0bec5;")
        self.lbl_mmr_detail.setStyleSheet("color: #b0bec5;")
        self.lbl_mmr_state.setWordWrap(True)
        self.lbl_mmr_detail.setWordWrap(True)

        # 위치/헤딩보다 상태 문구 칸을 조금 넓게 배치한다.
        mmr_lay.addLayout(self._bm1, 11)
        mmr_lay.addLayout(self._bm2, 10)
        mmr_lay.addLayout(self._bm3, 12)
        mmr_lay.addLayout(self._bm4, 12)

        # -------------------------
        # Target Frame (주황 느낌)
        # -------------------------
        target_frame = QFrame()
        target_frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        target_frame.setStyleSheet("""
            QFrame { background-color: #241607; border: 1px solid #a65f00; border-radius: 6px; }
            QLabel { color: #ffe0b2; }
        """)
        target_lay = QVBoxLayout(target_frame)
        target_lay.setContentsMargins(10, 3, 10, 3)
        target_lay.setSpacing(3)

        target_row = QHBoxLayout()
        target_row.setSpacing(12)
        self._bt0, self.lbl_target_status = make_item("표적 상태", "IDLE")
        self._bt1, self.lbl_target_latlon = make_item("표적 위경도")
        self._bt2, self.lbl_target_zr     = make_item("표적 고도 / 거리")

        self.lbl_target_status.setStyleSheet("color: #ffd54f;")
        self.lbl_target_latlon.setStyleSheet("color: #ff9800;")
        self.lbl_target_zr.setStyleSheet("color: #ff9800;")

        target_row.addLayout(self._bt0)
        target_row.addLayout(self._bt1)
        target_row.addLayout(self._bt2)

        impact_row = QHBoxLayout()
        impact_row.setSpacing(12)
        self._bi0, self.lbl_impact_status = make_item("탄착 상태", "탄착 대기")
        self._bi1, self.lbl_impact_latlon = make_item("탄착 위경도")
        self._bi2, self.lbl_impact_result = make_item("편차 / 판정")

        self.lbl_impact_status.setStyleSheet("color: #ffd54f;")
        self.lbl_impact_latlon.setStyleSheet("color: #ffb74d;")
        self.lbl_impact_result.setStyleSheet("color: #ffb74d;")

        impact_row.addLayout(self._bi0)
        impact_row.addLayout(self._bi1)
        impact_row.addLayout(self._bi2)

        target_lay.addLayout(target_row, 1)
        target_lay.addLayout(impact_row, 1)

        # row3에 좌/우로 배치
        row3.addWidget(mmr_frame, 1)
        row3.addWidget(target_frame, 1)
        
        outer.addLayout(row1, 1)
        outer.addLayout(row2, 1)
        outer.addLayout(row3, 1)
        return panel

    def update_target(self, x=None, y=None, rng=None, brg=None, conf=None, lat=None, lon=None, z=None):
        self.last_target.update(dict(x=x, y=y, rng=rng, brg=brg, conf=conf, lat=lat, lon=lon))

        # 1) 카메라 오버레이 갱신
        cam = None
        if self.stack.currentIndex() == 0:
            cam = self.cam_full
        elif self.stack.currentIndex() == 2:
            cam = self.split_cam

        if cam:
            cam.overlay.update_text(x, y, rng, brg, conf, lat, lon)

        # 2) HUD 갱신
        # Target LatLon/z는 3행 MMR 옆 빨간색 칸에 표시
        if lat is not None and lon is not None:
            self.lbl_target_latlon.setText(f"{float(lat):.6f}, {float(lon):.6f}")
        else:
            self.lbl_target_latlon.setText("—")

        if z is not None:
            try:
                z_f = float(z)
                r_txt = "—" if rng is None else f"{float(rng):.1f}m"
                self.lbl_target_zr.setText(f"{z_f:.2f}m / {r_txt}")
                self.lbl_demz.setText(f"{z_f:.3f}")
            except Exception:
                self.lbl_target_zr.setText("—")
        else:
            self.lbl_target_zr.setText("—")


        # 3) 로그
        #t = time.strftime("%H:%M:%S")
        #x_s = "—" if x is None else f"{x:.2f}"
        #y_s = "—" if y is None else f"{y:.2f}"
        #rng_s = "—" if rng is None else f"{rng:.2f}m"
        #brg_s = "—" if brg is None else f"{brg:.1f}°"
        #conf_s = "—" if conf is None else f"{conf:.0f}%"
        #lat_s = "—" if lat is None else f"{lat:.6f}"
        #lon_s = "—" if lon is None else f"{lon:.6f}"
        #z_s = "—" if z is None else f"{z:.3f}m"

        #self._log(
        #    f"{t}  Target x={x_s}, y={y_s}, range={rng_s}, brg={brg_s}, conf={conf_s}, "
        #    f"lat={lat_s}, lon={lon_s}, z={z_s}"
        #)
        
    def _apply_target_debug(self, data: dict):
        status = data.get("status", "UNKNOWN")
        reason = data.get("reason")
        reason_ko = data.get("reason_ko") or reason or ""

        seq = data.get("seq", data.get("frame_seq", "—"))
        u = data.get("u")
        v = data.get("v")
        pose_dt = data.get("pose_dt_ms")

        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        z = data.get("z_m")
        rng = data.get("range_m")

        # -------------------------
        # 1. HUD 색상
        # -------------------------
        if status in ("REQUESTED", "WAIT_FRAME", "COMPUTING"):
            color = "#ffd54f"   # 노랑
        elif status == "HIT_OK":
            color = "#00e676"   # 초록
        elif status == "HIT_WARN":
            color = "#ff9800"   # 주황
        else:
            color = "#ff5252"   # 빨강

        self.lbl_target_status.setText(str(status))
        self.lbl_target_status.setStyleSheet(f"color: {color};")

        # -------------------------
        # 2. Target LatLon 표시
        # -------------------------
        if status in ("HIT_OK", "HIT_WARN") and lat is not None and lon is not None:
            self.lbl_target_latlon.setText(f"{float(lat):.6f}, {float(lon):.6f}")
        elif u is not None and v is not None:
            self.lbl_target_latlon.setText(
                f"seq={seq}, uv=({float(u):.0f},{float(v):.0f})"
            )
        else:
            self.lbl_target_latlon.setText(f"seq={seq}")

        # -------------------------
        # 3. Target z/R 표시
        # -------------------------
        if status in ("HIT_OK", "HIT_WARN"):
            z_txt = "—" if z is None else f"{float(z):.2f}m"
            r_txt = "—" if rng is None else f"{float(rng):.1f}m"
            self.lbl_target_zr.setText(f"{z_txt} / {r_txt}")
        elif status == "FAIL":
            self.lbl_target_zr.setText(str(reason or "FAIL"))
        elif status == "WAIT_FRAME":
            self.lbl_target_zr.setText("프레임 대기")
        elif status == "COMPUTING":
            self.lbl_target_zr.setText("계산 중")
        else:
            self.lbl_target_zr.setText("—")

        # -------------------------
        # 4. 한글 로그 1줄
        # -------------------------
        uv_txt = ""
        if u is not None and v is not None:
            try:
                uv_txt = f", uv=({float(u):.0f},{float(v):.0f})"
            except Exception:
                pass

        dt_txt = ""
        if pose_dt is not None:
            try:
                dt_txt = f", 시간차={float(pose_dt):.1f}ms"
            except Exception:
                pass

        if status == "REQUESTED":
            self._log(f"[역투영 요청] seq={seq}{uv_txt}")
        elif status == "WAIT_FRAME":
            self._log(f"[역투영 대기] seq={seq}{uv_txt} → 클릭 프레임 수신 대기")
        elif status == "COMPUTING":
            self._log(f"[역투영 계산중] seq={seq}{uv_txt}{dt_txt}")
        elif status == "HIT_OK":
            r_txt = "—" if rng is None else f"{float(rng):.1f}m"
            z_txt = "—" if z is None else f"{float(z):.2f}m"
            self._log(
                f"[역투영 성공] seq={seq}{uv_txt}, 거리={r_txt}, DEM고도={z_txt}{dt_txt}"
            )
        elif status == "HIT_WARN":
            r_txt = "—" if rng is None else f"{float(rng):.1f}m"
            self._log(
                f"[역투영 주의] seq={seq}{uv_txt}, 거리={r_txt} → {reason_ko}"
            )
        elif status == "FAIL":
            self._log(f"[역투영 실패] seq={seq}{uv_txt} → {reason_ko}")
    def _apply_target_result(self, data: dict, reset_impacts: bool = False):
        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        z = data.get("z_m")
        rng = data.get("range_m")
        x = data.get("x_m")
        y = data.get("y_m")
        brg = data.get("bearing_deg")
        conf = data.get("conf")

        try:
            lat = float(lat) if lat is not None else None
            lon = float(lon) if lon is not None else None
            z   = float(z)   if z   is not None else None
            rng = float(rng) if rng is not None else None
            x   = float(x)   if x   is not None else None
            y   = float(y)   if y   is not None else None
            brg = float(brg) if brg is not None else None
            conf = float(conf) if conf is not None else None
        except Exception:
            pass

        if reset_impacts:
            self.clear_impacts(log_message=False)

        self._latest_target_data = {
            "seq": data.get("seq"),
            "lat_deg": lat,
            "lon_deg": lon,
            "z_m": z,
            "range_m": rng,
            "x_m": x,
            "y_m": y,
            "bearing_deg": brg,
            "conf": conf,
        }

        # HUD / 오버레이 반영
        self.update_target(
            x=x,
            y=y,
            rng=rng,
            brg=brg,
            conf=conf,
            lat=lat,
            lon=lon,
            z=z,
        )

        # 맵 반영: 표적 마커 + 실제 반경 15m 원
        self._draw_target_on_maps(self._latest_target_data)

    def _run_js_on_both_maps(self, js: str):
        for map_view in (self.map_full, self.split_map):
            try:
                map_view.web.page().runJavaScript(js)
            except Exception:
                pass

    def _draw_target_on_maps(self, data: dict | None):
        if not isinstance(data, dict):
            return
        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        if lat is None or lon is None:
            return
        try:
            lat = float(lat)
            lon = float(lon)
        except Exception:
            return

        z = data.get("z_m")
        rng = data.get("range_m")
        text = f"<b style='color:#ff1744'>표적</b><br>lat: {lat:.7f}<br>lon: {lon:.7f}"
        if z is not None:
            try:
                text += f"<br>DEM 고도: {float(z):.2f} m"
            except Exception:
                pass
        if rng is not None:
            try:
                text += f"<br>드론-표적 거리: {float(rng):.2f} m"
            except Exception:
                pass
        text += f"<br>판정 반경: {self._impact_threshold_m:.1f} m"
        js = f"showTargetResult({lat}, {lon}, {json.dumps(text, ensure_ascii=False)})"
        self._run_js_on_both_maps(js)

    def _has_valid_target(self) -> bool:
        data = self._latest_target_data
        if not isinstance(data, dict):
            return False
        return (
            data.get("lat_deg") is not None
            and data.get("lon_deg") is not None
            and data.get("x_m") is not None
            and data.get("y_m") is not None
        )

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2) -> float:
        r = 6371008.8
        p1 = math.radians(float(lat1))
        p2 = math.radians(float(lat2))
        dp = p2 - p1
        dl = math.radians(float(lon2) - float(lon1))
        a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
        return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))

    def _target_impact_distance_m(self, impact_data: dict) -> float | None:
        target = self._latest_target_data
        if not isinstance(target, dict):
            return None
        try:
            tx = float(target.get("x_m"))
            ty = float(target.get("y_m"))
            ix = float(impact_data.get("x_m"))
            iy = float(impact_data.get("y_m"))
            return math.hypot(ix - tx, iy - ty)
        except Exception:
            pass
        try:
            return self._haversine_m(
                target.get("lat_deg"), target.get("lon_deg"),
                impact_data.get("lat_deg"), impact_data.get("lon_deg")
            )
        except Exception:
            return None

    def _draw_impact_entry_on_maps(self, entry: dict):
        target = self._latest_target_data
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
        z = data.get("z_m")
        pose_dt = data.get("pose_dt_ms")
        text = (
            f"<b style='color:#ffb74d'>탄착 지정점 #{impact_id}</b>"
            f"<br>lat: {lat:.7f}<br>lon: {lon:.7f}"
            f"<br>표적 편차: {error_m:.2f} m"
            f"<br><b>판정: {result_ko}</b>"
        )
        if z is not None:
            try:
                text += f"<br>DEM 고도: {float(z):.2f} m"
            except Exception:
                pass
        if pose_dt is not None:
            try:
                text += f"<br>Pose Δt: {float(pose_dt):.1f} ms"
            except Exception:
                pass

        js = (
            f"showImpactResult({impact_id}, {lat}, {lon}, {target_lat}, {target_lon}, "
            f"{error_m}, {json.dumps(result)}, {json.dumps(text, ensure_ascii=False)})"
        )
        self._run_js_on_both_maps(js)

    def _redraw_impact_history(self):
        self._run_js_on_both_maps("clearImpactResults();")
        for entry in self._impact_history:
            self._draw_impact_entry_on_maps(entry)

    def clear_impacts(self, log_message: bool = True):
        self._impact_reset_ns = time.time_ns()
        self._impact_history.clear()
        self._impact_counter = 0
        self._latest_impact_data = None
        self._latest_impact_debug_data = None
        self._latest_impact_point_data = None
        self.lbl_impact_status.setText("탄착 대기")
        self.lbl_impact_status.setStyleSheet("color: #ffd54f;")
        self.lbl_impact_latlon.setText("—")
        self.lbl_impact_result.setText("—")
        self.lbl_impact_latlon.setStyleSheet("color: #ffb74d;")
        self.lbl_impact_result.setStyleSheet("color: #ffb74d;")
        self._run_js_on_both_maps("clearImpactResults();")
        if log_message:
            self._log("[탄착 초기화] 탄착 지점과 판정 이력을 삭제했습니다. 표적은 유지됩니다.")

    def _apply_impact_debug(self, data: dict):
        try:
            t_click_ns = int(data.get("t_click_ns") or 0)
        except Exception:
            t_click_ns = 0
        if t_click_ns and t_click_ns < self._impact_reset_ns:
            return

        status = str(data.get("status", "UNKNOWN"))
        if status in ("HIT_OK", "HIT_WARN") and isinstance(self._latest_impact_data, dict):
            latest_click_ns = self._latest_impact_data.get("data", {}).get("t_click_ns")
            try:
                if t_click_ns and int(latest_click_ns or 0) == t_click_ns:
                    return
            except Exception:
                pass
        reason = data.get("reason")
        reason_ko = data.get("reason_ko") or reason or ""
        seq = data.get("seq", data.get("frame_seq", "—"))
        u = data.get("u")
        v = data.get("v")
        pose_dt = data.get("pose_dt_ms")

        if status in ("REQUESTED", "WAIT_FRAME", "COMPUTING"):
            color = "#ffd54f"
        elif status == "HIT_OK":
            color = "#00e676"
        elif status == "HIT_WARN":
            color = "#ff9800"
        else:
            color = "#ff5252"

        status_text = {
            "REQUESTED": "탄착 계산 요청",
            "WAIT_FRAME": "프레임 대기",
            "COMPUTING": "탄착 계산 중",
            "HIT_OK": "좌표 계산 완료",
            "HIT_WARN": "좌표 계산 주의",
            "FAIL": "좌표 계산 실패",
        }.get(status, status)
        self.lbl_impact_status.setText(status_text)
        self.lbl_impact_status.setStyleSheet(f"color: {color};")

        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        if lat is not None and lon is not None:
            try:
                self.lbl_impact_latlon.setText(f"{float(lat):.6f}, {float(lon):.6f}")
            except Exception:
                pass
        elif u is not None and v is not None:
            try:
                self.lbl_impact_latlon.setText(f"seq={seq}, uv=({float(u):.0f},{float(v):.0f})")
            except Exception:
                self.lbl_impact_latlon.setText(f"seq={seq}")

        if status == "FAIL":
            self.lbl_impact_result.setText(str(reason_ko or "FAIL"))
        elif status == "WAIT_FRAME":
            self.lbl_impact_result.setText("클릭 프레임 수신 대기")
        elif status == "COMPUTING":
            self.lbl_impact_result.setText("DEM 교차 계산 중")
        elif status == "REQUESTED":
            self.lbl_impact_result.setText("—")

        dt_txt = ""
        if pose_dt is not None:
            try:
                dt_txt = f", 시간차={float(pose_dt):.1f}ms"
            except Exception:
                pass
        uv_txt = ""
        if u is not None and v is not None:
            try:
                uv_txt = f", uv=({float(u):.0f},{float(v):.0f})"
            except Exception:
                pass

        if status == "REQUESTED":
            self._log(f"[탄착 역투영 요청] seq={seq}{uv_txt}")
        elif status == "WAIT_FRAME":
            self._log(f"[탄착 역투영 대기] seq={seq}{uv_txt}")
        elif status == "COMPUTING":
            self._log(f"[탄착 역투영 계산중] seq={seq}{uv_txt}{dt_txt}")
        elif status == "HIT_WARN":
            self._log(f"[탄착 역투영 주의] seq={seq}{uv_txt} → {reason_ko}")
        elif status == "FAIL":
            self._log(f"[탄착 역투영 실패] seq={seq}{uv_txt} → {reason_ko}")

    def _apply_impact_result(self, data: dict):
        if not self._has_valid_target():
            self._log("[탄착 좌표 무시] 유효한 표적이 없습니다")
            return

        try:
            t_click_ns = int(data.get("t_click_ns") or 0)
        except Exception:
            t_click_ns = 0
        if t_click_ns and t_click_ns < self._impact_reset_ns:
            self._log("[탄착 좌표 무시] 초기화 이전 요청의 지연 응답입니다")
            return

        normalized = dict(data)
        for key in ("lat_deg", "lon_deg", "z_m", "range_m", "x_m", "y_m", "bearing_deg", "pose_dt_ms"):
            if normalized.get(key) is not None:
                try:
                    normalized[key] = float(normalized[key])
                except Exception:
                    pass

        error_m = self._target_impact_distance_m(normalized)
        if error_m is None:
            self.lbl_impact_status.setText("거리 계산 실패")
            self.lbl_impact_status.setStyleSheet("color: #ff5252;")
            self.lbl_impact_result.setText("좌표계 확인 필요")
            self._log("[탄착 판정 실패] 표적-탄착 수평거리 계산 불가")
            return

        self._impact_counter += 1
        result = "SUCCESS" if error_m <= self._impact_threshold_m else "REATTACK"
        entry = {
            "impact_id": self._impact_counter,
            "data": normalized,
            "error_m": float(error_m),
            "result": result,
        }
        self._impact_history.append(entry)
        self._latest_impact_data = entry

        lat = normalized.get("lat_deg")
        lon = normalized.get("lon_deg")
        self.lbl_impact_status.setText(f"탄착 #{self._impact_counter}")
        if lat is not None and lon is not None:
            self.lbl_impact_latlon.setText(f"{float(lat):.6f}, {float(lon):.6f}")

        if result == "SUCCESS":
            result_ko = "타격 성공"
            color = "#00e676"
        else:
            result_ko = "재타격 요청"
            color = "#ff5252"
        self.lbl_impact_status.setStyleSheet(f"color: {color};")
        self.lbl_impact_result.setText(f"{error_m:.1f}m / {result_ko}")
        self.lbl_impact_result.setStyleSheet(f"color: {color};")

        self._draw_impact_entry_on_maps(entry)
        self._log(
            f"[탄착 판정] #{self._impact_counter} 편차={error_m:.2f}m / "
            f"기준={self._impact_threshold_m:.1f}m → {result_ko}"
        )

    def copy_target_to_clipboard(self):
        lat = self.last_target.get("lat")
        lon = self.last_target.get("lon")

        if lat is not None and lon is not None:
            text = f"{float(lat):.7f}, {float(lon):.7f}"
            QGuiApplication.clipboard().setText(text)
            self._log(f"[복사] 표적 위경도 복사 완료: {text}")
            return

        x = self.last_target.get("x")
        y = self.last_target.get("y")
        if x is not None and y is not None:
            text = f"{float(x):.3f}, {float(y):.3f}"
            QGuiApplication.clipboard().setText(text)
            self._log(f"[복사] 표적 XY 좌표 복사 완료: {text}")
            return

        self._log("[복사] 복사할 표적 좌표가 없음")

    def _demo_click_update(self, pos: QPoint, click_type: str = "target"):
        click_type = str(click_type).strip().lower()
        if click_type not in ("target", "impact"):
            click_type = "target"

        if click_type == "impact" and not self._has_valid_target():
            self.lbl_impact_status.setText("표적 먼저 지정")
            self.lbl_impact_status.setStyleSheet("color: #ff5252;")
            self.lbl_impact_result.setText("우클릭 거부")
            self._log("[탄착 요청 거부] 표적을 먼저 지정하십시오")
            return

        u_raw, v_raw = float(pos.x()), float(pos.y())
        u, v = u_raw, v_raw
        evt = {
            "seq"       : int(getattr(self, "_display_seq", -1)),
            "u"         : float(u_raw),
            "v"         : float(v_raw),
            "t_click_ns": time.time_ns(),
            "view_mode" : int(self.stack.currentIndex()),
            "click_type": click_type,
        }
        if self._ros_bridge is not None:
            self._ros_bridge.publish_click(evt)
            debug_evt = {
                "status": "REQUESTED",
                "click_type": click_type,
                "seq": evt["seq"],
                "u": u_raw,
                "v": v_raw,
            }
            if click_type == "impact":
                self._apply_impact_debug(debug_evt)
            else:
                self._apply_target_debug(debug_evt)
        if DEBUG_CAMERA_RAY_LOG:        
            if self.K is not None and self.dist is not None:
                pts     = np.array([[[u_raw, v_raw]]], dtype=np.float64)
                und_pix = cv2.undistortPoints(pts, self.K, self.dist, P=self.K)
                u = float(und_pix[0, 0, 0]); v = float(und_pix[0, 0, 1])
                und_n = cv2.undistortPoints(pts, self.K, self.dist)
                x = float(und_n[0, 0, 0]); y = float(und_n[0, 0, 1])
                ray = np.array([x, y, 1.0], dtype=np.float64)
                ray = ray / np.linalg.norm(ray)
                cx_ = float(self.K[0, 2]); cy_ = float(self.K[1, 2])
                side_x = "RIGHT" if u_raw > cx_ else "LEFT"
                side_y = "DOWN"  if v_raw > cy_ else "UP"
                sx = "+" if x >= 0 else "-"; sy = "+" if y >= 0 else "-"
                self._log(f"[CAM] raw(u,v)=({u_raw:.1f},{v_raw:.1f}) -> und_pix=({u:.1f},{v:.1f})")
                self._log(
                    f"[RAY_RAW] {side_x}/{side_y}  norm(x,y)=({x:+.4f},{y:+.4f}) sign=({sx},{sy})  "
                    f"ray=({ray[0]:+.4f},{ray[1]:+.4f},{ray[2]:+.4f})"
                )
            else:
                self._log(f"[CAM] raw(u,v)=({u_raw:.1f},{v_raw:.1f}) (no calib)")

    def set_split_half(self):
        w = max(1, self.splitter.width())
        self.splitter.setSizes([w // 2, w // 2])

    def switch_mode(self, idx: int):
        names_ko = {
            0: "카메라",
            1: "지도",
            2: "분할",
        }

        current_idx = self.stack.currentIndex()

        # 같은 모드를 다시 누른 경우 중복 로그 방지
        if idx == current_idx:
            return

        self.stack.setCurrentIndex(idx)
        self._log(f"[시스템] 화면 전환 → {names_ko.get(idx, '알 수 없음')}")

        if idx == 2:
            QTimer.singleShot(0, self.set_split_half)

        # 새로 보이는 맵에 최신값 1회 즉시 반영
        # 새로 보이는 맵에 최신값 1회 즉시 반영
        if idx in (1, 2):
            if self._latest_drone_pose is not None:
                lat, lon, yaw = self._latest_drone_pose
                self._update_visible_map_drone(lat, lon, yaw)

            if self._latest_mmr_pose is not None:
                lat, lon, hdg = self._latest_mmr_pose
                self._update_visible_map_mmr(lat, lon, hdg)

            if self._mission_path_points:
                try:
                    self.map_full.update_mission_path(self._mission_path_points, self._mission_current_wp)
                    self.split_map.update_mission_path(self._mission_path_points, self._mission_current_wp)
                except Exception as e:
                    self._log(f"[MISSION/ERR] 미션 경로 재표시 실패: {type(e).__name__}: {e}")

        if self._latest_target_data is not None:
            self._draw_target_on_maps(self._latest_target_data)
            self._redraw_impact_history()

    def _log(self, msg: str):
        msg = str(msg)

        # 같은 로그가 연속으로 반복될 경우 출력 생략
        if msg == getattr(self, "_last_log_msg", None):
            return
        self._last_log_msg = msg

        color = "#cfcfcf"  # 기본 회색
        
        # 역투영 로그
        if msg.startswith("[역투영 성공]"):
            color = "#00e676"  # 초록
        elif msg.startswith("[역투영 주의]"):
            color = "#ff9800"  # 주황
        elif msg.startswith("[역투영 실패]"):
            color = "#ff5252"  # 빨강
        elif msg.startswith("[역투영 대기]") or msg.startswith("[역투영 계산중]") or msg.startswith("[역투영 요청]"):
            color = "#ffd54f"  # 노랑
        elif msg.startswith("[탄착 판정]"):
            color = "#00e676" if "타격 성공" in msg else "#ff5252"
        elif msg.startswith("[탄착 역투영 실패]") or msg.startswith("[탄착 요청 거부]"):
            color = "#ff5252"
        elif msg.startswith("[탄착 역투영 주의]"):
            color = "#ff9800"
        elif msg.startswith("[탄착 역투영") or msg.startswith("[탄착 초기화]"):
            color = "#ffd54f"
            
        # 이륙 관련
        elif msg.startswith("[이륙 확인]"):
            color = "#80cbc4"

        # 텔레메트리/ROS
        elif msg.startswith("[TEL/WARN]"):
            color = "#ff9800"
        elif msg.startswith("[ROS/ERR]"):
            color = "#ff5252"

        # 일반 정보
        elif msg.startswith("[지도 클릭]"):
            color = "#90caf9"
        elif msg.startswith("[시스템]"):
            color = "#b0bec5"
        elif msg.startswith("[복사]"):
            color = "#b39ddb"

        # 디버그 로그
        elif msg.startswith("[CAM/DEBUG]") or msg.startswith("[RAY/DEBUG]"):
            color = "#78909c"

        safe_msg = (
            msg.replace("&", "&amp;")
               .replace("<", "&lt;")
               .replace(">", "&gt;")
        )

        self.log.append(f'<span style="color:{color};">{safe_msg}</span>')
        self.log.moveCursor(self.log.textCursor().MoveOperation.End)
        
    def resizeEvent(self, e):
        super().resizeEvent(e)

        if hasattr(self, "menu_header"):
            QTimer.singleShot(0, self._position_menu_header)

        if self.stack.currentIndex() == 2:
            QTimer.singleShot(0, self.set_split_half)
            
    def closeEvent(self, event):
        try:
            if self._h264_receiver is not None:
                self._h264_receiver.stop()
        except Exception:
            pass

        try:
            if self._ros_bridge is not None:
                self._ros_bridge.shutdown()
        except Exception:
            pass

        super().closeEvent(event)

# ---------- 실행 ----------
def main():
    print("[CHECK] ui.py actually running", flush=True)
    app = QApplication(sys.argv)
    win = MainWindow()

    # 창 크기 조절 / 최대화 가능하도록 강제 설정
    win.setMinimumSize(900, 600)
    win.setMaximumSize(16777215, 16777215)
    win.setWindowFlags(
        win.windowFlags()
        | Qt.WindowType.WindowMinMaxButtonsHint
        | Qt.WindowType.WindowCloseButtonHint
    )

    # 시작하자마자 최대화
    win.showMaximized()

    # QtWebEngine / 카메라 뷰 초기 레이아웃 반영을 위해 한 번 더 최대화 보정
    QTimer.singleShot(300, win.showMaximized)
    QTimer.singleShot(800, win.showMaximized)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
