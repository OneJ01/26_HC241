#!/usr/bin/env python3
#26.04.25
import os
import json
import time
import math
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from cam_msgs.msg import FramePacket

import rasterio
from pyproj import Transformer

TELEMETRY_TOPIC = "/drone/telemetry"
ATTITUDE_SAMPLE_TOPIC = "/drone/attitude_sample"
POSITION_SAMPLE_TOPIC = "/drone/position_sample"
ALTITUDE_TOPIC = "/compute/drone_altitude"

FRAME_TOPIC = "/camera/frame_packet"
FRAME_META_TOPIC = "/camera/frame_meta"
CLICK_TOPIC = "/ui/click_event"
TARGET_TOPIC = "/compute/target_point"
TARGET_DEBUG_TOPIC = "/compute/target_debug"
IMPACT_TOPIC = "/compute/impact_point"
IMPACT_DEBUG_TOPIC = "/compute/impact_debug"
# MMR fire_solution_node로 보낼 표적 좌표 JSON 토픽
MMR_TARGET_POINT_TOPIC = "/target/point"

KEEP_FRAMES_N = 200

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR = os.path.join(BASE_DIR, "gcs_logs")
os.makedirs(SAVE_DIR, exist_ok=True)

# UI와 동일 DEM
DEM_PATH = os.path.expanduser("~/gcs_ws/data/dem_tin_1m_pot.tif")

def now_str():
    return time.strftime("%Y%m%d_%H%M%S")


class ComputeLogger(Node):

    def __init__(self):
        super().__init__("compute_logger")

        # DEM 로드
        self.dem = rasterio.open(DEM_PATH)
        self.dem_crs = self.dem.crs
        print(f"[DEM CRS] {self.dem_crs}")

        # WGS84 -> DEM 좌표 변환기
        if str(self.dem_crs) != "EPSG:4326":
            self.transformer = Transformer.from_crs(
                "EPSG:4326",
                self.dem_crs,
                always_xy=True
            )
        else:
            print("[CRS] DEM이 WGS84 → 좌표 변환기 사용 안함")
            self.transformer = None
            
            # DEM -> WGS84 역변환기
        if str(self.dem_crs) != "EPSG:4326":
            self.transformer_inv = Transformer.from_crs(
                self.dem_crs,
                "EPSG:4326",
                always_xy=True
            )
        else:
            self.transformer_inv = None

        # 고도 관련 변수
        self.home_ground_alt = None
        self.last_ground_alt = None

        # ROS pub/sub
        qos_frame = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_frame = self.create_subscription(
            FramePacket, FRAME_TOPIC, self.on_frame, qos_frame)

        # H.264 직접 수신 구조에서는 JPEG FramePacket 대신
        # /camera/frame_meta만으로 seq/time 정보를 저장한다.
        self.sub_frame_meta = self.create_subscription(
            String, FRAME_META_TOPIC, self.on_frame_meta, 10)

        self.sub_click = self.create_subscription(
            String, CLICK_TOPIC, self.on_click, 10)

        self.sub_telemetry = self.create_subscription(
            String, TELEMETRY_TOPIC, self.on_telemetry, 10)
        self.sub_attitude_sample = self.create_subscription(
            String, ATTITUDE_SAMPLE_TOPIC, self.on_attitude_sample, 50)
        self.sub_position_sample = self.create_subscription(
            String, POSITION_SAMPLE_TOPIC, self.on_position_sample, 50)

        self.pub_altitude = self.create_publisher(
            String, ALTITUDE_TOPIC, 10)
        
        self.pub_target = self.create_publisher(
            String, TARGET_TOPIC, 10)
        self.pub_target_debug = self.create_publisher(
            String, TARGET_DEBUG_TOPIC, 10)
        self.pub_impact = self.create_publisher(
            String, IMPACT_TOPIC, 10)
        self.pub_impact_debug = self.create_publisher(
            String, IMPACT_DEBUG_TOPIC, 10)
        # MMR 입력용 표적 좌표 JSON publisher
        self.pub_mmr_target_point = self.create_publisher(
            String, MMR_TARGET_POINT_TOPIC, 10)

        # pose cache
        # - pose_history: 기존 통합 telemetry 호환/fallback
        # - attitude/position_history: 프레임 시각 보간용 실제 스트림별 샘플
        self.latest_pose = None
        self.pose_history = deque(maxlen=500)
        self.attitude_history = deque(maxlen=500)
        self.position_history = deque(maxlen=500)
        self.latest_attitude_sample = None
        self.latest_position_sample = None
        self.max_attitude_span_ns = int(0.200 * 1e9)
        self.max_position_span_ns = int(0.300 * 1e9)
        self.max_nearest_dt_ns = int(0.150 * 1e9)
        # camera intrinsic / distortion
        self.K = None
        self.dist = None
        calib_path = os.path.join(BASE_DIR, "calib_result.json")
        try:
            with open(calib_path, "r") as f:
                cd = json.load(f)
            self.K = np.array(cd["K"], dtype=np.float64)
            self.dist = np.array(cd["dist"], dtype=np.float64).reshape(1, -1)
            self.get_logger().info(f"[CALIB] loaded: {calib_path}")
        except Exception as e:
            self.get_logger().warn(f"[CALIB] load failed: {e}")

        # camera extrinsic: Camera(OpenCV raw sensor frame) -> Body(FRD)
        # 조건:
        # - yaw offset = 0 deg
        # - physical camera roll = 180 deg
        # - pitch = 30 deg downward
        #
        # 현재 UI는 화면 표시용으로 이미지를 180도 회전해서 보여주고,
        # 클릭 좌표는 다시 raw 이미지 좌표로 되돌려 publish한다.
        # 따라서 ComputeLogger에서는 raw 이미지 좌표 기준으로 ray를 만들고,
        # 물리 장착 roll 180도는 R_cam_to_body에 반영한다.

        R_nominal = np.array([
            [0.0, -0.5,       0.8660254],
            [1.0,  0.0,       0.0      ],
            [0.0,  0.8660254, 0.5      ],
        ], dtype=np.float64)

        R_roll180 = np.array([
            [-1.0,  0.0, 0.0],
            [ 0.0, -1.0, 0.0],
            [ 0.0,  0.0, 1.0],
        ], dtype=np.float64)

        self.R_cam_to_body = R_nominal @ R_roll180

        # camera center position in body frame [m]
        # body FRD, origin = FC center
        # x: forward, y: right, z: down
        self.t_body_cam = np.array([0.0798, 0.0, 0.0045], dtype=np.float64)

        # frame cache
        self.frames = {}
        self.seq_fifo = deque()
        self.pending = {}

        self.event_log_path = os.path.join(
            SAVE_DIR, f"click_events_{now_str()}.jsonl")

        self.get_logger().info(f"Save events -> {self.event_log_path}")
        self.get_logger().info(
            "[POSE_SYNC] attitude/position timestamps separated; "
            "SLERP + projected-position interpolation enabled; "
            "nearest fallback <= 150ms"
        )

    # ---------------- DEM 샘플링 ----------------
    def sample_dem(self, lat, lon):

        try:
            if self.transformer:
                x, y = self.transformer.transform(lon, lat)
            else:
                x, y = lon, lat

            val = list(self.dem.sample([(x, y)]))[0][0]

            if val is None:
                return None

            val = float(val)
            nodata = self.dem.nodata

            if (nodata is not None and val == nodata) or np.isnan(val):
                self.get_logger().warn(
                    f"[DEM] invalid ground_alt: val={val}, lat={lat}, lon={lon}"
                )
                return None

            self.last_ground_alt = val
            return val

        except Exception:
            return self.last_ground_alt
        
    def _get_frame_time_ns(self, meta: dict):
        for key in ("t_cam_ns", "frame_t_cam_ns", "FrameWallClock", "host_time_ns"):
            v = meta.get(key)
            if v is None:
                continue
            try:
                return int(v)
            except Exception:
                pass
        return None


    @staticmethod
    def _to_float_or_none(value):
        try:
            return float(value) if value is not None else None
        except Exception:
            return None

    @staticmethod
    def _to_int_or_none(value):
        try:
            return int(value) if value is not None else None
        except Exception:
            return None

    def _append_attitude_sample(self, data: dict):
        t_ns = self._to_int_or_none(
            data.get("attitude_time_ns", data.get("t_ns"))
        )
        roll = self._to_float_or_none(data.get("roll_deg"))
        pitch = self._to_float_or_none(data.get("pitch_deg"))
        yaw = self._to_float_or_none(data.get("yaw_deg"))
        if t_ns is None or t_ns <= 0 or None in (roll, pitch, yaw):
            return False

        sample = {
            "t_ns": int(t_ns),
            "roll_deg": float(roll),
            "pitch_deg": float(pitch),
            "yaw_deg": float(yaw),
        }
        # 전용 샘플 토픽과 통합 telemetry가 같은 샘플을 중복 전달할 수 있다.
        if any(item.get("t_ns") == t_ns for item in self.attitude_history):
            return False
        self.attitude_history.append(sample)
        self.latest_attitude_sample = sample
        return True

    def _append_position_sample(self, data: dict):
        t_ns = self._to_int_or_none(
            data.get("position_time_ns", data.get("t_ns"))
        )
        lat = self._to_float_or_none(data.get("lat_deg"))
        lon = self._to_float_or_none(data.get("lon_deg"))
        if t_ns is None or t_ns <= 0 or lat is None or lon is None:
            return False

        sample = {
            "t_ns": int(t_ns),
            "lat_deg": float(lat),
            "lon_deg": float(lon),
            "abs_alt_m": self._to_float_or_none(data.get("abs_alt_m")),
            "rel_alt_m": self._to_float_or_none(data.get("rel_alt_m")),
        }
        if any(item.get("t_ns") == t_ns for item in self.position_history):
            return False
        self.position_history.append(sample)
        self.latest_position_sample = sample
        return True

    def on_attitude_sample(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._append_attitude_sample(data)

    def on_position_sample(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._append_position_sample(data)

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray):
        """3x3 회전행렬 -> 정규화 quaternion [w, x, y, z]."""
        R = np.asarray(R, dtype=np.float64).reshape(3, 3)
        trace = float(np.trace(R))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax(np.diag(R)))
            if i == 0:
                s = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
                if s <= 1e-12:
                    return None
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = math.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
                if s <= 1e-12:
                    return None
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = math.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
                if s <= 1e-12:
                    return None
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s

        q = np.array([w, x, y, z], dtype=np.float64)
        n = float(np.linalg.norm(q))
        if n <= 1e-12:
            return None
        return q / n

    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray):
        q = np.asarray(q, dtype=np.float64).reshape(4)
        n = float(np.linalg.norm(q))
        if n <= 1e-12:
            return None
        w, x, y, z = q / n
        return np.array([
            [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w)],
            [2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
            [2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)],
        ], dtype=np.float64)

    @staticmethod
    def _slerp_quaternion(q0: np.ndarray, q1: np.ndarray, alpha: float):
        q0 = np.asarray(q0, dtype=np.float64).reshape(4)
        q1 = np.asarray(q1, dtype=np.float64).reshape(4)
        q0 = q0 / max(float(np.linalg.norm(q0)), 1e-12)
        q1 = q1 / max(float(np.linalg.norm(q1)), 1e-12)
        alpha = min(1.0, max(0.0, float(alpha)))

        dot = float(np.dot(q0, q1))
        # q와 -q는 같은 자세이므로 최단 회전 경로를 선택한다.
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        dot = min(1.0, max(-1.0, dot))

        if dot > 0.9995:
            q = q0 + alpha * (q1 - q0)
            return q / max(float(np.linalg.norm(q)), 1e-12)

        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        if abs(sin_theta_0) <= 1e-12:
            return q0.copy()
        theta = theta_0 * alpha
        s0 = math.sin(theta_0 - theta) / sin_theta_0
        s1 = math.sin(theta) / sin_theta_0
        q = s0 * q0 + s1 * q1
        return q / max(float(np.linalg.norm(q)), 1e-12)

    @staticmethod
    def _lerp_angle_deg_shortest(a_deg: float, b_deg: float, alpha: float):
        delta = ((float(b_deg) - float(a_deg) + 180.0) % 360.0) - 180.0
        return (float(a_deg) + float(alpha) * delta) % 360.0

    @staticmethod
    def _lerp_optional(a, b, alpha):
        if a is None and b is None:
            return None
        if a is None:
            return float(b)
        if b is None:
            return float(a)
        return float(a) + float(alpha) * (float(b) - float(a))

    @staticmethod
    def _temporal_neighbors(history, frame_time_ns: int):
        samples = [s for s in history if s.get("t_ns") is not None]
        if not samples:
            return None, None, None
        samples.sort(key=lambda s: int(s["t_ns"]))
        before = None
        after = None
        nearest = None
        nearest_dt = None
        for sample in samples:
            t_ns = int(sample["t_ns"])
            dt_ns = abs(t_ns - int(frame_time_ns))
            if nearest_dt is None or dt_ns < nearest_dt:
                nearest = sample
                nearest_dt = dt_ns
            if t_ns <= frame_time_ns:
                before = sample
            if t_ns >= frame_time_ns and after is None:
                after = sample
        return before, after, nearest

    def _match_attitude_by_frame_time(self, frame_time_ns: int):
        before, after, nearest = self._temporal_neighbors(
            self.attitude_history, frame_time_ns
        )
        if nearest is None:
            return None

        nearest_dt_ns = abs(int(nearest["t_ns"]) - int(frame_time_ns))
        if nearest_dt_ns == 0:
            R = self._rpy_deg_to_R_body_to_world(
                nearest["roll_deg"], nearest["pitch_deg"], nearest["yaw_deg"]
            )
            out = dict(nearest)
            out.update({
                "t_ns": int(frame_time_ns),
                "R_body_to_world": R.tolist() if R is not None else None,
                "match_mode": "exact",
                "match_dt_ms": 0.0,
                "sample_span_ms": 0.0,
            })
            return out

        if before is not None and after is not None:
            t0 = int(before["t_ns"])
            t1 = int(after["t_ns"])
            span_ns = t1 - t0
            if 0 < span_ns <= self.max_attitude_span_ns and t0 <= frame_time_ns <= t1:
                alpha = (int(frame_time_ns) - t0) / float(span_ns)
                R0 = self._rpy_deg_to_R_body_to_world(
                    before["roll_deg"], before["pitch_deg"], before["yaw_deg"]
                )
                R1 = self._rpy_deg_to_R_body_to_world(
                    after["roll_deg"], after["pitch_deg"], after["yaw_deg"]
                )
                if R0 is not None and R1 is not None:
                    q0 = self._rotation_matrix_to_quaternion(R0)
                    q1 = self._rotation_matrix_to_quaternion(R1)
                    if q0 is not None and q1 is not None:
                        qi = self._slerp_quaternion(q0, q1, alpha)
                        Ri = self._quaternion_to_rotation_matrix(qi)
                        return {
                            "t_ns": int(frame_time_ns),
                            "roll_deg": self._lerp_optional(before["roll_deg"], after["roll_deg"], alpha),
                            "pitch_deg": self._lerp_optional(before["pitch_deg"], after["pitch_deg"], alpha),
                            "yaw_deg": self._lerp_angle_deg_shortest(before["yaw_deg"], after["yaw_deg"], alpha),
                            "R_body_to_world": Ri.tolist() if Ri is not None else None,
                            "match_mode": "interpolated_slerp",
                            "match_dt_ms": 0.0,
                            "sample_span_ms": span_ns / 1e6,
                            "source_before_ns": t0,
                            "source_after_ns": t1,
                            "alpha": float(alpha),
                        }

        if nearest_dt_ns <= self.max_nearest_dt_ns:
            R = self._rpy_deg_to_R_body_to_world(
                nearest["roll_deg"], nearest["pitch_deg"], nearest["yaw_deg"]
            )
            out = dict(nearest)
            out.update({
                "R_body_to_world": R.tolist() if R is not None else None,
                "match_mode": "nearest_fallback",
                "match_dt_ms": nearest_dt_ns / 1e6,
                "sample_span_ms": None,
                "source_sample_ns": int(nearest["t_ns"]),
            })
            return out
        return None

    def _match_position_by_frame_time(self, frame_time_ns: int):
        before, after, nearest = self._temporal_neighbors(
            self.position_history, frame_time_ns
        )
        if nearest is None:
            return None

        nearest_dt_ns = abs(int(nearest["t_ns"]) - int(frame_time_ns))
        if nearest_dt_ns == 0:
            out = dict(nearest)
            out.update({
                "t_ns": int(frame_time_ns),
                "match_mode": "exact",
                "match_dt_ms": 0.0,
                "sample_span_ms": 0.0,
            })
            return out

        if before is not None and after is not None:
            t0 = int(before["t_ns"])
            t1 = int(after["t_ns"])
            span_ns = t1 - t0
            if 0 < span_ns <= self.max_position_span_ns and t0 <= frame_time_ns <= t1:
                alpha = (int(frame_time_ns) - t0) / float(span_ns)
                lat0, lon0 = float(before["lat_deg"]), float(before["lon_deg"])
                lat1, lon1 = float(after["lat_deg"]), float(after["lon_deg"])
                lat_i = None
                lon_i = None

                # 이후 Ray-DEM 계산과 동일한 투영좌표계에서 위치를 선형 보간한다.
                try:
                    if self.transformer is not None:
                        x0, y0 = self.transformer.transform(lon0, lat0)
                        x1, y1 = self.transformer.transform(lon1, lat1)
                        xi = float(x0) + alpha * (float(x1) - float(x0))
                        yi = float(y0) + alpha * (float(y1) - float(y0))
                        lat_i, lon_i = self._dem_world_to_wgs84(xi, yi)
                    else:
                        lon_i = lon0 + alpha * (lon1 - lon0)
                        lat_i = lat0 + alpha * (lat1 - lat0)
                except Exception:
                    lon_i = lon0 + alpha * (lon1 - lon0)
                    lat_i = lat0 + alpha * (lat1 - lat0)

                if lat_i is not None and lon_i is not None:
                    return {
                        "t_ns": int(frame_time_ns),
                        "lat_deg": float(lat_i),
                        "lon_deg": float(lon_i),
                        "abs_alt_m": self._lerp_optional(before.get("abs_alt_m"), after.get("abs_alt_m"), alpha),
                        "rel_alt_m": self._lerp_optional(before.get("rel_alt_m"), after.get("rel_alt_m"), alpha),
                        "match_mode": "interpolated_projected",
                        "match_dt_ms": 0.0,
                        "sample_span_ms": span_ns / 1e6,
                        "source_before_ns": t0,
                        "source_after_ns": t1,
                        "alpha": float(alpha),
                    }

        if nearest_dt_ns <= self.max_nearest_dt_ns:
            out = dict(nearest)
            out.update({
                "match_mode": "nearest_fallback",
                "match_dt_ms": nearest_dt_ns / 1e6,
                "sample_span_ms": None,
                "source_sample_ns": int(nearest["t_ns"]),
            })
            return out
        return None

    def _match_legacy_pose_by_frame_time(self, frame_time_ns: int):
        """전용 샘플 토픽이 없는 구버전 mission_manager 호환용."""
        if frame_time_ns is None or not self.pose_history:
            return None
        best_pose = None
        best_dt_ns = None
        for pose in self.pose_history:
            t_ns = pose.get("telem_time_ns")
            if t_ns is None:
                continue
            dt_ns = abs(int(t_ns) - int(frame_time_ns))
            if best_dt_ns is None or dt_ns < best_dt_ns:
                best_dt_ns = dt_ns
                best_pose = pose
        if best_pose is None or best_dt_ns > self.max_nearest_dt_ns:
            return None
        out = dict(best_pose)
        out["match_dt_ms"] = best_dt_ns / 1e6
        out["pose_match_mode"] = "legacy_nearest_history"
        return out

    def _match_pose_by_frame_time(self, frame_time_ns: int):
        if frame_time_ns is None:
            return None

        attitude = self._match_attitude_by_frame_time(frame_time_ns)
        position = self._match_position_by_frame_time(frame_time_ns)

        if attitude is None or position is None:
            # 두 전용 버퍼가 아직 모두 비어 있는 시작 구간에만 구형 fallback 허용.
            if not self.attitude_history and not self.position_history:
                return self._match_legacy_pose_by_frame_time(frame_time_ns)
            return None

        lat = position.get("lat_deg")
        lon = position.get("lon_deg")
        rel_alt = position.get("rel_alt_m")
        ground_alt = None
        if lat is not None and lon is not None:
            ground_alt = self.sample_dem(lat, lon)

        drone_alt = None
        height_agl = None
        if self.home_ground_alt is not None and rel_alt is not None:
            drone_alt = float(self.home_ground_alt) + float(rel_alt)
        if drone_alt is not None and ground_alt is not None:
            height_agl = float(drone_alt) - float(ground_alt)

        attitude_dt_ms = float(attitude.get("match_dt_ms", 0.0))
        position_dt_ms = float(position.get("match_dt_ms", 0.0))
        pose_dt_ms = max(attitude_dt_ms, position_dt_ms)
        attitude_mode = str(attitude.get("match_mode", "unknown"))
        position_mode = str(position.get("match_mode", "unknown"))

        return {
            # 보간 성공 시 이 pose는 정확히 frame_time_ns 시점의 추정값이다.
            "telem_time_ns": int(frame_time_ns),
            "attitude_time_ns": int(attitude.get("t_ns", frame_time_ns)),
            "position_time_ns": int(position.get("t_ns", frame_time_ns)),
            "lat_deg": lat,
            "lon_deg": lon,
            "abs_alt_m": position.get("abs_alt_m"),
            "rel_alt_m": rel_alt,
            "roll_deg": attitude.get("roll_deg"),
            "pitch_deg": attitude.get("pitch_deg"),
            "yaw_deg": attitude.get("yaw_deg"),
            "R_body_to_world": attitude.get("R_body_to_world"),
            "ground_alt_m": ground_alt,
            "drone_alt_amsl_m": drone_alt,
            "height_agl_m": height_agl,
            "match_dt_ms": pose_dt_ms,
            "attitude_dt_ms": attitude_dt_ms,
            "position_dt_ms": position_dt_ms,
            "attitude_match_mode": attitude_mode,
            "position_match_mode": position_mode,
            "attitude_sample_span_ms": attitude.get("sample_span_ms"),
            "position_sample_span_ms": position.get("sample_span_ms"),
            "pose_match_mode": f"att={attitude_mode};pos={position_mode}",
            "attitude_source_before_ns": attitude.get("source_before_ns"),
            "attitude_source_after_ns": attitude.get("source_after_ns"),
            "position_source_before_ns": position.get("source_before_ns"),
            "position_source_after_ns": position.get("source_after_ns"),
        }

    def _rpy_deg_to_R_body_to_world(self, roll_deg, pitch_deg, yaw_deg):
        """
        Body(FRD) -> World(ENU)

        가정:
        - roll_deg, pitch_deg, yaw_deg 는 ArduPilot/MAVSDK attitude_euler 값
        - yaw_deg 는 North 기준 heading, clockwise positive
        - Body 는 FRD
        - World 는 ENU
        """
        if roll_deg is None or pitch_deg is None or yaw_deg is None:
            return None

        phi = math.radians(float(roll_deg))    # roll
        theta = math.radians(float(pitch_deg)) # pitch
        psi = math.radians(float(yaw_deg))     # yaw / heading

        cphi, sphi = math.cos(phi), math.sin(phi)
        cth,  sth  = math.cos(theta), math.sin(theta)
        cpsi, spsi = math.cos(psi), math.sin(psi)

        # Body(FRD) -> NED
        R_body_to_ned = np.array([
            [cth * cpsi,  sphi * sth * cpsi - cphi * spsi,  cphi * sth * cpsi + sphi * spsi],
            [cth * spsi,  sphi * sth * spsi + cphi * cpsi,  cphi * sth * spsi - sphi * cpsi],
            [-sth,        sphi * cth,                       cphi * cth],
        ], dtype=np.float64)

        # NED -> ENU
        T_ned_to_enu = np.array([
            [0.0, 1.0,  0.0],
            [1.0, 0.0,  0.0],
            [0.0, 0.0, -1.0],
        ], dtype=np.float64)

        R_body_to_world = T_ned_to_enu @ R_body_to_ned
        return R_body_to_world
    
    def _lla_to_dem_world(self, lat_deg, lon_deg, alt_m):
        """
        WGS84(lat, lon, alt) -> DEM/world position [x, y, z]
        x, y : DEM projected CRS meters (e.g. EPSG:5179)
        z    : Up [m]
        """
        if lat_deg is None or lon_deg is None or alt_m is None:
            return None

        try:
            lat_deg = float(lat_deg)
            lon_deg = float(lon_deg)
            alt_m = float(alt_m)

            if self.transformer is not None:
                x, y = self.transformer.transform(lon_deg, lat_deg)
            else:
                x, y = lon_deg, lat_deg

            return np.array([float(x), float(y), float(alt_m)], dtype=np.float64)

        except Exception:
            return None
        
    def _dem_world_to_wgs84(self, x_m, y_m):
        """
        DEM/world XY -> WGS84(lat, lon)
        """
        try:
            x_m = float(x_m)
            y_m = float(y_m)

            if self.transformer_inv is not None:
                lon, lat = self.transformer_inv.transform(x_m, y_m)
            else:
                lon, lat = x_m, y_m

            return float(lat), float(lon)
        except Exception:
            return None, None
        
    def _sample_dem_world_xy(self, x: float, y: float):
        """
        DEM projected XY 에서 높이 z를 읽는다.
        return: (z, status)
        status = "OK" | "OUT_OF_DEM" | "DEM_NODATA" | "DEM_SAMPLE_FAIL"
        """
        try:
            row, col = self.dem.index(float(x), float(y))
        except Exception:
            return None, "OUT_OF_DEM"

        if not (0 <= row < self.dem.height and 0 <= col < self.dem.width):
            return None, "OUT_OF_DEM"

        try:
            z = float(list(self.dem.sample([(float(x), float(y))]))[0][0])
        except Exception:
            return None, "DEM_SAMPLE_FAIL"

        nodata = self.dem.nodata
        if (nodata is not None and z == nodata) or np.isnan(z):
            return None, "DEM_NODATA"

        return z, "OK"
    
    def _intersect_ray_with_dem(
        self,
        origin_world: np.ndarray,
        ray_world: np.ndarray,
        step_m: float = 1.0,
        max_range_m: float = 300.0,
        bisection_iters: int = 12,
        max_nodata_steps: int = 3,
    ):
        """
        camera_origin_world + t * ray_world 와 DEM의 교차점을 찾는다.
        world:
        x, y = DEM projected meters
        z    = Up [m]
        """
        if origin_world is None or ray_world is None:
            return {"status": "INVALID_INPUT"}

        origin_world = np.asarray(origin_world, dtype=np.float64).reshape(-1)
        ray_world = np.asarray(ray_world, dtype=np.float64).reshape(-1)

        if origin_world.shape[0] != 3 or ray_world.shape[0] != 3:
            return {"status": "INVALID_SHAPE"}

        norm = np.linalg.norm(ray_world)
        if norm <= 1e-12:
            return {"status": "INVALID_RAY"}

        ray_world = ray_world / norm

        # 지면과 만나려면 아래로 향해야 함 (world z = up)
        if ray_world[2] >= -1e-6:
            return {"status": "RAY_UPWARD"}
        
        # 너무 수평이면 교차가 지나치게 멀거나 불안정
        if abs(ray_world[2]) < 1e-3:
            return {"status": "RAY_NEAR_HORIZONTAL"}

        x0, y0, z0 = origin_world
        z_dem0, st0 = self._sample_dem_world_xy(x0, y0)
        if st0 != "OK":
            return {"status": st0}

        diff_prev = z0 - z_dem0
        t_prev = 0.0

        if diff_prev <= 0.0:
            return {
                "status": "ORIGIN_BELOW_GROUND",
                "point_world_m": [float(x0), float(y0), float(z0)],
                "dem_z_m": float(z_dem0),
            }

        t = step_m
        while t <= max_range_m:
            p = origin_world + t * ray_world
            z_dem, st = self._sample_dem_world_xy(p[0], p[1])

            if st != "OK":
                if st == "DEM_NODATA":
                    max_nodata_steps -= 1
                    if max_nodata_steps < 0:
                        return {
                            "status": "DEM_NODATA",
                            "last_t_m": float(t),
                            "last_xy_m": [float(p[0]), float(p[1])],
                        }
                    t_prev = t
                    t += step_m
                    continue

                return {
                    "status": st,
                    "last_t_m": float(t),
                    "last_xy_m": [float(p[0]), float(p[1])],
                }


            diff = p[2] - z_dem

            # 지면을 통과한 구간 발견
            if diff <= 0.0:
                lo, hi = t_prev, t

                for _ in range(bisection_iters):
                    mid = 0.5 * (lo + hi)
                    pm = origin_world + mid * ray_world
                    z_mid, st_mid = self._sample_dem_world_xy(pm[0], pm[1])

                    if st_mid != "OK":
                        return {
                            "status": st_mid,
                            "last_t_m": float(mid),
                            "last_xy_m": [float(pm[0]), float(pm[1])],
                        }

                    diff_mid = pm[2] - z_mid
                    if diff_mid > 0.0:
                        lo = mid
                    else:
                        hi = mid

                t_hit = 0.5 * (lo + hi)
                p_hit = origin_world + t_hit * ray_world
                z_hit, st_hit = self._sample_dem_world_xy(p_hit[0], p_hit[1])

                if st_hit != "OK":
                    return {
                        "status": st_hit,
                        "last_t_m": float(t_hit),
                        "last_xy_m": [float(p_hit[0]), float(p_hit[1])],
                    }

                return {
                    "status": "HIT",
                    "range_m": float(t_hit),
                    "point_world_m": [float(p_hit[0]), float(p_hit[1]), float(z_hit)],
                    "dem_z_m": float(z_hit),
                }

            t_prev = t
            t += step_m

        return {
            "status": "NO_INTERSECTION",
            "max_range_m": float(max_range_m),
        }

    def _pixel_to_camera_ray(self, u_raw: float, v_raw: float):
        """
        raw pixel -> undistorted normalized camera ray
        Camera frame(OpenCV):
        x_c = right, y_c = down, z_c = forward
        """
        if self.K is None or self.dist is None:
            return None

        pts = np.array([[[float(u_raw), float(v_raw)]]], dtype=np.float64)

        # 보정 후 픽셀좌표
        und_pix = cv2.undistortPoints(pts, self.K, self.dist, P=self.K)
        u_und = float(und_pix[0, 0, 0])
        v_und = float(und_pix[0, 0, 1])

        # 정규화 카메라 좌표
        und_n = cv2.undistortPoints(pts, self.K, self.dist)
        x = float(und_n[0, 0, 0])
        y = float(und_n[0, 0, 1])

        ray_cam = np.array([x, y, 1.0], dtype=np.float64)
        norm = np.linalg.norm(ray_cam)
        if norm <= 1e-12:
            return None
        ray_cam = ray_cam / norm

        return {
            "u_raw": float(u_raw),
            "v_raw": float(v_raw),
            "u_und": u_und,
            "v_und": v_und,
            "ray_cam": ray_cam.tolist(),
        }


    def _build_click_geometry(self, evt: dict, pose_for_event: dict | None):
        """
        click evt + matched pose + extrinsic -> geometry bundle
        아직 body->world는 안 함.
        """
        u_raw = evt.get("u")
        v_raw = evt.get("v")
        if u_raw is None or v_raw is None:
            return None

        cam_info = self._pixel_to_camera_ray(float(u_raw), float(v_raw))
        if cam_info is None:
            return None

        ray_cam = np.array(cam_info["ray_cam"], dtype=np.float64)

        # Camera -> Body
        ray_body = self.R_cam_to_body @ ray_cam
        norm = np.linalg.norm(ray_body)
        if norm > 1e-12:
            ray_body = ray_body / norm

        geom = {
            "pixel_raw": [float(u_raw), float(v_raw)],
            "pixel_undistorted": [float(cam_info["u_und"]), float(cam_info["v_und"])],
            "ray_cam": ray_cam.tolist(),
            "ray_body": ray_body.tolist(),
            "camera_origin_body_m": self.t_body_cam.tolist(),
            "R_cam_to_body": self.R_cam_to_body.tolist(),
        }

        if pose_for_event is not None:
            geom["pose_rpy_deg"] = {
                "roll_deg": pose_for_event.get("roll_deg"),
                "pitch_deg": pose_for_event.get("pitch_deg"),
                "yaw_deg": pose_for_event.get("yaw_deg"),
            }
            geom["pose_llh"] = {
                "lat_deg": pose_for_event.get("lat_deg"),
                "lon_deg": pose_for_event.get("lon_deg"),
                "rel_alt_m": pose_for_event.get("rel_alt_m"),
                "drone_alt_amsl_m": pose_for_event.get("drone_alt_amsl_m"),
                "height_agl_m": pose_for_event.get("height_agl_m"),
            }
            R_body_to_world = None
            R_from_pose = pose_for_event.get("R_body_to_world")
            if R_from_pose is not None:
                try:
                    R_body_to_world = np.asarray(R_from_pose, dtype=np.float64).reshape(3, 3)
                except Exception:
                    R_body_to_world = None
            if R_body_to_world is None:
                R_body_to_world = self._rpy_deg_to_R_body_to_world(
                    pose_for_event.get("roll_deg"),
                    pose_for_event.get("pitch_deg"),
                    pose_for_event.get("yaw_deg"),
                )

            if R_body_to_world is not None:
                ray_world = R_body_to_world @ ray_body
                norm = np.linalg.norm(ray_world)
                if norm > 1e-12:
                    ray_world = ray_world / norm

                cam_offset_world = R_body_to_world @ self.t_body_cam

                geom["ray_world"] = ray_world.tolist()
                geom["camera_offset_world_m"] = cam_offset_world.tolist()
                geom["R_body_to_world"] = R_body_to_world.tolist()

                # 드론 절대 위치 -> DEM/world 좌표
                drone_pos_world = self._lla_to_dem_world(
                    pose_for_event.get("lat_deg"),
                    pose_for_event.get("lon_deg"),
                    pose_for_event.get("drone_alt_amsl_m"),
                )

                if drone_pos_world is not None:
                    camera_origin_world = drone_pos_world + cam_offset_world

                    geom["drone_pos_world_m"] = drone_pos_world.tolist()
                    geom["camera_origin_world_m"] = camera_origin_world.tolist()

                geom["world_frame"] = "DEM projected XY + Up"
                geom["world_horizontal_crs"] = str(self.dem_crs)
                geom["attitude_assumption"] = "yaw_deg interpreted as heading from North, clockwise positive"
        return geom
    
    def _publish_mmr_target_json(
        self,
        *,
        seq: int,
        lat_t: float,
        lon_t: float,
        z_m: float,
        range_m: float,
        x_m: float,
        y_m: float,
        bearing_deg,
        frame_time_ns,
        pose_for_event,
        pose_dt_ms,
        pose_match_mode: str,
        click_geometry,
    ):
        """
        MMR fire_solution_node 입력용 표적 JSON 발행.

        /target/point:
        - GCS에서 계산한 표적 좌표
        - DEM 교차 결과
        - 시간동기/투영 검증 메타데이터
        를 하나의 JSON으로 묶어 발행한다.
        """

        sync_error_ms = float(pose_dt_ms) if pose_dt_ms is not None else None

        # 현재 compute_logger는 pose_dt_ms > 150ms이면 HIT 계산을 막고 있음.
        # 따라서 GCS 기준 sync_ok는 150ms 이하로 둔다.
        # MMR에서 더 엄격히 보고 싶으면 MMR 쪽에서 50ms 등으로 재검증하면 됨.
        sync_ok = bool(sync_error_ms is not None and sync_error_ms <= 150.0)

        t_pose_ns = None
        if pose_for_event is not None:
            t_pose_ns = pose_for_event.get("telem_time_ns")

        target_payload = {
            # -----------------------------
            # 1. 기본 식별 정보
            # -----------------------------
            "target_id": int(seq),
            "seq": int(seq),
            "source": "gcs_compute_logger_pixel_backprojection",
            "status": "HIT",

            # -----------------------------
            # 2. 표적 WGS84 좌표
            # -----------------------------
            "lat_deg": float(lat_t),
            "lon_deg": float(lon_t),

            # -----------------------------
            # 3. 표적 DEM/world 좌표
            # x_m, y_m: GCS DEM CRS 기준 meter 좌표
            # z_m: ray-DEM 교차점의 DEM 지면고도
            # -----------------------------
            "x_m": float(x_m),
            "y_m": float(y_m),
            "z_m": float(z_m),
            "world_frame": str(self.dem_crs),
            "vertical_reference": "DEM",
            "altitude_mode": "dem_surface",

            # -----------------------------
            # 4. 디버깅용 거리/방위
            # -----------------------------
            "range_m": float(range_m),
            "bearing_deg": bearing_deg,

            # -----------------------------
            # 5. MMR 검증용 메타
            # -----------------------------
            "sync_ok": sync_ok,
            "projection_ok": True,
            "dem_intersection_ok": True,
            "sync_error_ms": sync_error_ms,
            "pose_match_mode": pose_match_mode,
            "attitude_dt_ms": pose_for_event.get("attitude_dt_ms") if pose_for_event else None,
            "position_dt_ms": pose_for_event.get("position_dt_ms") if pose_for_event else None,
            "attitude_match_mode": pose_for_event.get("attitude_match_mode") if pose_for_event else None,
            "position_match_mode": pose_for_event.get("position_match_mode") if pose_for_event else None,
            "attitude_sample_span_ms": pose_for_event.get("attitude_sample_span_ms") if pose_for_event else None,
            "position_sample_span_ms": pose_for_event.get("position_sample_span_ms") if pose_for_event else None,

            # -----------------------------
            # 6. 시간 메타
            # -----------------------------
            "t_cam_ns": frame_time_ns,
            "t_pose_ns": t_pose_ns,
            "t_publish_ns": time.time_ns(),

            # -----------------------------
            # 7. 계산 방식 명시
            # -----------------------------
            "calculation": "pixel_ray_dem_intersection",
        }

        if click_geometry is not None:
            target_payload["world_horizontal_crs"] = click_geometry.get("world_horizontal_crs")
            target_payload["camera_origin_world_m"] = click_geometry.get("camera_origin_world_m")
            target_payload["ray_world"] = click_geometry.get("ray_world")

        msg = String()
        msg.data = json.dumps(target_payload, ensure_ascii=False)
        self.pub_mmr_target_point.publish(msg)

        self.get_logger().info(
            "[MMR_TARGET_JSON_PUB] "
            f"topic={MMR_TARGET_POINT_TOPIC} "
            f"seq={seq} "
            f"lat={lat_t:.7f}, lon={lon_t:.7f}, z={z_m:.2f}m "
            f"sync_error={sync_error_ms}ms sync_ok={sync_ok}"
        )

        
    def _reason_ko(self, reason: str) -> str:
        table = {
            "NO_FRAME": "클릭한 프레임을 찾지 못함",
            "INVALID_CLICK": "클릭 이벤트 정보가 잘못됨",
            "NO_POSE_MATCH": "프레임 시간에 맞는 드론 자세 데이터 없음",
            "POSE_DT_TOO_LARGE": "프레임과 자세 데이터 시간차가 큼",
            "NO_DRONE_ALT_AMSL": "드론 절대고도 계산값 없음",
            "HOME_NOT_SET": "HOME 지면고도 초기화 안 됨",
            "NO_CLICK_GEOMETRY": "클릭 픽셀 기반 Ray 계산 실패",
            "NO_RAY_WORLD": "Ray를 월드 좌표로 변환 실패",
            "NO_CAMERA_ORIGIN_WORLD": "카메라 월드 원점 계산 실패",
            "RAY_UPWARD": "카메라 Ray가 지면이 아닌 위쪽을 향함",
            "RAY_NEAR_HORIZONTAL": "Ray가 너무 수평에 가까움",
            "OUT_OF_DEM": "DEM 범위 밖",
            "DEM_NODATA": "DEM 고도값 없음",
            "DEM_SAMPLE_FAIL": "DEM 샘플링 실패",
            "NO_INTERSECTION": "최대 거리 안에서 지면과 교차하지 않음",
            "RANGE_TOO_SHORT": "표적 거리가 너무 가까움",
            "RANGE_TOO_LONG": "표적 거리가 너무 멂",
            "INVALID_INPUT": "Ray/카메라 원점 입력값이 잘못됨",
            "INVALID_SHAPE": "Ray/카메라 원점 배열 형태가 잘못됨",
            "INVALID_RAY": "Ray 벡터가 유효하지 않음",
            "ORIGIN_BELOW_GROUND": "카메라 원점이 DEM 지면보다 낮음",
            "UNKNOWN_FAIL": "알 수 없는 역투영 실패",
            "WGS84_CONVERT_FAIL": "DEM 좌표를 위도/경도로 변환하지 못함",
            "RANGE_TOO_SHORT_WITH_SHALLOW_RAY": "표적 거리가 너무 가깝고 Ray가 수평에 가까움",
        }
        return table.get(str(reason), str(reason))
    
    def _publish_target_debug(
        self,
        status: str,
        *,
        evt: dict | None = None,
        meta: dict | None = None,
        pose_dt_ms=None,
        pose_match_mode: str | None = None,
        reason: str | None = None,
        lat_deg=None,
        lon_deg=None,
        z_m=None,
        range_m=None,
    ):
        payload = {
            "status": status,
            "reason": reason,
            "reason_ko": self._reason_ko(reason) if reason else "",
            "t_publish_ns": time.time_ns(),
        }

        click_type = "target"
        if evt is not None:
            click_type = str(evt.get("click_type", "target")).strip().lower()
            if click_type not in ("target", "impact"):
                click_type = "target"
            payload["click_type"] = click_type
            payload["seq"] = int(evt.get("seq", -1))
            payload["u"] = evt.get("u")
            payload["v"] = evt.get("v")
            payload["t_click_ns"] = evt.get("t_click_ns")
        else:
            payload["click_type"] = click_type

        if meta is not None:
            payload["frame_seq"] = int(meta.get("seq", -1))

        if pose_dt_ms is not None:
            try:
                payload["pose_dt_ms"] = float(pose_dt_ms)
            except Exception:
                pass

        if pose_match_mode is not None:
            payload["pose_match_mode"] = pose_match_mode

        if lat_deg is not None and lon_deg is not None:
            payload["lat_deg"] = float(lat_deg)
            payload["lon_deg"] = float(lon_deg)

        if z_m is not None:
            payload["z_m"] = float(z_m)

        if range_m is not None:
            payload["range_m"] = float(range_m)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if click_type == "impact":
            self.pub_impact_debug.publish(msg)
        else:
            self.pub_target_debug.publish(msg)

    # ---------------- telemetry ----------------
    def on_telemetry(self, msg: String):

        try:
            data = json.loads(msg.data)
            
        except Exception:
            return
        
        pose_time_ns = data.get("telem_time_ns")
        try:
            pose_time_ns = int(pose_time_ns) if pose_time_ns is not None else time.time_ns()
        except Exception:
            pose_time_ns = time.time_ns()

        # 전용 샘플 토픽이 유실되더라도 통합 telemetry 안의 원본 stream timestamp로
        # 같은 버퍼를 보완한다. 동일 timestamp는 _append_*에서 중복 제거된다.
        self._append_attitude_sample({
            "attitude_time_ns": data.get("attitude_time_ns"),
            "roll_deg": data.get("roll_deg"),
            "pitch_deg": data.get("pitch_deg"),
            "yaw_deg": data.get("yaw_deg"),
        })
        self._append_position_sample({
            "position_time_ns": data.get("position_time_ns"),
            "lat_deg": data.get("lat_deg"),
            "lon_deg": data.get("lon_deg"),
            "abs_alt_m": data.get("abs_alt_m"),
            "rel_alt_m": data.get("rel_alt_m"),
        })
            
        lat = data.get("lat_deg")
        lon = data.get("lon_deg")
        rel_alt = data.get("rel_alt_m")
        gps_fix = data.get("gps_fix_type")
        
        try:
            rel_alt_f = float(rel_alt) if rel_alt is not None else None
        except Exception:
            rel_alt_f = None

        # gps_fix 문자열/숫자 모두 대응
        # MAVSDK usable fix: FIX_3D, FIX_DGPS, FIX_RTK_FLOAT, FIX_RTK_FIXED
        gps_fix_ok = False
        if isinstance(gps_fix, str):
            gps_fix_upper = gps_fix.upper()
            gps_fix_ok = any(
                fix_name in gps_fix_upper
                for fix_name in ("3D", "DGPS", "RTK")
            )
        else:
            try:
                gps_fix_ok = int(gps_fix) >= 3
            except Exception:
                gps_fix_ok = False
                
        # lat/lon이 없어도 rel_alt는 UI에 보여줘야 하므로 여기서 return 하지 않음
        ground_alt = None

        if lat is not None and lon is not None:
            ground_alt = self.sample_dem(lat, lon)

        # HOME 고도 자동 설정
        if (
            self.home_ground_alt is None
            and gps_fix_ok
            and rel_alt_f is not None
            and rel_alt_f < 2.0
            and ground_alt is not None
        ):
            self.home_ground_alt = ground_alt
            self.get_logger().info(
                f"[HOME SET] home_ground_alt={self.home_ground_alt}")

        drone_alt = None
        height_agl = None

        if self.home_ground_alt is not None and rel_alt_f is not None:
            drone_alt = self.home_ground_alt + rel_alt_f

        if drone_alt is not None and ground_alt is not None:
            height_agl = drone_alt - ground_alt

        takeoff_status = "CHECKING"
        takeoff_reason = "Waiting for GPS fix / rel_alt / HOME altitude"

        if not gps_fix_ok:
            takeoff_status = "NOT READY"
            takeoff_reason = f"GPS fix not usable ({gps_fix})"
        elif rel_alt is None:
            takeoff_status = "CHECKING"
            takeoff_reason = "rel_alt not received yet"
        else:
            if rel_alt_f is None:
                takeoff_status = "NOT READY"
                takeoff_reason = "rel_alt parse failed"
            elif rel_alt_f >= 2.0:
                takeoff_status = "NOT READY"
                takeoff_reason = f"rel_alt too high ({rel_alt_f:.2f}m)"
            elif self.home_ground_alt is None:
                takeoff_status = "CHECKING"
                takeoff_reason = "HOME altitude not initialized yet"
            else:
                takeoff_status = "READY"
                takeoff_reason = "Takeoff condition satisfied"

        # UI publish
        alt_msg = {
            "rel_alt_m": rel_alt_f,     # GPS Fix 없어도 표시할 FC 상대고도
            "gps_fix_type": gps_fix,
            "gps_fix_ok": gps_fix_ok,

            "home_ground_alt_m": self.home_ground_alt,
            "ground_alt_m": ground_alt,
            "drone_alt_amsl_m": drone_alt,
            "height_agl_m": height_agl,

            "takeoff_status": takeoff_status,
            "takeoff_reason": takeoff_reason,
        }

        ros_msg = String()
        ros_msg.data = json.dumps(alt_msg)

        self.pub_altitude.publish(ros_msg)

        pose_snapshot = {
            "telem_time_ns": pose_time_ns,
            "telemetry_publish_time_ns": data.get("telemetry_publish_time_ns", pose_time_ns),
            "attitude_time_ns": data.get("attitude_time_ns"),
            "position_time_ns": data.get("position_time_ns"),
            "lat_deg": lat,
            "lon_deg": lon,
            "abs_alt_m": data.get("abs_alt_m"),
            "rel_alt_m": rel_alt_f,
            "roll_deg": data.get("roll_deg"),
            "pitch_deg": data.get("pitch_deg"),
            "yaw_deg": data.get("yaw_deg"),
            "ground_alt_m": ground_alt,
            "drone_alt_amsl_m": drone_alt,
            "height_agl_m": height_agl,
        }

        self.latest_pose = pose_snapshot
        self.pose_history.append(pose_snapshot)
    def _cache_frame(self, seq: int, meta: dict, jpg=None):
        """
        seq 기준으로 프레임 메타를 저장한다.

        기존 구조:
        - /camera/frame_packet → meta + jpg 저장

        신규 H.264 직접 구조:
        - /camera/frame_meta → meta만 저장, jpg=None

        클릭 계산에는 jpg 이미지가 필수가 아니고,
        seq/t_cam_ns/pose/u/v가 핵심이다.
        """
        try:
            seq = int(seq)
        except Exception:
            return

        if seq < 0:
            return

        meta2 = dict(meta or {})
        meta2["seq"] = seq

        old_item = self.frames.get(seq)

        if old_item is None:
            self.frames[seq] = (meta2, jpg)
            self.seq_fifo.append(seq)
        else:
            old_meta, old_jpg = old_item

            merged_meta = dict(old_meta or {})
            merged_meta.update(meta2)

            # 기존 FramePacket JPEG가 있으면 유지하고,
            # frame_meta만 들어온 경우에는 jpg=None 유지
            final_jpg = old_jpg if old_jpg is not None else jpg
            self.frames[seq] = (merged_meta, final_jpg)

        while len(self.seq_fifo) > KEEP_FRAMES_N:
            old = self.seq_fifo.popleft()
            self.frames.pop(old, None)

        if seq in self.pending:
            meta_for_event, jpg_for_event = self.frames.get(seq, (meta2, jpg))
            for evt in self.pending.pop(seq):
                self._handle_event_with_frame(evt, meta_for_event, jpg_for_event)
    # ---------------- frame ----------------
    def on_frame(self, msg: FramePacket):

        seq = int(msg.seq)

        meta = {
            "seq": seq,
            "width": int(msg.width),
            "height": int(msg.height),
            "t_cam_ns": int(msg.t_cam_ns),
            "FrameWallClock": int(msg.frame_wallclock_ns),
            "SensorTimestamp": int(msg.sensor_timestamp_ns),
            "ExposureTime_us": int(msg.exposure_us),
            "AnalogueGain": float(msg.analogue_gain),
            "DigitalGain": float(msg.digital_gain),
            "FrameDuration": int(msg.frame_duration_ns),
            "host_time_ns": int(msg.host_time_ns),
        }

        jpg = bytes(msg.jpg)
        self._cache_frame(seq, meta, jpg)
    
    def on_frame_meta(self, msg: String):
        """
        H.264 직접 수신 구조용 frame_meta callback.

        /camera/frame_meta는 JPEG 이미지를 포함하지 않고,
        seq / pts_ns / frame_t_cam_ns / width / height 같은 시간 동기용 메타만 가진다.

        compute_logger의 역투영 계산은 실제 이미지 bytes가 아니라
        클릭 좌표 u,v + 프레임 시간 + 드론 pose를 사용하므로
        frame_meta만으로도 계산 가능하다.
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        def to_int(v, default=0):
            try:
                if v is None:
                    return int(default)
                return int(v)
            except Exception:
                return int(default)

        try:
            seq = int(data.get("seq", -1))
        except Exception:
            return

        if seq < 0:
            return

        frame_t_cam_ns = data.get("frame_t_cam_ns", data.get("t_cam_ns"))
        if frame_t_cam_ns is None:
            return

        frame_t_cam_ns = to_int(frame_t_cam_ns, 0)
        if frame_t_cam_ns <= 0:
            return

        meta = dict(data)
        meta["seq"] = seq

        # 기존 FramePacket 기반 코드와 호환되도록 key 정규화
        meta["t_cam_ns"] = frame_t_cam_ns
        meta["frame_t_cam_ns"] = frame_t_cam_ns
        meta["FrameWallClock"] = to_int(
            meta.get("FrameWallClock", meta.get("frame_wallclock_ns", frame_t_cam_ns)),
            frame_t_cam_ns
        )
        meta["SensorTimestamp"] = to_int(
            meta.get("SensorTimestamp", meta.get("sensor_timestamp_ns", 0)),
            0
        )
        meta["ExposureTime_us"] = to_int(
            meta.get("ExposureTime_us", meta.get("exposure_us", 0)),
            0
        )
        meta["FrameDuration"] = to_int(
            meta.get("FrameDuration", meta.get("frame_duration_ns", 0)),
            0
        )
        meta["host_time_ns"] = to_int(meta.get("host_time_ns", 0), 0)
        meta["width"] = to_int(meta.get("width", 1456), 1456)
        meta["height"] = to_int(meta.get("height", 1088), 1088)
        meta["_frame_source"] = "frame_meta"

        # jpg 없이 meta만 저장
        self._cache_frame(seq, meta, jpg=None)

    # ---------------- click ----------------
    def on_click(self, msg: String):

        try:
            evt = json.loads(msg.data)
        except Exception:
            return

        click_type = str(evt.get("click_type", "target")).strip().lower()
        if click_type not in ("target", "impact"):
            click_type = "target"
        evt["click_type"] = click_type

        seq = int(evt.get("seq", -1))

        if seq < 0:
            self._publish_target_debug(
                "FAIL",
                evt=evt,
                reason="INVALID_CLICK",
            )
            return

        item = self.frames.get(seq)

        if item is None:
            self.pending.setdefault(seq, []).append(evt)

            self._publish_target_debug(
                "WAIT_FRAME",
                evt=evt,
                reason="NO_FRAME",
            )
            return

        meta, jpg = item
        self._handle_event_with_frame(evt, meta, jpg)

    # ---------------- event 처리 ----------------
    def _handle_event_with_frame(self, evt, meta, jpg):

        frame_time_ns = self._get_frame_time_ns(meta)
        matched_pose = self._match_pose_by_frame_time(frame_time_ns)

        if matched_pose is not None:
            pose_for_event = matched_pose
            pose_match_mode = str(
                matched_pose.get("pose_match_mode", "time_aligned_pose")
            )
            pose_dt_ms = matched_pose.get("match_dt_ms")
        else:
            # 정확도 패치 이후에는 오래된 latest 통합 pose를 억지로 사용하지 않는다.
            # 앞뒤 보간 또는 150ms 이내 nearest가 모두 불가능하면 계산을 거부한다.
            pose_for_event = None
            pose_match_mode = "no_time_aligned_pose"
            pose_dt_ms = None

        self._publish_target_debug(
            "COMPUTING",
            evt=evt,
            meta=meta,
            pose_dt_ms=pose_dt_ms,
            pose_match_mode=pose_match_mode,
        )
        
        click_geometry = self._build_click_geometry(evt, pose_for_event)
        out = {
            "evt": evt,
            "meta": meta,
            "frame_time_ns": frame_time_ns,
            "pose": pose_for_event,
            "pose_match_mode": pose_match_mode,
            "pose_match_dt_ms": pose_dt_ms,
            "click_geometry": click_geometry,
        }

        hit_result = None

        if pose_for_event is None:
            hit_result = {"status": "NO_POSE_MATCH"}

        elif click_geometry is None:
            hit_result = {"status": "NO_CLICK_GEOMETRY"}

        elif "ray_world" not in click_geometry:
            hit_result = {"status": "NO_RAY_WORLD"}

        elif "camera_origin_world_m" not in click_geometry:
            hit_result = {"status": "NO_CAMERA_ORIGIN_WORLD"}

        else:
            # pose 품질 체크
            if pose_dt_ms is not None and pose_dt_ms > 150.0:
                hit_result = {
                    "status": "POSE_DT_TOO_LARGE",
                    "pose_dt_ms": float(pose_dt_ms),
                }
            elif pose_for_event.get("drone_alt_amsl_m") is None:
                hit_result = {"status": "NO_DRONE_ALT_AMSL"}
            elif self.home_ground_alt is None:
                hit_result = {"status": "HOME_NOT_SET"}
            else:
                hit_result = self._intersect_ray_with_dem(
                    np.array(click_geometry["camera_origin_world_m"], dtype=np.float64),
                    np.array(click_geometry["ray_world"], dtype=np.float64),
                    step_m=1.0,
                    max_range_m=300.0,
                    bisection_iters=12,
                    max_nodata_steps=3,
                )

        out["dem_intersection"] = hit_result
            
        seq = meta["seq"]

        img_path = None

        if jpg is not None:
            img_path = os.path.join(SAVE_DIR, f"frame_{seq:06d}.jpg")

            if not os.path.exists(img_path):
                try:
                    with open(img_path, "wb") as f:
                        f.write(jpg)
                except Exception:
                    img_path = None

        out["image_path"] = img_path

        try:
            with open(self.event_log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(out, ensure_ascii=False) + "\n")
        except Exception as e:
            self.get_logger().warn(f"log write fail: {e}")
            
        if click_geometry is not None:
            rb = click_geometry["ray_body"]
            cam_o = click_geometry["camera_origin_body_m"]
            self.get_logger().info(
                "[RAY_BODY] "
                f"seq={seq} "
                f"ray_body=({rb[0]:+.4f},{rb[1]:+.4f},{rb[2]:+.4f}) "
                f"cam_body=({cam_o[0]:+.4f},{cam_o[1]:+.4f},{cam_o[2]:+.4f})"
            )
        if click_geometry is not None and "ray_world" in click_geometry:
            rw = click_geometry["ray_world"]
            cow = click_geometry["camera_offset_world_m"]
            self.get_logger().info(
                "[RAY_WORLD] "
                f"seq={seq} "
                f"ray_world=({rw[0]:+.4f},{rw[1]:+.4f},{rw[2]:+.4f}) "
                f"cam_offset_world=({cow[0]:+.4f},{cow[1]:+.4f},{cow[2]:+.4f})"
            )
        if click_geometry is not None and "camera_origin_world_m" in click_geometry:
            dp = click_geometry["drone_pos_world_m"]
            cp = click_geometry["camera_origin_world_m"]
            self.get_logger().info(
                "[POSE_WORLD] "

                f"seq={seq} "
                f"drone_world=({dp[0]:.3f},{dp[1]:.3f},{dp[2]:.3f}) "
                f"camera_world=({cp[0]:.3f},{cp[1]:.3f},{cp[2]:.3f})"
            )
        if hit_result is not None:
            if hit_result.get("status") == "HIT":
                tp = hit_result["point_world_m"]
                rng = hit_result["range_m"]
                
                lat_t, lon_t = self._dem_world_to_wgs84(tp[0], tp[1])
                if lat_t is not None and lon_t is not None:
                    bearing_deg = None
                    try:
                        if click_geometry is not None and "camera_origin_world_m" in click_geometry:
                            cam_o = click_geometry["camera_origin_world_m"]
                            dx = float(tp[0]) - float(cam_o[0])
                            dy = float(tp[1]) - float(cam_o[1])
                            bearing_deg = math.degrees(math.atan2(dx, dy))
                    except Exception:
                        bearing_deg = None

                    click_type = str(evt.get("click_type", "target")).strip().lower()
                    if click_type not in ("target", "impact"):
                        click_type = "target"

                    point_msg = {
                        "seq": int(seq),
                        "click_type": click_type,
                        "t_click_ns": evt.get("t_click_ns"),
                        "lat_deg": lat_t,
                        "lon_deg": lon_t,
                        "z_m": float(tp[2]),
                        "range_m": float(rng),
                        "x_m": float(tp[0]),
                        "y_m": float(tp[1]),
                        "bearing_deg": bearing_deg,
                        "pose_dt_ms": float(pose_dt_ms) if pose_dt_ms is not None else None,
                        "pose_match_mode": pose_match_mode,
                        "attitude_dt_ms": pose_for_event.get("attitude_dt_ms") if pose_for_event else None,
                        "position_dt_ms": pose_for_event.get("position_dt_ms") if pose_for_event else None,
                        "attitude_match_mode": pose_for_event.get("attitude_match_mode") if pose_for_event else None,
                        "position_match_mode": pose_for_event.get("position_match_mode") if pose_for_event else None,
                        "attitude_sample_span_ms": pose_for_event.get("attitude_sample_span_ms") if pose_for_event else None,
                        "position_sample_span_ms": pose_for_event.get("position_sample_span_ms") if pose_for_event else None,
                        "status": "HIT",
                    }
                    ros_msg = String()
                    ros_msg.data = json.dumps(point_msg, ensure_ascii=False)
                    if click_type == "impact":
                        self.pub_impact.publish(ros_msg)
                    else:
                        self.pub_target.publish(ros_msg)
                    
                    debug_status = "HIT_OK"
                    warn_reason = None

                    try:
                        ray_z = None
                        if click_geometry is not None and "ray_world" in click_geometry:
                            ray_z = float(click_geometry["ray_world"][2])

                        # world z 기준:
                        # ray_z < 0  : 지면 방향
                        # ray_z >= 0 : 상승 방향
                        # 기존 RAY_UPWARD는 _intersect_ray_with_dem()에서 FAIL로 먼저 걸러짐

                        if pose_dt_ms is not None and float(pose_dt_ms) > 100.0:
                            debug_status = "HIT_WARN"
                            warn_reason = "POSE_DT_TOO_LARGE"

                        #elif float(rng) < 3.0 and ray_z is not None and abs(ray_z) < 0.20:
                        #    debug_status = "HIT_WARN"
                        #    warn_reason = "RANGE_TOO_SHORT_WITH_SHALLOW_RAY"

                        #elif float(rng) < 3.0:
                        #    debug_status = "HIT_WARN"
                        #    warn_reason = "RANGE_TOO_SHORT"

                        elif float(rng) > 250.0:
                            debug_status = "HIT_WARN"
                            warn_reason = "RANGE_TOO_LONG"
                    except Exception:
                        pass

                    self._publish_target_debug(
                        debug_status,
                        evt=evt,
                        meta=meta,
                        pose_dt_ms=pose_dt_ms,
                        pose_match_mode=pose_match_mode,
                        reason=warn_reason,
                        lat_deg=lat_t,
                        lon_deg=lon_t,
                        z_m=float(tp[2]),
                        range_m=float(rng),
                    )
                    # MMR 입력용 /target/point는 표적 클릭에만 발행한다.
                    # 탄착 클릭은 MMR의 현재 목표 좌표를 절대 덮어쓰지 않는다.
                    if click_type == "target":
                        self._publish_mmr_target_json(
                            seq=int(seq),
                            lat_t=float(lat_t),
                            lon_t=float(lon_t),
                            z_m=float(tp[2]),
                            range_m=float(rng),
                            x_m=float(tp[0]),
                            y_m=float(tp[1]),
                            bearing_deg=bearing_deg,
                            frame_time_ns=frame_time_ns,
                            pose_for_event=pose_for_event,
                            pose_dt_ms=pose_dt_ms,
                            pose_match_mode=pose_match_mode,
                            click_geometry=click_geometry,
                        )
                else:
                    self._publish_target_debug(
                        "FAIL",
                        evt=evt,
                        meta=meta,
                        pose_dt_ms=pose_dt_ms,
                        pose_match_mode=pose_match_mode,
                        reason="WGS84_CONVERT_FAIL",
                    )
                self.get_logger().info(
                    f"[{'IMPACT' if evt.get('click_type') == 'impact' else 'TARGET'}_WORLD] "
                    f"seq={seq} "
                    f"hit=({tp[0]:.3f},{tp[1]:.3f},{tp[2]:.3f}) "
                    f"range={rng:.2f}m"
                )
            else:
                fail_reason = hit_result.get("status", "UNKNOWN_FAIL")

                self._publish_target_debug(
                    "FAIL",
                    evt=evt,
                    meta=meta,
                    pose_dt_ms=pose_dt_ms,
                    pose_match_mode=pose_match_mode,
                    reason=fail_reason,
                )

                self.get_logger().warn(
                    f"[{'IMPACT' if evt.get('click_type') == 'impact' else 'TARGET'}_WORLD] "
                    f"seq={seq} status={fail_reason}"
                )
        dt_s = "N/A" if pose_dt_ms is None else f"{pose_dt_ms:.1f}ms"
        self.get_logger().info(
            f"[OK] {evt.get('click_type', 'target')} click matched seq={seq} "
            f"uv=({evt.get('u')},{evt.get('v')}) pose_dt={dt_s} mode={pose_match_mode}"
        )
def main():
    rclpy.init()
    node = ComputeLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()