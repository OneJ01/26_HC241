"""
fire_solution_node.py
─────────────────────────────────────────────────────────────────────────────
역할
- GCS가 계산한 표적 좌표와 메타데이터를 /target/point JSON으로 수신
- MMR 위치(/gnss/fix_hp), MMR 헤딩(/gnss/heading)을 수신
- 입력 freshness / 유효성 / 시간동기 메타를 먼저 검증
- 검증 통과 시에만 BallisticsEngine.solve() 호출
- 결과를 /mmr/fire_solution, 상태를 /mmr/fire_solution/status 로 발행
- 진단 정보를 /mmr/fire_solution/diagnostics 로 발행
- 표적 유무와 관계없이 현재 포신 기구/탄속 기준 발사 가능 범위(/mmr/range_info)를 주기 발행

이번 수정에서 핵심적으로 바꾼 점
- 기존에는 /target/gps만 들어오면 거의 바로 solve()를 호출하던 구조였음
- 이제는 /target/meta를 함께 받고 sync/projection 검증까지 통과해야 계산하도록 변경
- 기존에는 수신 시각을 따로 저장하지 않았음
- 이제는 fix / heading / target / target_meta 각각의 수신 시각을 저장하여 stale 검사를 수행
- 기존에는 diagnostics publisher가 없거나 누락될 수 있었음
- 이제는 diagnostics publisher를 명시적으로 생성하고, 실패/성공 모두 진단 JSON을 발행
- 기존에는 publish_rate_hz 변경 시 타이머가 재생성되지 않았음
- 이제는 파라미터 변경 콜백에서 타이머 재생성까지 처리
- 기존에는 ballistics 엔진 쪽 world frame / vertical datum 설정이 충분히 연동되지 않았음
- 이제는 해당 파라미터를 BallisticsConfig에 명확히 전달
- /mmr/range_info는 표적 계산 결과가 아니라 현재 MMR 기준 발사 가능 영역으로 취급하여 타이머마다 발행

5단계 통합 변경
- 팬축 제거 구조에 맞춰 GNSS heading을 필수 입력으로 사용
- 마지막 표적을 새 표적 수신 또는 명시적 취소 전까지 latch하여 동일 표적 재계산
- MMR 위치·고도·헤딩 또는 표적 변경 시 solution_revision 증가
- /mmr/fire_solution에 입력 스냅샷과 계산 시각을 함께 기록
- 기존 GUI 호환을 위해 /mmr/range_info는 현재 헤딩 기준 부채꼴(SECTOR)로 유지
- publish_rate_hz 타이머는 입력 감시와 결과 heartbeat에 사용
- 탄도 solve는 입력이 설정 임계값 이상 변했을 때만 수행
- /mmr/drive/status가 정지 상태로 전환되는 순간 즉시 최종 solve 수행
─────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import json
import math
import queue
import time
from dataclasses import asdict
from typing import Any, Dict, Optional, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String

from .ballistic_completed import BallisticsConfig, BallisticsEngine
from .wind_correction import WindCorrectionConfig, WindCorrector
from .wind_provider import ManualWindSource, OpenMeteoWindProvider
from .fire_solution_tracking import (
    ConditionalRecomputeGate,
    DriveStopTransitionDetector,
    SolutionInputSnapshot,
    SolutionRevisionTracker,
    is_target_point_stale,
    make_target_key,
)

class FireSolutionNode(Node):
    def __init__(self) -> None:
        super().__init__("fire_solution_node")

        # ─────────────────────────────────────────────────────────
        # 1) 입력 검증 파라미터
        # ─────────────────────────────────────────────────────────
        self.declare_parameter("max_mmr_fix_age_sec", 1.0)
        self.declare_parameter("max_heading_age_sec", 1.0)
        self.declare_parameter("mmr_fix_topic", "/gnss/fix_hp") 
        self.declare_parameter("gnss_heading_topic", "/gnss/heading")
        # latch 활성 시 마지막 표적은 새 표적 수신 또는 명시적 clear 전까지 유지한다.
        # max_target_age_sec는 latch 비활성(legacy) 모드에서만 적용한다.
        self.declare_parameter("target_latch_enabled", True)
        self.declare_parameter("max_target_age_sec", 1.0)

        self.declare_parameter("max_sync_error_ms", 150.0)
        self.declare_parameter("require_target_sync_ok", True)
        self.declare_parameter("require_projection_ok", True)

        # 팬축이 제거되어 차체 회전으로 편각을 맞추므로 헤딩은 필수다.
        self.declare_parameter("require_heading_for_solution", True)
        self.declare_parameter("strict_validation_mode", True)
        self.declare_parameter("publish_diagnostics", True)

        # ─────────────────────────────────────────────────────────
        # 2) 좌표계 / 기준 정보
        # ─────────────────────────────────────────────────────────
        self.declare_parameter("world_frame", "EPSG:32652")
        self.declare_parameter("vertical_datum", "AMSL")
        self.declare_parameter("yaw_convention", "CW_FROM_NORTH")
        self.declare_parameter("input_altitude_mode", "ellipsoid")   # ellipsoid | amsl
        self.declare_parameter("prefer_dem_for_ground", True)

        # ─────────────────────────────────────────────────────────
        # 3) 탄도 / 하드웨어 파라미터
        # ─────────────────────────────────────────────────────────
        self.declare_parameter("muzzle_velocity", 15.0)
        self.declare_parameter("barrel_height_m", 0.35)
        self.declare_parameter("barrel_length_m", 0.40)
        self.declare_parameter("elevation_min_deg", 45.0)
        self.declare_parameter("elevation_max_deg", 61.0)
        # 실제 수평 조준은 차체 회전으로 수행한다. 다만 기존 지도 GUI의
        # 현재 헤딩 기준 부채꼴 사거리 표시를 유지하기 위해 ±45°를 사용한다.
        self.declare_parameter("horizontal_aim_mode", "BODY_ROTATION")
        self.declare_parameter("pan_half_angle_deg", 45.0)
        self.declare_parameter("revision_position_threshold_m", 0.05)
        self.declare_parameter("revision_altitude_threshold_m", 0.05)
        self.declare_parameter("revision_heading_threshold_deg", 0.10)
        self.declare_parameter("revision_target_threshold_m", 0.01)
        self.declare_parameter("drive_status_topic", "/mmr/drive/status")
        self.declare_parameter("recompute_on_drive_stop", True)
        self.declare_parameter("dem_path", "")
        self.declare_parameter("publish_rate_hz", 5.0)
        # 표적 좌표가 없어도 GUI가 현재 MMR 기준 발사 가능 범위를 계속 표시할 수 있도록
        # /mmr/range_info를 주기 발행한다. 0 이하이면 publish_rate_hz와 같은 주기로 발행한다.
        self.declare_parameter("range_info_publish_rate_hz", 1.0)
        self.declare_parameter("gravity_ms2", 9.8044)
        self.declare_parameter("geoid_undulation_m", 26.5)
        self.declare_parameter("trajectory_check_step_m", 1.0)
        self.declare_parameter("newton_max_iter", 60)
        self.declare_parameter("newton_tol_m", 0.05)

        # ─────────────────────────────────────────────────────────
        # 3-b) [신규] 풍속 보정 파라미터
        #      전부 declare_parameter 기반이므로 YAML에 없어도 기본값으로 동작한다.
        # ─────────────────────────────────────────────────────────
        self.declare_parameter("wind_correction_enabled", True)
        self.declare_parameter("wind_api_enabled", True)
        self.declare_parameter("wind_use_mmr_fix_as_station", True)
        self.declare_parameter("wind_station_lat", 37.3402)     # 한국공학대학교
        self.declare_parameter("wind_station_lon", 126.7336)
        self.declare_parameter("wind_cache_ttl_sec", 900.0)
        self.declare_parameter("wind_max_api_calls", 3)
        self.declare_parameter("wind_min_call_interval_sec", 30.0)
        self.declare_parameter("wind_http_timeout_sec", 10.0)
        self.declare_parameter("wind_max_retry_attempts", 3)
        self.declare_parameter("wind_retry_initial_sec", 30.0)
        self.declare_parameter("wind_retry_max_sec", 120.0)
        self.declare_parameter("wind_max_data_age_sec", 3600.0)
        self.declare_parameter("wind_coupling_per_sec", 0.05)
        self.declare_parameter("wind_height_exponent", 0.143)
        self.declare_parameter("wind_gust_blend", 0.0)
        self.declare_parameter("wind_max_offset_m", 5.0)
        self.declare_parameter("wind_iterations", 2)
        self.declare_parameter("wind_min_speed_ms", 0.3)
        self.declare_parameter("wind_manual_speed_ms", -1.0)
        self.declare_parameter("wind_manual_dir_from_deg", 0.0)
        self.declare_parameter("wind_refresh_topic", "/mmr/wind/refresh")
        self.declare_parameter("wind_status_topic", "/mmr/wind/status")
        self.declare_parameter("wind_provider_status_topic", "/mmr/wind/provider_status")
        self.declare_parameter("wind_provider_status_publish_rate_hz", 1.0)
        self.declare_parameter("wind_refresh_on_new_target", True)
        self.declare_parameter("wind_refresh_on_drive_stop", True)

        # 풍속 API 워커 -> ROS 타이머 전달용 스레드 안전 큐
        self._wind_update_queue = queue.SimpleQueue()
        self._last_wind_provider_status_pub_time: float = 0.0

        # 엔진 초기화
        self._engine = self._build_engine()
        self._build_wind_stack()

        # ─────────────────────────────────────────────────────────
        # 4) 최신 입력 상태 + 수신 시각
        # ─────────────────────────────────────────────────────────
        self._mmr_fix_msg: Optional[NavSatFix] = None
        self._mmr_fix_rx_time: Optional[float] = None

        self._heading_deg: Optional[float] = None
        self._heading_rx_time: Optional[float] = None

        self._target_point: Dict[str, Any] = {}
        self._target_point_rx_time: Optional[float] = None
        self._target_latched_at: Optional[float] = None

        self._revision_tracker = SolutionRevisionTracker(
            position_threshold_m=float(self.get_parameter("revision_position_threshold_m").value),
            altitude_threshold_m=float(self.get_parameter("revision_altitude_threshold_m").value),
            heading_threshold_deg=float(self.get_parameter("revision_heading_threshold_deg").value),
            target_threshold_m=float(self.get_parameter("revision_target_threshold_m").value),
        )
        self._recompute_gate = ConditionalRecomputeGate(self._revision_tracker)
        self._drive_stop_detector = DriveStopTransitionDetector()
        self._drive_stopped: Optional[bool] = None

        # 5Hz timer remains a lightweight monitor/heartbeat.  The expensive
        # ballistic solve runs only when inputs change materially, or when a
        # moving drivetrain becomes confirmed stopped.
        self._inputs_dirty: bool = True
        # 비동기 풍속 완료 이벤트는 _wind_update_queue로 전달한다.
        self._cached_solution_payload: Optional[Dict[str, Any]] = None
        self._cached_solution_status: Optional[str] = None
        self._cached_solution_diag: Optional[Dict[str, Any]] = None

        # /mmr/range_info 주기 발행 제어
        self._last_range_info_pub_time: float = 0.0

        # ─────────────────────────────────────────────────────────
        # 5) Subscribers
        # ─────────────────────────────────────────────────────────
        mmr_fix_topic = str(self.get_parameter("mmr_fix_topic").value)
        self.create_subscription(NavSatFix, mmr_fix_topic, self._cb_mmr_fix, 10)

        gnss_heading_topic = str(self.get_parameter("gnss_heading_topic").value)
        self.create_subscription(Float64, gnss_heading_topic, self._cb_heading, 10)

        # GCS compute_logger.py에서 발행하는 표적 좌표 JSON
        self.create_subscription(String, "/target/point", self._cb_target_point, 10)

        drive_status_topic = str(self.get_parameter("drive_status_topic").value)
        self.create_subscription(String, drive_status_topic, self._cb_drive_status, 10)

        # [신규] 풍속 수동 갱신 트리거 (아무 메시지나 오면 1회 취득 시도)
        self.create_subscription(
            String,
            str(self.get_parameter("wind_refresh_topic").value),
            self._cb_wind_refresh,
            10,
        )

        # ─────────────────────────────────────────────────────────
        # 6) Publishers
        # ─────────────────────────────────────────────────────────
        self._pub_sol = self.create_publisher(String, "/mmr/fire_solution", 10)
        self._pub_status = self.create_publisher(String, "/mmr/fire_solution/status", 10)
        self._pub_diag = self.create_publisher(String, "/mmr/fire_solution/diagnostics", 10)
        # [신규] 유효 사거리 정보 발행 — GUI 지도 사거리 링 표시용
        self._pub_range = self.create_publisher(String, "/mmr/range_info", 10)
        # [신규] 풍속 상태 발행 — GUI 표시 및 API 호출 횟수 감시용
        self._pub_wind = self.create_publisher(
            String, str(self.get_parameter("wind_status_topic").value), 10
        )
        # API 수집 상태는 계산 성공 여부와 독립적으로 발행한다.
        self._pub_wind_provider = self.create_publisher(
            String, str(self.get_parameter("wind_provider_status_topic").value), 10
        )

        # 노드 기동 시 캐시 프라이밍 1회. 즉시 시작하지 못하면 큐에 보관된다.
        self._request_wind_refresh(force=True, reason="startup")
        self._publish_wind_provider_status(force=True)

        # ─────────────────────────────────────────────────────────
        # 7) 타이머
        # ─────────────────────────────────────────────────────────
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)

        # 파라미터 변경 콜백
        self.add_on_set_parameters_callback(self._on_param_change)

        # 노드 시작 시 유효 사거리 1회 발행. 이후에는 타이머에서 주기 발행한다.
        self._publish_range_info()

        self.get_logger().info(
            "FireSolutionNode 초기화 완료 | "
            f"v0={self.get_parameter('muzzle_velocity').value} m/s | "
            f"포구 높이={self.get_parameter('barrel_height_m').value} m | "
            f"world={self.get_parameter('world_frame').value} | "
            f"DEM={'활성' if self.get_parameter('dem_path').value else '비활성'}"
        )


    # =====================================================================
    # 유효 사거리 계산 및 발행
    # =====================================================================

    @staticmethod
    def _range_from_elev(elev_deg: float, v0: float, g: float) -> float:
        """
        평지 기준 포물선 사거리.
        공식: R = v0² · sin(2θ) / g
        출처: 고전 탄도학 기초 (McCoy, "Modern Exterior Ballistics" §2)
        """
        theta = math.radians(elev_deg)
        return (v0 ** 2) * math.sin(2.0 * theta) / g

    def _publish_range_info(self) -> None:
        """
        현재 포신 가동범위와 발사체 물리 파라미터를 기준으로
        /mmr/range_info를 발행한다.

        이 토픽은 특정 표적에 대한 탄도해가 아니라, 현재 MMR 위치/헤딩에서
        지도에 표시할 수 있는 기본 발사 가능 영역 정보다. 따라서 표적 좌표가
        없어도 주기적으로 발행한다. GUI는 이 값을 MMR 위치/헤딩과 결합해
        부채꼴 범위를 표시한다.
        """
        v0 = float(self.get_parameter("muzzle_velocity").value)
        g = float(self.get_parameter("gravity_ms2").value)
        emin = float(self.get_parameter("elevation_min_deg").value)
        emax = float(self.get_parameter("elevation_max_deg").value)
        pan_half = float(self.get_parameter("pan_half_angle_deg").value)

        # elev_min/elev_max가 정수가 아니어도 안정적으로 샘플링한다.
        if emax < emin:
            emin, emax = emax, emin

        samples = [emin, emax]
        step = 0.5
        n = max(1, int(math.ceil((emax - emin) / step)))
        for i in range(n + 1):
            a = emin + (emax - emin) * i / n
            samples.append(a)

        # 평지 사거리 최대점인 45°가 가동범위 안에 있으면 반드시 포함한다.
        if emin <= 45.0 <= emax:
            samples.append(45.0)

        ranges = [self._range_from_elev(a, v0, g) for a in samples]
        r_max = max(ranges)
        r_min = min(ranges)

        horizontal_aim_mode = str(self.get_parameter("horizontal_aim_mode").value)
        payload = {
            "elev_min_deg": round(emin, 3),
            "elev_max_deg": round(emax, 3),
            "range_at_elev_min": round(self._range_from_elev(emin, v0, g), 2),
            "range_at_elev_max": round(self._range_from_elev(emax, v0, g), 2),
            "range_max": round(r_max, 2),
            "range_min": round(r_min, 2),
            "pan_half_angle_deg": round(pan_half, 3),
            "horizontal_aim_mode": horizontal_aim_mode,
            "horizontal_coverage_deg": round(2.0 * pan_half, 3),
            "display_shape": "SECTOR",
            "muzzle_velocity_ms": round(v0, 3),
            "gravity_ms2": round(g, 4),
            "target_required": False,
            "source": "fire_solution_capability",
            "note": "flat_terrain_capability_preview",
            "stamp": time.time(),
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_range.publish(msg)

        self.get_logger().debug(
            f"[range_info] {emin:.1f}°~{emax:.1f}° → {r_min:.1f}m ~ {r_max:.1f}m"
        )

    def _maybe_publish_range_info(self, force: bool = False) -> None:
        """/mmr/range_info를 설정된 주기로 발행한다."""
        now = time.time()
        rate = float(self.get_parameter("range_info_publish_rate_hz").value)
        if rate <= 0.0:
            rate = float(self.get_parameter("publish_rate_hz").value)
        rate = max(rate, 0.1)
        period = 1.0 / rate

        if force or (now - self._last_range_info_pub_time) >= period:
            self._publish_range_info()
            self._last_range_info_pub_time = now

    # =====================================================================
    # 파라미터 / 엔진 초기화
    # =====================================================================

    def _build_engine(self) -> BallisticsEngine:
        cfg = BallisticsConfig(
            world_epsg=str(self.get_parameter("world_frame").value),
            vertical_datum=str(self.get_parameter("vertical_datum").value),
            heading_convention=str(self.get_parameter("yaw_convention").value),
            input_altitude_mode=str(self.get_parameter("input_altitude_mode").value),
            prefer_dem_for_ground=bool(self.get_parameter("prefer_dem_for_ground").value),
            muzzle_velocity_ms=float(self.get_parameter("muzzle_velocity").value),
            gravity_ms2=float(self.get_parameter("gravity_ms2").value),
            barrel_height_m=float(self.get_parameter("barrel_height_m").value),
            barrel_length_m=float(self.get_parameter("barrel_length_m").value),
            elevation_min_deg=float(self.get_parameter("elevation_min_deg").value),
            elevation_max_deg=float(self.get_parameter("elevation_max_deg").value),
            geoid_undulation_m=float(self.get_parameter("geoid_undulation_m").value),
            dem_path=str(self.get_parameter("dem_path").value),
            trajectory_check_step_m=float(self.get_parameter("trajectory_check_step_m").value),
            newton_max_iter=int(self.get_parameter("newton_max_iter").value),
            newton_tol_m=float(self.get_parameter("newton_tol_m").value),
        )
        return BallisticsEngine(cfg)

    # =====================================================================
    # [신규] 풍속 보정 스택
    # =====================================================================

    def _build_wind_stack(self, overrides: Optional[Dict[str, Any]] = None) -> None:
        """풍속 제공자/보정기를 재생성한다. 기존 API 제공자는 안전하게 닫는다."""
        values = overrides or {}

        def _value(name: str):
            return values[name] if name in values else self.get_parameter(name).value

        old_provider = getattr(self, "_wind_provider", None)
        if old_provider is not None:
            old_provider.close()

        self._wind_corrector = WindCorrector(
            WindCorrectionConfig(
                enabled=bool(_value("wind_correction_enabled")),
                coupling_per_sec=float(_value("wind_coupling_per_sec")),
                height_exponent=float(_value("wind_height_exponent")),
                gust_blend=float(_value("wind_gust_blend")),
                max_offset_m=float(_value("wind_max_offset_m")),
                iterations=int(_value("wind_iterations")),
                min_speed_ms=float(_value("wind_min_speed_ms")),
            )
        )

        self._wind_manual = ManualWindSource(
            speed_ms=float(_value("wind_manual_speed_ms")),
            direction_from_deg=float(_value("wind_manual_dir_from_deg")),
        )

        self._wind_provider = OpenMeteoWindProvider(
            ttl_sec=float(_value("wind_cache_ttl_sec")),
            max_calls_per_engagement=int(_value("wind_max_api_calls")),
            min_interval_sec=float(_value("wind_min_call_interval_sec")),
            timeout_sec=float(_value("wind_http_timeout_sec")),
            max_retry_attempts=int(_value("wind_max_retry_attempts")),
            retry_initial_sec=float(_value("wind_retry_initial_sec")),
            retry_max_sec=float(_value("wind_retry_max_sec")),
            enabled=bool(_value("wind_api_enabled")),
            on_update=self._on_wind_updated,
            logger=lambda m: self.get_logger().info(m),
        )

    def _wind_station_coords(self) -> Tuple[float, float]:
        """
        API 조회 좌표를 결정한다.
        Open-Meteo 격자 해상도는 1~11km이므로 운동장 내 수 m 차이는 무의미하다.
        MMR 실측 fix가 있으면 그것을, 없으면 고정 좌표(한국공학대)를 쓴다.
        """
        if bool(self.get_parameter("wind_use_mmr_fix_as_station").value):
            fix = self._mmr_fix_msg
            if fix is not None and math.isfinite(fix.latitude) and math.isfinite(fix.longitude):
                return float(fix.latitude), float(fix.longitude)
        return (
            float(self.get_parameter("wind_station_lat").value),
            float(self.get_parameter("wind_station_lon").value),
        )

    def _request_wind_refresh(self, *, force: bool, reason: str) -> bool:
        """예산/주기 조건을 만족할 때만 백그라운드 취득을 시작한다 (논블로킹)."""
        if not bool(self.get_parameter("wind_correction_enabled").value):
            return False
        if self._wind_manual.active:
            return False
        lat, lon = self._wind_station_coords()
        return self._wind_provider.request_refresh(lat, lon, force=force, reason=reason)

    def _on_wind_updated(self, sample) -> None:
        """API 워커에서 호출된다. ROS 상태는 건드리지 않고 이벤트만 큐에 넣는다."""
        self._wind_update_queue.put(sample.fetch_stamp)

    def _cb_wind_refresh(self, msg: String) -> None:
        accepted = self._request_wind_refresh(force=True, reason="manual_request")
        stats = self._wind_provider.stats()
        self.get_logger().info(
            f"[wind] 수동 갱신 요청 | 접수={accepted} | "
            f"판정={stats.get('last_request_decision')} | "
            f"잔여예산={self._wind_provider.budget_remaining()}"
        )
        self._publish_wind_provider_status(force=True)

    def _current_wind_sample(self):
        """수동 입력 우선, 없으면 API 캐시. 너무 오래된 값은 버린다."""
        manual = self._wind_manual.get_sample()
        if manual is not None:
            return manual
        return self._wind_provider.get_sample(
            max_age_sec=float(self.get_parameter("wind_max_data_age_sec").value)
        )

    def _publish_wind_status(self, wind_meta: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(wind_meta, ensure_ascii=False)
        self._pub_wind.publish(msg)

    def _publish_wind_provider_status(self, *, force: bool = False) -> None:
        """표적/헤딩/탄도 계산과 무관하게 API 상태를 주기 발행한다."""
        now = time.time()
        rate = float(self.get_parameter("wind_provider_status_publish_rate_hz").value)
        if not force and rate > 0.0:
            min_period = 1.0 / rate
            if (now - self._last_wind_provider_status_pub_time) < min_period:
                return
        elif not force and rate <= 0.0:
            return

        payload = self._wind_provider.snapshot(
            max_age_sec=float(self.get_parameter("wind_max_data_age_sec").value)
        )
        payload["manual_active"] = self._wind_manual.active
        if self._wind_manual.active:
            manual = self._wind_manual.get_sample()
            payload["valid"] = manual is not None
            payload["stale"] = False
            payload["sample"] = manual.as_dict(now) if manual is not None else None
            payload["active_source"] = "manual"
        else:
            payload["active_source"] = "open_meteo"

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_wind_provider.publish(msg)
        self._last_wind_provider_status_pub_time = now

    def _solve_with_wind(self, **kwargs):
        """
        BallisticsEngine.solve()의 드롭인 대체.
        풍속 데이터가 없거나 보정이 실패하면 원본 해를 그대로 반환한다.
        반환: (FireSolution, wind_meta_dict)
        """
        sample = self._current_wind_sample()

        sol, res = self._wind_corrector.solve(
            self._engine,
            wind_speed_ms=sample.speed_ms if sample else None,
            wind_direction_from_deg=sample.direction_from_deg if sample else None,
            wind_gust_ms=sample.gust_ms if sample else None,
            wind_source=sample.source if sample else "",
            wind_age_sec=sample.age_sec() if sample else None,
            **kwargs,
        )

        meta = res.as_dict()
        meta["provider"] = self._wind_provider.stats()
        meta["sample"] = sample.as_dict() if sample else None
        return sol, meta

    def _on_param_change(self, params):
        param_map = {p.name: p.value for p in params}

        def _get(name: str, default: Any) -> Any:
            if name in param_map:
                return param_map[name]
            return self.get_parameter(name).value if self.has_parameter(name) else default

        muzzle_velocity = float(_get("muzzle_velocity", 15.0))
        publish_rate_hz = float(_get("publish_rate_hz", 5.0))
        range_info_publish_rate_hz = float(_get("range_info_publish_rate_hz", 1.0))
        elevation_min_deg = float(_get("elevation_min_deg", 5.0))
        elevation_max_deg = float(_get("elevation_max_deg", 80.0))
        barrel_height_m = float(_get("barrel_height_m", 0.35))
        barrel_length_m = float(_get("barrel_length_m", 0.40))
        max_sync_error_ms = float(_get("max_sync_error_ms", 50.0))
        world_frame = str(_get("world_frame", "EPSG:32652"))
        input_altitude_mode = str(_get("input_altitude_mode", "ellipsoid"))
        wind_timeout = float(_get("wind_http_timeout_sec", 10.0))
        wind_retry_count = int(_get("wind_max_retry_attempts", 3))
        wind_retry_initial = float(_get("wind_retry_initial_sec", 30.0))
        wind_retry_max = float(_get("wind_retry_max_sec", 120.0))
        wind_status_rate = float(_get("wind_provider_status_publish_rate_hz", 1.0))

        if muzzle_velocity <= 0.0:
            return SetParametersResult(successful=False, reason="muzzle_velocity must be > 0")
        if publish_rate_hz <= 0.0:
            return SetParametersResult(successful=False, reason="publish_rate_hz must be > 0")
        if range_info_publish_rate_hz < 0.0:
            return SetParametersResult(successful=False, reason="range_info_publish_rate_hz must be >= 0")
        if elevation_min_deg >= elevation_max_deg:
            return SetParametersResult(successful=False, reason="elevation_min_deg must be < elevation_max_deg")
        if barrel_height_m < 0.0:
            return SetParametersResult(successful=False, reason="barrel_height_m must be >= 0")
        if barrel_length_m < 0.0:
            return SetParametersResult(successful=False, reason="barrel_length_m must be >= 0")
        if max_sync_error_ms < 0.0:
            return SetParametersResult(successful=False, reason="max_sync_error_ms must be >= 0")
        if not world_frame.startswith("EPSG:"):
            return SetParametersResult(successful=False, reason="world_frame must look like EPSG:XXXX")
        if input_altitude_mode not in ("ellipsoid", "amsl"):
            return SetParametersResult(successful=False, reason="input_altitude_mode must be 'ellipsoid' or 'amsl'")
        if wind_timeout <= 0.0:
            return SetParametersResult(successful=False, reason="wind_http_timeout_sec must be > 0")
        if wind_retry_count < 0:
            return SetParametersResult(successful=False, reason="wind_max_retry_attempts must be >= 0")
        if wind_retry_initial < 0.0 or wind_retry_max < wind_retry_initial:
            return SetParametersResult(
                successful=False,
                reason="wind retry seconds must satisfy 0 <= initial <= max",
            )
        if wind_status_rate < 0.0:
            return SetParametersResult(
                successful=False, reason="wind_provider_status_publish_rate_hz must be >= 0"
            )

        # 엔진 재생성
        self._engine = self._build_engine()
        self._build_wind_stack(param_map)
        self._revision_tracker = SolutionRevisionTracker(
            position_threshold_m=float(_get("revision_position_threshold_m", 0.05)),
            altitude_threshold_m=float(_get("revision_altitude_threshold_m", 0.05)),
            heading_threshold_deg=float(_get("revision_heading_threshold_deg", 0.10)),
            target_threshold_m=float(_get("revision_target_threshold_m", 0.01)),
        )
        self._recompute_gate = ConditionalRecomputeGate(self._revision_tracker)
        self._drive_stop_detector.reset()
        self._cached_solution_payload = None
        self._cached_solution_status = None
        self._cached_solution_diag = None
        self._inputs_dirty = True

        # publish rate 변경 대응: 기존 타이머 취소 후 재생성
        try:
            self._timer.cancel()
        except Exception:
            pass
        new_rate = float(_get("publish_rate_hz", 5.0))
        self._timer = self.create_timer(1.0 / new_rate, self._timer_cb)

        cfg = self._engine.cfg
        self._maybe_publish_range_info(force=True)

        self.get_logger().info(
            "파라미터 갱신 완료 | "
            f"v0={cfg.muzzle_velocity_ms} m/s | "
            f"포구 높이={cfg.barrel_height_m} m | "
            f"world={cfg.world_epsg} | "
            f"DEM={'활성' if cfg.dem_path else '비활성'}"
        )
        return SetParametersResult(successful=True)

    # =====================================================================
    # Subscriber 콜백
    # =====================================================================

    def _cb_mmr_fix(self, msg: NavSatFix) -> None:
        self._mmr_fix_msg = msg
        self._mmr_fix_rx_time = time.time()
        self._inputs_dirty = True

    def _cb_heading(self, msg: Float64) -> None:
        """ZED-F9P Moving Baseline RTK /gnss/heading 수신."""
        self._heading_deg = float(msg.data)
        self._heading_rx_time = time.time()
        self._inputs_dirty = True

    def _cb_drive_status(self, msg: String) -> None:
        """Detect a confirmed drivetrain stop and immediately refresh the solve."""
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
            stopped = bool(payload.get("all_stopped", False))
        except Exception:
            return

        self._drive_stopped = stopped
        stopped_now = self._drive_stop_detector.update(stopped)
        if stopped_now and bool(self.get_parameter("wind_refresh_on_drive_stop").value):
            # [신규] 정렬 완료 직후가 조준 직전 실풍속을 반영할 최적 시점
            self._request_wind_refresh(force=True, reason="drive_stopped")
        if (
            stopped_now
            and bool(self.get_parameter("recompute_on_drive_stop").value)
        ):
            self._compute_and_publish(force_recompute=True, trigger="drive_stopped")

    def _cb_target_point(self, msg: String) -> None:
        """
        GCS compute_logger.py에서 발행한 /target/point JSON 수신.

        포함 정보:
        - lat_deg, lon_deg, z_m
        - sync_ok, projection_ok, dem_intersection_ok
        - sync_error_ms, t_cam_ns, t_pose_ns
        - target_id, source 등
        """
        now = time.time()
        try:
            target = json.loads(msg.data)

            if not isinstance(target, dict):
                raise ValueError("target point JSON root must be object")

            clear_requested = bool(target.get("clear_target", False))
            active_false = target.get("active") is False
            clear_status = str(target.get("status", "")).upper() in {
                "CLEAR", "CLEARED", "CANCEL", "CANCELLED"
            }
            if clear_requested or active_false or clear_status:
                self._target_point = {}
                self._target_point_rx_time = None
                self._target_latched_at = None
                self._recompute_gate.invalidate()
                self._cached_solution_payload = None
                self._cached_solution_status = None
                self._cached_solution_diag = None
                self._inputs_dirty = False
                self._publish_status_only("TARGET_POINT_CLEARED")
                self._publish_diagnostics(
                    "TARGET_POINT_CLEARED",
                    {"message": "Active target was explicitly cleared"},
                )
                return

            self._target_point = target
            self._target_latched_at = now
            self._inputs_dirty = True

        except Exception as exc:
            self._target_point = {
                "parse_error": str(exc),
                "raw": msg.data,
            }

        self._target_point_rx_time = now
        # [신규] 새 표적 = 새 교전 → API 호출 예산 초기화 후 1회 갱신
        if bool(self.get_parameter("wind_refresh_on_new_target").value):
            self._wind_provider.reset_budget()
            self._request_wind_refresh(force=True, reason="new_target")
        self._maybe_publish_range_info(force=True)
        self._compute_and_publish(force_recompute=True, trigger="new_target")

    # =====================================================================
    # 타이머 / 메인 계산
    # =====================================================================

    def _timer_cb(self) -> None:
        # 발사 가능 범위는 표적 유무와 무관한 현재 MMR 기준 capability 정보다.
        self._maybe_publish_range_info()

        # 최소 호출 간격 대기/백오프 재시도 요청을 논블로킹으로 진행한다.
        self._wind_provider.poll()
        self._publish_wind_provider_status()

        # SimpleQueue를 비워 API 갱신 이벤트를 유실 없이 1회 재계산으로 합친다.
        wind_updated = False
        while True:
            try:
                self._wind_update_queue.get_nowait()
                wind_updated = True
            except queue.Empty:
                break
        if wind_updated:
            self._inputs_dirty = True

        self._compute_and_publish(
            force_recompute=wind_updated,
            trigger="wind_updated" if wind_updated else "timer",
        )

    def _compute_and_publish(
        self,
        *,
        force_recompute: bool = False,
        trigger: str = "unspecified",
    ) -> None:
        ok, status, diag = self._validate_all_inputs()

        if not ok:
            self._publish_status_only(status)
            self._publish_diagnostics(status, diag)
            return

        mmr = self._mmr_fix_msg
        target = self._target_point

        # validate_all_inputs()에서 이미 보장하지만, 정적 분석/안전성 때문에 한 번 더 방어
        if mmr is None or not target:
            self._publish_status_only("INTERNAL_INPUT_MISSING")
            self._publish_diagnostics(
                "INTERNAL_INPUT_MISSING",
                {"message": "Validated inputs unexpectedly missing before solve()"},
            )
            return

        target_lat = float(target["lat_deg"])
        target_lon = float(target["lon_deg"])
        target_alt = float(target["z_m"])

        heading_deg = float(self._heading_deg)
        target_key = make_target_key(target)
        snapshot = SolutionInputSnapshot(
            target_key=target_key,
            mmr_lat_deg=float(mmr.latitude),
            mmr_lon_deg=float(mmr.longitude),
            mmr_alt_m=float(mmr.altitude),
            mmr_heading_deg=heading_deg,
            target_lat_deg=target_lat,
            target_lon_deg=target_lon,
            target_alt_m=target_alt,
        )

        # With no newly received input, a cached solution can be republished as
        # a heartbeat without even re-evaluating the thresholds.
        if not self._inputs_dirty and not force_recompute:
            if self._publish_cached_solution(trigger=trigger):
                return

        revision, revision_changed, should_recompute = self._recompute_gate.evaluate(
            snapshot,
            force=force_recompute,
        )
        self._inputs_dirty = False

        if not should_recompute and self._publish_cached_solution(trigger=trigger):
            return

        # [변경] 풍속 보정 래퍼 경유. 풍속 데이터가 없으면 기존과 완전히 동일한 결과.
        sol, wind_meta = self._solve_with_wind(
            mmr_lat=mmr.latitude,
            mmr_lon=mmr.longitude,
            mmr_gnss_alt=mmr.altitude,
            target_lat=target_lat,
            target_lon=target_lon,
            target_gnss_alt=target_alt,
            mmr_heading_deg=self._heading_deg,
        )
        self._publish_wind_status(wind_meta)
        solution_stamp = time.time()

        payload = {k: v for k, v in asdict(sol).items()}
        payload.update(
            {
                "input_validated": True,
                "validation_status": status,
                "solution_revision": revision,
                "solution_revision_changed": revision_changed,
                "solution_stamp": solution_stamp,
                "publish_stamp": solution_stamp,
                "recalculated": True,
                "recalculate_trigger": trigger,
                "target_key": target_key,
                "horizontal_aim_mode": self.get_parameter("horizontal_aim_mode").value,

                # GCS target JSON 메타
                "target_id": target.get("target_id"),
                "target_source": target.get("source"),
                "target_seq": target.get("seq"),
                "target_status": target.get("status"),

                "sync_error_ms": target.get("sync_error_ms"),
                "sync_ok": target.get("sync_ok"),
                "projection_ok": target.get("projection_ok"),
                "dem_intersection_ok": target.get("dem_intersection_ok"),

                "t_cam_ns": target.get("t_cam_ns"),
                "t_pose_ns": target.get("t_pose_ns"),
                "t_publish_ns": target.get("t_publish_ns"),

                # GCS에서 계산한 표적 원본 좌표 기록
                "target_lat_deg": target.get("lat_deg"),
                "target_lon_deg": target.get("lon_deg"),
                "target_z_m": target.get("z_m"),
                "target_x_m": target.get("x_m"),
                "target_y_m": target.get("y_m"),
                "target_world_frame_from_gcs": target.get("world_frame"),
                "target_vertical_reference": target.get("vertical_reference"),

                # MMR 계산 기준 및 해당 revision의 입력 스냅샷
                "mmr_lat_deg": float(mmr.latitude),
                "mmr_lon_deg": float(mmr.longitude),
                "mmr_alt_m": float(mmr.altitude),
                "mmr_heading_deg": heading_deg,
                "drive_stopped_at_solve": self._drive_stopped,
                "mmr_fix_rx_time": self._mmr_fix_rx_time,
                "heading_rx_time": self._heading_rx_time,
                "target_rx_time": self._target_point_rx_time,
                "target_latched": bool(self.get_parameter("target_latch_enabled").value),
                "target_latched_at": self._target_latched_at,
                "world_frame": self.get_parameter("world_frame").value,
                "vertical_datum": self.get_parameter("vertical_datum").value,
                "yaw_convention": self.get_parameter("yaw_convention").value,

                # [신규] 풍속 보정 메타
                "wind": wind_meta,
                "wind_corrected": bool(wind_meta.get("applied", False)),
            }
        )

        sol_msg = String()
        sol_msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_sol.publish(sol_msg)

        final_status = sol.solution_type.upper()
        self._publish_status_only(final_status)

        diag_payload = {
                **diag,
                "solution_type": sol.solution_type,
                "warning": sol.warning,
                "horizontal_dist_m": sol.horizontal_dist_m,
                "effective_alt_diff_m": sol.effective_alt_diff_m,
                "turret_yaw_deg": sol.turret_yaw_deg,
                "elevation_deg": sol.elevation_deg,
                "time_of_flight_s": sol.time_of_flight_s,
                "dem_used": sol.dem_used,
                "solver_iterations": sol.solver_iterations,
                "solver_residual_m": sol.solver_residual_m,
                "solution_revision": revision,
                "solution_revision_changed": revision_changed,
                "solution_stamp": solution_stamp,
                "target_key": target_key,
                "mmr_heading_deg": heading_deg,
                "recalculated": True,
                "recalculate_trigger": trigger,
                "wind": wind_meta,
            }
        self._publish_diagnostics(final_status, diag_payload)

        self._cached_solution_payload = dict(payload)
        self._cached_solution_status = final_status
        self._cached_solution_diag = dict(diag_payload)

        if sol.in_range:
            dist_str = f"{sol.horizontal_dist_m:.1f}m" if sol.horizontal_dist_m is not None else "N/A"
            elev_str = f"{sol.elevation_deg:.2f}°" if sol.elevation_deg is not None else "N/A"
            yaw_str = f"{sol.turret_yaw_deg:+.2f}°" if sol.turret_yaw_deg is not None else "N/A"
            tof_str = f"{sol.time_of_flight_s:.2f}s" if sol.time_of_flight_s is not None else "N/A"

            self.get_logger().debug(
                "해 계산 완료 | "
                f"거리={dist_str} | "
                f"앙각={elev_str} | "
                f"방위={yaw_str} | "
                f"TOF={tof_str} | "
                f"{'DEM' if sol.dem_used else 'GNSS'} 고도 보정"
            )
        else:
            self.get_logger().warn(
                f"[{sol.solution_type}] {sol.warning}",
                throttle_duration_sec=5.0,
            )

    def _publish_cached_solution(self, *, trigger: str) -> bool:
        """Republish the latest solve without running BallisticsEngine.solve()."""
        if self._cached_solution_payload is None or self._cached_solution_status is None:
            return False

        payload = dict(self._cached_solution_payload)
        payload["publish_stamp"] = time.time()
        payload["solution_revision_changed"] = False
        payload["recalculated"] = False
        payload["recalculate_trigger"] = trigger

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_sol.publish(msg)
        self._publish_status_only(self._cached_solution_status)

        diag = dict(self._cached_solution_diag or {})
        diag["solution_revision_changed"] = False
        diag["recalculated"] = False
        diag["recalculate_trigger"] = trigger
        self._publish_diagnostics(self._cached_solution_status, diag)
        return True

    # =====================================================================
    # 공용 유틸
    # =====================================================================

    def _is_finite(self, x: Any) -> bool:
        return isinstance(x, (int, float)) and math.isfinite(float(x))

    def _is_valid_latlon(self, lat: float, lon: float) -> bool:
        return (
            self._is_finite(lat)
            and self._is_finite(lon)
            and (-90.0 <= float(lat) <= 90.0)
            and (-180.0 <= float(lon) <= 180.0)
        )

    def _is_valid_altitude(self, alt: float) -> bool:
        return self._is_finite(alt)

    def _normalize_heading_deg(self, heading_deg: float) -> float:
        return float(heading_deg) % 360.0

    def _age_sec(self, rx_time: Optional[float]) -> Optional[float]:
        if rx_time is None:
            return None
        return max(0.0, time.time() - rx_time)

    def _publish_diagnostics(self, status: str, diag: Dict[str, Any]) -> None:
        if not bool(self.get_parameter("publish_diagnostics").value):
            return

        msg = String()
        payload = {
            "status": status,
            "world_frame": self.get_parameter("world_frame").value,
            "vertical_datum": self.get_parameter("vertical_datum").value,
            "yaw_convention": self.get_parameter("yaw_convention").value,
            **diag,
        }
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_diag.publish(msg)

    def _publish_status_only(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    # =====================================================================
    # 입력 검증
    # =====================================================================

    def _validate_mmr_fix(self) -> Tuple[bool, str, Dict[str, Any]]:
        msg = self._mmr_fix_msg
        age = self._age_sec(self._mmr_fix_rx_time)
        max_age = float(self.get_parameter("max_mmr_fix_age_sec").value)

        if msg is None:
            return False, "MMR_FIX_MISSING", {"message": "MMR fix not received yet"}

        if age is None or age > max_age:
            return False, "MMR_FIX_STALE", {"message": "MMR fix is stale", "mmr_fix_age_sec": age}

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        if not self._is_valid_latlon(lat, lon):
            return False, "MMR_FIX_INVALID", {"message": "Invalid MMR latitude/longitude", "lat": lat, "lon": lon}

        if not self._is_valid_altitude(alt):
            return False, "MMR_FIX_INVALID", {"message": "Invalid MMR altitude", "alt": alt}

        return True, "OK", {
            "mmr_fix_age_sec": age,
            "mmr_lat": lat,
            "mmr_lon": lon,
            "mmr_alt": alt,
        }

    def _validate_heading(self) -> Tuple[bool, str, Dict[str, Any]]:
        require_heading = bool(self.get_parameter("require_heading_for_solution").value)
        age = self._age_sec(self._heading_rx_time)
        max_age = float(self.get_parameter("max_heading_age_sec").value)

        if self._heading_deg is None:
            if require_heading:
                return False, "HEADING_MISSING", {"message": "Heading not received yet"}
            return True, "OK", {"heading_missing_allowed": True}

        if age is None or age > max_age:
            if require_heading:
                return False, "HEADING_STALE", {"message": "Heading is stale", "heading_age_sec": age}
            return True, "OK", {"heading_stale_allowed": True, "heading_age_sec": age}

        if not self._is_finite(self._heading_deg):
            if require_heading:
                return False, "HEADING_INVALID", {"message": "Heading is NaN/inf"}
            return True, "OK", {"heading_invalid_allowed": True}

        self._heading_deg = self._normalize_heading_deg(self._heading_deg)
        return True, "OK", {
            "heading_age_sec": age,
            "heading_deg": self._heading_deg,
        }

    def _validate_target_point(self) -> Tuple[bool, str, Dict[str, Any]]:
        target = self._target_point
        age = self._age_sec(self._target_point_rx_time)
        max_age = float(self.get_parameter("max_target_age_sec").value)

        if not target:
            return False, "TARGET_POINT_MISSING", {
                "message": "Target point JSON not received yet"
            }

        latch_enabled = bool(self.get_parameter("target_latch_enabled").value)
        if is_target_point_stale(
            age,
            latch_enabled=latch_enabled,
            max_age_sec=max_age,
        ):
            return False, "TARGET_POINT_STALE", {
                "message": "Target point JSON is stale",
                "target_point_age_sec": age,
                "target_latch_enabled": latch_enabled,
                "max_target_age_sec": max_age,
            }

        if "parse_error" in target:
            return False, "TARGET_POINT_PARSE_ERROR", {
                "message": "Target point JSON parse failed",
                "parse_error": target.get("parse_error"),
            }

        lat = target.get("lat_deg")
        lon = target.get("lon_deg")
        alt = target.get("z_m")

        if lat is None or lon is None:
            return False, "TARGET_POINT_INVALID", {
                "message": "lat_deg/lon_deg missing",
                "lat_deg": lat,
                "lon_deg": lon,
            }

        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except Exception:
            return False, "TARGET_POINT_INVALID", {
                "message": "lat_deg/lon_deg parse failed",
                "lat_deg": lat,
                "lon_deg": lon,
            }

        if not self._is_valid_latlon(lat_f, lon_f):
            return False, "TARGET_POINT_INVALID", {
                "message": "Invalid target latitude/longitude",
                "lat_deg": lat,
                "lon_deg": lon,
            }

        if alt is None:
            return False, "TARGET_POINT_INVALID", {
                "message": "z_m missing",
                "z_m": alt,
            }

        try:
            alt_f = float(alt)
        except Exception:
            return False, "TARGET_POINT_INVALID", {
                "message": "z_m parse failed",
                "z_m": alt,
            }

        if not self._is_valid_altitude(alt_f):
            return False, "TARGET_POINT_INVALID", {
                "message": "Invalid target altitude z_m",
                "z_m": alt,
            }

        # -----------------------------
        # GCS 좌표계산 상태 검증
        # -----------------------------
        status = target.get("status")
        if status != "HIT":
            return False, "TARGET_NOT_HIT", {
                "message": "Target status is not HIT",
                "target_status": status,
            }

        sync_error_ms = target.get("sync_error_ms")
        if sync_error_ms is not None and not self._is_finite(sync_error_ms):
            return False, "TARGET_POINT_INVALID", {
                "message": "sync_error_ms is not finite",
                "sync_error_ms": sync_error_ms,
            }

        require_sync_ok = bool(self.get_parameter("require_target_sync_ok").value)
        require_projection_ok = bool(self.get_parameter("require_projection_ok").value)
        max_sync_error_ms = float(self.get_parameter("max_sync_error_ms").value)

        sync_ok = target.get("sync_ok", None)
        projection_ok = target.get("projection_ok", None)
        dem_intersection_ok = target.get("dem_intersection_ok", None)

        if require_sync_ok and sync_ok is not True:
            return False, "TARGET_SYNC_FAILED", {
                "message": "Target sync_ok is False or missing",
                "sync_ok": sync_ok,
            }

        if require_projection_ok and projection_ok is not True:
            return False, "TARGET_PROJECTION_FAILED", {
                "message": "Target projection_ok is False or missing",
                "projection_ok": projection_ok,
            }

        if dem_intersection_ok is not True:
            return False, "TARGET_DEM_INTERSECTION_FAILED", {
                "message": "Target DEM intersection is False or missing",
                "dem_intersection_ok": dem_intersection_ok,
            }

        if sync_error_ms is not None and float(sync_error_ms) > max_sync_error_ms:
            return False, "TARGET_SYNC_ERROR_TOO_LARGE", {
                "message": "sync_error_ms exceeds threshold",
                "sync_error_ms": sync_error_ms,
                "max_sync_error_ms": max_sync_error_ms,
            }

        return True, "OK", {
            "target_point_age_sec": age,
            "target_latch_enabled": latch_enabled,
            "target_latched_at": self._target_latched_at,
            "target_id": target.get("target_id"),
            "target_seq": target.get("seq"),
            "source": target.get("source"),
            "target_lat": lat_f,
            "target_lon": lon_f,
            "target_alt": alt_f,
            "target_x_m": target.get("x_m"),
            "target_y_m": target.get("y_m"),
            "target_world_frame_from_gcs": target.get("world_frame"),
            "target_vertical_reference": target.get("vertical_reference"),
            "sync_ok": sync_ok,
            "projection_ok": projection_ok,
            "dem_intersection_ok": dem_intersection_ok,
            "sync_error_ms": sync_error_ms,
            "t_cam_ns": target.get("t_cam_ns"),
            "t_pose_ns": target.get("t_pose_ns"),
        }

    def _validate_all_inputs(self) -> Tuple[bool, str, Dict[str, Any]]:
        validators = [
            self._validate_mmr_fix,
            self._validate_heading,
            self._validate_target_point,
        ]

        merged: Dict[str, Any] = {}
        for validator in validators:
            ok, status, diag = validator()
            merged.update(diag)
            if not ok:
                return False, status, merged
        return True, "OK", merged


    def destroy_node(self):
        provider = getattr(self, "_wind_provider", None)
        if provider is not None:
            provider.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FireSolutionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()