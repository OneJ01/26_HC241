"""
ballistics.py
─────────────────────────────────────────────────────────────────────────────
역할
- world frame(EPSG) 기반 수평 거리 / 방위각 계산
- DEM CRS 대응 지형 고도 조회
- 고도 기준(ellipsoid / AMSL) 명시 처리
- 포신 길이 2-pass 보정
- 포물선 운동 기반 앙각 계산
- DEM 기반 탄도 경로 충돌 검사
- 결과를 FireSolution dataclass로 반환

이번 수정에서 핵심적으로 바꾼 점
- 기존에는 UTM52N 고정 유틸 중심이라 좌표계 일반화가 약했음
- 이제는 BallisticsConfig.world_epsg 기반 transformer를 엔진 내부에 명시적으로 둠
- 기존에는 DEM에 (lon, lat)를 바로 넣는 위험이 있었음
- 이제는 DEM CRS를 읽고 WGS84 -> DEM CRS 변환 후 row/col 계산
- 기존에는 launch/target 고도 출처가 결과에 남지 않았음
- 이제는 launch_alt_source / target_alt_source / vertical_datum 등을 결과에 포함
- 기존에는 FireSolution 반환부마다 진단 필드 일관성이 부족했음
- 이제는 성공/실패 모든 반환부에 핵심 메타를 최대한 동일하게 채움
- 기존에는 포신 길이 보정이 1회 근사 수준이었음
- 이제는 1차 앙각 계산 후 barrel_length*cos(theta)로 다시 유효 사거리를 보정하는 2-pass 방식 사용
- _bearing_to_target을 UTM 그리드 북 기준에서 WGS84 타원체 진북 기준(Geod.inv)으로 변경
  → /gnss/heading(진북 기준)과 동일한 기준으로 turret_yaw 계산되어 그리드 수렴각 오차 제거
─────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from pyproj import Geod, Transformer  # ← Geod 추가

try:
    import rasterio
    _HAS_RASTERIO = True
except ImportError:
    rasterio = None
    _HAS_RASTERIO = False

try:
    from osgeo import gdal as _gdal
    _HAS_GDAL = not _HAS_RASTERIO
except ImportError:
    _gdal = None
    _HAS_GDAL = False


# =====================================================================
# 설정값
# =====================================================================

@dataclass
class BallisticsConfig:
    # ── 좌표계 / 기준
    world_epsg: str = "EPSG:32652"
    vertical_datum: str = "AMSL"
    heading_convention: str = "CW_FROM_NORTH"

    # ── 고도 입력 정책
    # ellipsoid: 들어온 alt를 타원체고로 보고 geoid를 빼서 AMSL로 환산
    # amsl     : 들어온 alt를 이미 AMSL로 간주
    input_altitude_mode: str = "ellipsoid"
    prefer_dem_for_ground: bool = True

    # ── 발사체 / 물리
    muzzle_velocity_ms: float = 15.0
    gravity_ms2: float = 9.8044

    # ── 포신 기구
    barrel_height_m: float = 0.35
    barrel_length_m: float = 0.40
    elevation_min_deg: float = 5.0
    elevation_max_deg: float = 80.0

    # ── 지오이드
    geoid_undulation_m: float = 26.5

    # ── DEM
    dem_path: str = ""
    dem_nodata: float = -9999.0

    # ── 수치해석
    trajectory_check_step_m: float = 1.0
    newton_max_iter: int = 60
    newton_tol_m: float = 0.05


# =====================================================================
# DEM 래스터 핸들러
# =====================================================================

class DEMHandler:
    def __init__(self, dem_path: str, nodata: float = -9999.0):
        self._loaded = False
        self._nodata = nodata
        self._data: Optional[np.ndarray] = None
        self._transform = None
        self._to_dem_crs = None
        self._nrows = 0
        self._ncols = 0
        self._gt = None

        if _HAS_RASTERIO and os.path.isfile(dem_path):
            self._src = rasterio.open(dem_path)
            self._transform = self._src.transform
            self._data = self._src.read(1).astype(np.float64)
            self._data[self._data == nodata] = np.nan
            self._nrows, self._ncols = self._data.shape

            crs = self._src.crs
            if crs is not None:
                self._to_dem_crs = Transformer.from_crs("EPSG:4326", crs, always_xy=True)

            self._loaded = True
            return

        if _HAS_GDAL and os.path.isfile(dem_path):
            ds = _gdal.Open(dem_path)
            if ds:
                self._gt = ds.GetGeoTransform()
                band = ds.GetRasterBand(1)
                self._data = band.ReadAsArray().astype(np.float64)
                self._data[self._data == nodata] = np.nan
                self._nrows, self._ncols = self._data.shape

                proj = ds.GetProjection()
                if proj:
                    try:
                        self._to_dem_crs = Transformer.from_crs("EPSG:4326", proj, always_xy=True)
                    except Exception:
                        self._to_dem_crs = None

                self._loaded = True

    @property
    def loaded(self) -> bool:
        return self._loaded

    def elevation_at(self, lat: float, lon: float) -> Optional[float]:
        if not self._loaded or self._data is None:
            return None

        # WGS84 -> DEM CRS
        x, y = lon, lat
        if self._to_dem_crs is not None:
            try:
                x, y = self._to_dem_crs.transform(lon, lat)
            except Exception:
                return None

        if _HAS_RASTERIO and self._transform is not None:
            try:
                row_f, col_f = rasterio.transform.rowcol(self._transform, x, y, op=float)
            except Exception:
                return None
        elif self._gt is not None:
            gt = self._gt
            col_f = (x - gt[0]) / gt[1]
            row_f = (y - gt[3]) / gt[5]
        else:
            return None

        return self._bilinear(row_f, col_f)

    def _bilinear(self, row_f: float, col_f: float) -> Optional[float]:
        r0 = int(math.floor(row_f))
        c0 = int(math.floor(col_f))
        dr = row_f - r0
        dc = col_f - c0

        def _safe(r: int, c: int) -> Optional[float]:
            if 0 <= r < self._nrows and 0 <= c < self._ncols:
                v = self._data[r, c]
                return None if np.isnan(v) else float(v)
            return None

        v00 = _safe(r0, c0)
        v01 = _safe(r0, c0 + 1)
        v10 = _safe(r0 + 1, c0)
        v11 = _safe(r0 + 1, c0 + 1)

        vals = [v for v in [v00, v01, v10, v11] if v is not None]
        if not vals:
            return None

        # nodata가 일부 섞이면 주변 유효값 평균으로 완만히 대체
        avg = sum(vals) / len(vals)
        v00 = avg if v00 is None else v00
        v01 = avg if v01 is None else v01
        v10 = avg if v10 is None else v10
        v11 = avg if v11 is None else v11

        return (
            v00 * (1 - dr) * (1 - dc)
            + v01 * (1 - dr) * dc
            + v10 * dr * (1 - dc)
            + v11 * dr * dc
        )


# =====================================================================
# 결과 컨테이너
# =====================================================================

@dataclass
class FireSolution:
    # 필수 결과는 앞에 둔다.
    in_range: bool
    solution_type: str

    # 검증 / 메타
    input_validated: bool = False
    validation_status: str = ""
    target_id: Optional[int] = None
    target_source: Optional[str] = None
    sync_error_ms: Optional[float] = None

    # 좌표계 / 기준
    world_frame: str = ""
    vertical_datum: str = ""
    launch_alt_source: str = ""
    target_alt_source: str = ""

    # 추가 진단
    launch_alt_m: Optional[float] = None
    target_alt_m: Optional[float] = None
    solver_iterations: Optional[int] = None
    solver_residual_m: Optional[float] = None
    terrain_check_performed: bool = False

    # 유효 해
    elevation_deg: Optional[float] = None
    turret_yaw_deg: Optional[float] = None
    time_of_flight_s: Optional[float] = None

    # 거리 / 고도
    horizontal_dist_m: float = 0.0
    effective_alt_diff_m: float = 0.0
    terrain_clearance_m: Optional[float] = None

    # 메타
    max_range_m: float = 0.0
    bearing_deg: float = 0.0
    muzzle_velocity_ms: float = 0.0
    barrel_height_m: float = 0.0
    dem_used: bool = False
    warning: str = ""


# =====================================================================
# 엔진
# =====================================================================

class BallisticsEngine:
    def __init__(self, config: BallisticsConfig):
        self.cfg = config
        self._dem = DEMHandler(config.dem_path, config.dem_nodata) if config.dem_path else None
        self._to_world = Transformer.from_crs("EPSG:4326", config.world_epsg, always_xy=True)
        self._from_world = Transformer.from_crs(config.world_epsg, "EPSG:4326", always_xy=True)
        self._geod = Geod(ellps="WGS84")  # ← 추가: 진북 기준 방위각 계산용

    # -----------------------------------------------------------------
    # 좌표 유틸
    # -----------------------------------------------------------------

    def _latlon_to_world(self, lat: float, lon: float) -> Tuple[float, float]:
        x, y = self._to_world.transform(lon, lat)
        return x, y

    def _world_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        lon, lat = self._from_world.transform(x, y)
        return lat, lon

    def _horizontal_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        x1, y1 = self._latlon_to_world(lat1, lon1)
        x2, y2 = self._latlon_to_world(lat2, lon2)
        return math.hypot(x2 - x1, y2 - y1)

    def _bearing_to_target(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        # ← 수정: UTM 그리드 북 기준 → WGS84 타원체 진북 기준(Geod.inv)
        # /gnss/heading이 진북 기준이므로 동일한 기준으로 turret_yaw를 계산하기 위함
        az, _, _ = self._geod.inv(lon1, lat1, lon2, lat2)
        return float(az % 360.0)

    def _offset_latlon(self, lat: float, lon: float, bearing_deg: float, dist_m: float) -> Tuple[float, float]:
        x, y = self._latlon_to_world(lat, lon)
        rad = math.radians(bearing_deg)
        x2 = x + dist_m * math.sin(rad)
        y2 = y + dist_m * math.cos(rad)
        return self._world_to_latlon(x2, y2)

    def _compute_turret_yaw(self, mmr_heading_deg: float, bearing_deg: float) -> float:
        delta = (bearing_deg - mmr_heading_deg + 360.0) % 360.0
        return delta - 360.0 if delta > 180.0 else delta

    # -----------------------------------------------------------------
    # 고도 유틸
    # -----------------------------------------------------------------

    def _normalize_input_altitude(self, alt_in: float) -> float:
        cfg = self.cfg
        if cfg.input_altitude_mode == "amsl":
            return alt_in
        # 기본: ellipsoid -> AMSL
        return alt_in - cfg.geoid_undulation_m

    def _common_result_kwargs(
        self,
        *,
        dem_used: bool,
        launch_alt_source: str,
        target_alt_source: str,
        alt_launch: float,
        alt_target: float,
        solver_iterations: Optional[int],
        solver_residual: Optional[float],
        terrain_check_performed: bool,
        h_dist: float,
        eff_dh: float,
        max_range: float,
        bearing: float,
        v0: float,
    ):
        cfg = self.cfg
        return {
            "input_validated": True,
            "validation_status": "OK",
            "world_frame": cfg.world_epsg,
            "vertical_datum": cfg.vertical_datum,
            "launch_alt_source": launch_alt_source,
            "target_alt_source": target_alt_source,
            "launch_alt_m": round(alt_launch, 3),
            "target_alt_m": round(alt_target, 3),
            "solver_iterations": solver_iterations,
            "solver_residual_m": round(solver_residual, 6) if solver_residual is not None else None,
            "terrain_check_performed": terrain_check_performed,
            "horizontal_dist_m": round(h_dist, 3),
            "effective_alt_diff_m": round(eff_dh, 3),
            "max_range_m": round(max_range, 3),
            "bearing_deg": round(bearing, 3),
            "muzzle_velocity_ms": v0,
            "barrel_height_m": cfg.barrel_height_m,
            "dem_used": dem_used,
        }

    # -----------------------------------------------------------------
    # 공개 인터페이스
    # -----------------------------------------------------------------

    def solve(
        self,
        mmr_lat: float,
        mmr_lon: float,
        mmr_gnss_alt: float,
        target_lat: float,
        target_lon: float,
        target_gnss_alt: float,
        mmr_heading_deg: Optional[float] = None,
    ) -> FireSolution:
        cfg = self.cfg
        g = cfg.gravity_ms2
        v0 = cfg.muzzle_velocity_ms

        # 1) 수평 거리 / 방위각
        h_dist = self._horizontal_distance(mmr_lat, mmr_lon, target_lat, target_lon)
        bearing = self._bearing_to_target(mmr_lat, mmr_lon, target_lat, target_lon)
        max_range = v0 ** 2 / g

        # 2) 고도 기준 결정
        dem_used = False
        solver_iterations: Optional[int] = None
        solver_residual: Optional[float] = None

        base_mmr_alt = self._normalize_input_altitude(mmr_gnss_alt)
        base_target_alt = self._normalize_input_altitude(target_gnss_alt)

        alt_launch = base_mmr_alt + cfg.barrel_height_m
        alt_target = base_target_alt
        launch_alt_source = "gnss+barrel_height"
        target_alt_source = "gnss"

        if cfg.prefer_dem_for_ground and self._dem and self._dem.loaded:
            dem_mmr = self._dem.elevation_at(mmr_lat, mmr_lon)
            dem_tgt = self._dem.elevation_at(target_lat, target_lon)
            if dem_mmr is not None and dem_tgt is not None:
                alt_launch = dem_mmr + cfg.barrel_height_m
                alt_target = dem_tgt
                dem_used = True
                launch_alt_source = "dem+barrel_height"
                target_alt_source = "dem_surface"

        eff_dh = alt_target - alt_launch

        # 3) 포신 길이 2-pass 보정
        initial_elev_rad = math.radians((cfg.elevation_min_deg + cfg.elevation_max_deg) * 0.5)
        initial_barrel_offset = cfg.barrel_length_m * math.cos(initial_elev_rad)
        effective_dist = max(0.01, h_dist - initial_barrel_offset)
        elevation_rad, sol_type, warning, solver_iterations, solver_residual = self._solve_elevation(
            effective_dist, eff_dh, v0, g, cfg
        )

        if elevation_rad is not None:
            barrel_horiz_offset = cfg.barrel_length_m * math.cos(elevation_rad)
            effective_dist = max(0.01, h_dist - barrel_horiz_offset)
            elevation_rad, sol_type, warning, solver_iterations, solver_residual = self._solve_elevation(
                effective_dist, eff_dh, v0, g, cfg
            )

        # 4) 해 없음
        if elevation_rad is None:
            return FireSolution(
                in_range=False,
                solution_type=sol_type,
                warning=warning,
                **self._common_result_kwargs(
                    dem_used=dem_used,
                    launch_alt_source=launch_alt_source,
                    target_alt_source=target_alt_source,
                    alt_launch=alt_launch,
                    alt_target=alt_target,
                    solver_iterations=solver_iterations,
                    solver_residual=solver_residual,
                    terrain_check_performed=False,
                    h_dist=h_dist,
                    eff_dh=eff_dh,
                    max_range=max_range,
                    bearing=bearing,
                    v0=v0,
                ),
            )

        elevation_deg = math.degrees(elevation_rad)

        # 5) 기계 한계 검사
        if not (cfg.elevation_min_deg <= elevation_deg <= cfg.elevation_max_deg):
            return FireSolution(
                in_range=False,
                solution_type="mechanical_limit",
                elevation_deg=round(elevation_deg, 3),
                warning=(
                    f"계산된 앙각 {elevation_deg:.2f}° 가 기계 한계 "
                    f"[{cfg.elevation_min_deg}° ~ {cfg.elevation_max_deg}°] 초과"
                ),
                **self._common_result_kwargs(
                    dem_used=dem_used,
                    launch_alt_source=launch_alt_source,
                    target_alt_source=target_alt_source,
                    alt_launch=alt_launch,
                    alt_target=alt_target,
                    solver_iterations=solver_iterations,
                    solver_residual=solver_residual,
                    terrain_check_performed=False,
                    h_dist=h_dist,
                    eff_dh=eff_dh,
                    max_range=max_range,
                    bearing=bearing,
                    v0=v0,
                ),
            )

        # 6) 비행 시간
        cos_e = math.cos(elevation_rad)
        tof = effective_dist / (v0 * cos_e) if cos_e > 1e-9 else None

        # 7) DEM 지형 충돌 검사
        terrain_clearance = None
        terrain_check_performed = bool(dem_used and tof is not None)
        if dem_used and tof is not None:
            blocked, clearance = self._terrain_clearance_check(
                mmr_lat=mmr_lat,
                mmr_lon=mmr_lon,
                launch_alt_m=alt_launch,
                bearing_deg=bearing,
                effective_dist_m=effective_dist,
                elevation_rad=elevation_rad,
                v0=v0,
                g=g,
                cfg=cfg,
            )
            terrain_clearance = clearance

            if blocked:
                return FireSolution(
                    in_range=False,
                    solution_type="terrain_blocked",
                    elevation_deg=round(elevation_deg, 3),
                    time_of_flight_s=round(tof, 4),
                    terrain_clearance_m=round(clearance, 3) if clearance is not None else None,
                    warning=f"탄도 경로가 지형과 충돌 (최소 여유: {clearance:.2f}m)",
                    **self._common_result_kwargs(
                        dem_used=dem_used,
                        launch_alt_source=launch_alt_source,
                        target_alt_source=target_alt_source,
                        alt_launch=alt_launch,
                        alt_target=alt_target,
                        solver_iterations=solver_iterations,
                        solver_residual=solver_residual,
                        terrain_check_performed=True,
                        h_dist=h_dist,
                        eff_dh=eff_dh,
                        max_range=max_range,
                        bearing=bearing,
                        v0=v0,
                    ),
                )

        # 8) 상대 포신 yaw
        turret_yaw = None
        if mmr_heading_deg is not None:
            turret_yaw = self._compute_turret_yaw(mmr_heading_deg, bearing)

        return FireSolution(
            in_range=True,
            solution_type=sol_type,
            elevation_deg=round(elevation_deg, 3),
            turret_yaw_deg=round(turret_yaw, 3) if turret_yaw is not None else None,
            time_of_flight_s=round(tof, 4) if tof is not None else None,
            terrain_clearance_m=round(terrain_clearance, 3) if terrain_clearance is not None else None,
            warning=warning,
            **self._common_result_kwargs(
                dem_used=dem_used,
                launch_alt_source=launch_alt_source,
                target_alt_source=target_alt_source,
                alt_launch=alt_launch,
                alt_target=alt_target,
                solver_iterations=solver_iterations,
                solver_residual=solver_residual,
                terrain_check_performed=terrain_check_performed,
                h_dist=h_dist,
                eff_dh=eff_dh,
                max_range=max_range,
                bearing=bearing,
                v0=v0,
            ),
        )

    # -----------------------------------------------------------------
    # 내부: 앙각 수치해
    # -----------------------------------------------------------------

    def _solve_elevation(
        self,
        R: float,
        dh: float,
        v0: float,
        g: float,
        cfg: BallisticsConfig,
    ):
        """
        반환:
        (theta_rad | None, solution_type, warning_str, iterations, residual_m)

        수정 이력
        ---------
        [수정] 포신 가동범위가 45°~61° (고각 전용) 이므로 고각 해를 우선 탐색.

        포물선 방정식 R = v²·sin(2θ)/g 는 θ와 (90°-θ) 두 해를 가진다.
        기존 solver는 해석해 θ = 0.5·arcsin(gR/v²) — 저각 해 (< 45°) 를 반환.
        포신 가동범위가 45° 이상이므로 고각 해 = 90° - 저각 해 를 사용해야 한다.

        평지(dh≈0): 고각 해석해 = π/2 - 저각 해석해 를 직접 계산.
        고도차 있음: Newton-Raphson 초기값을 고각 해석해로 설정, 탐색 범위 min~max 제한.
        """
        warning = ""
        theta_min = math.radians(cfg.elevation_min_deg)
        theta_max = math.radians(cfg.elevation_max_deg)

        disc = (g * R) / (v0 ** 2)

        # 사거리 초과 검사 (disc > 1 이면 어떤 각도로도 도달 불가)
        if disc > 1.0:
            return None, "out_of_range", f"사거리 초과 (필요 {R:.1f}m, 최대 {v0**2/g:.1f}m)", 0, None

        # 저각 해석해 → 고각 해석해 (90° - 저각)
        # 근거: sin(2θ) = sin(2(90°-θ)) 이므로 두 해는 대칭
        low_angle_rad  = 0.5 * math.asin(disc)
        high_angle_rad = math.pi / 2.0 - low_angle_rad

        # 평지 근사 (|dh| < 0.01m)
        if abs(dh) < 0.01:
            # 고각 해가 가동범위 내이면 우선 사용
            if theta_min <= high_angle_rad <= theta_max:
                return high_angle_rad, "flat", warning, 1, 0.0
            # 저각 해 시도 (fallback — 거의 발생 안 함)
            if theta_min <= low_angle_rad <= theta_max:
                return low_angle_rad, "flat", warning, 1, 0.0
            return None, "mechanical_limit", (
                f"평지 해 {math.degrees(high_angle_rad):.2f}° / {math.degrees(low_angle_rad):.2f}° 가 "
                f"가동범위 [{cfg.elevation_min_deg}°~{cfg.elevation_max_deg}°] 밖"
            ), 1, None

        # 고도차 있음: Newton-Raphson
        # 초기값: 고각 해석해를 min/max 범위로 클램핑
        theta = max(theta_min, min(theta_max, high_angle_rad))

        def f(th: float) -> float:
            c = math.cos(th)
            if abs(c) < 1e-9:
                return 1e9
            return R * math.tan(th) - (g * R * R) / (2.0 * v0 * v0 * c * c) - dh

        def df(th: float) -> float:
            c = math.cos(th)
            if abs(c) < 1e-9:
                return 1e9
            sec2 = 1.0 / (c * c)
            return R * sec2 - (g * R * R / (v0 * v0)) * sec2 * math.tan(th)

        last_residual = None
        for i in range(1, cfg.newton_max_iter + 1):
            yerr = f(theta)
            last_residual = abs(yerr)
            if abs(yerr) <= cfg.newton_tol_m:
                return theta, "altitude_corrected", warning, i, last_residual

            slope = df(theta)
            if abs(slope) < 1e-12:
                break

            next_theta = theta - yerr / slope
            if not math.isfinite(next_theta):
                break
            theta = max(theta_min, min(theta_max, next_theta))

        return None, "out_of_range", "Newton-Raphson 미수렴 또는 해 없음", cfg.newton_max_iter, last_residual

    # -----------------------------------------------------------------
    # 내부: DEM 기반 탄도 경로 검사
    # -----------------------------------------------------------------

    def _terrain_clearance_check(
        self,
        *,
        mmr_lat: float,
        mmr_lon: float,
        launch_alt_m: float,
        bearing_deg: float,
        effective_dist_m: float,
        elevation_rad: float,
        v0: float,
        g: float,
        cfg: BallisticsConfig,
    ) -> Tuple[bool, Optional[float]]:
        """
        탄도 궤적을 일정 간격으로 샘플링하여 DEM 지형보다 낮아지는 지점이 있는지 검사.
        반환: (blocked, minimum_clearance_m)
        """
        if not self._dem or not self._dem.loaded:
            return False, None

        min_clearance = None
        step = max(0.1, cfg.trajectory_check_step_m)

        dist = step
        while dist < effective_dist_m:
            lat_i, lon_i = self._offset_latlon(mmr_lat, mmr_lon, bearing_deg, dist)
            ground_alt = self._dem.elevation_at(lat_i, lon_i)
            if ground_alt is None:
                dist += step
                continue

            t = dist / (v0 * math.cos(elevation_rad))
            z = launch_alt_m + v0 * math.sin(elevation_rad) * t - 0.5 * g * t * t
            clearance = z - ground_alt

            if min_clearance is None or clearance < min_clearance:
                min_clearance = clearance

            if clearance < 0.0:
                return True, min_clearance

            dist += step

        return False, min_clearance