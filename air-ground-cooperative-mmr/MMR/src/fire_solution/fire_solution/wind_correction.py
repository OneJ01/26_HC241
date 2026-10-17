"""
wind_correction.py
─────────────────────────────────────────────────────────────────────────────
역할
- 풍속/풍향을 받아 탄착점 편류(drift)를 추정한다.
- 표적 좌표를 편류의 반대 방향으로 이동시킨 "가상 조준점(aim point)"을 만든다.
- 기존 BallisticsEngine.solve()를 그 가상 조준점으로 재호출한다.
  → 헤딩(turret_yaw_deg)과 앙각(elevation_deg)이 동시에 자동 보정된다.

비침습 설계
- BallisticsEngine / BallisticsConfig / DEMHandler 를 일절 수정하지 않는다.
- 엔진의 private 메서드에 의존하지 않는다 (자체 pyproj.Geod 사용).
- 풍속이 없거나 보정이 실패하면 원본 해를 그대로 반환한다.

─────────────────────────────────────────────────────────────────────────────
물리 모델 근거
─────────────────────────────────────────────────────────────────────────────
현재 BallisticsEngine은 무항력 포물선 모델이다. 무항력 모델에서 일정한 바람은
탄체에 아무런 영향을 주지 못한다(탄체가 공기를 느끼지 않으므로). 따라서 바람
효과를 반영하려면 최소한의 항력 결합항을 도입해야 한다.

선형 항력(저속 소형 탄체 근사) 운동방정식:

    dv/dt = -β (v - W) - g ẑ            β = 1/τ  [1/s]

W = const 로 두고 해석하면

    r(t) = [W - (g/β)ẑ] t
           + (1/β)(v₀ - W + (g/β)ẑ)(1 - e^(-βt))

βt << 1 로 전개하면

    r(t) ≈ v₀t - (g t²/2)ẑ  +  (β/2) W t²  +  O(β²)
           └── 기존 무항력 해 ──┘  └ 바람 항 ┘

∴ 바람에 의한 수평 편류:

    Δ = (1/2) · β · W · t_f²                                  ... (식 1)

여기서 β = wind_coupling_per_sec 이며, 물리적으로는

    β ≈ ρ · C_d · A / (2m) · v̄                               ... (식 2)

ρ=공기밀도[kg/m³], C_d=항력계수, A=단면적[m²], m=질량[kg], v̄=평균 속도[m/s].

중요
----
탄체 자체의 평균 항력(무풍 상태의 사거리 감소)은 이미 경험적으로 역산한
muzzle_velocity(24.13 m/s, 56.62°/55m 기준)에 흡수되어 있다. 따라서 여기서는
바람에 의한 "차분(differential)" 성분만 더한다. 이중 계상이 발생하지 않는다.

고도 보정
--------
API가 주는 값은 지상 10m 풍속이다. 탄도 평균 고도는

    apex = (v₀ sinθ)² / (2g),   h̄ = (2/3)·apex

멱법칙 프로파일 W(h) = W₁₀ · (h/10)^α 로 환산한다 (α=0.143, 개활지 표준값).
55m 사격의 경우 apex ≈ 20.7m, h̄ ≈ 13.8m → 보정계수 ≈ 1.05 로 작다.

풍향 규약
--------
기상학 관례로 풍향은 "불어오는 방향(from)"이다.
바람이 불어가는 방향의 ENU 성분은

    u_east  = -W · sin(dir_from)
    v_north = -W · cos(dir_from)

─────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from pyproj import Geod

_GEOD = Geod(ellps="WGS84")


# =====================================================================
# 설정
# =====================================================================

@dataclass
class WindCorrectionConfig:
    enabled: bool = True

    # 식 (1)의 β [1/s]. 반드시 실사격으로 캘리브레이션할 것.
    coupling_per_sec: float = 0.05

    # 지상 풍속 고도 프로파일
    reference_height_m: float = 10.0
    height_exponent: float = 0.143
    min_profile_height_m: float = 2.0

    # 돌풍 반영 비율 0.0(평균풍만) ~ 1.0(돌풍값 사용)
    gust_blend: float = 0.0

    # 안전 클램프: 조준점 이동 최대량 [m]
    max_offset_m: float = 5.0

    # 가상 조준점 재계산 반복 횟수 (1~3 권장)
    iterations: int = 2

    # 이 풍속 미만이면 보정을 생략한다 [m/s]
    min_speed_ms: float = 0.3


@dataclass
class WindCorrectionResult:
    applied: bool = False
    reason: str = "not_attempted"

    wind_speed_ms: Optional[float] = None
    wind_direction_from_deg: Optional[float] = None
    wind_source: str = ""
    wind_age_sec: Optional[float] = None

    effective_speed_ms: Optional[float] = None
    mean_trajectory_height_m: Optional[float] = None

    drift_east_m: float = 0.0
    drift_north_m: float = 0.0
    drift_total_m: float = 0.0
    drift_clamped: bool = False

    aim_lat_deg: Optional[float] = None
    aim_lon_deg: Optional[float] = None

    delta_yaw_deg: Optional[float] = None
    delta_elevation_deg: Optional[float] = None

    iterations_used: int = 0
    coupling_per_sec: float = 0.0

    def as_dict(self) -> Dict[str, Any]:
        def _r(v, n=3):
            return round(float(v), n) if v is not None else None

        return {
            "applied": self.applied,
            "reason": self.reason,
            "wind_speed_ms": _r(self.wind_speed_ms),
            "wind_direction_from_deg": _r(self.wind_direction_from_deg, 1),
            "wind_source": self.wind_source,
            "wind_age_sec": _r(self.wind_age_sec, 1),
            "effective_speed_ms": _r(self.effective_speed_ms),
            "mean_trajectory_height_m": _r(self.mean_trajectory_height_m, 2),
            "drift_east_m": _r(self.drift_east_m),
            "drift_north_m": _r(self.drift_north_m),
            "drift_total_m": _r(self.drift_total_m),
            "drift_clamped": self.drift_clamped,
            "aim_lat_deg": _r(self.aim_lat_deg, 8),
            "aim_lon_deg": _r(self.aim_lon_deg, 8),
            "delta_yaw_deg": _r(self.delta_yaw_deg),
            "delta_elevation_deg": _r(self.delta_elevation_deg),
            "iterations_used": self.iterations_used,
            "coupling_per_sec": _r(self.coupling_per_sec, 5),
        }


# =====================================================================
# 유틸
# =====================================================================

def wind_vector_enu(speed_ms: float, direction_from_deg: float) -> Tuple[float, float]:
    """
    기상학 풍향(불어오는 방향)을 바람이 불어가는 ENU 벡터로 변환한다.
    반환: (east_ms, north_ms)
    """
    rad = math.radians(direction_from_deg)
    return (-speed_ms * math.sin(rad), -speed_ms * math.cos(rad))


def coupling_from_projectile(
    mass_kg: float,
    diameter_m: float,
    drag_coefficient: float = 0.5,
    air_density: float = 1.225,
    reference_speed_ms: float = 18.0,
) -> float:
    """
    식 (2)로 β를 이론 추정한다. 실사격 캘리브레이션 전 초기값 산출용이다.

        β ≈ ρ · C_d · A / (2m) · v̄

    예) m=0.15kg, d=0.042m, Cd=0.5, v̄=18m/s → β ≈ 0.051 [1/s]
    """
    area = math.pi * (diameter_m * 0.5) ** 2
    return air_density * drag_coefficient * area / (2.0 * mass_kg) * reference_speed_ms


def mean_trajectory_height(v0_ms: float, elevation_deg: float, gravity: float = 9.8044) -> float:
    """포물선 평균 고도 = (2/3)·정점고도."""
    vy = v0_ms * math.sin(math.radians(elevation_deg))
    apex = (vy * vy) / (2.0 * gravity)
    return (2.0 / 3.0) * apex


def scale_wind_to_height(
    speed_10m: float,
    height_m: float,
    *,
    reference_height_m: float = 10.0,
    exponent: float = 0.143,
    min_height_m: float = 2.0,
) -> float:
    """멱법칙 풍속 프로파일 W(h) = W_ref · (h/h_ref)^α."""
    h = max(min_height_m, float(height_m))
    return float(speed_10m) * (h / reference_height_m) ** exponent


def offset_latlon_enu(lat: float, lon: float, east_m: float, north_m: float) -> Tuple[float, float]:
    """WGS84 측지선 기준으로 (동,북) 오프셋을 적용한 위경도를 반환한다."""
    dist = math.hypot(east_m, north_m)
    if dist < 1e-9:
        return lat, lon
    azimuth = math.degrees(math.atan2(east_m, north_m)) % 360.0
    lon2, lat2, _ = _GEOD.fwd(lon, lat, azimuth, dist)
    return lat2, lon2


# =====================================================================
# 보정기
# =====================================================================

class WindCorrector:
    """
    BallisticsEngine을 감싸서 풍향/풍속을 반영한 해를 만든다.
    엔진 자체는 전혀 수정하지 않는다.
    """

    def __init__(self, config: Optional[WindCorrectionConfig] = None) -> None:
        self.cfg = config or WindCorrectionConfig()

    # -----------------------------------------------------------------

    def solve(
        self,
        engine,
        *,
        mmr_lat: float,
        mmr_lon: float,
        mmr_gnss_alt: float,
        target_lat: float,
        target_lon: float,
        target_gnss_alt: float,
        mmr_heading_deg: Optional[float] = None,
        wind_speed_ms: Optional[float] = None,
        wind_direction_from_deg: Optional[float] = None,
        wind_gust_ms: Optional[float] = None,
        wind_source: str = "",
        wind_age_sec: Optional[float] = None,
    ):
        """
        반환: (FireSolution, WindCorrectionResult)

        보정 불가/불필요 시에도 반드시 원본 해를 반환한다 (fail-soft).
        """
        cfg = self.cfg

        base_sol = engine.solve(
            mmr_lat=mmr_lat,
            mmr_lon=mmr_lon,
            mmr_gnss_alt=mmr_gnss_alt,
            target_lat=target_lat,
            target_lon=target_lon,
            target_gnss_alt=target_gnss_alt,
            mmr_heading_deg=mmr_heading_deg,
        )

        res = WindCorrectionResult(
            wind_speed_ms=wind_speed_ms,
            wind_direction_from_deg=wind_direction_from_deg,
            wind_source=wind_source,
            wind_age_sec=wind_age_sec,
            coupling_per_sec=cfg.coupling_per_sec,
        )

        # ── 조기 반환 조건 ──────────────────────────────────────
        if not cfg.enabled:
            res.reason = "disabled"
            return base_sol, res
        if wind_speed_ms is None or wind_direction_from_deg is None:
            res.reason = "no_wind_data"
            return base_sol, res
        if not base_sol.in_range or base_sol.time_of_flight_s is None:
            res.reason = "base_solution_invalid"
            return base_sol, res
        if base_sol.elevation_deg is None:
            res.reason = "base_elevation_missing"
            return base_sol, res

        # 돌풍 블렌딩
        speed = float(wind_speed_ms)
        if wind_gust_ms is not None and cfg.gust_blend > 0.0:
            b = max(0.0, min(1.0, cfg.gust_blend))
            speed = (1.0 - b) * speed + b * float(wind_gust_ms)

        if speed < cfg.min_speed_ms:
            res.reason = "wind_below_threshold"
            res.effective_speed_ms = speed
            return base_sol, res

        v0 = float(getattr(engine.cfg, "muzzle_velocity_ms", 0.0))
        g = float(getattr(engine.cfg, "gravity_ms2", 9.8044))
        if v0 <= 0.0:
            res.reason = "invalid_muzzle_velocity"
            return base_sol, res

        # ── 반복 보정 ───────────────────────────────────────────
        best_sol = base_sol
        cur_sol = base_sol
        iterations = max(1, min(5, int(cfg.iterations)))

        for i in range(iterations):
            tof = cur_sol.time_of_flight_s
            elev = cur_sol.elevation_deg
            if tof is None or elev is None:
                break

            h_bar = mean_trajectory_height(v0, elev, g)
            w_eff = scale_wind_to_height(
                speed,
                h_bar,
                reference_height_m=cfg.reference_height_m,
                exponent=cfg.height_exponent,
                min_height_m=cfg.min_profile_height_m,
            )

            w_e, w_n = wind_vector_enu(w_eff, float(wind_direction_from_deg))

            # 식 (1): Δ = ½ β W t²
            k = 0.5 * cfg.coupling_per_sec * (tof ** 2)
            drift_e = k * w_e
            drift_n = k * w_n

            total = math.hypot(drift_e, drift_n)
            clamped = False
            if total > cfg.max_offset_m and total > 1e-9:
                scale = cfg.max_offset_m / total
                drift_e *= scale
                drift_n *= scale
                total = cfg.max_offset_m
                clamped = True

            # 조준점은 편류의 반대 방향(풍상측)으로 이동
            aim_lat, aim_lon = offset_latlon_enu(target_lat, target_lon, -drift_e, -drift_n)

            trial = engine.solve(
                mmr_lat=mmr_lat,
                mmr_lon=mmr_lon,
                mmr_gnss_alt=mmr_gnss_alt,
                target_lat=aim_lat,
                target_lon=aim_lon,
                target_gnss_alt=target_gnss_alt,
                mmr_heading_deg=mmr_heading_deg,
            )

            if not trial.in_range or trial.elevation_deg is None:
                res.reason = f"corrected_solution_out_of_range_iter{i + 1}"
                res.effective_speed_ms = w_eff
                res.mean_trajectory_height_m = h_bar
                res.drift_east_m = drift_e
                res.drift_north_m = drift_n
                res.drift_total_m = total
                res.drift_clamped = clamped
                return best_sol, res

            best_sol = trial
            cur_sol = trial

            res.applied = True
            res.reason = "ok"
            res.effective_speed_ms = w_eff
            res.mean_trajectory_height_m = h_bar
            res.drift_east_m = drift_e
            res.drift_north_m = drift_n
            res.drift_total_m = total
            res.drift_clamped = clamped
            res.aim_lat_deg = aim_lat
            res.aim_lon_deg = aim_lon
            res.iterations_used = i + 1

        if res.applied:
            if base_sol.elevation_deg is not None and best_sol.elevation_deg is not None:
                res.delta_elevation_deg = best_sol.elevation_deg - base_sol.elevation_deg
            if base_sol.turret_yaw_deg is not None and best_sol.turret_yaw_deg is not None:
                res.delta_yaw_deg = best_sol.turret_yaw_deg - base_sol.turret_yaw_deg

        return best_sol, res
