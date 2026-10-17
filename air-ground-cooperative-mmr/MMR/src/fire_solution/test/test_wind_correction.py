"""
test_wind_correction.py
─────────────────────────────────────────────────────────────────────────────
네트워크 없이 실행 가능한 풍속 보정 검증.
- 회귀 안전성: 풍속 데이터가 없으면 기존 해와 완전히 동일해야 한다.
- 물리 정합성: 측풍은 조준점을 풍상측으로, 배풍은 앙각을 낮추는 방향으로 민다.
- 예산 제어: API 호출은 설정된 상한을 절대 넘지 않는다.
"""

import math

import pytest

from fire_solution.ballistic_completed import BallisticsConfig, BallisticsEngine
from fire_solution.wind_correction import (
    WindCorrectionConfig,
    WindCorrector,
    coupling_from_projectile,
    mean_trajectory_height,
    offset_latlon_enu,
    scale_wind_to_height,
    wind_vector_enu,
)
from fire_solution.wind_provider import ManualWindSource, OpenMeteoWindProvider

# 한국공학대학교 운동장 부근
MMR_LAT, MMR_LON, MMR_ALT = 37.34020, 126.73360, 30.0
# 정북 약 55m (45~61° 가동범위 내 해가 존재하는 거리. 50m는 61.3°로 한계 초과)
TGT_LAT, TGT_LON, TGT_ALT = 37.3406941, 126.73360, 30.0
HEADING = 0.0


def _engine():
    return BallisticsEngine(
        BallisticsConfig(
            world_epsg="EPSG:32652",
            input_altitude_mode="amsl",
            prefer_dem_for_ground=False,
            muzzle_velocity_ms=24.13,
            gravity_ms2=9.8044,
            barrel_height_m=0.96,
            barrel_length_m=0.30,
            elevation_min_deg=45.0,
            elevation_max_deg=61.0,
            dem_path="",
        )
    )


def _solve(corrector, **wind):
    return corrector.solve(
        _engine(),
        mmr_lat=MMR_LAT, mmr_lon=MMR_LON, mmr_gnss_alt=MMR_ALT,
        target_lat=TGT_LAT, target_lon=TGT_LON, target_gnss_alt=TGT_ALT,
        mmr_heading_deg=HEADING,
        **wind,
    )


# =====================================================================
# 벡터 / 프로파일 유틸
# =====================================================================

def test_wind_vector_convention():
    """풍향은 '불어오는 방향'. 서풍(270°)은 동쪽으로 불어간다."""
    e, n = wind_vector_enu(5.0, 270.0)
    assert e == pytest.approx(5.0, abs=1e-9)
    assert n == pytest.approx(0.0, abs=1e-9)

    e, n = wind_vector_enu(5.0, 0.0)          # 북풍 → 남쪽으로
    assert n == pytest.approx(-5.0, abs=1e-9)
    assert e == pytest.approx(0.0, abs=1e-9)


def test_height_profile_is_mild_at_low_apex():
    h = mean_trajectory_height(24.13, 56.62, 9.8044)
    assert 12.0 < h < 16.0                     # apex≈20.7m → h̄≈13.8m
    assert scale_wind_to_height(5.0, h) == pytest.approx(5.0 * (h / 10.0) ** 0.143)


def test_offset_latlon_enu_roundtrip():
    lat2, lon2 = offset_latlon_enu(MMR_LAT, MMR_LON, 3.0, 4.0)
    from pyproj import Geod
    _, _, dist = Geod(ellps="WGS84").inv(MMR_LON, MMR_LAT, lon2, lat2)
    assert dist == pytest.approx(5.0, abs=1e-3)


def test_coupling_estimate_is_physically_sane():
    beta = coupling_from_projectile(mass_kg=0.15, diameter_m=0.042,
                                    drag_coefficient=0.5, reference_speed_ms=18.0)
    assert 0.01 < beta < 0.2


# =====================================================================
# 회귀 안전성 — 가장 중요한 검증
# =====================================================================

def test_no_wind_data_is_bitwise_identical_to_base():
    corrector = WindCorrector(WindCorrectionConfig(enabled=True))
    base = _engine().solve(
        mmr_lat=MMR_LAT, mmr_lon=MMR_LON, mmr_gnss_alt=MMR_ALT,
        target_lat=TGT_LAT, target_lon=TGT_LON, target_gnss_alt=TGT_ALT,
        mmr_heading_deg=HEADING,
    )
    sol, res = _solve(corrector)
    assert res.applied is False
    assert res.reason == "no_wind_data"
    assert sol.elevation_deg == base.elevation_deg
    assert sol.turret_yaw_deg == base.turret_yaw_deg


def test_disabled_flag_is_identical_to_base():
    corrector = WindCorrector(WindCorrectionConfig(enabled=False))
    sol, res = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=270.0)
    assert res.applied is False and res.reason == "disabled"


def test_calm_wind_below_threshold_skips_correction():
    corrector = WindCorrector(WindCorrectionConfig(min_speed_ms=0.3))
    sol, res = _solve(corrector, wind_speed_ms=0.1, wind_direction_from_deg=90.0)
    assert res.applied is False and res.reason == "wind_below_threshold"


# =====================================================================
# 물리 정합성
# =====================================================================

def test_crosswind_shifts_aim_point_upwind():
    """서풍(270°) → 탄체는 동쪽으로 밀림 → 조준점은 서쪽으로 이동해야 한다."""
    corrector = WindCorrector(WindCorrectionConfig(coupling_per_sec=0.05))
    sol, res = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=270.0)

    assert res.applied is True
    assert res.drift_east_m > 0.0                  # 편류는 동쪽
    assert res.aim_lon_deg < TGT_LON               # 조준점은 서쪽
    assert res.delta_yaw_deg is not None and res.delta_yaw_deg < 0.0   # 반시계


def test_crosswind_direction_is_antisymmetric():
    corrector = WindCorrector(WindCorrectionConfig(coupling_per_sec=0.05))
    _, west = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=270.0)
    _, east = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=90.0)
    assert west.drift_east_m == pytest.approx(-east.drift_east_m, rel=1e-6)


def test_tailwind_raises_required_elevation():
    """
    남풍(180°, 남→북)은 북쪽 표적 방향으로 부는 배풍.
    탄체가 더 멀리 날아가므로 조준점을 가깝게 당겨야 하고,
    45~61° 고각 영역에서는 사거리가 짧을수록 앙각이 커진다.
    """
    corrector = WindCorrector(WindCorrectionConfig(coupling_per_sec=0.05))
    base, _ = _solve(corrector)
    tail, res = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=180.0)
    assert res.applied is True
    assert res.drift_north_m > 0.0
    assert tail.elevation_deg > base.elevation_deg


def test_headwind_is_opposite_of_tailwind():
    corrector = WindCorrector(WindCorrectionConfig(coupling_per_sec=0.05))
    base, _ = _solve(corrector)
    head, res = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=0.0)
    assert res.drift_north_m < 0.0
    assert head.elevation_deg < base.elevation_deg


def test_drift_scales_quadratically_with_time_of_flight():
    """Δ ∝ t² 이므로 풍속 2배는 편류 2배여야 한다 (t는 풍속과 무관)."""
    corrector = WindCorrector(WindCorrectionConfig(coupling_per_sec=0.05, iterations=1))
    _, a = _solve(corrector, wind_speed_ms=4.0, wind_direction_from_deg=270.0)
    _, b = _solve(corrector, wind_speed_ms=8.0, wind_direction_from_deg=270.0)
    assert b.drift_total_m == pytest.approx(2.0 * a.drift_total_m, rel=1e-6)


def test_max_offset_clamp_is_enforced():
    corrector = WindCorrector(
        WindCorrectionConfig(coupling_per_sec=0.5, max_offset_m=1.0)
    )
    _, res = _solve(corrector, wind_speed_ms=20.0, wind_direction_from_deg=270.0)
    assert res.drift_clamped is True
    assert res.drift_total_m == pytest.approx(1.0, abs=1e-9)


def test_gust_blend_increases_drift():
    cfg_mean = WindCorrectionConfig(coupling_per_sec=0.05, gust_blend=0.0, iterations=1)
    cfg_gust = WindCorrectionConfig(coupling_per_sec=0.05, gust_blend=1.0, iterations=1)
    _, mean = _solve(WindCorrector(cfg_mean),
                     wind_speed_ms=5.0, wind_direction_from_deg=270.0, wind_gust_ms=12.0)
    _, gust = _solve(WindCorrector(cfg_gust),
                     wind_speed_ms=5.0, wind_direction_from_deg=270.0, wind_gust_ms=12.0)
    assert gust.drift_total_m > mean.drift_total_m


# =====================================================================
# API 호출 예산
# =====================================================================

def test_provider_never_exceeds_budget(monkeypatch):
    calls = {"n": 0}

    class _P(OpenMeteoWindProvider):
        def _fetch(self, latitude, longitude):
            calls["n"] += 1
            raise RuntimeError("network disabled in test")

    p = _P(max_calls_per_engagement=3, min_interval_sec=0.0, ttl_sec=0.0, max_retry_attempts=0)
    for _ in range(50):
        p.request_refresh(MMR_LAT, MMR_LON, force=True)
        if p._thread:
            p._thread.join(timeout=2.0)

    assert calls["n"] <= 3
    assert p.budget_remaining() == 0


def test_budget_resets_per_engagement():
    p = OpenMeteoWindProvider(max_calls_per_engagement=3, enabled=False)
    assert p.budget_remaining() == 3
    p.reset_budget()
    assert p.budget_remaining() == 3


def test_min_interval_queues_rapid_calls_without_starting_second_request():
    class _P(OpenMeteoWindProvider):
        def _fetch(self, latitude, longitude):
            raise RuntimeError("network disabled in test")

    p = _P(
        max_calls_per_engagement=10,
        min_interval_sec=999.0,
        max_retry_attempts=0,
    )
    assert p.request_refresh(MMR_LAT, MMR_LON, force=True, reason="first") is True
    if p._thread:
        p._thread.join(timeout=2.0)
    assert p.request_refresh(MMR_LAT, MMR_LON, force=True, reason="second") is True
    stats = p.stats()
    assert stats["queue_depth"] == 1
    assert stats["last_request_decision"] == "queued"


def test_network_failure_returns_none_and_never_raises():
    p = OpenMeteoWindProvider(
        endpoint="http://127.0.0.1:1/nonexistent",
        max_calls_per_engagement=1,
        min_interval_sec=0.0,
        timeout_sec=0.5,
    )
    p.request_refresh(MMR_LAT, MMR_LON, force=True)
    if p._thread:
        p._thread.join(timeout=5.0)
    assert p.get_sample() is None
    assert p.stats()["last_error"] != ""


def test_manual_source_overrides_and_deactivates():
    assert ManualWindSource(-1.0).get_sample() is None
    s = ManualWindSource(6.0, 225.0).get_sample()
    assert s is not None and s.source == "manual"
    assert s.speed_ms == 6.0 and s.direction_from_deg == 225.0
