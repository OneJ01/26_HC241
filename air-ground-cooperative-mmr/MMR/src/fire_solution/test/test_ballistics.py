"""
test_ballistics.py
─────────────────────────────────────────────────────────────────────────────
ballistics.py 핵심 계산 단위 테스트.
DEM 없이 실행 가능 (DEM 의존 테스트는 별도 표시).

실행:
    python -m pytest test/test_ballistics.py -v
─────────────────────────────────────────────────────────────────────────────
"""

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fire_solution.ballistic_completed import (
    BallisticsConfig, BallisticsEngine,
)

# ── 공통 기본 설정 ──────────────────────────────────────────────────────────
BASE_CFG = BallisticsConfig(
    muzzle_velocity_ms  = 20.0,
    barrel_height_m     = 0.35,
    barrel_length_m     = 0.40,
    elevation_min_deg   = 5.0,
    elevation_max_deg   = 80.0,
    dem_path            = '',       # DEM 비활성
    gravity_ms2         = 9.8044,
)
ENG = BallisticsEngine(BASE_CFG)

# BallisticsEngine 인스턴스 메서드를 전역 함수처럼 사용하기 위한 래퍼
# ballistic_completed.py 에서 해당 함수들은 클래스 메서드로 구현되어 있음
def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """수평 거리 (m) — UTM52N 유클리드 거리 기반."""
    return ENG._horizontal_distance(lat1, lon1, lat2, lon2)

def bearing_to_target(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """진북 기준 방위각 (0~360°)."""
    return ENG._bearing_to_target(lat1, lon1, lat2, lon2)

def compute_turret_yaw(mmr_heading_deg: float, bearing_deg: float) -> float:
    """포탑 회전 각도 (-180~+180°)."""
    return ENG._compute_turret_yaw(mmr_heading_deg, bearing_deg)



# ── 기준 좌표 (인천 인근 임의값) ────────────────────────────────────────────
MMR_LAT, MMR_LON, MMR_ALT   = 37.4500, 126.6500, 10.0
TGT_LAT, TGT_LON, TGT_ALT   = 37.4500, 126.6514, 10.0   # 정동 ~120m


# ══════════════════════════════════════════════════════════════════════════
# 유틸리티 함수
# ══════════════════════════════════════════════════════════════════════════

def test_haversine_known():
    """적도 1° 경도 차 ≈ 111.195 km"""
    d = haversine_distance(0.0, 0.0, 0.0, 1.0)
    assert abs(d - 111195.0) < 10, f"하버사인 오차 과다: {d:.1f}m"


def test_haversine_zero():
    assert haversine_distance(37.0, 127.0, 37.0, 127.0) == 0.0


def test_bearing_north():
    """정북 방향: 위도 증가, 경도 동일 → bearing ≈ 0°"""
    b = bearing_to_target(37.0, 127.0, 37.01, 127.0)
    assert abs(b) < 1.0 or abs(b - 360) < 1.0, f"정북 방위각 오류: {b}"


def test_bearing_east():
    """정동 방향: 경도 증가, 위도 동일 → bearing ≈ 90°"""
    b = bearing_to_target(37.0, 127.0, 37.0, 127.01)
    assert abs(b - 90.0) < 1.0, f"정동 방위각 오류: {b}"


def test_turret_yaw_wrap():
    """180° 경계 래핑 테스트"""
    yaw = compute_turret_yaw(350.0, 10.0)
    assert abs(yaw - 20.0) < 0.01, f"yaw 래핑 오류: {yaw}"

    yaw2 = compute_turret_yaw(10.0, 350.0)
    assert abs(yaw2 - (-20.0)) < 0.01, f"yaw 반대 방향 오류: {yaw2}"


# ══════════════════════════════════════════════════════════════════════════
# 포물선 역학 검증
# ══════════════════════════════════════════════════════════════════════════

def test_flat_45deg_max_range():
    """
    45° 발사 = 최대 사거리 (포구 동일 고도 기준).
    barrel_length 오프셋 제거 후 수평 사거리 = R_max 이면 arcsin(1.0) → 45°.
    barrel 오프셋이 effective_dist를 줄이므로, 원시 거리 R+barrel_length 로 조정.
    """
    v0  = 20.0
    g   = 9.8044
    bl  = BASE_CFG.barrel_length_m
    R   = v0**2 / g          # 물리적 최대 사거리 (effective_dist 기준)

    # solve() 내부에서 effective_dist = h_dist - barrel_length 이므로
    # h_dist = R + bl 로 넣으면 effective_dist = R → 정확히 45°
    muzzle_alt = MMR_ALT + BASE_CFG.barrel_height_m
    sol = ENG.solve(
        MMR_LAT, MMR_LON, MMR_ALT,
        *_target_at_dist(R + bl, alt=muzzle_alt),
    )
    assert sol.in_range, f"최대 사거리 내에서 in_range=False: {sol.warning}"
    assert abs(sol.elevation_deg - 45.0) < 1.0, f"45° 근방 이탈: {sol.elevation_deg}°"


def test_flat_short_range():
    """단거리 20m, 포구와 동일 고도 표적 → 진짜 평지 → solution_type='flat', 앙각 < 45°"""
    muzzle_alt = MMR_ALT + BASE_CFG.barrel_height_m
    sol = ENG.solve(
        MMR_LAT, MMR_LON, MMR_ALT,
        *_target_at_dist(20.0, alt=muzzle_alt),
    )
    assert sol.in_range
    assert sol.elevation_deg < 45.0
    assert sol.solution_type == 'flat'


def test_out_of_range():
    """사거리 초과 → in_range=False"""
    v0 = BASE_CFG.muzzle_velocity_ms
    g  = BASE_CFG.gravity_ms2
    over_range = v0**2 / g * 1.5  # 최대 사거리의 150%

    sol = ENG.solve(
        MMR_LAT, MMR_LON, MMR_ALT,
        *_target_at_dist(over_range),
    )
    assert not sol.in_range
    assert sol.solution_type in ('out_of_range', 'mechanical_limit')


def test_altitude_corrected_target_higher():
    """표적이 포구보다 5m 높은 경우 — 앙각이 평지보다 커야 함"""
    dist = 30.0
    flat_sol = ENG.solve(
        MMR_LAT, MMR_LON, 10.0,
        *_target_at_dist(dist, alt=10.0),
    )
    high_sol = ENG.solve(
        MMR_LAT, MMR_LON, 10.0,
        *_target_at_dist(dist, alt=10.0 + BASE_CFG.barrel_height_m + 5.0),
    )
    assert flat_sol.in_range and high_sol.in_range
    assert high_sol.elevation_deg > flat_sol.elevation_deg, (
        f"표적 높을 때 앙각이 더 커야 함: "
        f"flat={flat_sol.elevation_deg:.2f}° high={high_sol.elevation_deg:.2f}°"
    )


def test_altitude_corrected_target_lower():
    """표적이 포구보다 5m 낮은 경우 — 앙각이 평지보다 작아야 함"""
    dist = 30.0
    flat_sol = ENG.solve(
        MMR_LAT, MMR_LON, 10.0,
        *_target_at_dist(dist, alt=10.0),
    )
    low_sol = ENG.solve(
        MMR_LAT, MMR_LON, 10.0,
        *_target_at_dist(dist, alt=10.0 + BASE_CFG.barrel_height_m - 5.0),
    )
    assert flat_sol.in_range and low_sol.in_range
    assert low_sol.elevation_deg < flat_sol.elevation_deg


def test_mechanical_limit_low_elevation():
    """포구 초속 대비 너무 가까운 거리 → 앙각이 하한 미만 → mechanical_limit"""
    cfg = BallisticsConfig(
        muzzle_velocity_ms=20.0,
        barrel_height_m=0.35,
        barrel_length_m=0.40,
        elevation_min_deg=30.0,   # 인위적으로 하한 높임
        elevation_max_deg=80.0,
        dem_path='',
    )
    eng = BallisticsEngine(cfg)
    sol = eng.solve(
        MMR_LAT, MMR_LON, MMR_ALT,
        *_target_at_dist(5.0),   # 매우 근접 → 낮은 앙각 필요
    )
    # 앙각이 5°보다 훨씬 작을 것이므로 30° 하한에 걸림
    assert sol.solution_type == 'mechanical_limit' or not sol.in_range


def test_trajectory_roundtrip():
    """
    계산된 앙각과 TOF로 탄착 위치를 역검증.
    x(t) = v0*cos(θ)*t → R,  y(t) = v0*sin(θ)*t - 0.5*g*t² → dh
    """
    dist = 25.0
    dh   = 3.0   # 표적이 3m 높음
    launch_alt = MMR_ALT + BASE_CFG.barrel_height_m

    sol = ENG.solve(
        MMR_LAT, MMR_LON, MMR_ALT,
        *_target_at_dist(dist, alt=MMR_ALT + dh),
    )
    assert sol.in_range, f"역검증 케이스 in_range=False: {sol.warning}"

    θ  = math.radians(sol.elevation_deg)
    t  = sol.time_of_flight_s
    v0 = BASE_CFG.muzzle_velocity_ms
    g  = BASE_CFG.gravity_ms2

    computed_R  = v0 * math.cos(θ) * t
    computed_dh = v0 * math.sin(θ) * t - 0.5 * g * t**2

    # effective_dist 보정값과 비교
    assert abs(computed_R  - sol.horizontal_dist_m + BASE_CFG.barrel_length_m) < 0.5
    assert abs(computed_dh - sol.effective_alt_diff_m) < 0.1, (
        f"고도차 역검증 실패: computed={computed_dh:.3f}m, sol={sol.effective_alt_diff_m:.3f}m"
    )


# ══════════════════════════════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════════════════════════════

def _target_at_dist(dist_m: float, alt: float = MMR_ALT):
    """MMR 기준 정동 방향으로 dist_m 거리의 표적 (lat, lon, alt)"""
    # 정동: 경도만 증가, 위도 동일
    # 인천 위도 37.45°에서 1° 경도 ≈ cos(37.45°) * 111195m
    cos_lat = math.cos(math.radians(MMR_LAT))
    dlon = dist_m / (cos_lat * 111195.0)
    return MMR_LAT, MMR_LON + dlon, alt


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
