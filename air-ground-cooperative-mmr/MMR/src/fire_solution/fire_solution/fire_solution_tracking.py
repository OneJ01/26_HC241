"""Pure-Python helpers for fire-solution input tracking.

This module intentionally has no ROS dependency so revision/latching behaviour can
be unit-tested without hardware or a ROS graph.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

_EARTH_RADIUS_M = 6_371_000.0


def angular_distance_deg(a_deg: float, b_deg: float) -> float:
    """Return the smallest absolute angular separation in degrees."""
    return abs((float(a_deg) - float(b_deg) + 180.0) % 360.0 - 180.0)


def surface_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Small-distance equirectangular approximation on WGS84-like coordinates."""
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    dphi = phi2 - phi1
    dlambda = math.radians(float(lon2) - float(lon1))
    x = dlambda * math.cos((phi1 + phi2) * 0.5)
    return _EARTH_RADIUS_M * math.hypot(x, dphi)


def make_target_key(payload: dict[str, Any]) -> str:
    """Build a stable target key while preserving explicit target identifiers."""
    target_id = payload.get("target_id")
    if target_id not in {None, ""}:
        return f"id:{target_id}"

    seq = payload.get("seq", payload.get("target_seq"))
    if seq not in {None, ""}:
        return f"seq:{seq}"

    try:
        lat = float(payload["lat_deg"])
        lon = float(payload["lon_deg"])
        alt = float(payload["z_m"])
        return f"geo:{lat:.8f},{lon:.8f},{alt:.3f}"
    except (KeyError, TypeError, ValueError):
        return "target:anonymous"


def is_target_point_stale(
    age_sec: Optional[float],
    *,
    latch_enabled: bool,
    max_age_sec: float,
) -> bool:
    """Return whether a target input must be rejected as stale.

    When latching is enabled, the most recently accepted target remains valid
    until a new target replaces it or an explicit clear command is received.
    ``max_age_sec`` is retained only for legacy/non-latched operation.
    """
    if age_sec is None:
        return True
    if latch_enabled:
        return False
    return age_sec > max(0.0, float(max_age_sec))


@dataclass(frozen=True)
class SolutionInputSnapshot:
    target_key: str
    mmr_lat_deg: float
    mmr_lon_deg: float
    mmr_alt_m: float
    mmr_heading_deg: float
    target_lat_deg: float
    target_lon_deg: float
    target_alt_m: float


class SolutionRevisionTracker:
    """Increment a revision only when solve inputs change materially."""

    def __init__(
        self,
        *,
        position_threshold_m: float = 0.05,
        altitude_threshold_m: float = 0.05,
        heading_threshold_deg: float = 0.10,
        target_threshold_m: float = 0.01,
    ) -> None:
        self.position_threshold_m = max(0.0, float(position_threshold_m))
        self.altitude_threshold_m = max(0.0, float(altitude_threshold_m))
        self.heading_threshold_deg = max(0.0, float(heading_threshold_deg))
        self.target_threshold_m = max(0.0, float(target_threshold_m))
        self.revision = 0
        self.last_snapshot: Optional[SolutionInputSnapshot] = None

    def update(self, snapshot: SolutionInputSnapshot) -> tuple[int, bool]:
        changed = self._changed(snapshot)
        if changed:
            self.revision += 1
            self.last_snapshot = snapshot
        return self.revision, changed

    def invalidate(self) -> None:
        """Forget the previous snapshot while preserving the monotonic revision."""
        self.last_snapshot = None

    def reset(self) -> None:
        """Reset both snapshot and revision; intended for isolated tests only."""
        self.revision = 0
        self.last_snapshot = None

    def _changed(self, current: SolutionInputSnapshot) -> bool:
        previous = self.last_snapshot
        if previous is None:
            return True
        if current.target_key != previous.target_key:
            return True

        if surface_distance_m(
            previous.mmr_lat_deg,
            previous.mmr_lon_deg,
            current.mmr_lat_deg,
            current.mmr_lon_deg,
        ) > self.position_threshold_m:
            return True

        if abs(current.mmr_alt_m - previous.mmr_alt_m) > self.altitude_threshold_m:
            return True

        if angular_distance_deg(
            current.mmr_heading_deg,
            previous.mmr_heading_deg,
        ) > self.heading_threshold_deg:
            return True

        if surface_distance_m(
            previous.target_lat_deg,
            previous.target_lon_deg,
            current.target_lat_deg,
            current.target_lon_deg,
        ) > self.target_threshold_m:
            return True

        if abs(current.target_alt_m - previous.target_alt_m) > self.altitude_threshold_m:
            return True

        return False


class ConditionalRecomputeGate:
    """Decide whether an expensive ballistic solve is required.

    The underlying revision changes only when the solve inputs change beyond
    their configured thresholds.  ``force=True`` requests a fresh solve using
    the same revision, which is used when the drivetrain transitions to a
    confirmed stopped state.
    """

    def __init__(self, tracker: SolutionRevisionTracker) -> None:
        self.tracker = tracker

    def evaluate(
        self,
        snapshot: SolutionInputSnapshot,
        *,
        force: bool = False,
    ) -> tuple[int, bool, bool]:
        revision, changed = self.tracker.update(snapshot)
        return revision, changed, bool(force or changed)

    def invalidate(self) -> None:
        self.tracker.invalidate()


class DriveStopTransitionDetector:
    """Report only a moving/not-stopped -> stopped transition."""

    def __init__(self) -> None:
        self.last_stopped: Optional[bool] = None

    def update(self, stopped: bool) -> bool:
        current = bool(stopped)
        transitioned = self.last_stopped is False and current is True
        self.last_stopped = current
        return transitioned

    def reset(self) -> None:
        self.last_stopped = None
