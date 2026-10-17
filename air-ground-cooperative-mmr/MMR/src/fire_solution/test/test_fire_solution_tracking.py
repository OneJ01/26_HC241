import math

from fire_solution.fire_solution_tracking import (
    ConditionalRecomputeGate,
    DriveStopTransitionDetector,
    SolutionInputSnapshot,
    SolutionRevisionTracker,
    angular_distance_deg,
    is_target_point_stale,
    make_target_key,
    surface_distance_m,
)


def snap(**kwargs):
    data = dict(
        target_key="id:T1",
        mmr_lat_deg=37.0,
        mmr_lon_deg=127.0,
        mmr_alt_m=50.0,
        mmr_heading_deg=10.0,
        target_lat_deg=37.0001,
        target_lon_deg=127.0001,
        target_alt_m=0.0,
    )
    data.update(kwargs)
    return SolutionInputSnapshot(**data)


def test_target_key_prefers_explicit_id():
    assert make_target_key({"target_id": "ABC", "seq": 9}) == "id:ABC"


def test_target_key_falls_back_to_seq():
    assert make_target_key({"seq": 9}) == "seq:9"


def test_target_key_falls_back_to_coordinates():
    key = make_target_key({"lat_deg": 37.1, "lon_deg": 127.2, "z_m": 3.0})
    assert key.startswith("geo:")


def test_angular_distance_wraparound():
    assert math.isclose(angular_distance_deg(359.9, 0.1), 0.2, abs_tol=1e-9)


def test_surface_distance_zero():
    assert surface_distance_m(37.0, 127.0, 37.0, 127.0) == 0.0


def test_first_snapshot_creates_revision_one():
    tracker = SolutionRevisionTracker()
    assert tracker.update(snap()) == (1, True)


def test_identical_snapshot_does_not_increment():
    tracker = SolutionRevisionTracker()
    tracker.update(snap())
    assert tracker.update(snap()) == (1, False)


def test_heading_change_increments_revision():
    tracker = SolutionRevisionTracker(heading_threshold_deg=0.1)
    tracker.update(snap())
    assert tracker.update(snap(mmr_heading_deg=10.2)) == (2, True)


def test_heading_wrap_small_change_does_not_increment():
    tracker = SolutionRevisionTracker(heading_threshold_deg=0.5)
    tracker.update(snap(mmr_heading_deg=359.9))
    assert tracker.update(snap(mmr_heading_deg=0.1)) == (1, False)


def test_mmr_position_change_increments_revision():
    tracker = SolutionRevisionTracker(position_threshold_m=0.05)
    tracker.update(snap())
    assert tracker.update(snap(mmr_lat_deg=37.000001)) == (2, True)


def test_target_change_increments_revision():
    tracker = SolutionRevisionTracker()
    tracker.update(snap())
    assert tracker.update(snap(target_key="id:T2")) == (2, True)


def test_reset_restarts_revision():
    tracker = SolutionRevisionTracker()
    tracker.update(snap())
    tracker.reset()
    assert tracker.update(snap()) == (1, True)


def test_invalidate_preserves_monotonic_revision():
    tracker = SolutionRevisionTracker()
    tracker.update(snap())
    tracker.invalidate()
    assert tracker.update(snap(target_key="id:T2")) == (2, True)


def test_latched_target_never_expires_by_age():
    assert not is_target_point_stale(999999.0, latch_enabled=True, max_age_sec=1.0)


def test_non_latched_target_uses_age_limit():
    assert is_target_point_stale(1.1, latch_enabled=False, max_age_sec=1.0)
    assert not is_target_point_stale(0.9, latch_enabled=False, max_age_sec=1.0)


def test_recompute_gate_first_snapshot_solves():
    gate = ConditionalRecomputeGate(SolutionRevisionTracker())
    assert gate.evaluate(snap()) == (1, True, True)


def test_recompute_gate_identical_snapshot_uses_cache():
    gate = ConditionalRecomputeGate(SolutionRevisionTracker())
    gate.evaluate(snap())
    assert gate.evaluate(snap()) == (1, False, False)


def test_recompute_gate_force_recomputes_without_revision_increment():
    gate = ConditionalRecomputeGate(SolutionRevisionTracker())
    gate.evaluate(snap())
    assert gate.evaluate(snap(), force=True) == (1, False, True)


def test_recompute_gate_changed_input_solves_and_increments():
    gate = ConditionalRecomputeGate(SolutionRevisionTracker())
    gate.evaluate(snap())
    assert gate.evaluate(snap(mmr_heading_deg=11.0)) == (2, True, True)


def test_drive_stop_detector_only_fires_on_false_to_true():
    detector = DriveStopTransitionDetector()
    assert detector.update(True) is False
    assert detector.update(True) is False
    assert detector.update(False) is False
    assert detector.update(False) is False
    assert detector.update(True) is True
    assert detector.update(True) is False
