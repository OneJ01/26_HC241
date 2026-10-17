"""Open-Meteo 제공자의 호출 안정성 회귀 테스트."""

import io
import json
import urllib.error

import pytest

from fire_solution.wind_provider import OpenMeteoWindProvider, WindSample

LAT, LON = 37.3402, 126.7336


class FakeClock:
    def __init__(self, value=1_000_000.0):
        self.value = float(value)

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += float(seconds)


def _join(provider):
    if provider._thread is not None:
        provider._thread.join(timeout=2.0)


def test_retry_backoff_recovers_without_consuming_extra_trigger_budget():
    clock = FakeClock()

    class Provider(OpenMeteoWindProvider):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.fetch_count = 0

        def _fetch(self, latitude, longitude):
            self.fetch_count += 1
            if self.fetch_count < 3:
                raise RuntimeError("temporary network failure")
            return WindSample(
                speed_ms=3.0,
                direction_from_deg=220.0,
                gust_ms=5.0,
                model_time="",
                fetch_stamp=clock(),
                latitude=latitude,
                longitude=longitude,
            ), 200

    provider = Provider(
        clock=clock,
        min_interval_sec=0.0,
        max_calls_per_engagement=3,
        max_retry_attempts=3,
        retry_initial_sec=30.0,
        retry_max_sec=120.0,
    )
    assert provider.request_refresh(LAT, LON, force=True, reason="startup") is True
    _join(provider)
    assert provider.stats()["last_result"] == "retry_scheduled"
    assert provider.budget_remaining() == 2

    clock.advance(29.0)
    assert provider.poll() is False
    clock.advance(1.0)
    assert provider.poll() is True
    _join(provider)

    clock.advance(60.0)
    assert provider.poll() is True
    _join(provider)

    assert provider.get_sample() is not None
    stats = provider.stats()
    assert stats["successful_calls_since_start"] == 1
    assert stats["failed_calls_since_start"] == 2
    assert stats["total_calls_since_start"] == 3
    assert provider.budget_remaining() == 2


def test_model_time_controls_data_staleness_even_when_just_fetched():
    clock = FakeClock(2_000_000.0)
    provider = OpenMeteoWindProvider(clock=clock)
    provider._sample = WindSample(
        speed_ms=2.0,
        direction_from_deg=90.0,
        model_time="",
        model_stamp=clock() - 7200.0,
        fetch_stamp=clock(),
    )
    assert provider.get_sample(max_age_sec=3600.0) is None
    snapshot = provider.snapshot(max_age_sec=3600.0)
    assert snapshot["valid"] is False
    assert snapshot["stale"] is True
    assert snapshot["sample"]["fetch_age_sec"] == pytest.approx(0.0)
    assert snapshot["sample"]["model_age_sec"] == pytest.approx(7200.0)


def test_http_error_preserves_status_and_api_reason():
    clock = FakeClock()
    body = json.dumps({"error": True, "reason": "Invalid variable name"}).encode()

    def failing_urlopen(request, timeout):
        raise urllib.error.HTTPError(
            request.full_url,
            400,
            "Bad Request",
            hdrs=None,
            fp=io.BytesIO(body),
        )

    provider = OpenMeteoWindProvider(
        clock=clock,
        urlopen=failing_urlopen,
        max_retry_attempts=3,
        min_interval_sec=0.0,
    )
    assert provider.request_refresh(LAT, LON, force=True, reason="test") is True
    _join(provider)
    stats = provider.stats()
    assert stats["last_http_status"] == 400
    assert "Invalid variable name" in stats["last_error"]
    assert "Invalid variable name" in stats["last_response_excerpt"]
    assert stats["retry_pending"] is False
    assert stats["queue_depth"] == 0


def test_force_request_during_inflight_is_queued_not_lost():
    clock = FakeClock()
    provider = OpenMeteoWindProvider(
        clock=clock,
        max_calls_per_engagement=3,
        min_interval_sec=30.0,
        max_retry_attempts=0,
    )
    with provider._lock:
        provider._stats.in_flight = True
        provider._stats.last_call_stamp = clock()

    assert provider.request_refresh(LAT, LON, force=True, reason="drive_stopped") is True
    stats = provider.stats()
    assert stats["queue_depth"] == 1
    assert stats["last_request_decision"] == "queued"


def test_close_discards_pending_requests_and_blocks_new_requests():
    clock = FakeClock()
    provider = OpenMeteoWindProvider(clock=clock, min_interval_sec=100.0)
    with provider._lock:
        provider._stats.last_call_stamp = clock()
    assert provider.request_refresh(LAT, LON, force=True, reason="queued") is True
    assert provider.stats()["queue_depth"] == 1
    provider.close()
    assert provider.stats()["queue_depth"] == 0
    assert provider.request_refresh(LAT, LON, force=True, reason="after_close") is False
