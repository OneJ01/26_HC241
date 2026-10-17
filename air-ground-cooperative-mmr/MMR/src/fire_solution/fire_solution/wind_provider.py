"""
wind_provider.py
─────────────────────────────────────────────────────────────────────────────
역할
- Open-Meteo Forecast API에서 현재 지상 10m 풍속/풍향/돌풍을 취득한다.
- ROS2 executor를 블로킹하지 않는 백그라운드 스레드로 HTTP 요청한다.
- 최소 호출 간격 동안 들어온 갱신 요청을 버리지 않고 큐에 보관한다.
- 일시적 네트워크 오류는 지수 백오프로 자동 재시도한다.
- API 자료 시각(model_time)과 로컬 수신 시각(fetch_stamp)을 분리한다.
- HTTP 상태 코드와 짧은 오류 응답 본문을 진단 정보로 보존한다.
- 네트워크 실패 시 마지막 정상 캐시 또는 None을 반환한다(fail-soft).

주의
- 이 모듈은 풍속 API의 수집 안정성만 담당한다.
- 풍속 보정 계산식이나 탄도 계산 로직은 포함하거나 변경하지 않는다.
- 외부 의존성 없이 Python 표준 라이브러리만 사용한다.
─────────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Deque, Dict, Optional, Tuple

OPEN_METEO_FORECAST_URL = "https://api.open-meteo.com/v1/forecast"


class WindFetchError(RuntimeError):
    """HTTP 상태와 응답 일부를 보존하는 풍속 API 오류."""

    def __init__(
        self,
        message: str,
        *,
        http_status: Optional[int] = None,
        response_excerpt: str = "",
    ) -> None:
        super().__init__(message)
        self.http_status = http_status
        self.response_excerpt = response_excerpt


@dataclass
class WindSample:
    """단일 풍속 데이터 스냅샷."""

    speed_ms: float
    direction_from_deg: float
    gust_ms: Optional[float] = None
    source: str = "open_meteo"
    model_time: str = ""
    model_stamp: Optional[float] = None
    fetch_stamp: float = 0.0
    latitude: Optional[float] = None
    longitude: Optional[float] = None

    def fetch_age_sec(self, now: Optional[float] = None) -> float:
        current = time.time() if now is None else float(now)
        if self.fetch_stamp <= 0.0:
            return float("inf")
        return max(0.0, current - self.fetch_stamp)

    def model_age_sec(self, now: Optional[float] = None) -> Optional[float]:
        if self.model_stamp is None:
            return None
        current = time.time() if now is None else float(now)
        return max(0.0, current - self.model_stamp)

    def age_sec(self, now: Optional[float] = None) -> float:
        """호환용 유효 데이터 나이. 모델 시각과 수신 시각 중 더 오래된 값을 사용."""
        fetch_age = self.fetch_age_sec(now)
        model_age = self.model_age_sec(now)
        return max(fetch_age, model_age) if model_age is not None else fetch_age

    def as_dict(self, now: Optional[float] = None) -> Dict[str, Any]:
        model_age = self.model_age_sec(now)
        return {
            "speed_ms": round(float(self.speed_ms), 3),
            "direction_from_deg": round(float(self.direction_from_deg), 1),
            "gust_ms": round(float(self.gust_ms), 3) if self.gust_ms is not None else None,
            "source": self.source,
            "model_time": self.model_time,
            "model_stamp": self.model_stamp,
            "fetch_stamp": self.fetch_stamp,
            "fetch_age_sec": round(self.fetch_age_sec(now), 2),
            "model_age_sec": round(model_age, 2) if model_age is not None else None,
            "age_sec": round(self.age_sec(now), 2),
            "latitude": self.latitude,
            "longitude": self.longitude,
        }


@dataclass
class PendingRequest:
    latitude: float
    longitude: float
    reason: str
    force: bool
    due_stamp: float
    retry_index: int = 0
    sequence_id: int = 0
    generation: int = 0


@dataclass
class ProviderStats:
    calls_used: int = 0
    calls_budget: int = 0
    total_calls: int = 0
    successful_calls: int = 0
    failed_calls: int = 0
    consecutive_failures: int = 0
    last_call_stamp: float = 0.0
    last_success_stamp: float = 0.0
    last_failure_stamp: float = 0.0
    last_error: str = ""
    last_http_status: Optional[int] = None
    last_response_excerpt: str = ""
    last_result: str = "never_called"
    last_request_decision: str = ""
    in_flight: bool = False
    active_reason: str = ""
    active_retry_index: int = 0

    def as_dict(
        self,
        *,
        queue_depth: int,
        next_action_stamp: Optional[float],
        retry_pending: bool,
        closed: bool,
    ) -> Dict[str, Any]:
        return {
            # 기존 키 유지
            "calls_used_this_engagement": self.calls_used,
            "calls_budget": self.calls_budget,
            "total_calls_since_start": self.total_calls,
            "last_call_stamp": self.last_call_stamp,
            "last_error": self.last_error,
            "last_http_status": self.last_http_status,
            "in_flight": self.in_flight,
            # 안정성 진단 키
            "successful_calls_since_start": self.successful_calls,
            "failed_calls_since_start": self.failed_calls,
            "consecutive_failures": self.consecutive_failures,
            "last_success_stamp": self.last_success_stamp,
            "last_failure_stamp": self.last_failure_stamp,
            "last_response_excerpt": self.last_response_excerpt,
            "last_result": self.last_result,
            "last_request_decision": self.last_request_decision,
            "active_reason": self.active_reason,
            "active_retry_index": self.active_retry_index,
            "queue_depth": queue_depth,
            "retry_pending": bool(retry_pending),
            "next_action_stamp": next_action_stamp,
            "closed": closed,
        }


def _parse_utc_model_time(value: str) -> Optional[float]:
    """Open-Meteo의 ISO 시각을 UTC epoch로 변환한다."""
    text = str(value or "").strip()
    if not text:
        return None
    try:
        normalized = text[:-1] + "+00:00" if text.endswith("Z") else text
        parsed = datetime.fromisoformat(normalized)
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return parsed.timestamp()
    except (TypeError, ValueError, OverflowError):
        return None


class OpenMeteoWindProvider:
    """큐, 재시도, 진단 상태를 포함한 비동기 Open-Meteo 제공자."""

    def __init__(
        self,
        *,
        ttl_sec: float = 900.0,
        max_calls_per_engagement: int = 3,
        min_interval_sec: float = 30.0,
        timeout_sec: float = 10.0,
        max_retry_attempts: int = 3,
        retry_initial_sec: float = 30.0,
        retry_max_sec: float = 120.0,
        endpoint: str = OPEN_METEO_FORECAST_URL,
        enabled: bool = True,
        on_update: Optional[Callable[[WindSample], None]] = None,
        logger: Optional[Callable[[str], None]] = None,
        clock: Optional[Callable[[], float]] = None,
        urlopen: Optional[Callable[..., Any]] = None,
    ) -> None:
        self.ttl_sec = max(0.0, float(ttl_sec))
        self.max_calls_per_engagement = max(0, int(max_calls_per_engagement))
        self.min_interval_sec = max(0.0, float(min_interval_sec))
        self.timeout_sec = max(0.1, float(timeout_sec))
        self.max_retry_attempts = max(0, int(max_retry_attempts))
        self.retry_initial_sec = max(0.0, float(retry_initial_sec))
        self.retry_max_sec = max(self.retry_initial_sec, float(retry_max_sec))
        self.endpoint = str(endpoint)
        self.enabled = bool(enabled)

        self._on_update = on_update
        self._log = logger or (lambda _msg: None)
        self._clock = clock or time.time
        self._urlopen = urlopen or urllib.request.urlopen

        self._lock = threading.Lock()
        self._sample: Optional[WindSample] = None
        self._stats = ProviderStats(calls_budget=self.max_calls_per_engagement)
        self._pending: Deque[PendingRequest] = deque()
        self._thread: Optional[threading.Thread] = None
        self._sequence_id = 0
        self._generation = 0
        self._closed = False

    # -----------------------------------------------------------------
    # 공개 조회/상태
    # -----------------------------------------------------------------

    def get_sample(self, max_age_sec: Optional[float] = None) -> Optional[WindSample]:
        """캐시된 값을 반환한다. 네트워크 접근은 하지 않는다."""
        now = self._clock()
        with self._lock:
            sample = self._sample
        if sample is None:
            return None
        if max_age_sec is not None and sample.age_sec(now) > float(max_age_sec):
            return None
        return sample

    def stats(self) -> Dict[str, Any]:
        with self._lock:
            self._stats.calls_budget = self.max_calls_per_engagement
            next_stamp = self._pending[0].due_stamp if self._pending else None
            retry_pending = any(item.retry_index > 0 for item in self._pending)
            return self._stats.as_dict(
                queue_depth=len(self._pending),
                next_action_stamp=next_stamp,
                retry_pending=retry_pending,
                closed=self._closed,
            )

    def snapshot(self, max_age_sec: Optional[float] = None) -> Dict[str, Any]:
        now = self._clock()
        with self._lock:
            sample = self._sample
            next_stamp = self._pending[0].due_stamp if self._pending else None
            retry_pending = any(item.retry_index > 0 for item in self._pending)
            stats = self._stats.as_dict(
                queue_depth=len(self._pending),
                next_action_stamp=next_stamp,
                retry_pending=retry_pending,
                closed=self._closed,
            )
        age = sample.age_sec(now) if sample is not None else None
        stale = bool(sample is not None and max_age_sec is not None and age > float(max_age_sec))
        return {
            "enabled": self.enabled and not self._closed,
            "valid": sample is not None and not stale,
            "stale": stale,
            "timestamp": now,
            "sample": sample.as_dict(now) if sample is not None else None,
            "provider": stats,
        }

    def is_cache_fresh(self, now: Optional[float] = None) -> bool:
        current = self._clock() if now is None else float(now)
        with self._lock:
            sample = self._sample
        return sample is not None and sample.age_sec(current) <= self.ttl_sec

    def reset_budget(self) -> None:
        """새 운용 사이클 시작 시 예산과 이전 대기/재시도 요청을 초기화한다."""
        with self._lock:
            self._generation += 1
            self._stats.calls_used = 0
            self._pending.clear()
            self._stats.last_request_decision = "budget_reset"

    def budget_remaining(self) -> int:
        with self._lock:
            return max(0, self.max_calls_per_engagement - self._stats.calls_used)

    def close(self) -> None:
        """새 요청/콜백을 막고 대기열을 비운다. 진행 중 HTTP 요청은 자연 종료시킨다."""
        with self._lock:
            self._closed = True
            self.enabled = False
            self._pending.clear()
            self._stats.last_request_decision = "closed"

    # -----------------------------------------------------------------
    # 요청/큐 처리
    # -----------------------------------------------------------------

    def request_refresh(
        self,
        latitude: float,
        longitude: float,
        *,
        force: bool = False,
        reason: str = "",
    ) -> bool:
        """
        갱신 요청을 접수한다.

        반환값 True는 즉시 시작 또는 대기열 접수를 뜻한다. 최소 호출 간격이나
        진행 중 요청 때문에 즉시 시작하지 못해도 요청은 버리지 않는다.
        """
        lat = float(latitude)
        lon = float(longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)):
            self._set_decision("invalid_coordinates")
            return False
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self._set_decision("invalid_coordinates")
            return False

        now = self._clock()
        thread_request: Optional[PendingRequest] = None
        with self._lock:
            if self._closed or not self.enabled:
                self._stats.last_request_decision = "disabled"
                return False
            if self._stats.calls_used >= self.max_calls_per_engagement:
                self._stats.last_request_decision = "budget_exhausted"
                return False
            if not force and self._sample is not None and self._sample.age_sec(now) <= self.ttl_sec:
                self._stats.last_request_decision = "cache_fresh"
                return False
            if self._is_duplicate_locked(lat, lon, reason):
                self._stats.last_request_decision = "duplicate_queued"
                return False

            self._sequence_id += 1
            due = max(now, self._stats.last_call_stamp + self.min_interval_sec)
            request = PendingRequest(
                latitude=lat,
                longitude=lon,
                reason=str(reason or "unspecified"),
                force=bool(force),
                due_stamp=due,
                retry_index=0,
                sequence_id=self._sequence_id,
                generation=self._generation,
            )
            self._pending.append(request)
            self._stats.calls_used += 1
            self._stats.last_request_decision = "queued" if due > now or self._stats.in_flight else "accepted"
            thread_request = self._take_due_locked(now)

        if thread_request is not None:
            self._start_worker(thread_request)
        return True

    def poll(self) -> bool:
        """ROS 타이머에서 호출한다. 기한이 된 대기/재시도 요청을 시작한다."""
        now = self._clock()
        with self._lock:
            request = self._take_due_locked(now)
        if request is None:
            return False
        self._start_worker(request)
        return True

    def _set_decision(self, decision: str) -> None:
        with self._lock:
            self._stats.last_request_decision = decision

    def _is_duplicate_locked(self, latitude: float, longitude: float, reason: str) -> bool:
        normalized_reason = str(reason or "unspecified")
        for item in self._pending:
            if (
                item.reason == normalized_reason
                and abs(item.latitude - latitude) < 1e-6
                and abs(item.longitude - longitude) < 1e-6
                and item.retry_index == 0
            ):
                return True
        return False

    def _take_due_locked(self, now: float) -> Optional[PendingRequest]:
        if self._closed or not self.enabled or self._stats.in_flight or not self._pending:
            return None
        request = self._pending[0]
        if request.due_stamp > now:
            return None
        self._pending.popleft()
        self._stats.in_flight = True
        self._stats.active_reason = request.reason
        self._stats.active_retry_index = request.retry_index
        self._stats.total_calls += 1
        self._stats.last_call_stamp = now
        self._stats.last_result = "in_flight"
        self._stats.last_request_decision = "started"
        return request

    def _start_worker(self, request: PendingRequest) -> None:
        self._thread = threading.Thread(
            target=self._worker,
            args=(request,),
            daemon=True,
            name="open_meteo_wind_fetch",
        )
        self._thread.start()

    # -----------------------------------------------------------------
    # HTTP 워커
    # -----------------------------------------------------------------

    def _worker(self, request: PendingRequest) -> None:
        sample: Optional[WindSample] = None
        error = ""
        response_excerpt = ""
        status: Optional[int] = None
        try:
            sample, status = self._fetch(request.latitude, request.longitude)
        except WindFetchError as exc:
            error = str(exc)
            status = exc.http_status
            response_excerpt = exc.response_excerpt
        except Exception as exc:  # noqa: BLE001 - 노드 생존이 우선
            error = f"{type(exc).__name__}: {exc}"

        now = self._clock()
        callback_allowed = False
        retry_scheduled = False
        retry_delay = 0.0
        with self._lock:
            self._stats.in_flight = False
            self._stats.active_reason = ""
            self._stats.active_retry_index = 0
            self._stats.last_http_status = status
            self._stats.last_response_excerpt = response_excerpt

            if self._closed:
                return

            if sample is not None:
                self._sample = sample
                self._stats.successful_calls += 1
                self._stats.consecutive_failures = 0
                self._stats.last_success_stamp = now
                self._stats.last_error = ""
                self._stats.last_result = "success"
                callback_allowed = True
            else:
                self._stats.failed_calls += 1
                self._stats.consecutive_failures += 1
                self._stats.last_failure_stamp = now
                self._stats.last_error = error or "unknown fetch error"
                self._stats.last_result = "failed"

                if (
                    request.generation == self._generation
                    and request.retry_index < self.max_retry_attempts
                    and self._is_retryable_status(status)
                ):
                    retry_delay = min(
                        self.retry_initial_sec * (2 ** request.retry_index),
                        self.retry_max_sec,
                    )
                    retry_due = max(
                        now + retry_delay,
                        self._stats.last_call_stamp + self.min_interval_sec,
                    )
                    retry = PendingRequest(
                        latitude=request.latitude,
                        longitude=request.longitude,
                        reason=request.reason,
                        force=True,
                        due_stamp=retry_due,
                        retry_index=request.retry_index + 1,
                        sequence_id=request.sequence_id,
                        generation=request.generation,
                    )
                    self._pending.appendleft(retry)
                    retry_scheduled = True
                    self._stats.last_result = "retry_scheduled"

        if sample is not None:
            self._log(
                f"[wind] Open-Meteo 취득 성공 ({request.reason}) | "
                f"{sample.speed_ms:.2f} m/s @ {sample.direction_from_deg:.0f}° from | "
                f"model_age={sample.model_age_sec(now)}"
            )
            if callback_allowed and self._on_update is not None:
                try:
                    self._on_update(sample)
                except Exception:  # noqa: BLE001
                    pass
        else:
            retry_text = (
                f" | {retry_delay:.0f}초 후 재시도 {request.retry_index + 1}/{self.max_retry_attempts}"
                if retry_scheduled
                else " | 재시도 종료"
            )
            self._log(
                f"[wind] Open-Meteo 취득 실패 ({request.reason}) | "
                f"status={status} | {error}{retry_text}"
            )

    def _fetch(self, latitude: float, longitude: float) -> Tuple[WindSample, Optional[int]]:
        params = {
            "latitude": f"{latitude:.4f}",
            "longitude": f"{longitude:.4f}",
            "current": "wind_speed_10m,wind_direction_10m,wind_gusts_10m",
            "wind_speed_unit": "ms",
            "timezone": "UTC",
        }
        url = f"{self.endpoint}?{urllib.parse.urlencode(params)}"
        req = urllib.request.Request(
            url,
            headers={
                "User-Agent": "mmr-weather-client/2.0",
                "Accept": "application/json",
                "Connection": "close",
            },
        )

        try:
            with self._urlopen(req, timeout=self.timeout_sec) as resp:
                status = getattr(resp, "status", None)
                raw = resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            body = ""
            try:
                body = exc.read(1024).decode("utf-8", errors="replace")
            except Exception:  # noqa: BLE001
                body = ""
            reason = self._extract_api_error(body) or str(exc.reason or exc)
            raise WindFetchError(
                f"HTTP {exc.code}: {reason}",
                http_status=int(exc.code),
                response_excerpt=body[:512],
            ) from exc
        except urllib.error.URLError as exc:
            raise WindFetchError(f"URL 오류: {exc.reason}") from exc
        except TimeoutError as exc:
            raise WindFetchError(f"요청 시간 초과({self.timeout_sec:.1f}s)") from exc

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise WindFetchError(
                f"JSON 파싱 실패: {exc}",
                http_status=status,
                response_excerpt=raw[:512],
            ) from exc

        if not isinstance(data, dict):
            raise WindFetchError("응답 JSON root가 object가 아님", http_status=status)
        if data.get("error") is True:
            reason = str(data.get("reason", "Open-Meteo error response"))
            raise WindFetchError(reason, http_status=status, response_excerpt=raw[:512])

        cur = data.get("current")
        if not isinstance(cur, dict):
            raise WindFetchError("응답에 'current' 블록이 없음", http_status=status)

        speed = cur.get("wind_speed_10m")
        direction = cur.get("wind_direction_10m")
        if speed is None or direction is None:
            raise WindFetchError("응답에 풍속/풍향 필드가 없음", http_status=status)

        try:
            speed_value = float(speed)
            direction_value = float(direction) % 360.0
            gust_raw = cur.get("wind_gusts_10m")
            gust_value = float(gust_raw) if gust_raw is not None else None
        except (TypeError, ValueError) as exc:
            raise WindFetchError(
                f"풍속 필드 형식 오류: {exc}",
                http_status=status,
                response_excerpt=raw[:512],
            ) from exc

        if not math.isfinite(speed_value) or speed_value < 0.0:
            raise WindFetchError("풍속 값이 유효하지 않음", http_status=status)
        if not math.isfinite(direction_value):
            raise WindFetchError("풍향 값이 유효하지 않음", http_status=status)
        if gust_value is not None and (not math.isfinite(gust_value) or gust_value < 0.0):
            raise WindFetchError("돌풍 값이 유효하지 않음", http_status=status)

        model_time = str(cur.get("time", ""))
        fetch_stamp = self._clock()
        sample = WindSample(
            speed_ms=speed_value,
            direction_from_deg=direction_value,
            gust_ms=gust_value,
            source="open_meteo",
            model_time=model_time,
            model_stamp=_parse_utc_model_time(model_time),
            fetch_stamp=fetch_stamp,
            latitude=float(data.get("latitude", latitude)),
            longitude=float(data.get("longitude", longitude)),
        )
        return sample, status


    @staticmethod
    def _is_retryable_status(status: Optional[int]) -> bool:
        """네트워크 오류/일시 서버 오류만 재시도하고 잘못된 요청 4xx는 반복하지 않는다."""
        if status is None:
            return True
        return status in (408, 425, 429) or status >= 500 or status == 200

    @staticmethod
    def _extract_api_error(body: str) -> str:
        try:
            parsed = json.loads(body)
            if isinstance(parsed, dict):
                return str(parsed.get("reason") or parsed.get("message") or "")
        except Exception:  # noqa: BLE001
            pass
        return body.strip()[:200]


class ManualWindSource:
    """speed_ms < 0이면 비활성인 수동 풍속 입력."""

    def __init__(self, speed_ms: float = -1.0, direction_from_deg: float = 0.0) -> None:
        self.speed_ms = float(speed_ms)
        self.direction_from_deg = float(direction_from_deg)

    @property
    def active(self) -> bool:
        return self.speed_ms >= 0.0

    def get_sample(self) -> Optional[WindSample]:
        if not self.active:
            return None
        now = time.time()
        return WindSample(
            speed_ms=self.speed_ms,
            direction_from_deg=self.direction_from_deg % 360.0,
            gust_ms=None,
            source="manual",
            model_time="",
            model_stamp=None,
            fetch_stamp=now,
        )
