#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""6륜 BLDC 목표 RPM/실제 RPM/전류 기반 부하 관측기.

이 모듈은 Shadow mode 전용이다. 모터 명령이나 RS-485 패킷을 변경하지 않고,
현재 상태를 분류하여 /mmr/drive/status에 넣을 진단 정보만 반환한다.

핵심 구분:
- LOAD_PROGRESSING: 부하는 있으나 실제 회전이 유지됨
- UNLOAD_SUSPECT: 조향 중 특정 바퀴만 상대 전류가 낮고 RPM 추종은 정상
- LOCAL_STALL_SUSPECT: 일부 바퀴만 고전류·저진행률
- GLOBAL_STALL_SUSPECT: 다수 바퀴가 저진행률 상태
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import median
import time
from typing import Any


LEFT_IDS = (1, 3, 5)
RIGHT_IDS = (2, 4, 6)
ALL_IDS = LEFT_IDS + RIGHT_IDS

# 전류가 이 값보다 작으면 측면 중앙값 비율을 신뢰하지 않는다.
# 실제 하드웨어 데이터 확보 후 필요하면 파라미터화할 수 있다.
_MIN_VALID_SIDE_CURRENT_A = 0.30
_UNLOAD_TRACKING_MIN_RATIO = 0.80
_PROGRESS_FILTER_ALPHA = 0.25
_CURRENT_FILTER_ALPHA = 0.25


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(value), float(high)))


def _ewma(previous: float | None, value: float, alpha: float) -> float:
    if previous is None:
        return float(value)
    return alpha * float(value) + (1.0 - alpha) * float(previous)


@dataclass
class _TimerState:
    unload_s: float = 0.0
    local_stall_s: float = 0.0


class LoadObserver:
    """RPM·전류 기반 상태 관측기.

    update()는 상태를 계산하기만 하며 모터 목표값을 수정하지 않는다.
    """

    def __init__(
        self,
        *,
        min_target_rpm: float = 1.0,
        feedback_max_age_s: float = 0.5,
        progressing_min_ratio: float = 0.45,
        stall_max_ratio: float = 0.20,
        unload_current_ratio: float = 0.40,
        over_current_ratio: float = 1.50,
        turning_threshold: float = 0.20,
        suspect_confirm_s: float = 0.30,
        stall_confirm_s: float = 0.60,
        recovery_confirm_s: float = 0.50,
    ) -> None:
        self.min_target_rpm = max(0.1, float(min_target_rpm))
        self.feedback_max_age_s = max(0.05, float(feedback_max_age_s))
        self.progressing_min_ratio = _clamp(progressing_min_ratio, 0.05, 1.50)
        self.stall_max_ratio = _clamp(stall_max_ratio, 0.0, 0.95)
        self.unload_current_ratio = max(0.0, float(unload_current_ratio))
        self.over_current_ratio = max(1.0, float(over_current_ratio))
        self.turning_threshold = _clamp(turning_threshold, 0.0, 1.0)
        self.suspect_confirm_s = max(0.0, float(suspect_confirm_s))
        self.stall_confirm_s = max(0.0, float(stall_confirm_s))
        self.recovery_confirm_s = max(0.0, float(recovery_confirm_s))

        self._filtered_current: dict[int, float | None] = {
            mid: None for mid in ALL_IDS
        }
        self._filtered_progress: dict[int, float | None] = {
            mid: None for mid in ALL_IDS
        }
        self._timers: dict[int, _TimerState] = {
            mid: _TimerState() for mid in ALL_IDS
        }
        self._global_stall_s = 0.0
        self._last_update = time.monotonic()
        self._confirmed_state = 'NORMAL'
        self._confirmed_suspect_ids: list[int] = []
        self._recovery_started_at: float | None = None
        self._latest = self.empty_result()

    @staticmethod
    def empty_result() -> dict[str, Any]:
        return {
            'state': 'DISABLED',
            'candidate_state': 'DISABLED',
            'load_score': 0,
            'suspect_wheel_ids': [],
            'observer_confidence': 0.0,
            'turning': False,
            'active_wheel_count': 0,
            'fresh_active_wheel_count': 0,
            'progressing_wheel_count': 0,
            'stalled_wheel_count': 0,
            'side_current_median_a': {'left': None, 'right': None},
            'wheel_metrics': {mid: {} for mid in ALL_IDS},
        }

    @property
    def latest(self) -> dict[str, Any]:
        return self._latest

    @staticmethod
    def _update_duration(current: float, condition: bool, dt: float) -> float:
        if condition:
            return current + dt
        # 후보가 사라지면 빠르게 감쇠시켜 순간 샘플이 오래 남지 않게 한다.
        return max(0.0, current - 2.0 * dt)

    def update(
        self,
        *,
        targets: dict[int, float],
        feedback: dict[int, dict[str, Any]],
        steer: float,
        now: float | None = None,
    ) -> dict[str, Any]:
        now = time.monotonic() if now is None else float(now)
        dt = _clamp(now - self._last_update, 0.0, 0.30)
        self._last_update = now
        turning = abs(float(steer)) >= self.turning_threshold

        wheel_metrics: dict[int, dict[str, Any]] = {}
        active_ids: list[int] = []
        fresh_active_ids: list[int] = []

        for mid in ALL_IDS:
            target = float(targets.get(mid, 0.0) or 0.0)
            fb = feedback.get(mid, {}) or {}
            actual = float(fb.get('actual_rpm', 0.0) or 0.0)
            current_raw = fb.get('current_a')
            current = None if current_raw is None else max(0.0, float(current_raw))
            age_raw = fb.get('age_sec')
            age = None if age_raw is None else max(0.0, float(age_raw))
            fresh = (
                bool(fb.get('ok', False))
                and age is not None
                and age <= self.feedback_max_age_s
            )
            active = abs(target) >= self.min_target_rpm

            if active:
                active_ids.append(mid)
            if active and fresh:
                fresh_active_ids.append(mid)

            progress = None
            if active and fresh:
                progress = _clamp(
                    abs(actual) / max(abs(target), self.min_target_rpm),
                    0.0,
                    2.0,
                )
                self._filtered_progress[mid] = _ewma(
                    self._filtered_progress[mid],
                    progress,
                    _PROGRESS_FILTER_ALPHA,
                )

            if fresh and current is not None:
                self._filtered_current[mid] = _ewma(
                    self._filtered_current[mid],
                    current,
                    _CURRENT_FILTER_ALPHA,
                )

            wheel_metrics[mid] = {
                'observer_valid': bool(active and fresh),
                'active': bool(active),
                'fresh': bool(fresh),
                'target_rpm': target,
                'actual_rpm': actual,
                'current_a': current,
                'filtered_current_a': self._filtered_current[mid],
                'progress_ratio': progress,
                'filtered_progress_ratio': self._filtered_progress[mid],
                'current_ratio': None,
                'age_sec': age,
                'state': 'INACTIVE' if not active else ('NORMAL' if fresh else 'INVALID_FEEDBACK'),
                'candidate_state': 'NONE',
                'unload_duration_s': self._timers[mid].unload_s,
                'local_stall_duration_s': self._timers[mid].local_stall_s,
            }

        side_medians: dict[str, float | None] = {'left': None, 'right': None}
        for side_name, ids in (('left', LEFT_IDS), ('right', RIGHT_IDS)):
            values = [
                float(self._filtered_current[mid])
                for mid in ids
                if mid in fresh_active_ids
                and self._filtered_current[mid] is not None
            ]
            if values:
                side_medians[side_name] = float(median(values))

        unload_ids: list[int] = []
        local_stall_ids: list[int] = []
        progressing_ids: list[int] = []
        global_stall_candidates: list[int] = []

        for mid in ALL_IDS:
            metric = wheel_metrics[mid]
            if not metric['observer_valid']:
                self._timers[mid].unload_s = self._update_duration(
                    self._timers[mid].unload_s, False, dt
                )
                self._timers[mid].local_stall_s = self._update_duration(
                    self._timers[mid].local_stall_s, False, dt
                )
                continue

            side_name = 'left' if mid in LEFT_IDS else 'right'
            side_median = side_medians[side_name]
            filtered_current = metric['filtered_current_a']
            filtered_progress = metric['filtered_progress_ratio']

            current_ratio = None
            if (
                filtered_current is not None
                and side_median is not None
                and side_median >= _MIN_VALID_SIDE_CURRENT_A
            ):
                current_ratio = float(filtered_current) / max(side_median, 1e-6)
            metric['current_ratio'] = current_ratio

            meaningful_current = (
                filtered_current is not None
                and float(filtered_current) >= _MIN_VALID_SIDE_CURRENT_A
            )
            unload_condition = (
                turning
                and filtered_progress is not None
                and float(filtered_progress) >= _UNLOAD_TRACKING_MIN_RATIO
                and current_ratio is not None
                and current_ratio <= self.unload_current_ratio
            )
            local_stall_condition = (
                filtered_progress is not None
                and float(filtered_progress) <= self.stall_max_ratio
                and current_ratio is not None
                and current_ratio >= self.over_current_ratio
                and meaningful_current
            )

            self._timers[mid].unload_s = self._update_duration(
                self._timers[mid].unload_s,
                unload_condition,
                dt,
            )
            self._timers[mid].local_stall_s = self._update_duration(
                self._timers[mid].local_stall_s,
                local_stall_condition,
                dt,
            )

            if unload_condition:
                metric['candidate_state'] = 'UNLOAD_SUSPECT'
            if local_stall_condition:
                metric['candidate_state'] = 'LOCAL_STALL_SUSPECT'

            if self._timers[mid].unload_s >= self.suspect_confirm_s:
                unload_ids.append(mid)
                metric['state'] = 'UNLOAD_SUSPECT'
            if self._timers[mid].local_stall_s >= self.stall_confirm_s:
                local_stall_ids.append(mid)
                metric['state'] = 'LOCAL_STALL_SUSPECT'

            if (
                filtered_progress is not None
                and self.progressing_min_ratio <= float(filtered_progress) < 0.90
                and meaningful_current
            ):
                progressing_ids.append(mid)

            if (
                filtered_progress is not None
                and float(filtered_progress) <= self.stall_max_ratio
                and meaningful_current
            ):
                global_stall_candidates.append(mid)

            metric['unload_duration_s'] = self._timers[mid].unload_s
            metric['local_stall_duration_s'] = self._timers[mid].local_stall_s

        global_stall_condition = len(global_stall_candidates) >= 4
        self._global_stall_s = self._update_duration(
            self._global_stall_s,
            global_stall_condition,
            dt,
        )

        if active_ids and len(fresh_active_ids) < len(active_ids):
            raw_state = 'INVALID_FEEDBACK'
            raw_ids = sorted(set(active_ids) - set(fresh_active_ids))
            load_score = 3
        elif self._global_stall_s >= self.stall_confirm_s:
            raw_state = 'GLOBAL_STALL_SUSPECT'
            raw_ids = list(global_stall_candidates)
            load_score = 8
        elif local_stall_ids:
            raw_state = 'LOCAL_STALL_SUSPECT'
            raw_ids = list(local_stall_ids)
            load_score = 6
        elif unload_ids:
            raw_state = 'UNLOAD_SUSPECT'
            raw_ids = list(unload_ids)
            load_score = 5
        elif len(progressing_ids) >= 2:
            raw_state = 'LOAD_PROGRESSING'
            raw_ids = list(progressing_ids)
            load_score = 2
        else:
            raw_state = 'NORMAL'
            raw_ids = []
            load_score = 0

        # 비정상 상태가 사라진 뒤 recovery_confirm_s 동안 유지하여 표시가
        # 한 샘플마다 깜빡이지 않게 한다.
        if raw_state == 'NORMAL' and self._confirmed_state != 'NORMAL':
            if self._recovery_started_at is None:
                self._recovery_started_at = now
            if now - self._recovery_started_at >= self.recovery_confirm_s:
                self._confirmed_state = 'NORMAL'
                self._confirmed_suspect_ids = []
                self._recovery_started_at = None
        else:
            self._recovery_started_at = None
            if raw_state != 'NORMAL':
                self._confirmed_state = raw_state
                self._confirmed_suspect_ids = list(raw_ids)
            elif self._confirmed_state == 'NORMAL':
                self._confirmed_suspect_ids = []

        state = self._confirmed_state
        suspect_ids = list(self._confirmed_suspect_ids)
        if state == 'NORMAL':
            load_score = 0

        active_count = len(active_ids)
        fresh_active_count = len(fresh_active_ids)
        confidence = (
            float(fresh_active_count) / float(active_count)
            if active_count > 0
            else 1.0
        )

        result = {
            'state': state,
            'candidate_state': raw_state,
            'load_score': int(load_score),
            'suspect_wheel_ids': suspect_ids,
            'observer_confidence': float(confidence),
            'turning': bool(turning),
            'active_wheel_count': int(active_count),
            'fresh_active_wheel_count': int(fresh_active_count),
            'progressing_wheel_count': int(len(progressing_ids)),
            'stalled_wheel_count': int(len(global_stall_candidates)),
            'global_stall_duration_s': float(self._global_stall_s),
            'side_current_median_a': side_medians,
            'wheel_metrics': wheel_metrics,
        }
        self._latest = result
        return result
