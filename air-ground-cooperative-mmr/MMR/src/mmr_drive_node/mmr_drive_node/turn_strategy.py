#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional steering strategies for the MMR 6-wheel skid-steer drive.

The normal turn strategies in this module only change the *magnitude* of the
already-computed left/right RPM commands.  They never change motor IDs,
forward/reverse direction tables, or packet direction bytes.

The manual pivot pulse controller only decides whether the existing pivot
command is in its ON or RELIEF phase.  The actual left/right direction table is
still owned by ``bldc_drive_node.py`` and remains unchanged.

Every feature is disabled by default.  With all options disabled, the original
command path is preserved.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(float(value), high))


def _cap_pair(left: float, right: float, cap_rpm: float) -> tuple[float, float]:
    """Scale a non-negative RPM pair proportionally to an outer-wheel cap."""
    left = max(0.0, float(left))
    right = max(0.0, float(right))
    cap_rpm = max(0.0, float(cap_rpm))
    outer = max(left, right)
    if outer <= 0.0 or cap_rpm <= 0.0 or outer <= cap_rpm:
        return left, right
    scale = cap_rpm / outer
    return left * scale, right * scale


def _equal_pair(left: float, right: float, cap_rpm: float) -> tuple[float, float]:
    """Return a straight-driving pair based on the current outer-wheel RPM."""
    base = max(0.0, max(float(left), float(right)))
    if cap_rpm > 0.0:
        base = min(base, float(cap_rpm))
    return base, base


def _blend_differential(
    left: float,
    right: float,
    factor: float,
) -> tuple[float, float]:
    """Blend from equal RPM (factor=0) to the input pair (factor=1)."""
    left = max(0.0, float(left))
    right = max(0.0, float(right))
    factor = _clamp(factor, 0.0, 1.0)
    outer = max(left, right)
    if outer <= 0.0:
        return 0.0, 0.0

    if left < right:
        inner = outer - (outer - left) * factor
        return inner, outer
    if right < left:
        inner = outer - (outer - right) * factor
        return outer, inner
    return outer, outer


def _scale_speed_and_differential(
    left: float,
    right: float,
    speed_scale: float,
    differential_scale: float,
) -> tuple[float, float]:
    """Reduce total speed and left/right differential without reversing a side."""
    left = max(0.0, float(left))
    right = max(0.0, float(right))
    speed_scale = _clamp(speed_scale, 0.0, 1.0)
    differential_scale = _clamp(differential_scale, 0.0, 1.0)

    outer = max(left, right)
    inner = min(left, right)
    if outer <= 0.0:
        return 0.0, 0.0

    scaled_outer = outer * speed_scale
    scaled_diff = (outer - inner) * speed_scale * differential_scale
    scaled_inner = max(0.0, scaled_outer - scaled_diff)

    if left < right:
        return scaled_inner, scaled_outer
    if right < left:
        return scaled_outer, scaled_inner
    return scaled_outer, scaled_outer


@dataclass(frozen=True)
class TurnStrategyConfig:
    """Configuration for optional normal-driving turn strategies."""

    start_threshold: float = 0.15
    release_threshold: float = 0.08

    wide_turn_enabled: bool = False
    wide_turn_min_inner_ratio: float = 0.70

    # Stronger curvature during the turn pulse without reversing either side.
    # At full effect, inner_rpm = outer_rpm * tight_turn_min_inner_ratio.
    tight_turn_enabled: bool = False
    tight_turn_start_steer: float = 0.55
    tight_turn_full_steer: float = 0.90
    tight_turn_min_inner_ratio: float = 0.20
    tight_turn_outer_speed_cap_rpm: float = 15.0
    tight_turn_ramp_s: float = 0.15
    tight_turn_only_with_pulse: bool = True

    turn_entry_enabled: bool = False
    turn_entry_speed_cap_rpm: float = 3.0
    turn_entry_hold_s: float = 0.30
    turn_entry_ramp_s: float = 0.50

    pulse_turn_enabled: bool = False
    pulse_turn_on_s: float = 0.30
    pulse_turn_relief_s: float = 0.20
    pulse_turn_relief_speed_cap_rpm: float = 3.0

    turn_release_enabled: bool = False
    turn_release_straight_s: float = 0.40
    turn_release_speed_cap_rpm: float = 2.0
    # True이면 조향 해제 시 현재 직진 요청 속도를 유지한다. 기존처럼
    # 2~3RPM으로 강제 제한하지 않으므로 운전자가 전진 스틱을 유지한 상태에서
    # 로봇이 잠시 정지하는 듯한 현상을 없앤다.
    turn_release_keep_drive_speed: bool = False

    counter_steer_enabled: bool = False
    counter_steer_ratio: float = 0.05
    counter_steer_duration_s: float = 0.15
    counter_steer_speed_cap_rpm: float = 1.5
    counter_steer_forward_only: bool = True

    load_adaptive_turn_enabled: bool = False
    load_turn_unload_diff_scale: float = 0.70
    load_turn_unload_speed_scale: float = 0.85
    load_turn_local_stall_diff_scale: float = 0.50
    load_turn_local_stall_speed_scale: float = 0.70

    def normalized(self) -> "TurnStrategyConfig":
        start = _clamp(self.start_threshold, 0.01, 1.0)
        release = _clamp(self.release_threshold, 0.0, start)
        tight_start = _clamp(self.tight_turn_start_steer, start, 1.0)
        tight_full = _clamp(self.tight_turn_full_steer, tight_start, 1.0)
        return TurnStrategyConfig(
            start_threshold=start,
            release_threshold=release,
            wide_turn_enabled=bool(self.wide_turn_enabled),
            wide_turn_min_inner_ratio=_clamp(
                self.wide_turn_min_inner_ratio, 0.0, 1.0
            ),
            tight_turn_enabled=bool(self.tight_turn_enabled),
            tight_turn_start_steer=tight_start,
            tight_turn_full_steer=tight_full,
            tight_turn_min_inner_ratio=_clamp(
                self.tight_turn_min_inner_ratio, 0.0, 1.0
            ),
            tight_turn_outer_speed_cap_rpm=max(
                0.0, self.tight_turn_outer_speed_cap_rpm
            ),
            tight_turn_ramp_s=max(0.0, self.tight_turn_ramp_s),
            tight_turn_only_with_pulse=bool(
                self.tight_turn_only_with_pulse
            ),
            turn_entry_enabled=bool(self.turn_entry_enabled),
            turn_entry_speed_cap_rpm=max(0.0, self.turn_entry_speed_cap_rpm),
            turn_entry_hold_s=max(0.0, self.turn_entry_hold_s),
            turn_entry_ramp_s=max(0.0, self.turn_entry_ramp_s),
            pulse_turn_enabled=bool(self.pulse_turn_enabled),
            pulse_turn_on_s=max(0.05, self.pulse_turn_on_s),
            pulse_turn_relief_s=max(0.05, self.pulse_turn_relief_s),
            pulse_turn_relief_speed_cap_rpm=max(
                0.0, self.pulse_turn_relief_speed_cap_rpm
            ),
            turn_release_enabled=bool(self.turn_release_enabled),
            turn_release_straight_s=max(0.0, self.turn_release_straight_s),
            turn_release_speed_cap_rpm=max(
                0.0, self.turn_release_speed_cap_rpm
            ),
            turn_release_keep_drive_speed=bool(
                self.turn_release_keep_drive_speed
            ),
            counter_steer_enabled=bool(self.counter_steer_enabled),
            counter_steer_ratio=_clamp(self.counter_steer_ratio, 0.0, 0.5),
            counter_steer_duration_s=max(0.0, self.counter_steer_duration_s),
            counter_steer_speed_cap_rpm=max(
                0.0, self.counter_steer_speed_cap_rpm
            ),
            counter_steer_forward_only=bool(self.counter_steer_forward_only),
            load_adaptive_turn_enabled=bool(
                self.load_adaptive_turn_enabled
            ),
            load_turn_unload_diff_scale=_clamp(
                self.load_turn_unload_diff_scale, 0.0, 1.0
            ),
            load_turn_unload_speed_scale=_clamp(
                self.load_turn_unload_speed_scale, 0.0, 1.0
            ),
            load_turn_local_stall_diff_scale=_clamp(
                self.load_turn_local_stall_diff_scale, 0.0, 1.0
            ),
            load_turn_local_stall_speed_scale=_clamp(
                self.load_turn_local_stall_speed_scale, 0.0, 1.0
            ),
        )


class TurnStrategyController:
    """Stateful optional normal-driving turn strategy controller."""

    def __init__(self, config: TurnStrategyConfig):
        self.config = config.normalized()
        self.reset(0.0)

    @property
    def enabled_features(self) -> list[str]:
        c = self.config
        features: list[str] = []
        if c.wide_turn_enabled:
            features.append('WIDE_TURN')
        if c.tight_turn_enabled:
            features.append('TIGHT_TURN')
        if c.turn_entry_enabled:
            features.append('TURN_ENTRY')
        if c.pulse_turn_enabled:
            features.append('PULSE_TURN')
        if c.turn_release_enabled:
            features.append('TURN_RELEASE')
        if c.counter_steer_enabled:
            features.append('COUNTER_STEER')
        if c.load_adaptive_turn_enabled:
            features.append('LOAD_ADAPTIVE')
        return features

    @property
    def enabled(self) -> bool:
        return bool(self.enabled_features)

    def reset(self, now: float = 0.0) -> None:
        self._turning_latched = False
        self._entry_started_at: float | None = None
        self._pulse_started_at: float | None = None
        self._release_phase = 'IDLE'
        self._release_started_at: float | None = None
        self._last_turn_sign = 0
        self._last_state = (
            'DISABLED' if not getattr(self, 'config', None) else 'IDLE'
        )
        self._latest_debug: dict[str, Any] = self._empty_debug()
        self._latest_debug['timestamp'] = float(now)

    @property
    def latest_debug(self) -> dict[str, Any]:
        return dict(self._latest_debug)

    def _empty_debug(self) -> dict[str, Any]:
        features = self.enabled_features if hasattr(self, 'config') else []
        return {
            'enabled': bool(features),
            'features': features,
            'state': 'IDLE' if features else 'DISABLED',
            'state_changed': False,
            'input_steer': 0.0,
            'turning': False,
            'last_turn_sign': 0,
            'entry_progress': 0.0,
            'pulse_phase': 'IDLE',
            'release_phase': 'IDLE',
            'counter_steer_active': False,
            'release_speed_preserved': False,
            'tight_turn_active': False,
            'tight_turn_progress': 0.0,
            'tight_turn_inner_ratio': 1.0,
            'load_overlay': 'NONE',
            'speed_scale': 1.0,
            'differential_scale': 1.0,
            'input_left_rpm': 0.0,
            'input_right_rpm': 0.0,
            'output_left_rpm': 0.0,
            'output_right_rpm': 0.0,
            'timestamp': 0.0,
        }

    def _turning_with_hysteresis(self, steer: float) -> bool:
        magnitude = abs(float(steer))
        if self._turning_latched:
            return magnitude > self.config.release_threshold
        return magnitude >= self.config.start_threshold

    def _apply_wide_turn(
        self,
        left: float,
        right: float,
    ) -> tuple[float, float, bool]:
        outer = max(left, right)
        inner = min(left, right)
        if outer <= 0.0 or inner == outer:
            return left, right, False

        min_inner = outer * self.config.wide_turn_min_inner_ratio
        if inner >= min_inner:
            return left, right, False

        if left < right:
            return min_inner, right, True
        return left, min_inner, True

    def _apply_tight_turn(
        self,
        left: float,
        right: float,
        steer: float,
        now: float,
    ) -> tuple[float, float, bool, float, float]:
        """Increase curvature by reducing the inner side during a pulse.

        Both outputs remain non-negative, so this function never commands an
        inner-wheel reversal.  The feature is therefore tighter than the
        original 50% reduction but less aggressive than a true pivot blend.
        """
        c = self.config
        left = max(0.0, float(left))
        right = max(0.0, float(right))
        outer = max(left, right)
        inner = min(left, right)
        if outer <= 0.0 or left == right:
            return left, right, False, 0.0, 1.0

        steer_abs = abs(float(steer))
        if steer_abs < c.tight_turn_start_steer:
            return left, right, False, 0.0, inner / outer

        if c.tight_turn_full_steer <= c.tight_turn_start_steer:
            steer_progress = 1.0
        else:
            steer_progress = _clamp(
                (steer_abs - c.tight_turn_start_steer)
                / (c.tight_turn_full_steer - c.tight_turn_start_steer),
                0.0,
                1.0,
            )

        time_progress = 1.0
        if c.tight_turn_ramp_s > 0.0 and self._entry_started_at is not None:
            time_progress = _clamp(
                (now - self._entry_started_at) / c.tight_turn_ramp_s,
                0.0,
                1.0,
            )

        progress = min(steer_progress, time_progress)
        current_ratio = _clamp(inner / outer, 0.0, 1.0)
        target_ratio = current_ratio + (
            c.tight_turn_min_inner_ratio - current_ratio
        ) * progress
        # This strategy only tightens.  It must never raise the inner speed.
        target_ratio = min(current_ratio, target_ratio)

        capped_left, capped_right = _cap_pair(
            left,
            right,
            c.tight_turn_outer_speed_cap_rpm,
        )
        capped_outer = max(capped_left, capped_right)
        target_inner = capped_outer * target_ratio

        if left < right:
            return target_inner, capped_outer, True, progress, target_ratio
        return capped_outer, target_inner, True, progress, target_ratio

    def _counter_pair(
        self,
        base_left: float,
        base_right: float,
        last_turn_sign: int,
        preserve_drive_speed: bool = False,
    ) -> tuple[float, float]:
        # 기존 구현은 counter_steer_speed_cap_rpm(통상 1.5~2RPM)으로
        # 강제 감속했기 때문에 전진 입력이 유지되어도 정지처럼 느껴질 수 있었다.
        # preserve_drive_speed=True이면 현재 직진 요청 속도를 그대로 사용하고
        # 좌우 차이만 counter_steer_ratio만큼 아주 작게 만든다.
        counter_cap = (
            0.0
            if preserve_drive_speed
            else self.config.counter_steer_speed_cap_rpm
        )
        base_left, base_right = _equal_pair(
            base_left,
            base_right,
            counter_cap,
        )
        base = max(base_left, base_right)
        inner = base * (1.0 - self.config.counter_steer_ratio)

        # Previous left turn (+) -> short right turn: right side becomes inner.
        if last_turn_sign > 0:
            return base, inner
        # Previous right turn (-) -> short left turn: left side becomes inner.
        if last_turn_sign < 0:
            return inner, base
        return base, base

    def apply(
        self,
        left_rpm: float,
        right_rpm: float,
        steer: float,
        forward: bool,
        load_state: str = 'DISABLED',
        now: float = 0.0,
    ) -> tuple[float, float, dict[str, Any]]:
        """Apply enabled strategies and return modified RPM magnitudes + debug."""
        left_in = max(0.0, float(left_rpm))
        right_in = max(0.0, float(right_rpm))
        steer = _clamp(steer, -1.0, 1.0)
        now = float(now)

        if not self.enabled:
            debug = self._empty_debug()
            debug.update({
                'input_steer': steer,
                'input_left_rpm': left_in,
                'input_right_rpm': right_in,
                'output_left_rpm': left_in,
                'output_right_rpm': right_in,
                'timestamp': now,
            })
            self._latest_debug = debug
            self._last_state = 'DISABLED'
            return left_in, right_in, debug

        c = self.config
        turning = self._turning_with_hysteresis(steer)
        rising_edge = turning and not self._turning_latched
        falling_edge = (not turning) and self._turning_latched

        if rising_edge:
            self._entry_started_at = now
            self._pulse_started_at = None
            self._release_phase = 'IDLE'
            self._release_started_at = None

        if turning:
            self._last_turn_sign = 1 if steer > 0.0 else -1
        elif falling_edge:
            if c.turn_release_enabled and c.turn_release_straight_s > 0.0:
                self._release_phase = 'STRAIGHT'
                self._release_started_at = now
            elif (
                c.counter_steer_enabled
                and c.counter_steer_duration_s > 0.0
                and (forward or not c.counter_steer_forward_only)
            ):
                self._release_phase = 'COUNTER'
                self._release_started_at = now
            else:
                self._release_phase = 'IDLE'
                self._release_started_at = None
            self._entry_started_at = None
            self._pulse_started_at = None

        if turning and self._release_phase != 'IDLE':
            self._release_phase = 'IDLE'
            self._release_started_at = None

        self._turning_latched = turning

        left = left_in
        right = right_in
        state = 'TURN_ACTIVE' if turning else 'IDLE'
        pulse_phase = 'IDLE'
        entry_progress = 1.0 if turning else 0.0
        counter_active = False
        release_speed_preserved = False
        tight_applied = False
        tight_progress = 0.0
        tight_inner_ratio = 1.0
        load_overlay = 'NONE'
        speed_scale = 1.0
        differential_scale = 1.0

        # 1) Wide-turn clamp.  If tight-turn is also enabled, tight-turn wins.
        wide_applied = False
        if turning and c.wide_turn_enabled and not c.tight_turn_enabled:
            left, right, wide_applied = self._apply_wide_turn(left, right)
            if wide_applied:
                state = 'WIDE_TURN'

        # 2) Turn-entry sequence.
        entry_active = False
        if turning and c.turn_entry_enabled:
            if self._entry_started_at is None:
                self._entry_started_at = now
            elapsed = max(0.0, now - self._entry_started_at)
            target_left, target_right = _cap_pair(
                left,
                right,
                c.turn_entry_speed_cap_rpm,
            )

            if elapsed < c.turn_entry_hold_s:
                left, right = _equal_pair(
                    target_left,
                    target_right,
                    c.turn_entry_speed_cap_rpm,
                )
                state = 'TURN_PREP'
                entry_progress = 0.0
                entry_active = True
            elif c.turn_entry_ramp_s > 0.0 and elapsed < (
                c.turn_entry_hold_s + c.turn_entry_ramp_s
            ):
                entry_progress = _clamp(
                    (elapsed - c.turn_entry_hold_s) / c.turn_entry_ramp_s,
                    0.0,
                    1.0,
                )
                left, right = _blend_differential(
                    target_left,
                    target_right,
                    entry_progress,
                )
                state = 'TURN_RAMP'
                entry_active = True
            else:
                left, right = target_left, target_right
                entry_progress = 1.0

        # 3) Pulse phase selection.
        if turning and c.pulse_turn_enabled and not entry_active:
            if self._pulse_started_at is None:
                self._pulse_started_at = now
            cycle = c.pulse_turn_on_s + c.pulse_turn_relief_s
            phase_elapsed = (now - self._pulse_started_at) % cycle
            if phase_elapsed < c.pulse_turn_on_s:
                pulse_phase = 'TURN'
                state = 'TURN_PULSE'
            else:
                pulse_phase = 'RELIEF'
                left, right = _equal_pair(
                    left,
                    right,
                    c.pulse_turn_relief_speed_cap_rpm,
                )
                state = 'STRAIGHT_RELIEF'
        elif not turning:
            self._pulse_started_at = None

        # 4) Tight-turn is applied only to the active turn portion by default.
        tight_allowed = (
            turning
            and c.tight_turn_enabled
            and not entry_active
            and (
                not c.tight_turn_only_with_pulse
                or (c.pulse_turn_enabled and pulse_phase == 'TURN')
            )
        )
        if tight_allowed:
            (
                left,
                right,
                tight_applied,
                tight_progress,
                tight_inner_ratio,
            ) = self._apply_tight_turn(left, right, steer, now)
            if tight_applied:
                state = 'TIGHT_TURN_PULSE' if pulse_phase == 'TURN' else 'TIGHT_TURN'

        # 5) Release straight and optional short counter-steer pulse.
        if not turning and self._release_phase != 'IDLE':
            phase_elapsed = max(
                0.0,
                now - (self._release_started_at or now),
            )

            if self._release_phase == 'STRAIGHT':
                if phase_elapsed < c.turn_release_straight_s:
                    release_cap = (
                        0.0
                        if c.turn_release_keep_drive_speed
                        else c.turn_release_speed_cap_rpm
                    )
                    left, right = _equal_pair(
                        left,
                        right,
                        release_cap,
                    )
                    release_speed_preserved = bool(
                        c.turn_release_keep_drive_speed
                    )
                    state = (
                        'RELEASE_STRAIGHT_KEEP_SPEED'
                        if release_speed_preserved
                        else 'RELEASE_STRAIGHT'
                    )
                else:
                    can_counter = (
                        c.counter_steer_enabled
                        and c.counter_steer_duration_s > 0.0
                        and (forward or not c.counter_steer_forward_only)
                    )
                    if can_counter:
                        self._release_phase = 'COUNTER'
                        self._release_started_at = now
                        phase_elapsed = 0.0
                    else:
                        self._release_phase = 'IDLE'
                        self._release_started_at = None

            if self._release_phase == 'COUNTER':
                phase_elapsed = max(
                    0.0,
                    now - (self._release_started_at or now),
                )
                if phase_elapsed < c.counter_steer_duration_s:
                    left, right = self._counter_pair(
                        left,
                        right,
                        self._last_turn_sign,
                        preserve_drive_speed=(
                            c.turn_release_keep_drive_speed
                        ),
                    )
                    release_speed_preserved = bool(
                        c.turn_release_keep_drive_speed
                    )
                    state = (
                        'COUNTER_STEER_KEEP_SPEED'
                        if release_speed_preserved
                        else 'COUNTER_STEER'
                    )
                    counter_active = True
                else:
                    self._release_phase = 'IDLE'
                    self._release_started_at = None
                    state = 'IDLE'

        # 6) Load-adaptive overlay only while actively turning.
        if turning and c.load_adaptive_turn_enabled:
            load_state = str(load_state or '').upper()
            if load_state == 'UNLOAD_SUSPECT':
                speed_scale = c.load_turn_unload_speed_scale
                differential_scale = c.load_turn_unload_diff_scale
                load_overlay = 'UNLOAD_REDUCTION'
            elif load_state == 'LOCAL_STALL_SUSPECT':
                speed_scale = c.load_turn_local_stall_speed_scale
                differential_scale = c.load_turn_local_stall_diff_scale
                load_overlay = 'LOCAL_STALL_REDUCTION'

            if load_overlay != 'NONE':
                left, right = _scale_speed_and_differential(
                    left,
                    right,
                    speed_scale,
                    differential_scale,
                )

        left = max(0.0, left)
        right = max(0.0, right)

        state_changed = state != self._last_state
        self._last_state = state
        debug = {
            'enabled': True,
            'features': self.enabled_features,
            'state': state,
            'state_changed': state_changed,
            'input_steer': steer,
            'turning': bool(turning),
            'last_turn_sign': int(self._last_turn_sign),
            'entry_progress': float(entry_progress),
            'pulse_phase': pulse_phase,
            'release_phase': self._release_phase,
            'counter_steer_active': bool(counter_active),
            'release_speed_preserved': bool(release_speed_preserved),
            'tight_turn_active': bool(tight_applied),
            'tight_turn_progress': float(tight_progress),
            'tight_turn_inner_ratio': float(tight_inner_ratio),
            'load_overlay': load_overlay,
            'speed_scale': float(speed_scale),
            'differential_scale': float(differential_scale),
            'input_left_rpm': float(left_in),
            'input_right_rpm': float(right_in),
            'output_left_rpm': float(left),
            'output_right_rpm': float(right),
            'wide_turn_applied': bool(wide_applied),
            'timestamp': now,
        }
        self._latest_debug = debug
        return left, right, debug


@dataclass(frozen=True)
class PivotPulseConfig:
    """Configuration for LB/RB manual pivot pulsing."""

    enabled: bool = False
    rpm: float = 4.0
    on_s: float = 0.20
    relief_s: float = 0.20

    def normalized(self) -> "PivotPulseConfig":
        return PivotPulseConfig(
            enabled=bool(self.enabled),
            rpm=max(0.0, float(self.rpm)),
            on_s=max(0.05, float(self.on_s)),
            relief_s=max(0.05, float(self.relief_s)),
        )


class PivotPulseController:
    """Pure state machine for manual pivot ON/RELIEF timing."""

    def __init__(self, config: PivotPulseConfig):
        self.config = config.normalized()
        self.reset(0.0)

    def reset(self, now: float = 0.0) -> None:
        self._turn_sign = 0
        self._cycle_started_at = float(now)
        self._last_phase = 'IDLE'
        self._latest_debug = {
            'enabled': bool(getattr(self, 'config', None) and self.config.enabled),
            'phase': 'IDLE',
            'state_changed': False,
            'turn_sign': 0,
            'command_rpm': 0.0,
            'timestamp': float(now),
        }

    @property
    def latest_debug(self) -> dict[str, Any]:
        return dict(self._latest_debug)

    def apply(self, turn_sign: int, now: float) -> tuple[float, dict[str, Any]]:
        """Return pivot RPM for the current phase.

        ``turn_sign``: +1 for the existing left-pivot direction table, -1 for
        the existing right-pivot direction table, and 0 for no pivot request.
        """
        turn_sign = 1 if turn_sign > 0 else -1 if turn_sign < 0 else 0
        now = float(now)

        if turn_sign == 0:
            self.reset(now)
            return 0.0, self.latest_debug

        if not self.config.enabled:
            phase = 'CONTINUOUS'
            rpm = self.config.rpm
        else:
            if turn_sign != self._turn_sign:
                self._turn_sign = turn_sign
                self._cycle_started_at = now
            cycle = self.config.on_s + self.config.relief_s
            phase_elapsed = (now - self._cycle_started_at) % cycle
            if phase_elapsed < self.config.on_s:
                phase = 'PIVOT'
                rpm = self.config.rpm
            else:
                phase = 'RELIEF'
                rpm = 0.0

        state_changed = phase != self._last_phase
        self._last_phase = phase
        self._turn_sign = turn_sign
        self._latest_debug = {
            'enabled': bool(self.config.enabled),
            'phase': phase,
            'state_changed': bool(state_changed),
            'turn_sign': int(turn_sign),
            'command_rpm': float(rpm),
            'timestamp': now,
        }
        return rpm, self.latest_debug
