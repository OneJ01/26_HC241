#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MMR 구동부 디버깅 GUI

/mmr/drive/status(std_msgs/String, JSON)을 구독하여 다음을 표시한다.
- 좌/우 대표 목표 RPM
- 6개 바퀴의 실제 RPM을 대형 카드 형태로 표시
- 바퀴별 RPM 오차, 전류, 피드백 상태, age, timeout

실행:
  ros2 run mmr_gui drive_debug_gui
"""

import json
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


LEFT_WHEELS = ['FL', 'ML', 'RL']
RIGHT_WHEELS = ['FR', 'MR', 'RR']
WHEEL_ORDER = LEFT_WHEELS + RIGHT_WHEELS
TARGET_MATCH_TOLERANCE_RPM = 0.05


class DriveStatusSubscriber(Node):
    """구동 상태 JSON 토픽을 수신해 마지막 payload를 저장한다."""

    def __init__(self) -> None:
        super().__init__('drive_debug_gui_node')
        self.declare_parameter('drive_status_topic', '/mmr/drive/status')
        topic = str(self.get_parameter('drive_status_topic').value)

        self.latest_payload: dict[str, Any] | None = None
        self.last_rx_wall_time = 0.0
        self.parse_error_count = 0

        self._sub = self.create_subscription(
            String,
            topic,
            self._status_callback,
            10,
        )
        self.get_logger().info(f'구동 상태 구독 시작: {topic}')

    def _status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('payload is not a JSON object')
        except Exception as exc:
            self.parse_error_count += 1
            self.get_logger().warn(
                f'/mmr/drive/status JSON 파싱 실패: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        self.latest_payload = payload
        self.last_rx_wall_time = time.monotonic()


class DriveDebugWindow(QMainWindow):
    """6륜 BLDC 상태 확인용 단독 디버깅 창."""

    def __init__(self, ros_node: DriveStatusSubscriber) -> None:
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('MMR Drive Debug GUI')
        self.resize(1320, 920)
        self.setMinimumSize(1080, 760)

        self._summary_labels: dict[str, QLabel] = {}
        self._target_value_labels: dict[str, QLabel] = {}
        self._target_state_labels: dict[str, QLabel] = {}
        self._wheel_labels: dict[str, dict[str, QLabel]] = {}

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(14, 12, 14, 12)
        root_layout.setSpacing(10)

        header_layout = QHBoxLayout()
        title = QLabel('MMR Drive Debug')
        title.setFont(QFont('Arial', 18, QFont.Weight.Bold))
        header_layout.addWidget(title)
        header_layout.addStretch(1)

        self._connection_label = QLabel('NO DATA')
        self._connection_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._connection_label.setMinimumWidth(150)
        self._connection_label.setStyleSheet(
            'QLabel { background: #6b7280; color: white; border-radius: 6px; '
            'padding: 7px 12px; font-weight: bold; }'
        )
        header_layout.addWidget(self._connection_label)
        root_layout.addLayout(header_layout)

        root_layout.addWidget(self._make_target_summary_box())
        root_layout.addWidget(self._make_status_strip())
        root_layout.addWidget(self._make_wheel_area(), stretch=1)

        hint = QLabel(
            '표시 기준: Target/Actual RPM은 로봇 전진 방향을 +로 통일합니다. '
            '좌·우 목표 RPM은 각 측 3개 모터의 대표값이며, 서로 다르면 MISMATCH로 표시합니다. '
            'LOAD는 Shadow 진단이고, Turn Strategy는 launch에서 명시적으로 켠 시험 기능만 표시합니다.'
        )
        hint.setWordWrap(True)
        hint.setStyleSheet('color: #4b5563;')
        root_layout.addWidget(hint)

        self.setCentralWidget(root)

        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(20)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(100)

    def _make_target_summary_box(self) -> QGroupBox:
        box = QGroupBox('Commanded Target RPM')
        layout = QHBoxLayout(box)
        layout.setSpacing(14)
        layout.addWidget(self._make_target_panel('left', 'LEFT TARGET', 'ID 1 · 3 · 5'))
        layout.addWidget(self._make_target_panel('right', 'RIGHT TARGET', 'ID 2 · 4 · 6'))
        return box

    def _make_target_panel(self, side: str, title: str, ids_text: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        frame.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        frame.setMinimumHeight(120)
        frame.setStyleSheet(
            'QFrame { background: #f3f4f6; border: 1px solid #d1d5db; '
            'border-radius: 8px; }'
        )

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(16, 10, 16, 10)
        layout.setSpacing(2)

        header = QLabel(f'{title}   ({ids_text})')
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header.setFont(QFont('Arial', 11, QFont.Weight.Bold))
        header.setStyleSheet('border: none; color: #374151;')
        layout.addWidget(header)

        value = QLabel('-')
        value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value.setFont(QFont('Arial', 30, QFont.Weight.Bold))
        value.setStyleSheet('border: none; color: #111827;')
        layout.addWidget(value, stretch=1)

        state = QLabel('WAITING')
        state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        state.setFont(QFont('Arial', 9, QFont.Weight.Bold))
        state.setStyleSheet('border: none; color: #6b7280;')
        layout.addWidget(state)

        self._target_value_labels[side] = value
        self._target_state_labels[side] = state
        return frame

    def _make_status_strip(self) -> QGroupBox:
        box = QGroupBox('Drive Status')
        layout = QGridLayout(box)
        layout.setHorizontalSpacing(14)
        layout.setVerticalSpacing(5)

        fields = [
            ('mode', 'Mode'),
            ('turn_strategy_features', 'Turn Features'),
            ('turn_strategy_state', 'Turn State'),
            ('turn_strategy_input_steer', 'Input Steer'),
            ('turn_strategy_pulse_phase', 'Pulse Phase'),
            ('turn_strategy_release_phase', 'Release Phase'),
            ('turn_strategy_counter_active', 'Counter'),
            ('turn_strategy_load_overlay', 'Load Overlay'),
            ('turn_strategy_speed_scale', 'Speed Scale'),
            ('turn_strategy_differential_scale', 'Diff Scale'),
            ('load_observer_mode', 'Observer'),
            ('load_state', 'Load State'),
            ('load_candidate_state', 'Candidate'),
            ('suspect_wheel_ids', 'Suspect IDs'),
            ('observer_confidence', 'Confidence'),
            ('feedback_all_fresh', 'Feedback Fresh'),
            ('all_stopped', 'Stopped'),
            ('left_avg_rpm', 'Left Avg'),
            ('right_avg_rpm', 'Right Avg'),
            ('status_age_sec', 'Status Age'),
            ('parse_errors', 'Parse Errors'),
        ]

        for index, (key, display_name) in enumerate(fields):
            row = index // 4
            col = (index % 4) * 2
            name = QLabel(display_name)
            name.setFont(QFont('Arial', 9, QFont.Weight.Bold))
            value = QLabel('-')
            value.setFont(QFont('Arial', 10))
            value.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextSelectableByMouse
            )
            layout.addWidget(name, row, col)
            layout.addWidget(value, row, col + 1)
            self._summary_labels[key] = value

        return box

    def _make_wheel_area(self) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self._make_side_group('LEFT WHEELS', LEFT_WHEELS))
        layout.addWidget(self._make_side_group('RIGHT WHEELS', RIGHT_WHEELS))
        return container

    def _make_side_group(self, title: str, wheels: list[str]) -> QGroupBox:
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        layout.setSpacing(8)
        for wheel in wheels:
            layout.addWidget(self._make_wheel_card(wheel))
        return group

    def _make_wheel_card(self, wheel: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        frame.setMinimumHeight(150)
        frame.setStyleSheet(
            'QFrame { background: white; border: 1px solid #cbd5e1; '
            'border-radius: 8px; }'
        )

        outer = QHBoxLayout(frame)
        outer.setContentsMargins(12, 8, 12, 8)
        outer.setSpacing(14)

        identity_layout = QVBoxLayout()
        wheel_label = QLabel(wheel)
        wheel_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        wheel_label.setFont(QFont('Arial', 18, QFont.Weight.Bold))
        wheel_label.setMinimumWidth(70)
        wheel_label.setStyleSheet('border: none; color: #111827;')
        identity_layout.addWidget(wheel_label)

        id_label = QLabel('ID -')
        id_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        id_label.setFont(QFont('Arial', 10, QFont.Weight.Bold))
        id_label.setStyleSheet('border: none; color: #6b7280;')
        identity_layout.addWidget(id_label)
        outer.addLayout(identity_layout)

        rpm_layout = QVBoxLayout()
        rpm_caption = QLabel('ACTUAL RPM')
        rpm_caption.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rpm_caption.setFont(QFont('Arial', 9, QFont.Weight.Bold))
        rpm_caption.setStyleSheet('border: none; color: #6b7280;')
        rpm_layout.addWidget(rpm_caption)

        rpm_value = QLabel('-')
        rpm_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        rpm_value.setFont(QFont('Arial', 28, QFont.Weight.Bold))
        rpm_value.setMinimumWidth(175)
        rpm_value.setStyleSheet('border: none; color: #111827;')
        rpm_layout.addWidget(rpm_value, stretch=1)
        outer.addLayout(rpm_layout, stretch=1)

        detail_layout = QGridLayout()
        detail_layout.setHorizontalSpacing(12)
        detail_layout.setVerticalSpacing(4)

        error_label = QLabel('Error: - RPM')
        current_label = QLabel('Current: - A')
        progress_label = QLabel('Progress: -')
        ratio_label = QLabel('Current ratio: -')
        age_label = QLabel('Age: - s')
        timeout_label = QLabel('Timeout: -')
        load_label = QLabel('LOAD: -')
        ok_label = QLabel('NO DATA')
        ok_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ok_label.setMinimumWidth(88)

        for detail in (
            error_label, current_label, progress_label, ratio_label,
            age_label, timeout_label, load_label,
        ):
            detail.setFont(QFont('Arial', 9))
            detail.setStyleSheet('border: none; color: #374151;')

        detail_layout.addWidget(error_label, 0, 0)
        detail_layout.addWidget(current_label, 0, 1)
        detail_layout.addWidget(progress_label, 1, 0)
        detail_layout.addWidget(ratio_label, 1, 1)
        detail_layout.addWidget(age_label, 2, 0)
        detail_layout.addWidget(timeout_label, 2, 1)
        detail_layout.addWidget(load_label, 3, 0, 1, 2)
        detail_layout.addWidget(ok_label, 4, 0, 1, 2)
        outer.addLayout(detail_layout)

        self._wheel_labels[wheel] = {
            'id': id_label,
            'actual_rpm': rpm_value,
            'error_rpm': error_label,
            'current_a': current_label,
            'progress_ratio': progress_label,
            'current_ratio': ratio_label,
            'age_sec': age_label,
            'timeout_count': timeout_label,
            'load_state': load_label,
            'ok': ok_label,
        }
        return frame

    def _spin_ros_once(self) -> None:
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def _refresh_ui(self) -> None:
        payload = self.ros_node.latest_payload or {}
        now = time.monotonic()

        status_age = None
        if self.ros_node.last_rx_wall_time > 0.0:
            status_age = now - self.ros_node.last_rx_wall_time

        self._update_connection_state(status_age)

        suspect_ids = payload.get('suspect_wheel_ids', [])
        if not isinstance(suspect_ids, list):
            suspect_ids = []
        turn_features = payload.get('turn_strategy_features', [])
        if not isinstance(turn_features, list):
            turn_features = []

        summary = {
            'mode': payload.get('mode', '-'),
            'turn_strategy_features': (
                ', '.join(str(v) for v in turn_features)
                if turn_features else 'OFF'
            ),
            'turn_strategy_state': payload.get('turn_strategy_state', '-'),
            'turn_strategy_input_steer': self._fmt_float(
                payload.get('turn_strategy_input_steer')
            ),
            'turn_strategy_pulse_phase': payload.get(
                'turn_strategy_pulse_phase', '-'
            ),
            'turn_strategy_release_phase': payload.get(
                'turn_strategy_release_phase', '-'
            ),
            'turn_strategy_counter_active': payload.get(
                'turn_strategy_counter_active', '-'
            ),
            'turn_strategy_load_overlay': payload.get(
                'turn_strategy_load_overlay', '-'
            ),
            'turn_strategy_speed_scale': self._fmt_float(
                payload.get('turn_strategy_speed_scale')
            ),
            'turn_strategy_differential_scale': self._fmt_float(
                payload.get('turn_strategy_differential_scale')
            ),
            'load_observer_mode': payload.get('load_observer_mode', '-'),
            'load_state': payload.get('load_state', '-'),
            'load_candidate_state': payload.get('load_candidate_state', '-'),
            'suspect_wheel_ids': (
                ', '.join(str(v) for v in suspect_ids) if suspect_ids else 'NONE'
            ),
            'observer_confidence': self._fmt_float(
                payload.get('observer_confidence')
            ),
            'feedback_all_fresh': payload.get('feedback_all_fresh', '-'),
            'all_stopped': payload.get('all_stopped', '-'),
            'left_avg_rpm': self._fmt_float(payload.get('left_avg_rpm')),
            'right_avg_rpm': self._fmt_float(payload.get('right_avg_rpm')),
            'status_age_sec': self._fmt_float(status_age),
            'parse_errors': str(self.ros_node.parse_error_count),
        }

        for key, value in summary.items():
            label = self._summary_labels.get(key)
            if label is not None:
                label.setText(str(value))

        wheels = payload.get('wheels', {})
        if not isinstance(wheels, dict):
            wheels = {}

        self._update_target_panel('left', wheels, LEFT_WHEELS)
        self._update_target_panel('right', wheels, RIGHT_WHEELS)

        for wheel in WHEEL_ORDER:
            data = wheels.get(wheel, {})
            if not isinstance(data, dict):
                data = {}
            self._update_wheel_card(wheel, data)

    def _update_connection_state(self, status_age: float | None) -> None:
        if status_age is None:
            text = 'NO DATA'
            style = 'background: #6b7280; color: white;'
        elif status_age <= 0.5:
            text = 'STATUS ONLINE'
            style = 'background: #15803d; color: white;'
        elif status_age <= 1.5:
            text = f'STATUS DELAY {status_age:.1f}s'
            style = 'background: #b45309; color: white;'
        else:
            text = f'STATUS STALE {status_age:.1f}s'
            style = 'background: #b91c1c; color: white;'

        self._connection_label.setText(text)
        self._connection_label.setStyleSheet(
            f'QLabel {{ {style} border-radius: 6px; padding: 7px 12px; '
            'font-weight: bold; }'
        )

    def _update_target_panel(
        self,
        side: str,
        wheels: dict[str, Any],
        wheel_names: list[str],
    ) -> None:
        targets: list[float] = []
        for wheel in wheel_names:
            wheel_data = wheels.get(wheel, {})
            if not isinstance(wheel_data, dict):
                continue
            value = self._as_float(wheel_data.get('target_rpm'))
            if value is not None:
                targets.append(value)

        value_label = self._target_value_labels[side]
        state_label = self._target_state_labels[side]

        if not targets:
            value_label.setText('-')
            value_label.setStyleSheet('border: none; color: #111827;')
            state_label.setText('WAITING')
            state_label.setStyleSheet('border: none; color: #6b7280;')
            return

        spread = max(targets) - min(targets)
        representative = sum(targets) / len(targets)

        if spread <= TARGET_MATCH_TOLERANCE_RPM:
            value_label.setText(f'{representative:.2f} RPM')
            value_label.setStyleSheet('border: none; color: #111827;')
            state_label.setText('3 MOTORS MATCH')
            state_label.setStyleSheet('border: none; color: #15803d;')
        else:
            value_label.setText('MISMATCH')
            value_label.setStyleSheet('border: none; color: #b91c1c;')
            state_label.setText(f'{min(targets):.2f} ~ {max(targets):.2f} RPM')
            state_label.setStyleSheet('border: none; color: #b91c1c;')

    def _update_wheel_card(self, wheel: str, data: dict[str, Any]) -> None:
        labels = self._wheel_labels[wheel]
        labels['id'].setText(f"ID {data.get('id', '-')}")
        labels['actual_rpm'].setText(
            f"{self._fmt_float(data.get('actual_rpm'))} RPM"
        )
        labels['error_rpm'].setText(
            f"Error: {self._fmt_float(data.get('error_rpm'))} RPM"
        )
        labels['current_a'].setText(
            f"Current: {self._fmt_float(data.get('current_a'))} A"
        )
        labels['progress_ratio'].setText(
            f"Progress: {self._fmt_float(data.get('filtered_progress_ratio'))}"
        )
        labels['current_ratio'].setText(
            f"Current ratio: {self._fmt_float(data.get('current_ratio'))}"
        )
        labels['age_sec'].setText(
            f"Age: {self._fmt_float(data.get('age_sec'))} s"
        )
        labels['timeout_count'].setText(
            f"Timeout: {data.get('timeout_count', '-')}"
        )

        load_state = str(data.get('load_state', '-'))
        labels['load_state'].setText(f'LOAD: {load_state}')
        if load_state in {'NORMAL', 'INACTIVE', 'LOAD_PROGRESSING'}:
            labels['load_state'].setStyleSheet(
                'border: none; color: #166534; font-weight: bold;'
            )
        elif load_state in {'DISABLED', 'INVALID_FEEDBACK', '-'}:
            labels['load_state'].setStyleSheet(
                'border: none; color: #6b7280; font-weight: bold;'
            )
        else:
            labels['load_state'].setStyleSheet(
                'border: none; color: #b91c1c; font-weight: bold;'
            )

        ok_value = data.get('ok')
        if ok_value is True:
            labels['ok'].setText('FEEDBACK OK')
            labels['ok'].setStyleSheet(
                'QLabel { border: none; border-radius: 5px; padding: 4px; '
                'background: #dcfce7; color: #166534; font-weight: bold; }'
            )
        elif ok_value is False:
            labels['ok'].setText('FEEDBACK FAIL')
            labels['ok'].setStyleSheet(
                'QLabel { border: none; border-radius: 5px; padding: 4px; '
                'background: #fee2e2; color: #991b1b; font-weight: bold; }'
            )
        else:
            labels['ok'].setText('NO DATA')
            labels['ok'].setStyleSheet(
                'QLabel { border: none; border-radius: 5px; padding: 4px; '
                'background: #e5e7eb; color: #4b5563; font-weight: bold; }'
            )

    @staticmethod
    def _as_float(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _fmt_float(value: Any) -> str:
        if value is None:
            return '-'
        try:
            return f'{float(value):.2f}'
        except Exception:
            return str(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    ros_node = DriveStatusSubscriber()

    app = QApplication(sys.argv)
    window = DriveDebugWindow(ros_node)
    window.show()

    try:
        app.exec()
    finally:
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
