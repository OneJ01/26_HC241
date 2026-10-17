"""
bldc_drive.launch.py

joy 노드와 bldc_drive_node를 함께 실행.

주요 파라미터 (launch argument 로 오버라이드 가능):
  serial_port    : RS-485 시리얼 포트 (기본 /dev/ttyUSB0)
  baudrate       : 통신 속도 (기본 115200)
  inter_delay_ms : ID 간 전송 간격 ms (기본 5)
  max_rpm        : 최대 RPM 제한 (기본 19.0)
  arrive_s       : 가감속 도달 시간 초 (기본 0.15)

조이스틱:
  - 블루투스 HID 조이스틱은 별도 tty 포트를 지정하지 않는다.
  - joy_node 기본 자동 인식을 사용한다.
  - /dev/input/js0 같은 경로를 device_name으로 넘기지 않는다.

사용 예:
  ros2 launch mmr_drive_node bldc_drive.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baudrate:=115200 \
    max_rpm:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_param(name: str) -> ParameterValue:
    return ParameterValue(
        PythonExpression([
            "'", LaunchConfiguration(name),
            "'.lower() in ['true', '1', 'yes', 'on']",
        ]),
        value_type=bool,
    )


def generate_launch_description():

    # ── Launch Arguments ────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB-RS485 시리얼 포트 경로'
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='RS-485 통신 속도 (현재 6대 설정값 115200)'
    )
    inter_delay_ms_arg = DeclareLaunchArgument(
        'inter_delay_ms',
        default_value='5',
        description='ID 간 패킷 전송 간격 (ms)'
    )
    max_rpm_arg = DeclareLaunchArgument(
        'max_rpm',
        default_value='19.0',
        description='전후진 최대 RPM 제한'
    )
    arrive_s_arg = DeclareLaunchArgument(
        'arrive_s',
        default_value='0.15',
        description='0x03 가감속 목표 도달 시간 (초)'
    )
    auto_align_motion_mode_arg = DeclareLaunchArgument(
        'auto_align_motion_mode',
        default_value='arc_forward',
        description='자동 정렬 방식: pivot 또는 arc_forward',
    )
    auto_align_arc_inner_ratio_arg = DeclareLaunchArgument(
        'auto_align_arc_inner_ratio',
        default_value='0.50',
        description='곡선 조향 시 내측 RPM/외측 RPM 비율',
    )
    feedback_period_s_arg = DeclareLaunchArgument(
        'feedback_period_s', default_value='0.15'
    )
    feedback_timeout_s_arg = DeclareLaunchArgument(
        'feedback_timeout_s', default_value='0.12'
    )
    feedback_fresh_timeout_s_arg = DeclareLaunchArgument(
        'feedback_fresh_timeout_s', default_value='1.5'
    )
    load_observer_enabled_arg = DeclareLaunchArgument(
        'load_observer_enabled', default_value='true'
    )
    load_protection_enabled_arg = DeclareLaunchArgument(
        'load_protection_enabled', default_value='false'
    )
    load_min_target_rpm_arg = DeclareLaunchArgument(
        'load_min_target_rpm', default_value='1.0'
    )
    load_feedback_max_age_s_arg = DeclareLaunchArgument(
        'load_feedback_max_age_s', default_value='0.5'
    )
    load_progressing_min_ratio_arg = DeclareLaunchArgument(
        'load_progressing_min_ratio', default_value='0.45'
    )
    load_stall_max_ratio_arg = DeclareLaunchArgument(
        'load_stall_max_ratio', default_value='0.20'
    )
    load_unload_current_ratio_arg = DeclareLaunchArgument(
        'load_unload_current_ratio', default_value='0.40'
    )
    load_over_current_ratio_arg = DeclareLaunchArgument(
        'load_over_current_ratio', default_value='1.50'
    )
    load_turning_threshold_arg = DeclareLaunchArgument(
        'load_turning_threshold', default_value='0.20'
    )
    load_suspect_confirm_s_arg = DeclareLaunchArgument(
        'load_suspect_confirm_s', default_value='0.30'
    )
    load_stall_confirm_s_arg = DeclareLaunchArgument(
        'load_stall_confirm_s', default_value='0.60'
    )
    load_recovery_confirm_s_arg = DeclareLaunchArgument(
        'load_recovery_confirm_s', default_value='0.50'
    )
    adaptive_steering_enabled_arg = DeclareLaunchArgument(
        'adaptive_steering_enabled',
        default_value='false',
        description='속도/조향량 연동 차동 제한과 조향 slew 활성화',
    )
    adaptive_command_period_s_arg = DeclareLaunchArgument(
        'adaptive_command_period_s', default_value='0.2'
    )
    steer_slew_rate_per_sec_arg = DeclareLaunchArgument(
        'steer_slew_rate_per_sec', default_value='1.0'
    )
    steer_speed_cap_full_rpm_arg = DeclareLaunchArgument(
        'steer_speed_cap_full_rpm', default_value='5.0'
    )

    # ── 선택형 조향 시험 전략(모두 기본 OFF) ────────────────
    turn_strategy_command_period_s_arg = DeclareLaunchArgument(
        'turn_strategy_command_period_s', default_value='0.10'
    )
    turn_strategy_start_threshold_arg = DeclareLaunchArgument(
        'turn_strategy_start_threshold', default_value='0.15'
    )
    turn_strategy_release_threshold_arg = DeclareLaunchArgument(
        'turn_strategy_release_threshold', default_value='0.08'
    )
    wide_turn_enabled_arg = DeclareLaunchArgument(
        'wide_turn_enabled', default_value='false'
    )
    wide_turn_min_inner_ratio_arg = DeclareLaunchArgument(
        'wide_turn_min_inner_ratio', default_value='0.70'
    )
    turn_entry_enabled_arg = DeclareLaunchArgument(
        'turn_entry_enabled', default_value='false'
    )
    turn_entry_speed_cap_rpm_arg = DeclareLaunchArgument(
        'turn_entry_speed_cap_rpm', default_value='3.0'
    )
    turn_entry_hold_s_arg = DeclareLaunchArgument(
        'turn_entry_hold_s', default_value='0.30'
    )
    turn_entry_ramp_s_arg = DeclareLaunchArgument(
        'turn_entry_ramp_s', default_value='0.50'
    )
    pulse_turn_enabled_arg = DeclareLaunchArgument(
        'pulse_turn_enabled', default_value='false'
    )
    pulse_turn_on_s_arg = DeclareLaunchArgument(
        'pulse_turn_on_s', default_value='0.30'
    )
    pulse_turn_relief_s_arg = DeclareLaunchArgument(
        'pulse_turn_relief_s', default_value='0.20'
    )
    pulse_turn_relief_speed_cap_rpm_arg = DeclareLaunchArgument(
        'pulse_turn_relief_speed_cap_rpm', default_value='3.0'
    )
    turn_release_enabled_arg = DeclareLaunchArgument(
        'turn_release_enabled', default_value='false'
    )
    turn_release_straight_s_arg = DeclareLaunchArgument(
        'turn_release_straight_s', default_value='0.40'
    )
    turn_release_speed_cap_rpm_arg = DeclareLaunchArgument(
        'turn_release_speed_cap_rpm', default_value='2.0'
    )
    counter_steer_enabled_arg = DeclareLaunchArgument(
        'counter_steer_enabled', default_value='false'
    )
    counter_steer_ratio_arg = DeclareLaunchArgument(
        'counter_steer_ratio', default_value='0.05'
    )
    counter_steer_duration_s_arg = DeclareLaunchArgument(
        'counter_steer_duration_s', default_value='0.15'
    )
    counter_steer_speed_cap_rpm_arg = DeclareLaunchArgument(
        'counter_steer_speed_cap_rpm', default_value='1.5'
    )
    counter_steer_forward_only_arg = DeclareLaunchArgument(
        'counter_steer_forward_only', default_value='true'
    )
    load_adaptive_turn_enabled_arg = DeclareLaunchArgument(
        'load_adaptive_turn_enabled', default_value='false'
    )
    load_turn_unload_diff_scale_arg = DeclareLaunchArgument(
        'load_turn_unload_diff_scale', default_value='0.70'
    )
    load_turn_unload_speed_scale_arg = DeclareLaunchArgument(
        'load_turn_unload_speed_scale', default_value='0.85'
    )
    load_turn_local_stall_diff_scale_arg = DeclareLaunchArgument(
        'load_turn_local_stall_diff_scale', default_value='0.50'
    )
    load_turn_local_stall_speed_scale_arg = DeclareLaunchArgument(
        'load_turn_local_stall_speed_scale', default_value='0.70'
    )

    # ── joy 노드 ─────────────────────────────────────────────
    # 블루투스 조이스틱은 joy_node 기본 자동 인식을 사용한다.
    # device_name에 /dev/input/js0 같은 경로를 넣지 않는다.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'autorepeat_rate': 20.0,
            'deadzone': 0.05,
        }],
        output='screen',
    )

    # ── bldc_drive_node ──────────────────────────────────────
    bldc_drive_node = Node(
        package='mmr_drive_node',
        executable='bldc_drive_node',
        name='bldc_drive_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': ParameterValue(
                LaunchConfiguration('baudrate'),
                value_type=int,
            ),
            'inter_delay_ms': ParameterValue(
                LaunchConfiguration('inter_delay_ms'),
                value_type=int,
            ),
            'max_rpm': ParameterValue(
                LaunchConfiguration('max_rpm'),
                value_type=float,
            ),
            'arrive_s': ParameterValue(
                LaunchConfiguration('arrive_s'),
                value_type=float,
            ),
            'auto_align_motion_mode': LaunchConfiguration('auto_align_motion_mode'),
            'auto_align_arc_inner_ratio': ParameterValue(
                LaunchConfiguration('auto_align_arc_inner_ratio'),
                value_type=float,
            ),
            'feedback_period_s': ParameterValue(
                LaunchConfiguration('feedback_period_s'),
                value_type=float,
            ),
            'feedback_timeout_s': ParameterValue(
                LaunchConfiguration('feedback_timeout_s'),
                value_type=float,
            ),
            'feedback_fresh_timeout_s': ParameterValue(
                LaunchConfiguration('feedback_fresh_timeout_s'),
                value_type=float,
            ),
            'load_observer_enabled': _bool_param('load_observer_enabled'),
            'load_protection_enabled': _bool_param('load_protection_enabled'),
            'load_min_target_rpm': ParameterValue(
                LaunchConfiguration('load_min_target_rpm'), value_type=float
            ),
            'load_feedback_max_age_s': ParameterValue(
                LaunchConfiguration('load_feedback_max_age_s'), value_type=float
            ),
            'load_progressing_min_ratio': ParameterValue(
                LaunchConfiguration('load_progressing_min_ratio'), value_type=float
            ),
            'load_stall_max_ratio': ParameterValue(
                LaunchConfiguration('load_stall_max_ratio'), value_type=float
            ),
            'load_unload_current_ratio': ParameterValue(
                LaunchConfiguration('load_unload_current_ratio'), value_type=float
            ),
            'load_over_current_ratio': ParameterValue(
                LaunchConfiguration('load_over_current_ratio'), value_type=float
            ),
            'load_turning_threshold': ParameterValue(
                LaunchConfiguration('load_turning_threshold'), value_type=float
            ),
            'load_suspect_confirm_s': ParameterValue(
                LaunchConfiguration('load_suspect_confirm_s'), value_type=float
            ),
            'load_stall_confirm_s': ParameterValue(
                LaunchConfiguration('load_stall_confirm_s'), value_type=float
            ),
            'load_recovery_confirm_s': ParameterValue(
                LaunchConfiguration('load_recovery_confirm_s'), value_type=float
            ),
            'adaptive_steering_enabled': _bool_param('adaptive_steering_enabled'),
            'adaptive_command_period_s': ParameterValue(
                LaunchConfiguration('adaptive_command_period_s'),
                value_type=float,
            ),
            'steer_slew_rate_per_sec': ParameterValue(
                LaunchConfiguration('steer_slew_rate_per_sec'),
                value_type=float,
            ),
            'steer_speed_cap_full_rpm': ParameterValue(
                LaunchConfiguration('steer_speed_cap_full_rpm'),
                value_type=float,
            ),
            'turn_strategy_command_period_s': ParameterValue(
                LaunchConfiguration('turn_strategy_command_period_s'), value_type=float
            ),
            'turn_strategy_start_threshold': ParameterValue(
                LaunchConfiguration('turn_strategy_start_threshold'), value_type=float
            ),
            'turn_strategy_release_threshold': ParameterValue(
                LaunchConfiguration('turn_strategy_release_threshold'), value_type=float
            ),
            'wide_turn_enabled': _bool_param('wide_turn_enabled'),
            'wide_turn_min_inner_ratio': ParameterValue(
                LaunchConfiguration('wide_turn_min_inner_ratio'), value_type=float
            ),
            'turn_entry_enabled': _bool_param('turn_entry_enabled'),
            'turn_entry_speed_cap_rpm': ParameterValue(
                LaunchConfiguration('turn_entry_speed_cap_rpm'), value_type=float
            ),
            'turn_entry_hold_s': ParameterValue(
                LaunchConfiguration('turn_entry_hold_s'), value_type=float
            ),
            'turn_entry_ramp_s': ParameterValue(
                LaunchConfiguration('turn_entry_ramp_s'), value_type=float
            ),
            'pulse_turn_enabled': _bool_param('pulse_turn_enabled'),
            'pulse_turn_on_s': ParameterValue(
                LaunchConfiguration('pulse_turn_on_s'), value_type=float
            ),
            'pulse_turn_relief_s': ParameterValue(
                LaunchConfiguration('pulse_turn_relief_s'), value_type=float
            ),
            'pulse_turn_relief_speed_cap_rpm': ParameterValue(
                LaunchConfiguration('pulse_turn_relief_speed_cap_rpm'), value_type=float
            ),
            'turn_release_enabled': _bool_param('turn_release_enabled'),
            'turn_release_straight_s': ParameterValue(
                LaunchConfiguration('turn_release_straight_s'), value_type=float
            ),
            'turn_release_speed_cap_rpm': ParameterValue(
                LaunchConfiguration('turn_release_speed_cap_rpm'), value_type=float
            ),
            'counter_steer_enabled': _bool_param('counter_steer_enabled'),
            'counter_steer_ratio': ParameterValue(
                LaunchConfiguration('counter_steer_ratio'), value_type=float
            ),
            'counter_steer_duration_s': ParameterValue(
                LaunchConfiguration('counter_steer_duration_s'), value_type=float
            ),
            'counter_steer_speed_cap_rpm': ParameterValue(
                LaunchConfiguration('counter_steer_speed_cap_rpm'), value_type=float
            ),
            'counter_steer_forward_only': _bool_param('counter_steer_forward_only'),
            'load_adaptive_turn_enabled': _bool_param('load_adaptive_turn_enabled'),
            'load_turn_unload_diff_scale': ParameterValue(
                LaunchConfiguration('load_turn_unload_diff_scale'), value_type=float
            ),
            'load_turn_unload_speed_scale': ParameterValue(
                LaunchConfiguration('load_turn_unload_speed_scale'), value_type=float
            ),
            'load_turn_local_stall_diff_scale': ParameterValue(
                LaunchConfiguration('load_turn_local_stall_diff_scale'), value_type=float
            ),
            'load_turn_local_stall_speed_scale': ParameterValue(
                LaunchConfiguration('load_turn_local_stall_speed_scale'), value_type=float
            ),
            'joy_topic': '/joy',
        }],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        inter_delay_ms_arg,
        max_rpm_arg,
        arrive_s_arg,
        auto_align_motion_mode_arg,
        auto_align_arc_inner_ratio_arg,
        feedback_period_s_arg,
        feedback_timeout_s_arg,
        feedback_fresh_timeout_s_arg,
        load_observer_enabled_arg,
        load_protection_enabled_arg,
        load_min_target_rpm_arg,
        load_feedback_max_age_s_arg,
        load_progressing_min_ratio_arg,
        load_stall_max_ratio_arg,
        load_unload_current_ratio_arg,
        load_over_current_ratio_arg,
        load_turning_threshold_arg,
        load_suspect_confirm_s_arg,
        load_stall_confirm_s_arg,
        load_recovery_confirm_s_arg,
        adaptive_steering_enabled_arg,
        adaptive_command_period_s_arg,
        steer_slew_rate_per_sec_arg,
        steer_speed_cap_full_rpm_arg,
        turn_strategy_command_period_s_arg,
        turn_strategy_start_threshold_arg,
        turn_strategy_release_threshold_arg,
        wide_turn_enabled_arg,
        wide_turn_min_inner_ratio_arg,
        turn_entry_enabled_arg,
        turn_entry_speed_cap_rpm_arg,
        turn_entry_hold_s_arg,
        turn_entry_ramp_s_arg,
        pulse_turn_enabled_arg,
        pulse_turn_on_s_arg,
        pulse_turn_relief_s_arg,
        pulse_turn_relief_speed_cap_rpm_arg,
        turn_release_enabled_arg,
        turn_release_straight_s_arg,
        turn_release_speed_cap_rpm_arg,
        counter_steer_enabled_arg,
        counter_steer_ratio_arg,
        counter_steer_duration_s_arg,
        counter_steer_speed_cap_rpm_arg,
        counter_steer_forward_only_arg,
        load_adaptive_turn_enabled_arg,
        load_turn_unload_diff_scale_arg,
        load_turn_unload_speed_scale_arg,
        load_turn_local_stall_diff_scale_arg,
        load_turn_local_stall_speed_scale_arg,
        joy_node,
        bldc_drive_node,
    ])