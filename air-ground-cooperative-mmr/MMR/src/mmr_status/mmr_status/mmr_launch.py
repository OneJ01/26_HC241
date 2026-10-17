"""
mmr_launch.py
─────────────────────────────────────────────────────────────────────────────
MMR 통합 실행 런치 파일.

목표:
  - 구동 BLDC, 탄도 계산, 조준 coordinator, 포신 STM, GUI를 한 번에 실행
  - 기본값은 실제 하드웨어 통합 실행 기준으로 true
  - 소프트웨어 점검 시에만 launch 인자로 use_drive/use_stm/use_imu/use_gnss 등을 false로 override

대표 실행:
  # 실제 하드웨어 통합 실행 기본
  ros2 launch mmr_status mmr_launch.py

  # 소프트웨어 점검 시에만 하드웨어 노드 OFF
  ros2 launch mmr_status mmr_launch.py \
    use_drive:=false use_stm:=false use_imu:=false use_gnss:=false use_ntrip:=false

주요 토픽 연결:
  /target/point
    → fire_solution_node
    → /mmr/fire_solution
    → aim_coordinator_node
    → /mmr/drive/align_cmd       → bldc_drive_node
    → /mmr/turret/aim_request    → turret_control_node
    → /mmr/fire_confirm          → turret_control_node

  main_gui
    → /mmr/operator_input
    ← /mmr/engagement_status, /mmr/fire_solution, /target/point
    ← /compute/impact_point
      · 표적 중심 유효 타격 반경 표시
      · 탄착 편차 및 SUCCESS/REATTACK 판정
─────────────────────────────────────────────────────────────────────────────
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


_TRUE_EXPR = "'.lower() in ['true', '1', 'yes', 'on']"


def _bool_param(name: str) -> ParameterValue:
    """LaunchConfiguration 문자열을 ROS bool 파라미터로 변환."""
    return ParameterValue(
        PythonExpression(["'", LaunchConfiguration(name), _TRUE_EXPR]),
        value_type=bool,
    )


def _declare_bool(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    fire_cfg = PathJoinSubstitution([
        FindPackageShare("fire_solution"),
        "config",
        "launch_params.yaml",
    ])

    # 기본값은 실제 장비 통합 실행 기준이다.
    # 소프트웨어 테스트 시에는 use_* 인자를 false로 넘겨 하드웨어 노드를 끈다.
    gnss_launch = PathJoinSubstitution([
        FindPackageShare("gnss_driver"),
        "launch",
        "gnss_stable_launch.py",
    ])

    # ── 실행 여부 스위치 ─────────────────────────────────────────
    use_tf_arg = _declare_bool("use_tf", "true", "base_link→gps_link static TF 실행")
    use_gnss_arg = _declare_bool("use_gnss", "true", "gnss_driver 통합 실행")
    use_imu_arg = _declare_bool("use_imu", "true", "stella_ahrs + tilt_monitor 실행")
    use_status_arg = _declare_bool("use_status", "true", "mmr_status_node 실행")
    use_drive_arg = _declare_bool("use_drive", "true", "BLDC bldc_drive_node 실행")
    use_joy_arg = _declare_bool("use_joy", "true", "joy_node 실행. use_drive:=true일 때만 의미 있음")
    use_fire_solution_arg = _declare_bool("use_fire_solution", "true", "fire_solution_node 실행")
    use_coordinator_arg = _declare_bool("use_coordinator", "true", "aim_coordinator_node 실행")
    use_turret_arg = _declare_bool("use_turret", "true", "turret_control_node 실행")
    use_stm_arg = _declare_bool("use_stm", "true", "turret_control_node UART 활성화")
    use_gui_arg = _declare_bool("use_gui", "true", "PyQt GUI 실행")

    # ── 포트/장치 ────────────────────────────────────────────────
    lite_port_arg = DeclareLaunchArgument("lite_port", default_value="/dev/ttyACM0")
    rover_port_arg = DeclareLaunchArgument("rover_port", default_value="/dev/ttyACM1")
    gnss_baudrate_arg = DeclareLaunchArgument("gnss_baudrate", default_value="115200")
    use_ntrip_arg = _declare_bool("use_ntrip", "true", "GNSS NTRIP 실행")

    ntrip_authenticate_arg = _declare_bool(
        "ntrip_authenticate",
        "false",
        "NTRIP 인증 사용 여부",
    )
    ntrip_username_arg = DeclareLaunchArgument(
        "ntrip_username",
        default_value="",
        description="NTRIP username",
    )
    ntrip_password_arg = DeclareLaunchArgument(
        "ntrip_password",
        default_value="",
        description="NTRIP password",
    )

    imu_port_arg = DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB1")
    imu_baudrate_arg = DeclareLaunchArgument("imu_baudrate", default_value="115200")
    imu_frame_id_arg = DeclareLaunchArgument("imu_frame_id", default_value="imu_link")

    bldc_port_arg = DeclareLaunchArgument("bldc_port", default_value="/dev/ttyUSB0")
    bldc_baudrate_arg = DeclareLaunchArgument("bldc_baudrate", default_value="115200")
    stm_port_arg = DeclareLaunchArgument("stm_port", default_value="/dev/ttyACM0")
    turret_startup_delay_sec_arg = DeclareLaunchArgument(
        "turret_startup_delay_sec",
        default_value="2.0",
        description="STM serial open 이후 startup HOME0 전 대기 시간(sec)",
    )

    dem_path_arg = DeclareLaunchArgument(
        "dem_path",
        default_value=os.path.expanduser("~/mmr_ws/data/dem_tin_1m_pot.tif"),
        description="DEM GeoTIFF 절대경로. 빈 문자열로 주면 DEM 비활성",
    )

    # ── 구동 파라미터 ────────────────────────────────────────────
    max_rpm_arg = DeclareLaunchArgument("max_rpm", default_value="19.0")
    arrive_s_arg = DeclareLaunchArgument("arrive_s", default_value="0.15")
    default_pivot_rpm_arg = DeclareLaunchArgument("default_pivot_rpm", default_value="5.0")
    manual_pivot_rpm_arg = DeclareLaunchArgument("manual_pivot_rpm", default_value="5.0")
    auto_align_motion_mode_arg = DeclareLaunchArgument(
        "auto_align_motion_mode",
        default_value="arc_forward",
        description="자동 정렬 방식: pivot 또는 arc_forward",
    )
    auto_align_arc_inner_ratio_arg = DeclareLaunchArgument(
        "auto_align_arc_inner_ratio",
        default_value="0.50",
        description="곡선 조향 시 내측 RPM/외측 RPM 비율",
    )
    align_fast_rpm_arg = DeclareLaunchArgument(
        "align_fast_rpm",
        default_value="7.0",
        description="자동 정렬 큰 방위 오차 구간의 외측 바퀴 RPM",
    )
    align_medium_rpm_arg = DeclareLaunchArgument(
        "align_medium_rpm",
        default_value="6.0",
        description="자동 정렬 중간 방위 오차 구간의 외측 바퀴 RPM",
    )
    align_slow_rpm_arg = DeclareLaunchArgument(
        "align_slow_rpm",
        default_value="5.0",
        description="자동 정렬 미세 방위 오차 구간의 외측 바퀴 RPM",
    )
    align_cmd_timeout_sec_arg = DeclareLaunchArgument("align_cmd_timeout_sec", default_value="0.5")
    stop_rpm_threshold_arg = DeclareLaunchArgument("stop_rpm_threshold", default_value="0.3")
    feedback_period_s_arg = DeclareLaunchArgument(
        "feedback_period_s", default_value="0.15"
    )
    feedback_timeout_s_arg = DeclareLaunchArgument(
        "feedback_timeout_s", default_value="0.12"
    )
    feedback_fresh_timeout_s_arg = DeclareLaunchArgument(
        "feedback_fresh_timeout_s", default_value="1.5"
    )

    # RPM·전류 부하 관측기. protection=false이면 진단만 수행하는 Shadow mode.
    load_observer_enabled_arg = _declare_bool(
        "load_observer_enabled",
        "true",
        "목표/실제 RPM과 전류 기반 부하 상태 관측",
    )
    load_protection_enabled_arg = _declare_bool(
        "load_protection_enabled",
        "false",
        "예약 파라미터. 현재 패치는 Shadow mode 전용",
    )
    load_min_target_rpm_arg = DeclareLaunchArgument(
        "load_min_target_rpm", default_value="1.0"
    )
    load_feedback_max_age_s_arg = DeclareLaunchArgument(
        "load_feedback_max_age_s", default_value="0.5"
    )
    load_progressing_min_ratio_arg = DeclareLaunchArgument(
        "load_progressing_min_ratio", default_value="0.45"
    )
    load_stall_max_ratio_arg = DeclareLaunchArgument(
        "load_stall_max_ratio", default_value="0.20"
    )
    load_unload_current_ratio_arg = DeclareLaunchArgument(
        "load_unload_current_ratio", default_value="0.40"
    )
    load_over_current_ratio_arg = DeclareLaunchArgument(
        "load_over_current_ratio", default_value="1.50"
    )
    load_turning_threshold_arg = DeclareLaunchArgument(
        "load_turning_threshold", default_value="0.20"
    )
    load_suspect_confirm_s_arg = DeclareLaunchArgument(
        "load_suspect_confirm_s", default_value="0.30"
    )
    load_stall_confirm_s_arg = DeclareLaunchArgument(
        "load_stall_confirm_s", default_value="0.60"
    )
    load_recovery_confirm_s_arg = DeclareLaunchArgument(
        "load_recovery_confirm_s", default_value="0.50"
    )

    # 일반 주행 적응형 차동 조향. false이면 기존 조향 계산으로 완전 복귀.
    adaptive_steering_enabled_arg = _declare_bool(
        "adaptive_steering_enabled",
        "false",
        "속도/조향량 연동 차동 제한과 조향 slew 활성화",
    )
    adaptive_command_period_s_arg = DeclareLaunchArgument(
        "adaptive_command_period_s", default_value="0.2"
    )
    steer_slew_rate_per_sec_arg = DeclareLaunchArgument(
        "steer_slew_rate_per_sec", default_value="1.0"
    )
    steer_low_speed_rpm_arg = DeclareLaunchArgument(
        "steer_low_speed_rpm", default_value="4.0"
    )
    steer_mid_speed_rpm_arg = DeclareLaunchArgument(
        "steer_mid_speed_rpm", default_value="8.0"
    )
    steer_high_speed_rpm_arg = DeclareLaunchArgument(
        "steer_high_speed_rpm", default_value="12.0"
    )
    steer_transition_rpm_arg = DeclareLaunchArgument(
        "steer_transition_rpm", default_value="1.0"
    )
    steer_reduction_low_arg = DeclareLaunchArgument(
        "steer_reduction_low", default_value="0.50"
    )
    steer_reduction_mid_arg = DeclareLaunchArgument(
        "steer_reduction_mid", default_value="0.30"
    )
    steer_reduction_high_arg = DeclareLaunchArgument(
        "steer_reduction_high", default_value="0.20"
    )
    steer_reduction_very_high_arg = DeclareLaunchArgument(
        "steer_reduction_very_high", default_value="0.12"
    )
    steer_speed_cap_mid_rpm_arg = DeclareLaunchArgument(
        "steer_speed_cap_mid_rpm", default_value="10.0"
    )
    steer_speed_cap_high_rpm_arg = DeclareLaunchArgument(
        "steer_speed_cap_high_rpm", default_value="7.0"
    )
    steer_speed_cap_full_rpm_arg = DeclareLaunchArgument(
        "steer_speed_cap_full_rpm", default_value="5.0"
    )

    # 선택형 조향 시험 전략. 모두 false이면 기존 조향 출력과 동일.
    turn_strategy_command_period_s_arg = DeclareLaunchArgument(
        "turn_strategy_command_period_s", default_value="0.10"
    )
    turn_strategy_start_threshold_arg = DeclareLaunchArgument(
        "turn_strategy_start_threshold", default_value="0.15"
    )
    turn_strategy_release_threshold_arg = DeclareLaunchArgument(
        "turn_strategy_release_threshold", default_value="0.08"
    )
    wide_turn_enabled_arg = _declare_bool(
        "wide_turn_enabled", "false", "완만한 곡선 조향: 안쪽 휠 최소 비율 제한"
    )
    wide_turn_min_inner_ratio_arg = DeclareLaunchArgument(
        "wide_turn_min_inner_ratio", default_value="0.70"
    )
    tight_turn_enabled_arg = _declare_bool(
        "tight_turn_enabled",
        "false",
        "큰 조향 입력의 펄스 ON 구간에서 내측 휠을 더 강하게 감속",
    )
    tight_turn_start_steer_arg = DeclareLaunchArgument(
        "tight_turn_start_steer", default_value="0.55"
    )
    tight_turn_full_steer_arg = DeclareLaunchArgument(
        "tight_turn_full_steer", default_value="0.90"
    )
    tight_turn_min_inner_ratio_arg = DeclareLaunchArgument(
        "tight_turn_min_inner_ratio", default_value="0.20"
    )
    tight_turn_outer_speed_cap_rpm_arg = DeclareLaunchArgument(
        "tight_turn_outer_speed_cap_rpm", default_value="15.0"
    )
    tight_turn_ramp_s_arg = DeclareLaunchArgument(
        "tight_turn_ramp_s", default_value="0.15"
    )
    tight_turn_only_with_pulse_arg = _declare_bool(
        "tight_turn_only_with_pulse",
        "true",
        "TIGHT_TURN을 펄스 TURN 구간에서만 적용",
    )
    turn_entry_enabled_arg = _declare_bool(
        "turn_entry_enabled", "false", "조향 진입 전 속도 제한/차동 램프"
    )
    turn_entry_speed_cap_rpm_arg = DeclareLaunchArgument(
        "turn_entry_speed_cap_rpm", default_value="3.0"
    )
    turn_entry_hold_s_arg = DeclareLaunchArgument(
        "turn_entry_hold_s", default_value="0.30"
    )
    turn_entry_ramp_s_arg = DeclareLaunchArgument(
        "turn_entry_ramp_s", default_value="0.50"
    )
    pulse_turn_enabled_arg = _declare_bool(
        "pulse_turn_enabled", "false", "조향/직진 해소 펄스 반복"
    )
    pulse_turn_on_s_arg = DeclareLaunchArgument(
        "pulse_turn_on_s", default_value="0.30"
    )
    pulse_turn_relief_s_arg = DeclareLaunchArgument(
        "pulse_turn_relief_s", default_value="0.20"
    )
    pulse_turn_relief_speed_cap_rpm_arg = DeclareLaunchArgument(
        "pulse_turn_relief_speed_cap_rpm", default_value="3.0"
    )
    pivot_pulse_enabled_arg = _declare_bool(
        "pivot_pulse_enabled",
        "false",
        "LB/RB 제자리 회전을 PIVOT/0RPM RELIEF 펄스로 실행",
    )
    pivot_pulse_rpm_arg = DeclareLaunchArgument(
        "pivot_pulse_rpm", default_value="4.0"
    )
    pivot_pulse_on_s_arg = DeclareLaunchArgument(
        "pivot_pulse_on_s", default_value="0.20"
    )
    pivot_pulse_relief_s_arg = DeclareLaunchArgument(
        "pivot_pulse_relief_s", default_value="0.20"
    )
    pivot_pulse_arrive_s_arg = DeclareLaunchArgument(
        "pivot_pulse_arrive_s", default_value="0.10"
    )
    turn_release_enabled_arg = _declare_bool(
        "turn_release_enabled", "false", "조향 해제 후 짧은 직진 해소"
    )
    turn_release_straight_s_arg = DeclareLaunchArgument(
        "turn_release_straight_s", default_value="0.40"
    )
    turn_release_speed_cap_rpm_arg = DeclareLaunchArgument(
        "turn_release_speed_cap_rpm", default_value="2.0"
    )
    turn_release_keep_drive_speed_arg = _declare_bool(
        "turn_release_keep_drive_speed",
        "false",
        "조향 해제/카운터 구간에도 현재 직진 요청 속도 유지",
    )
    counter_steer_enabled_arg = _declare_bool(
        "counter_steer_enabled", "false", "조향 해제 후 작은 반대 타각 펄스"
    )
    counter_steer_ratio_arg = DeclareLaunchArgument(
        "counter_steer_ratio", default_value="0.05"
    )
    counter_steer_duration_s_arg = DeclareLaunchArgument(
        "counter_steer_duration_s", default_value="0.15"
    )
    counter_steer_speed_cap_rpm_arg = DeclareLaunchArgument(
        "counter_steer_speed_cap_rpm", default_value="1.5"
    )
    counter_steer_forward_only_arg = _declare_bool(
        "counter_steer_forward_only", "true", "반대 타각은 전진에서만 허용"
    )
    load_adaptive_turn_enabled_arg = _declare_bool(
        "load_adaptive_turn_enabled", "false", "부하 관측 결과로 조향 차동/속도 완화"
    )
    load_turn_unload_diff_scale_arg = DeclareLaunchArgument(
        "load_turn_unload_diff_scale", default_value="0.70"
    )
    load_turn_unload_speed_scale_arg = DeclareLaunchArgument(
        "load_turn_unload_speed_scale", default_value="0.85"
    )
    load_turn_local_stall_diff_scale_arg = DeclareLaunchArgument(
        "load_turn_local_stall_diff_scale", default_value="0.50"
    )
    load_turn_local_stall_speed_scale_arg = DeclareLaunchArgument(
        "load_turn_local_stall_speed_scale", default_value="0.70"
    )

    return LaunchDescription([
        # 실행 스위치
        use_tf_arg,
        use_gnss_arg,
        use_imu_arg,
        use_status_arg,
        use_drive_arg,
        use_joy_arg,
        use_fire_solution_arg,
        use_coordinator_arg,
        use_turret_arg,
        use_stm_arg,
        use_gui_arg,

        # 포트/장치
        lite_port_arg,
        rover_port_arg,
        gnss_baudrate_arg,
        use_ntrip_arg,
        ntrip_authenticate_arg,
        ntrip_username_arg,
        ntrip_password_arg,
        imu_port_arg,
        imu_baudrate_arg,
        imu_frame_id_arg,
        bldc_port_arg,
        bldc_baudrate_arg,
        stm_port_arg,
        turret_startup_delay_sec_arg,
        dem_path_arg,

        # 구동 파라미터
        max_rpm_arg,
        arrive_s_arg,
        default_pivot_rpm_arg,
        manual_pivot_rpm_arg,
        auto_align_motion_mode_arg,
        auto_align_arc_inner_ratio_arg,
        align_fast_rpm_arg,
        align_medium_rpm_arg,
        align_slow_rpm_arg,
        align_cmd_timeout_sec_arg,
        stop_rpm_threshold_arg,
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
        steer_low_speed_rpm_arg,
        steer_mid_speed_rpm_arg,
        steer_high_speed_rpm_arg,
        steer_transition_rpm_arg,
        steer_reduction_low_arg,
        steer_reduction_mid_arg,
        steer_reduction_high_arg,
        steer_reduction_very_high_arg,
        steer_speed_cap_mid_rpm_arg,
        steer_speed_cap_high_rpm_arg,
        steer_speed_cap_full_rpm_arg,
        turn_strategy_command_period_s_arg,
        turn_strategy_start_threshold_arg,
        turn_strategy_release_threshold_arg,
        wide_turn_enabled_arg,
        wide_turn_min_inner_ratio_arg,
        tight_turn_enabled_arg,
        tight_turn_start_steer_arg,
        tight_turn_full_steer_arg,
        tight_turn_min_inner_ratio_arg,
        tight_turn_outer_speed_cap_rpm_arg,
        tight_turn_ramp_s_arg,
        tight_turn_only_with_pulse_arg,
        turn_entry_enabled_arg,
        turn_entry_speed_cap_rpm_arg,
        turn_entry_hold_s_arg,
        turn_entry_ramp_s_arg,
        pulse_turn_enabled_arg,
        pulse_turn_on_s_arg,
        pulse_turn_relief_s_arg,
        pulse_turn_relief_speed_cap_rpm_arg,
        pivot_pulse_enabled_arg,
        pivot_pulse_rpm_arg,
        pivot_pulse_on_s_arg,
        pivot_pulse_relief_s_arg,
        pivot_pulse_arrive_s_arg,
        turn_release_enabled_arg,
        turn_release_straight_s_arg,
        turn_release_speed_cap_rpm_arg,
        turn_release_keep_drive_speed_arg,
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

        # ── [0] GNSS 통합 실행 옵션 ─────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gnss_launch),
            launch_arguments={
                "lite_port": LaunchConfiguration("lite_port"),
                "rover_port": LaunchConfiguration("rover_port"),
                "baudrate": LaunchConfiguration("gnss_baudrate"),
                "use_ntrip": LaunchConfiguration("use_ntrip"),
                "ntrip_authenticate": LaunchConfiguration("ntrip_authenticate"),
                "ntrip_username": LaunchConfiguration("ntrip_username"),
                "ntrip_password": LaunchConfiguration("ntrip_password"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gnss")),
        ),

        # ── [1] TF: base_link → gps_link ────────────────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_gps",
            arguments=["0", "0", "0.2", "0", "0", "0", "base_link", "gps_link"],
            condition=IfCondition(LaunchConfiguration("use_tf")),
        ),

        # ── [2] IMU + roll/pitch 상태 감시 ─────────────────────
        Node(
            package="stella_ahrs",
            executable="stella_ahrs_node",
            name="stella_ahrs_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "port": LaunchConfiguration("imu_port"),
                "baudrate": ParameterValue(LaunchConfiguration("imu_baudrate"), value_type=int),
                "frame_id": LaunchConfiguration("imu_frame_id"),
            }],
            condition=IfCondition(LaunchConfiguration("use_imu")),
        ),
        Node(
            package="mmr_status",
            executable="tilt_monitor_node",
            name="tilt_monitor_node",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("use_imu")),
        ),

        # ── [3] 상태 집계 ──────────────────────────────────────
        Node(
            package="mmr_status",
            executable="mmr_status_node",
            name="mmr_status_node",
            output="screen",
            emulate_tty=True,
            parameters=[{"publish_rate_hz": 5.0}],
            condition=IfCondition(LaunchConfiguration("use_status")),
        ),

        # ── [4] 조이스틱 + BLDC 구동 ───────────────────────────
        # 블루투스 조이스틱은 joy_node 기본 자동 인식을 사용한다.
        # /dev/input/js0 같은 경로를 device_name으로 넘기지 않는다.
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "autorepeat_rate": 20.0,
                "deadzone": 0.05,
            }],
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("use_drive"), _TRUE_EXPR,
                    " and '", LaunchConfiguration("use_joy"), _TRUE_EXPR,
                ])
            ),
        ),
        Node(
            package="mmr_drive_node",
            executable="bldc_drive_node",
            name="bldc_drive_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "serial_port": LaunchConfiguration("bldc_port"),
                "baudrate": ParameterValue(LaunchConfiguration("bldc_baudrate"), value_type=int),
                "inter_delay_ms": 5,
                "max_rpm": ParameterValue(LaunchConfiguration("max_rpm"), value_type=float),
                "arrive_s": ParameterValue(LaunchConfiguration("arrive_s"), value_type=float),
                "joy_topic": "/joy",
                "align_cmd_topic": "/mmr/drive/align_cmd",
                "drive_status_topic": "/mmr/drive/status",
                "default_pivot_rpm": ParameterValue(LaunchConfiguration("default_pivot_rpm"), value_type=float),
                "manual_pivot_rpm": ParameterValue(LaunchConfiguration("manual_pivot_rpm"), value_type=float),
                "auto_align_motion_mode": LaunchConfiguration("auto_align_motion_mode"),
                "auto_align_arc_inner_ratio": ParameterValue(
                    LaunchConfiguration("auto_align_arc_inner_ratio"), value_type=float
                ),
                "manual_pivot_left_button": 4,
                "manual_pivot_right_button": 5,
                "align_cmd_timeout_sec": ParameterValue(LaunchConfiguration("align_cmd_timeout_sec"), value_type=float),
                "stop_rpm_threshold": ParameterValue(LaunchConfiguration("stop_rpm_threshold"), value_type=float),
                "feedback_period_s": ParameterValue(LaunchConfiguration("feedback_period_s"), value_type=float),
                "feedback_timeout_s": ParameterValue(LaunchConfiguration("feedback_timeout_s"), value_type=float),
                "feedback_fresh_timeout_s": ParameterValue(LaunchConfiguration("feedback_fresh_timeout_s"), value_type=float),
                "load_observer_enabled": _bool_param("load_observer_enabled"),
                "load_protection_enabled": _bool_param("load_protection_enabled"),
                "load_min_target_rpm": ParameterValue(LaunchConfiguration("load_min_target_rpm"), value_type=float),
                "load_feedback_max_age_s": ParameterValue(LaunchConfiguration("load_feedback_max_age_s"), value_type=float),
                "load_progressing_min_ratio": ParameterValue(LaunchConfiguration("load_progressing_min_ratio"), value_type=float),
                "load_stall_max_ratio": ParameterValue(LaunchConfiguration("load_stall_max_ratio"), value_type=float),
                "load_unload_current_ratio": ParameterValue(LaunchConfiguration("load_unload_current_ratio"), value_type=float),
                "load_over_current_ratio": ParameterValue(LaunchConfiguration("load_over_current_ratio"), value_type=float),
                "load_turning_threshold": ParameterValue(LaunchConfiguration("load_turning_threshold"), value_type=float),
                "load_suspect_confirm_s": ParameterValue(LaunchConfiguration("load_suspect_confirm_s"), value_type=float),
                "load_stall_confirm_s": ParameterValue(LaunchConfiguration("load_stall_confirm_s"), value_type=float),
                "load_recovery_confirm_s": ParameterValue(LaunchConfiguration("load_recovery_confirm_s"), value_type=float),
                "adaptive_steering_enabled": _bool_param("adaptive_steering_enabled"),
                "adaptive_command_period_s": ParameterValue(LaunchConfiguration("adaptive_command_period_s"), value_type=float),
                "steer_slew_rate_per_sec": ParameterValue(LaunchConfiguration("steer_slew_rate_per_sec"), value_type=float),
                "steer_low_speed_rpm": ParameterValue(LaunchConfiguration("steer_low_speed_rpm"), value_type=float),
                "steer_mid_speed_rpm": ParameterValue(LaunchConfiguration("steer_mid_speed_rpm"), value_type=float),
                "steer_high_speed_rpm": ParameterValue(LaunchConfiguration("steer_high_speed_rpm"), value_type=float),
                "steer_transition_rpm": ParameterValue(LaunchConfiguration("steer_transition_rpm"), value_type=float),
                "steer_reduction_low": ParameterValue(LaunchConfiguration("steer_reduction_low"), value_type=float),
                "steer_reduction_mid": ParameterValue(LaunchConfiguration("steer_reduction_mid"), value_type=float),
                "steer_reduction_high": ParameterValue(LaunchConfiguration("steer_reduction_high"), value_type=float),
                "steer_reduction_very_high": ParameterValue(LaunchConfiguration("steer_reduction_very_high"), value_type=float),
                "steer_speed_cap_mid_rpm": ParameterValue(LaunchConfiguration("steer_speed_cap_mid_rpm"), value_type=float),
                "steer_speed_cap_high_rpm": ParameterValue(LaunchConfiguration("steer_speed_cap_high_rpm"), value_type=float),
                "steer_speed_cap_full_rpm": ParameterValue(LaunchConfiguration("steer_speed_cap_full_rpm"), value_type=float),
                "turn_strategy_command_period_s": ParameterValue(LaunchConfiguration("turn_strategy_command_period_s"), value_type=float),
                "turn_strategy_start_threshold": ParameterValue(LaunchConfiguration("turn_strategy_start_threshold"), value_type=float),
                "turn_strategy_release_threshold": ParameterValue(LaunchConfiguration("turn_strategy_release_threshold"), value_type=float),
                "wide_turn_enabled": _bool_param("wide_turn_enabled"),
                "wide_turn_min_inner_ratio": ParameterValue(LaunchConfiguration("wide_turn_min_inner_ratio"), value_type=float),
                "tight_turn_enabled": _bool_param("tight_turn_enabled"),
                "tight_turn_start_steer": ParameterValue(LaunchConfiguration("tight_turn_start_steer"), value_type=float),
                "tight_turn_full_steer": ParameterValue(LaunchConfiguration("tight_turn_full_steer"), value_type=float),
                "tight_turn_min_inner_ratio": ParameterValue(LaunchConfiguration("tight_turn_min_inner_ratio"), value_type=float),
                "tight_turn_outer_speed_cap_rpm": ParameterValue(LaunchConfiguration("tight_turn_outer_speed_cap_rpm"), value_type=float),
                "tight_turn_ramp_s": ParameterValue(LaunchConfiguration("tight_turn_ramp_s"), value_type=float),
                "tight_turn_only_with_pulse": _bool_param("tight_turn_only_with_pulse"),
                "turn_entry_enabled": _bool_param("turn_entry_enabled"),
                "turn_entry_speed_cap_rpm": ParameterValue(LaunchConfiguration("turn_entry_speed_cap_rpm"), value_type=float),
                "turn_entry_hold_s": ParameterValue(LaunchConfiguration("turn_entry_hold_s"), value_type=float),
                "turn_entry_ramp_s": ParameterValue(LaunchConfiguration("turn_entry_ramp_s"), value_type=float),
                "pulse_turn_enabled": _bool_param("pulse_turn_enabled"),
                "pulse_turn_on_s": ParameterValue(LaunchConfiguration("pulse_turn_on_s"), value_type=float),
                "pulse_turn_relief_s": ParameterValue(LaunchConfiguration("pulse_turn_relief_s"), value_type=float),
                "pulse_turn_relief_speed_cap_rpm": ParameterValue(LaunchConfiguration("pulse_turn_relief_speed_cap_rpm"), value_type=float),
                "pivot_pulse_enabled": _bool_param("pivot_pulse_enabled"),
                "pivot_pulse_rpm": ParameterValue(LaunchConfiguration("pivot_pulse_rpm"), value_type=float),
                "pivot_pulse_on_s": ParameterValue(LaunchConfiguration("pivot_pulse_on_s"), value_type=float),
                "pivot_pulse_relief_s": ParameterValue(LaunchConfiguration("pivot_pulse_relief_s"), value_type=float),
                "pivot_pulse_arrive_s": ParameterValue(LaunchConfiguration("pivot_pulse_arrive_s"), value_type=float),
                "turn_release_enabled": _bool_param("turn_release_enabled"),
                "turn_release_straight_s": ParameterValue(LaunchConfiguration("turn_release_straight_s"), value_type=float),
                "turn_release_speed_cap_rpm": ParameterValue(LaunchConfiguration("turn_release_speed_cap_rpm"), value_type=float),
                "turn_release_keep_drive_speed": _bool_param("turn_release_keep_drive_speed"),
                "counter_steer_enabled": _bool_param("counter_steer_enabled"),
                "counter_steer_ratio": ParameterValue(LaunchConfiguration("counter_steer_ratio"), value_type=float),
                "counter_steer_duration_s": ParameterValue(LaunchConfiguration("counter_steer_duration_s"), value_type=float),
                "counter_steer_speed_cap_rpm": ParameterValue(LaunchConfiguration("counter_steer_speed_cap_rpm"), value_type=float),
                "counter_steer_forward_only": _bool_param("counter_steer_forward_only"),
                "load_adaptive_turn_enabled": _bool_param("load_adaptive_turn_enabled"),
                "load_turn_unload_diff_scale": ParameterValue(LaunchConfiguration("load_turn_unload_diff_scale"), value_type=float),
                "load_turn_unload_speed_scale": ParameterValue(LaunchConfiguration("load_turn_unload_speed_scale"), value_type=float),
                "load_turn_local_stall_diff_scale": ParameterValue(LaunchConfiguration("load_turn_local_stall_diff_scale"), value_type=float),
                "load_turn_local_stall_speed_scale": ParameterValue(LaunchConfiguration("load_turn_local_stall_speed_scale"), value_type=float),
            }],
            condition=IfCondition(LaunchConfiguration("use_drive")),
        ),

        # ── [5] 탄도해 계산 ────────────────────────────────────
        Node(
            package="fire_solution",
            executable="fire_solution_node",
            name="fire_solution_node",
            output="screen",
            emulate_tty=True,
            parameters=[fire_cfg, {"dem_path": LaunchConfiguration("dem_path")}],
            condition=IfCondition(LaunchConfiguration("use_fire_solution")),
        ),

        # ── [6] 조준 상태 머신 ─────────────────────────────────
        Node(
            package="fire_solution",
            executable="aim_coordinator_node",
            name="aim_coordinator_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                fire_cfg,
                {
                    "align_fast_rpm": ParameterValue(
                        LaunchConfiguration("align_fast_rpm"), value_type=float
                    ),
                    "align_medium_rpm": ParameterValue(
                        LaunchConfiguration("align_medium_rpm"), value_type=float
                    ),
                    "align_slow_rpm": ParameterValue(
                        LaunchConfiguration("align_slow_rpm"), value_type=float
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_coordinator")),
        ),

        # ── [7] 포신/발사 STM 제어 ─────────────────────────────
        Node(
            package="fire_solution",
            executable="turret_control_node",
            name="turret_control_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                fire_cfg,
                {
                    "uart_enabled": _bool_param("use_stm"),
                    "serial_port": LaunchConfiguration("stm_port"),
                    "startup_uart_settle_delay_sec": ParameterValue(LaunchConfiguration("turret_startup_delay_sec"), value_type=float),
                    "terminal_input_enabled": False,
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_turret")),
        ),

        # ── [8] GUI ────────────────────────────────────────────
        Node(
            package="mmr_gui",
            executable="main_gui",
            name="main_gui",
            output="screen",
            emulate_tty=True,
            parameters=[fire_cfg],
            condition=IfCondition(LaunchConfiguration("use_gui")),
        ),
    ])