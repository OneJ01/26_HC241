#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gnss_stable_launch.py
────────────────────────────────────────────────────────────────────────────
MMR 듀얼 ZED-F9P 안정 운용 런치 파일.

검증된 실제 운용 구조:
  1. Lite 보드(Moving Base/Base)
     - 센서 GGA 출력
     - PC/미니PC에서 NTRIP caster 접속
     - 수신한 RTCM을 Lite 보드로 직접 주입

  2. 큰 보드(Rover/Heading 출력 보드)
     - Lite가 만든 Moving Base RTCM만 수신
     - UBX-NAV-HPPOSLLH: 큰 보드 고정밀 위경도
     - UBX-NAV-RELPOSNED: heading / baseline

발행 주요 토픽:
  /gnss/fix_hp         sensor_msgs/NavSatFix
  /fix                 sensor_msgs/NavSatFix
  /gnss/fix_pvt        sensor_msgs/NavSatFix
  /gnss/heading        std_msgs/Float64
  /gnss/relposned_vec  geometry_msgs/Vector3
  /gnss/rover/status   std_msgs/String(JSON)
  /gnss/ntrip_status   std_msgs/String(JSON)

실행 예:
  ros2 launch gnss_driver gnss_stable_launch.py
  ros2 launch gnss_driver gnss_stable_launch.py lite_port:=/dev/ttyACM0 rover_port:=/dev/ttyACM1
  ros2 launch gnss_driver gnss_stable_launch.py use_ntrip:=false

주의:
  큰 보드 Rover에는 외부 VRS RTCM을 직접 넣지 마십시오.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("gnss_driver"),
        "config",
        "gnss_stable.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lite_port",
            default_value="/dev/ttyACM0",
            description="Lite 보드(Moving Base/Base) serial port",
        ),
        DeclareLaunchArgument(
            "rover_port",
            default_value="/dev/ttyACM1",
            description="큰 보드(Rover/Heading) serial port",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="ZED-F9P serial baudrate",
        ),
        DeclareLaunchArgument(
            "use_ntrip",
            default_value="true",
            description="Lite 보드 NTRIP 주입 노드 실행 여부",
        ),
        DeclareLaunchArgument(
            "ntrip_authenticate",
            default_value=os.environ.get("MMR_NTRIP_AUTHENTICATE", "false"),
            description="NTRIP Basic 인증 사용 여부. 환경변수 MMR_NTRIP_AUTHENTICATE로 기본값 지정 가능",
        ),
        DeclareLaunchArgument(
            "ntrip_username",
            default_value=os.environ.get("MMR_NTRIP_USERNAME", ""),
            description="NTRIP username. 환경변수 MMR_NTRIP_USERNAME으로 기본값 지정 가능",
        ),
        DeclareLaunchArgument(
            "ntrip_password",
            default_value=os.environ.get("MMR_NTRIP_PASSWORD", ""),
            description="NTRIP password. 환경변수 MMR_NTRIP_PASSWORD로 기본값 지정 가능",
        ),

        # Lite 보드 NTRIP 주입 노드
        Node(
            package="gnss_driver",
            executable="ntrip_lite",
            name="ntrip_lite_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                cfg,
                {
                    "serial_port": LaunchConfiguration("lite_port"),
                    "baudrate": LaunchConfiguration("baudrate"),
                    "authenticate": PythonExpression([
                        "'", LaunchConfiguration("ntrip_authenticate"), "'.lower() in ['true', '1', 'yes', 'on']"
                    ]),
                    "username": LaunchConfiguration("ntrip_username"),
                    "password": LaunchConfiguration("ntrip_password"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_ntrip")),
        ),

        # 큰 보드 Rover UBX 파싱 노드
        Node(
            package="gnss_driver",
            executable="ubx_rover",
            name="ubx_rover_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                cfg,
                {
                    "serial_port": LaunchConfiguration("rover_port"),
                    "baudrate": LaunchConfiguration("baudrate"),
                },
            ],
        ),
    ])
