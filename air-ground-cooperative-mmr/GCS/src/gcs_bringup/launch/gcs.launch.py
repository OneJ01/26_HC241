import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    drone_gcs_dir = '/home/leeguehoe/drone_gcs'
    python_exec = '/home/leeguehoe/drone_gcs/.venv/bin/python'

    # H.264 수신/BGR 변환 브릿지 위치
    h264_gcs_scripts_dir = '/home/leeguehoe/h264_gcs_ws/src/gcs_bringup/scripts'

    common_env = {
        'ROS_DOMAIN_ID': '42',
        'ROS_LOCALHOST_ONLY': '0',
    }

    return LaunchDescription([
        # 1. H.264 UDP 수신 → BGR ROS 토픽 변환
        ExecuteProcess(
            cmd=[
                python_exec,
                'h264_to_bgr_bridge.py',
                '--port',
                '5600',
            ],
            cwd=h264_gcs_scripts_dir,
            name='h264_to_bgr_bridge',
            output='screen',
            additional_env=common_env
        ),

        # 2. 좌표변환 / 계산 노드
        ExecuteProcess(
            cmd=[python_exec, 'compute_logger.py'],
            cwd=drone_gcs_dir,
            name='compute_logger',
            output='screen',
            additional_env=common_env
        ),

        # 3. GCS UI
        ExecuteProcess(
            cmd=[python_exec, 'ui.py'],
            cwd=drone_gcs_dir,
            name='gcs_ui',
            output='screen',
            additional_env=common_env
        ),
    ])