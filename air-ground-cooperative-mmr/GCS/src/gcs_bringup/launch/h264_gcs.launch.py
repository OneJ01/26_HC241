import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


PYTHON = "/home/leeguehoe/drone_gcs/.venv/bin/python3"


def generate_launch_description():
    pkg_share = get_package_share_directory("gcs_bringup")
    script_dir = os.path.join(pkg_share, "scripts")

    domain_id = LaunchConfiguration("domain_id")
    port = LaunchConfiguration("port")
    quality = LaunchConfiguration("quality")
    rate = LaunchConfiguration("rate")
    max_match_dt_ms = LaunchConfiguration("max_match_dt_ms")

    h264_bgr_bridge = os.path.join(script_dir, "h264_to_bgr_bridge.py")
    compute_logger = os.path.join(script_dir, "compute_logger.py")
    gcs_ui = os.path.join(script_dir, "gcs_ui.py")

    return LaunchDescription([
        # 기본 실행 옵션
        DeclareLaunchArgument("domain_id", default_value="42"),
        DeclareLaunchArgument("port", default_value="5600"),
        DeclareLaunchArgument("quality", default_value="80"),
        DeclareLaunchArgument("rate", default_value="30"),
        DeclareLaunchArgument("max_match_dt_ms", default_value="100"),

        # ROS domain
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),

        # PyQt/QWebEngine 안정화 옵션
        SetEnvironmentVariable(
            "QTWEBENGINE_CHROMIUM_FLAGS",
            "--disable-gpu --disable-gpu-compositing --disable-vulkan --disable-features=Vulkan"
        ),
        SetEnvironmentVariable("QT_OPENGL", "software"),

        ExecuteProcess(
            name="h264_to_bgr_bridge",
            cmd=[
                PYTHON,
                h264_bgr_bridge,
                "--port", port,
                "--rate", rate,
                "--max-match-dt-ms", max_match_dt_ms,
            ],
            output="screen",
        ),

        # 픽셀 역투영 / 좌표 계산 logger
        ExecuteProcess(
            name="compute_logger",
            cmd=[PYTHON, compute_logger],
            output="screen",
        ),

        # GCS UI
        ExecuteProcess(
            name="gcs_ui",
            cmd=[PYTHON, gcs_ui],
            output="screen",
        ),
    ])
