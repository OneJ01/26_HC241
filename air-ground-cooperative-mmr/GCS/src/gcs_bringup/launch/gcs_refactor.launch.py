import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


VENV_PYTHON = "/home/leeguehoe/drone_gcs/.venv/bin/python3"


def generate_launch_description():
    pkg_share = get_package_share_directory("gcs_bringup")
    scripts_dir = os.path.join(pkg_share, "scripts")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),

        ExecuteProcess(
            cmd=[
                VENV_PYTHON,
                os.path.join(scripts_dir, "compute_logger.py"),
            ],
            cwd=scripts_dir,
            name="compute_logger",
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                VENV_PYTHON,
                os.path.join(scripts_dir, "gcs_ui.py"),
            ],
            cwd=scripts_dir,
            name="gcs_ui",
            output="screen",
        ),
    ])
