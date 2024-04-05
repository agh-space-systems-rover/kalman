from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        get_package_share_path("kalman_bringup")
                        / "launch"
                        / "_commons.launch.py"
                    )
                ),
                launch_arguments={
                    "drivers": "true",
                    "drivers.composition": "false",
                    "drivers.rgbd_ids": "",
                    "drivers.master": "false",
                    "drivers.imu": "false",
                    "drivers.compasscal": "false",
                    "drivers.gps": "true",
                }.items(),
            ),
        ]
    )
