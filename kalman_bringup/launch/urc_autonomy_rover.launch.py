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
                    # "component_container": "true",
                    "description": "true",
                    "hardware": "true",
                    "hardware.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "hardware.master": "true",
                    "hardware.master.mode": "pc",
                    "hardware.imu": "true",
                    "hardware.gps": "true",
                    "clouds": "true",
                    "clouds.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "slam": "true",
                    "slam.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "nav2": "true",
                    "nav2.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "wheels": "true",
                    "aruco": "true",
                    "aruco.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "aruco.dict": "4X4_50",
                    "aruco.size": "0.15",
                    "yolo": "true",
                    "yolo.rgbd_ids": "d455_front d455_back d455_left d455_right",
                    "yolo.config": "urc2024",
                    "supervisor": "true",
                    "supervisor.deactivate_aruco": "true",
                }.items(),
            ),
        ]
    )
