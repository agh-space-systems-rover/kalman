from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path

REALSENSE_CAMERAS = ["d455_front"]


def launch_setup(context):
    description = []
    for camera in REALSENSE_CAMERAS:
        description += [
            Node(
                package="kalman_blocks",
                executable="blocks_detection",
                parameters=[
                    {
                        "camera_no": camera,
                    }
                ]
            )
        ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_no",
                default_value="",
                description="Each name of realsense camera by the topic"
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
