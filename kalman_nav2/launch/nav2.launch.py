from ament_index_python import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
import yaml
import os
import jinja2


def launch_setup(context):
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]

    # Obstacle detection
    description = [
        Node(
            package="obstacle_detection",
            executable="obstacle_detection",
            remappings={
                "input": f"/{camera_id}/depth/color/points/filtered",
                "output": f"/{camera_id}/depth/color/points/filtered/obstacles",
            }.items(),
        )
        for camera_id in rgbd_ids
    ]

    # Load Nav2 config template
    with open(
        str(get_package_share_path("kalman_nav2") / "param" / "nav2.yaml.j2"),
    ) as f:
        nav2_config_template = jinja2.Template(f.read())

    # Render config using camera IDs
    ukf_config_str = nav2_config_template.render(
        rgbd_ids=rgbd_ids,
    )

    # Load config
    nav2_params = yaml.load(ukf_config_str, Loader=yaml.FullLoader)

    # Save it to file
    nav2_params_path = "/tmp/kalman/nav2." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(nav2_params_path), exist_ok=True)
    with open(nav2_params_path, "w") as f:
        yaml.dump(nav2_params, f)

    description += [
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("kalman_nav2")
                    / "launch"
                    / "navigation_launch.launch.py"
                )
            ),
            launch_arguments={
                "params_file": nav2_params_path,
            }.items(),
        ),
        # path follower
        Node(
            package="kalman_nav2",
            executable="path_follower",
            parameters=[
                str(
                    get_package_share_path("kalman_nav2")
                    / "param"
                    / "path_follower.yaml"
                ),
            ],
        ),
    ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="d455_front d455_back d455_left d455_right",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
