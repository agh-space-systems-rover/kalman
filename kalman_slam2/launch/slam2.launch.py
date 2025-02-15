from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import jinja2
import yaml
import os


def load_local_ukf_config(rgbd_ids):
    # Load UKF config template
    with open(
        str(
            get_package_share_path("kalman_slam2")
            / "config"
            / "ukf_filter_node_local.yaml.j2"
        ),
        "r",
    ) as f:
        ukf_config_template = jinja2.Template(f.read())

    # Render UKF config with camera IDs
    ukf_config = ukf_config_template.render(
        rgbd_ids=rgbd_ids,
    )

    # Load UKF config
    ukf_params = yaml.load(ukf_config, Loader=yaml.FullLoader)

    # Save UKF config to file
    ukf_params_path = "/tmp/kalman/ukf_filter_node_local." + str(os.getpid()) + ".yaml"
    os.makedirs(os.path.dirname(ukf_params_path), exist_ok=True)
    with open(ukf_params_path, "w") as f:
        yaml.dump(ukf_params, f)

    # Return path to UKF config
    return ukf_params_path


def launch_setup(context):
    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]
    description = []

    description += [
        Node(
            namespace=f"{camera_id}",
            package="rtabmap_odom",
            executable="rgbd_odometry",
            parameters=[
                str(
                    get_package_share_path("kalman_slam2")
                    / "config"
                    / "rgbd_odometry.yaml"
                )
            ],
            remappings=[
                ("rgb/image", f"color/image_raw"),
                ("depth/image", f"aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", f"color/camera_info"),
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )
        for camera_id in rgbd_ids
    ]

    description += [
        Node(
            package="robot_localization",
            executable="ukf_node",
            name="ukf_filter_node",
            parameters=[load_local_ukf_config(rgbd_ids)],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
    ]
    return description


def create_slam_toolbox_node(
    package_name: str, is_sim: LaunchConfiguration, map_path: LaunchConfiguration
) -> object:
    launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), "launch", "online_async_launch.py"]
    )
    params_file = PathJoinSubstitution(
        [FindPackageShare("kalman_slam2"), "config", "mapper_params_online_async.yaml"]
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments={
            "use_sim_time": is_sim,
            "slam_params_file": params_file,
            "map_file_name": map_path,
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            OpaqueFunction(function=launch_setup),
            Node(
                package="kalman_slam2",
                executable="local_maxima_node",
                remappings=[("/input_cloud", "/point_cloud")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "-0.30",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "lidar_link",
                ],
            ),
            Node(
                package="kalman_slam2",
                executable="pointcloud_concat",
                parameters=[{"/target_frame": "lidar_link", "/clouds": 4, "/hz": 10.0}],
                remappings=[
                    ("cloud_in1", "/d455_front/point_cloud"),
                    ("cloud_in2", "/d455_back/point_cloud"),
                    ("cloud_in3", "/d455_left/point_cloud"),
                    ("cloud_in4", "/d455_right/point_cloud"),
                    ("cloud_out", "/point_cloud"),
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                parameters=[{"use_inf": True, "scan_time": 0.1}],
                remappings=[("cloud_in", "/filtered_cloud"), ("/scan", "/scan_raw")],
            ),
            Node(package="kalman_slam2", executable="laser_filter_node"),
            create_slam_toolbox_node('slam_toolbox', 'False', '/'),
        ]
    )
