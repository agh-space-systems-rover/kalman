import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "map_path",
            description="Map file path",
        ),
        DeclareLaunchArgument(
            "is_sim",
            default_value="true",
        )]

    robot_nodes = create_robot_node()

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_slam_toolbox_node(package_name: str, is_sim: LaunchConfiguration, map_path: LaunchConfiguration) -> object:
    launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'online_async_launch.py'])
    params_file = PathJoinSubstitution(
        [FindPackageShare('kalman_slam2'), "config", "mapper_params_online_async.yaml"])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments={'use_sim_time': is_sim, 'slam_params_file': params_file, 'map_file_name': map_path}.items()
    )


def create_robot_node() -> list:
    """

    :rtype: list
    """
    map_path = LaunchConfiguration("map_path", default='/')
    # is_sim = LaunchConfiguration('is_sim')
    is_sim = 'True'
    package_name = 'auto_mapper'

    # auto_mapper = Node(
    #     package=package_name,
    #     executable="auto_mapper",
    #     name="auto_mapper",
    #     parameters=[
    #         {"map_path": map_path}
    #     ]
    # )

    navigation_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'bringup_launch.py'])
    package_share = FindPackageShare(package_name)
    rviz_config_file = PathJoinSubstitution(
        [package_share, "rviz", "config.rviz"]
    )
    return [
        GroupAction(
            actions=[
                # SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
                create_slam_toolbox_node(package_name, is_sim, map_path),
                # nav2_bringup,
                # auto_mapper,
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                    arguments=["-d", rviz_config_file]
                )
            ]
        )
    ]