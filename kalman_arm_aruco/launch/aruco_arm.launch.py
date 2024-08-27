from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from ament_index_python import get_package_share_path
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
import xacro

REALSENSE_SERIAL_NUMBERS = {
   
    "d435_arm": "_936322070029",
}
   
def launch_setup(context):
    component_container = LaunchConfiguration("component_container").perform(context)

    description = []

    rgbd_ids = [
        x
        for x in LaunchConfiguration("rgbd_ids").perform(context).split(" ")
        if x != ""
    ]

    if len(rgbd_ids) > 0:
        rgbd_ids_sns = [
            (x, REALSENSE_SERIAL_NUMBERS[x])
            for x in rgbd_ids
            if x in REALSENSE_SERIAL_NUMBERS
        ]
    
    if len(rgbd_ids) > 0:

        description += [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(
                    get_package_share_path("realsense2_camera") / "launch" / "rs_launch.py"
                )
            ),
            launch_arguments={
                "camera_name": camera_name,
                "camera_namespace": "",
                "serial_no": serial_no,
                "config_file": str(  # Must use external config file for non-configurable options.
                    get_package_share_path("arm_aruco_detection")
                    / "config"
                    / "realsense2_camera.yaml"
                    ),
                }.items(),
            )
            for camera_name, serial_no in rgbd_ids_sns
        ]
    parameters = [
        str(get_package_share_path("arm_aruco_detection") / "config" / f"aruco_tracker.yaml")
    ]

    if component_container:
        description += [
            LoadComposableNodes(
                target_container=component_container,
                composable_node_descriptions=[
                    ComposableNode(
                        package="aruco_opencv",
                        plugin="aruco_opencv::ArucoTrackerAutostart",
                        namespace=rgbd_id,
                        name="aruco_tracker",
                        parameters=parameters,
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                    for rgbd_id in rgbd_ids
                ],
            )
        ]
    else:
        description += [
            Node(
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                namespace=rgbd_id,
                name="aruco_tracker",
                parameters=parameters,
            )
            for rgbd_id in rgbd_ids
        ]

    
    description += [
        Node(
            package='arm_aruco_detection',
            executable='marker_publisher',
            name='marker_publisher'
            
        ),
    ]
    description += [
        Node(
            package='arm_aruco_detection',
            executable='marker_broadcaster',
            name='marker_broadcaster'
            
        ),
    ]
    description += [
        Node(
            package='arm_aruco_detection',
            executable='panel_node',
            name='panel_node'
            
        ),
    ]
    return description

def generate_launch_description(): 
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty by default to disable composition.",
            ),
            DeclareLaunchArgument(
                "rgbd_ids",
                default_value="d435_arm",
                description="Space-separated IDs of the depth cameras to use.",
            ),
            

            OpaqueFunction(function=launch_setup)
        ]
    )
  