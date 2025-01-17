# AUTO-GENERATED FILE. DO NOT EDIT.
# SEE: kalman_bringup/tools/gen_type_hints

from typing import Literal, TypedDict


class Yolo(TypedDict):
    rgbd_ids: str
    "Space-separated IDs of the depth cameras to use."
    config: Literal["urc2024"]
    "name of the configuration to load. Valid choices are: {'urc2024'}"


class UnitySim(TypedDict):
    scene: str
    "The scene to load in Unity."


class Nav2(TypedDict):
    component_container: str
    "Name of an existing component container to use. Empty by default to disable composition."
    rgbd_ids: str
    "Space-separated IDs of the depth cameras to use."
    static_map: Literal["", "erc2023", "erc2024"]
    "Name of the static map to use. Maps are stored in kalman_nav2/maps. Empty by default to disable static map. Valid choices are: ['', 'erc2024', 'erc2023']"


class Slam(TypedDict):
    component_container: str
    "Name of an existing component container to use. Empty to disable composition."
    rgbd_ids: str
    "Space-separated IDs of the depth cameras to use."
    gps_datum: str
    "The 'latitude longitude' of the map frame. Only used if GPS is enabled. Empty to assume first recorded GPS fix."
    fiducials: Literal["", "erc2024", "terc2024"]
    "Name of the list of fiducials to use. Empty disables fiducial odometry. Valid choices are: ['', 'erc2024', 'terc2024']"
    use_mag: Literal["false", "true"]
    "Use IMU yaw readings for global EKF. If disabled, heading will drift over time. Valid choices are: ['true', 'false']"


class Gs(TypedDict):
    pass


class Clouds(TypedDict):
    component_container: str
    "Name of an existing component container to use. Empty to disable composition."
    rgbd_ids: str
    "Space-separated IDs of the depth cameras to use for point cloud generation."


class Wheels(TypedDict):
    joy: Literal["", "arduino", "gamepad"]
    "Joy device to use for headless driving. Choose 'gamepad' or 'arduino'. Empty disables headless teleop. Valid choices are: ['', 'gamepad', 'arduino']"


class Supervisor(TypedDict):
    aruco_rgbd_ids: str
    "Space-separated IDs of the depth cameras that were configured in kalman_aruco."
    aruco_deactivate_unused: Literal["false", "true"]
    "Deactivate ArUco detection nodes when supervisor is not actively looking for tags. Valid choices are: ['true', 'false']"
    yolo_enabled: Literal["false", "true"]
    "Whether YOLO detection is enabled. Valid choices are: ['true', 'false']"
    yolo_deactivate_unused: Literal["false", "true"]
    "Deactivate YOLO detection when supervisor is not actively looking for objects. Valid choices are: ['true', 'false']"


class Spacenav(TypedDict):
    pass


class Rviz(TypedDict):
    configs: str
    "Space separated RViz configuration file names without extensions, e.g. 'autonomy demo_rgbd'"


class Master(TypedDict):
    mode: Literal["arm", "gs", "pc"]
    "On what hardware is this module being run? Available modes: gs, pc, arm. Valid choices are: ['gs', 'pc', 'arm']"


class ArmGs(TypedDict):
    pass


class Hardware(TypedDict):
    component_container: str
    "Name of an existing component container to use. Empty by default to disable composition."
    master: Literal["", "arm", "gs", "pc"]
    "Start the master driver in a given mode ('pc', 'gs' or 'arm'). Leave empty to disable. Valid choices are: ['', 'pc', 'gs', 'arm']"
    rgbd_ids: str
    "Space-separated IDs of the depth cameras to use (d455_front, d455_back, ...). Leave empty to disable the cameras."
    imu: Literal["", "full", "no_mag"]
    "Start IMU. 'no_mag' disables magnetometer. Valid choices are: ['', 'no_mag', 'full']"
    compass_calibration: str
    "Start IMU compass calibration node for a given number of seconds. IMU must be disabled in order to calibrate the compass. Zero to run in normal mode, without calibration."
    gps: Literal["false", "true"]
    "Start the GPS driver. Valid choices are: ['true', 'false']"


class Description(TypedDict):
    layout: Literal["arm", "autonomy"]
    "layout of the robot: autonomy, arm. Valid choices are: ['autonomy', 'arm']"
    joint_state_publisher_gui: Literal["false", "true"]
    "Start the joint state publisher in GUI mode. Valid choices are: ['true', 'false']"


class Aruco(TypedDict):
    component_container: str
    "Name of an existing component container to use. Empty by default to disable composition."
    rgbd_ids: str
    "Space-separated IDs of the RGBD cameras to use. Regular cameras are not supported yet."
    dict: Literal["4X4_100", "4X4_1000", "4X4_250", "4X4_50", "5X5_100", "5X5_1000", "5X5_250", "5X5_50", "6X6_100", "6X6_1000", "6X6_250", "6X6_50", "7X7_100", "7X7_1000", "7X7_250", "7X7_50", "8X8_100", "8X8_1000", "8X8_250", "8X8_50", "ARUCO_ORIGINAL"]
    "Dictionary of markers to use. Valid choices are: ['4X4_50', '4X4_100', '4X4_250', '4X4_1000', '5X5_50', '5X5_100', '5X5_250', '5X5_1000', '6X6_50', '6X6_100', '6X6_250', '6X6_1000', '7X7_50', '7X7_100', '7X7_250', '7X7_1000', '8X8_50', '8X8_100', '8X8_250', '8X8_1000', 'ARUCO_ORIGINAL']"
    size: str
    "Size of the marker including black border in meters."


class Arm(TypedDict):
    debug: Literal["False", "True", "false", "true"]
    "One of: ['true', 'false', 'True', 'False']"
    allow_trajectory_execution: Literal["False", "True", "false", "true"]
    "One of: ['true', 'false', 'True', 'False']"
    publish_monitored_planning_scene: Literal["False", "True", "false", "true"]
    "One of: ['true', 'false', 'True', 'False']"
    monitor_dynamics: Literal["False", "True", "false", "true"]
    "One of: ['true', 'false', 'True', 'False']"


class BringupConfig(TypedDict):
    yolo: Yolo
    "models and configs for `yolo_ros`"
    unity_sim: UnitySim
    nav2: Nav2
    "configuration and launch files for Nav2 and related modules; Includes a custom path follower."
    slam: Slam
    "configuration + launch files for robot_localization and RTAB-Map"
    gs: Gs
    "ReactJS-based GUI for the rover"
    clouds: Clouds
    "point cloud generation and filtering"
    wheels: Wheels
    "controller node that converts Twist messages on `/cmd_vel` and similar topics to the actual wheel state; Also includes safeguards that can limit the acceleration and velocity or stop the rover to adjust wheel rotation."
    supervisor: Supervisor
    "Manages autonomous navigation missions."
    spacenav: Spacenav
    "Launch files and scripts for the SpaceMouse."
    rviz: Rviz
    "Launch files and configs for RViz"
    master: Master
    "driver for our custom Master device"
    arm_gs: ArmGs
    "scripts for gs site for interfacing with arm"
    hardware: Hardware
    "drivers, tools and launch scripts for the physical hardware onboard; Only to be run separately from the simulation on a physical robot."
    description: Description
    "Xacro / URDF descriptions + models for the rover"
    aruco: Aruco
    "pre-configured setup for aruco_opencv"
    arm: Arm
    "configuration and launch files for the arm"
