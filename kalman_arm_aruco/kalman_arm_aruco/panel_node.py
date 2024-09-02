

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from visualization_msgs.msg import Marker, MarkerArray
from typing import Tuple, Iterable
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformListener, Buffer, TransformStamped


def build_transform(parent_id: str,
                    child_id: str,
                    pos: Tuple[float, float, float],
                    quat: Tuple[float, float, float, float]) \
        -> TransformStamped:
    t = TransformStamped()
    # t.header.stamp = Time().to_msg()
    t.header.frame_id = parent_id
    t.child_frame_id = child_id
    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


def build_cube_marker(frame_id: str,
                      position: Tuple[float, float, float],
                      orientation: Tuple[float, float, float, float],
                      scale: Tuple[float, float, float],
                      color: Tuple[float, float, float, float]) \
        -> Marker:
    marker = Marker()

    marker.header.frame_id = frame_id
    # Note that the timestamp attached to the marker message above is
    # ros::Time(), which is time Zero (0). This is treated differently by RViz
    # than any other time. If you use ros::Time::now() or any other non-zero
    # value, rviz will only display the marker if that time is close enough to
    # the current time, where "close enough" depends on TF. With time 0
    # however, the marker will be displayed regardless of the current time.
    marker.header.stamp = Time().to_msg()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.frame_locked = True
    marker.lifetime = Duration(seconds=1.0).to_msg()

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    id = hash((frame_id, *position, *orientation, *scale, *color)) % 1000000
    marker.id = id

    return marker


def build_cylinder_marker(frame_id: str,
                          position: Tuple[float, float, float],
                          orientation: Tuple[float, float, float, float],
                          scale: Tuple[float, float, float],
                          color: Tuple[float, float, float, float]) \
        -> Marker:
    marker = build_cube_marker(frame_id, position, orientation, scale, color)
    marker.type = Marker.CYLINDER
    return marker


# Tag frames
TAG_PANEL_A_FRAME = 'tag_11'
TAG_PANEL_B1_TOP_FRAME = 'tag_14'
TAG_PANEL_B1_BOTTOM_FRAME = 'tag_15'
TAG_PANEL_B2_FRAME = 'tag_13'

# Marker colors
COLOR_PANEL = (0.7, 0.7, 0.8, 0.7)
COLOR_SWITCH = (1.0, 1.0, 1.0, 1.0)
COLOR_TAG = (0.0, 0.0, 0.0, 1.0)
COLOR_LOCK = (0.5, 0.5, 0.0, 1.0)
COLOR_RJ45 = (0.0, 0.0, 1.0, 1.0)

# Marker positions
COLOR_SOCKET = (1.0, 0.2, 0.2, 1.0)
POS_ZERO = (0.0, 0.0, 0.0)
POS_SWITCH = (0.0, -0.01, 0.0)
SCALE_SWITCH = (0.05, 0.01, 0.05)
SCALE_SWITCH_SMALL = (0.032, 0.01, 0.032)

# TF data
SWITCH_W = 0.05
SWITCH_H = 0.05
SWITCH_TO_SWITCH_DISTANCE_H = 0.084
SWITCH_TO_SWITCH_DISTANCE_V = 0.071
TOP_TAG_TO_B1_H = -0.062
TOP_TAG_TO_B1_V = -0.0355
BOTTOM_TAG_TO_B1_H = -0.062
BOTTOM_TAG_TO_B1_V = 0.0355
RJ45_OFFSET = -0.0125
B1_TO_LOCK_H = -0.1235
B1_TO_LOCK_V = 0.0
DEFAULT_ORIENT = (0.0, 0.0, 0.0, 1.0)
ROTATION_ORIENT = (0.0, -0.7071, 0.7071, 0.0)


PANEL_TRANSFORMS = [
    # PANEL A
    build_transform(TAG_PANEL_A_FRAME, 'tag_panel_a',
                    (0.0, 0.0, 0.0),
                    ROTATION_ORIENT),
    build_transform('tag_panel_a', 'panel_a',
                    (SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     -SWITCH_TO_SWITCH_DISTANCE_V),
                    DEFAULT_ORIENT),
    build_transform('panel_a', 'switch_0',
                    (SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     SWITCH_TO_SWITCH_DISTANCE_V),
                    DEFAULT_ORIENT),
    build_transform('panel_a', 'switch_1',
                    (-SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     0.0),
                    DEFAULT_ORIENT),
    build_transform('panel_a', 'switch_2',
                    (SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     0.0),
                    DEFAULT_ORIENT),
    build_transform('panel_a', 'switch_3',
                    (-SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     -SWITCH_TO_SWITCH_DISTANCE_V),
                    DEFAULT_ORIENT),
    build_transform('panel_a', 'switch_4',
                    (SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     -SWITCH_TO_SWITCH_DISTANCE_V),
                    DEFAULT_ORIENT),
    # PANEL B1
    build_transform(TAG_PANEL_B1_TOP_FRAME, 'tag_panel_b1',
                    (0.0,0.0,0.0),
                    ROTATION_ORIENT),
    build_transform('tag_panel_b1', 'panel_b1',
                    (TOP_TAG_TO_B1_H,
                     0.0,
                     TOP_TAG_TO_B1_V),
                    DEFAULT_ORIENT),
    build_transform('panel_b1', 'switch_power',
                    (0.0,
                     0.0,
                     SWITCH_TO_SWITCH_DISTANCE_V / 2),
                    DEFAULT_ORIENT),
    build_transform('panel_b1', 'socket',
                    (0.0,
                     0.0,
                     -SWITCH_TO_SWITCH_DISTANCE_V / 2),
                    DEFAULT_ORIENT),
    build_transform('panel_b1', 'lock',
                    (B1_TO_LOCK_H,
                     0.0,
                     B1_TO_LOCK_V),
                    DEFAULT_ORIENT),
    # PANEL B2
    build_transform(TAG_PANEL_B2_FRAME, 'tag_panel_b2',
                    (0.0, 0.0, 0.0),
                    ROTATION_ORIENT),
    build_transform('tag_panel_b2', 'panel_b2',
                    (SWITCH_TO_SWITCH_DISTANCE_H / 2,
                     0.0,
                     -SWITCH_TO_SWITCH_DISTANCE_V),
                    DEFAULT_ORIENT),
    build_transform('panel_b2', 'rj45',
                    (RJ45_OFFSET,
                     0.0,
                     SWITCH_TO_SWITCH_DISTANCE_V / 2),
                    DEFAULT_ORIENT),

]


PANEL_MARKERS = [
    # PANEL A
    build_cube_marker('panel_a', POS_ZERO, DEFAULT_ORIENT,
                      (0.35, 0.01, 0.45), COLOR_PANEL),
    build_cube_marker('panel_b1', POS_ZERO, DEFAULT_ORIENT,
                      (0.25, 0.01, 0.45), COLOR_PANEL),
    build_cube_marker('panel_b2', POS_ZERO, DEFAULT_ORIENT,
                      (0.25, 0.01, 0.45), COLOR_PANEL),
    build_cube_marker('switch_0', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_SWITCH),
    build_cube_marker('switch_1', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_SWITCH),
    build_cube_marker('switch_2', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_SWITCH),
    build_cube_marker('switch_3', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_SWITCH),
    build_cube_marker('switch_4', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_SWITCH),
    build_cube_marker('tag_panel_a', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_TAG),
    # PANEL B1
    build_cube_marker('switch_power', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_SWITCH),
    build_cube_marker('tag_panel_b1', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH_SMALL, COLOR_TAG),
    build_cylinder_marker("socket", POS_SWITCH, (0.71, 0.0, 0.0, 0.71),
                          (0.05, 0.05, 0.01), COLOR_SOCKET),
    build_cube_marker('lock', POS_ZERO, DEFAULT_ORIENT,
                      (0.005, 0.05, 0.157), COLOR_LOCK),
    # PANEL B2
    build_cube_marker('tag_panel_b2', POS_SWITCH, DEFAULT_ORIENT,
                      SCALE_SWITCH, COLOR_TAG),
    build_cube_marker("rj45", POS_SWITCH, DEFAULT_ORIENT,
                      (0.0125, 0.01, 0.0125), COLOR_RJ45),
]


class ErcPanelVis(Node):
    def __init__(self):
        super().__init__('erc_panel_vis')
        self.marker_pub = self.create_publisher(
            MarkerArray, 'erc_panel_markers', 10)

        self.timer_period = 1
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.publish_transforms(PANEL_TRANSFORMS)

    def timer_callback(self) -> None:
        self.publish_markers(PANEL_MARKERS)

    def publish_markers(self, markers: Iterable[Marker]) -> None:
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub.publish(marker_array)

    def update_transform_time(self, transform: TransformStamped) -> None:
        transform.header.stamp = self.get_clock().now().to_msg()

    def publish_transforms(self,
                           transforms: Iterable[TransformStamped]) \
            -> None:
        for transform in transforms:
            self.update_transform_time(transform)

        self.tf_static_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    marker_publisher_node = ErcPanelVis()

    try:
        rclpy.spin(marker_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        marker_publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
