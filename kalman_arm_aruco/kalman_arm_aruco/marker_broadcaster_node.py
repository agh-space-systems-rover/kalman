from launch_ros.actions import Node
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class marker_broadcaster(Node):

    def __init__(self):
        super().__init__('marker_frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_subscription = self.create_subscription(TransformStamped, '/d435_arm_frames', self.marker_broadcast, 10)
        self.marker_subscription

    def marker_broadcast(self, msg: TransformStamped):
        self.tf_broadcaster.sendTransform(msg)

def main():
    try:
        rclpy.init()
        node = marker_broadcaster()
        rclpy.spin(node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass