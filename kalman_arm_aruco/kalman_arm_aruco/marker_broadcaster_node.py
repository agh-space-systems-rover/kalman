from launch_ros.actions import Node
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class marker_broadcaster(Node):

    def __init__(self):
        super().__init__('marker_frame_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_subscription = self.create_subscription(TransformStamped, '/d435_arm_frames', self.marker_broadcast, 10)
        self.marker_subscription
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def marker_broadcast(self, msg: TransformStamped):
        # self.tf_broadcaster.sendTransform(msg)
        first_frame = 'world'
        second_frame = msg.child_frame_id
        self.tf_buffer.set_transform(msg, 'dupa')
        print(f"Set tf: {msg.child_frame_id}, {msg.header.frame_id}")
        try:
            trans = self.tf_buffer.lookup_transform(first_frame, second_frame, rclpy.time.Time())
            ts_stamped = TransformStamped()
            ts_stamped.transform = trans.transform
            ts_stamped.child_frame_id = second_frame
            ts_stamped.header.frame_id = first_frame
            self.tf_broadcaster.sendTransform(ts_stamped)
        except Exception as e:
            self.get_logger().error('Failed to get transform {}\n'.format(repr(e)))
        

def main():
    try:
        rclpy.init()
        node = marker_broadcaster()
        rclpy.spin(node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass