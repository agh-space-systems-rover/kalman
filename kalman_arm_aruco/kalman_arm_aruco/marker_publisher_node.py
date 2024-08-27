from launch_ros.actions import Node
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose


class marker_broadcaster(Node):

    def __init__(self):
        super().__init__('marker_frame_publisher')

        self.marker_subscription = self.create_subscription(ArucoDetection, '/d435_arm/aruco_detections', self.marker_pose_to_base_link, 10)
        self.marker_subscription

        self.marker_publisher = self.create_publisher(TransformStamped, 'd435_arm_frames', 10)

    def marker_pose_to_base_link(self, msg: ArucoDetection):
        MarkerTransform = TransformStamped()
        MarkerTransform.header.stamp = self.get_clock().now().to_msg()

        MarkerTransform.header.frame_id = 'd435_arm'
        markers: list[MarkerPose] = msg.markers

        for marker in markers:
            MarkerTransform.child_frame_id = f"Marker{marker.marker_id}"

            MarkerTransform.transform.translation.x = marker.pose.position.x
            MarkerTransform.transform.translation.y = marker.pose.position.y
            MarkerTransform.transform.translation.z = marker.pose.position.z

            MarkerTransform.transform.rotation.x = marker.pose.orientation.x
            MarkerTransform.transform.rotation.y = marker.pose.orientation.y
            MarkerTransform.transform.rotation.z = marker.pose.orientation.z
            MarkerTransform.transform.rotation.w = marker.pose.orientation.w

            self.marker_publisher.publish(MarkerTransform)


def main():
    try:
        rclpy.init()
        node = marker_broadcaster()
        rclpy.spin(node)
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

        
        


            

    
    

