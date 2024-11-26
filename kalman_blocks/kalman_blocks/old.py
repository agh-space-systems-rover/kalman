import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np
# red
# lower_red = np.array([160, 100, 100])
# upper_red = np.array([180, 255, 255])

color_ranges = {
    "red": {
        "lower": np.array([160, 100, 100]),
        "upper": np.array([180, 255, 255]),

    },
    # "green": {
    #     "lower": np.array([40, 50, 50]),
    #     "upper": np.array([80, 255, 255])
    # },
    # "blue": {
    #     "lower": np.array([100, 50, 50]),
    #     "upper": np.array([140, 255, 255])
    # },
    # "white": {
    #     "lower": np.array([0, 0, 200]),
    #     "upper": np.array([180, 20, 255])
    # }
}

K = np.array([
    [311.76, 0, 320],  # fx, 0 , cx
    [0, 311.76, 180],  # 0, fy, cy
    [0, 0, 1]
])  # square pixel fx = fy   !!! cx and cy depend of current settings


class ImageClass(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        # read params
        self.declare_parameter("cube_width", 0.1)  # [m]
        self.declare_parameter("image_width", 640)  # width in pixl
        self.declare_parameter("fov", 90)
        # subscribe the cameras
        self.bridge = CvBridge()
        self.create_subscription(
            Image, "/d455_front/color/image_raw", self.image_process_callback, 10)
        # write params
        self.cube_width = self.get_parameter("cube_width").value
        self.image_width = self.get_parameter("image_width").value
        self.fov = self.get_parameter("fov").value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.xBox = 0.1
        self.yBox = 0.1
        self.width = 0.1
        self.height = 0.1
        self.cube_id = "dupa"

    # def calculate_position(self, width_in_pixels, real_width, fov, image_width) -> float:
    #     fov_rad = np.deg2rad(fov)
    #     focal_length = image_width / (2 * np.tan(fov_rad / 2))
    #     return (real_width * focal_length) / width_in_pixels

    def camera_info_callback(self, camera_info: CameraInfo):
        tf = TransformStamped()
        bb_radius = ((self.width + self.height) / 2) / 2
        z = None
        z = (
            self.cube_width / bb_radius * camera_info.k[0]
        )
        x = z * ((self.xBox + self.width)/2 -
                 camera_info.k[2]) / camera_info.k[0]
        y = z * ((self.yBox + self.height)/2 -
                 camera_info.k[5]) / camera_info.k[4]
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'world'
        tf.child_frame_id = self.cube_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        self.tf_broadcaster.sendTransform(tf)

        # print(f"{self.cube_id} x: {x} \n y: {y} \n z: {z} ")

    def image_process_callback(self, msg: Image):   # add frame id
        image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
        # kernel = np.array([[0, -1, 0],
        #                    [-1, 5, -1],
        #                    [0, -1, 0]])
        kernel_erode = np.ones((2, 2), "uint8")
        kernel_dilate = np.ones((3, 3), "uint8")
        eroded = cv2.erode(hsv, kernel_erode, iterations=1)
        cv2.imshow("d", eroded)
        for color, ranges in color_ranges.items():
            lower = ranges["lower"]
            upper = ranges["upper"]
            mask = cv2.inRange(eroded, lower, upper)
            mask = cv2.dilate(mask, kernel_dilate)
            cv2.imshow("s", mask)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area >= 20 and area <= 700:
                    # print(area)
                    self.xBox, self.yBox, self.width, self.height = cv2.boundingRect(
                        contour)
                    self.cube_id = color
                    cv2.rectangle(image_raw, (self.xBox, self.yBox),
                                  (self.xBox + self.width, self.yBox + self.height), (0, 255, 0), 1)
                    cv2.putText(image_raw, f"{color}",
                                (self.xBox, self.yBox), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

                    # distance = self.calculate_position(
                    #     width, self.cube_width, self.fov, self.image_width)
                    # print(f"distance {color}: {distance:.2f} m")

        cv2.imshow("detected", image_raw)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main():
    try:
        rclpy.init()
        node = ImageClass()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
