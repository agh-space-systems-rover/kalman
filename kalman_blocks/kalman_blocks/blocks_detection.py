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
    # "red": {
    #     "lower": np.array([160, 100, 100]),
    #     "upper": np.array([180, 255, 255]),

    # },
    "green": {
        "lower": np.array([40, 50, 50]),
        "upper": np.array([80, 255, 255])
    },
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
])  # square pixel fx = fy | !!! cx and cy depend on current resolution !!!

fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]


class ImageClass(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        # read params
        self.declare_parameter("cube_width", 0.1)  # [m]
        self.declare_parameter("camera_no", "d455_front")
        self.bridge = CvBridge()
        # write params
        self.cube_width = self.get_parameter("cube_width").value
        self.camera_no = self.get_parameter("camera_no").value
        # subscribe the camera
        self.create_subscription(
            Image, f"/{self.camera_no}/color/image_raw", self.image_process_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def calculate_cube_pose(self, W_pixel: int, x_pos: int, y_pos: int, cube_id: str):
        tf = TransformStamped()
        z = (fx * self.cube_width) / W_pixel
        x = z * (x_pos - cx) / fx
        y = z * (y_pos - cy) / fy
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = cube_id
        tf.transform.translation.x = z
        tf.transform.translation.y = y
        tf.transform.translation.z = x
        self.tf_broadcaster.sendTransform(tf)

        # print(f"{cube_id} x: {x} \n y: {y} \n z: {z} ")

    def image_process_callback(self, msg: Image):   # add frame id
        image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
        # kernel = np.array([[0, -1, 0],
        #                    [-1, 5, -1],
        #                    [0, -1, 0]])
        kernel_erode = np.ones((2, 2), "uint8")
        kernel_dilate = np.ones((3, 3), "uint8")
        eroded = cv2.erode(hsv, kernel_erode, iterations=1)
        # cv2.imshow("d", eroded)
        for color, ranges in color_ranges.items():
            lower = ranges["lower"]
            upper = ranges["upper"]
            mask = cv2.inRange(eroded, lower, upper)
            mask = cv2.dilate(mask, kernel_dilate)
            # cv2.imshow("s", mask)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            # NOWE OBLICZANIE POZYCJI
            for contour in contours:
                area = cv2.contourArea(contour)
                if area >= 40 and area <= 1000:
                    # print(area)
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        x_pos = int(M["m10"] / M["m00"])
                        y_pos = int(M["m01"] / M["m00"])
                        wmin_x = np.min(contour[:, :, 0])
                        wmax_x = np.max(contour[:, :, 0])
                        hmin_y = np.min(contour[:, :, 1])
                        hmax_y = np.max(contour[:, :, 1])
                        width = wmax_x - wmin_x
                        height = hmax_y - hmin_y
                        x, y = int(x_pos - width / 2), int(y_pos - height / 2)
                    cv2.rectangle(image_raw, (x, y),
                                  (int(x_pos + width / 2), int(y_pos + height / 2)), (0, 255, 0), 1)
                    cv2.putText(image_raw, f"{color}",
                                (x_pos, y_pos), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
                    self.calculate_cube_pose(
                        width, x_pos=x_pos, y_pos=y_pos, cube_id=str(color))

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
