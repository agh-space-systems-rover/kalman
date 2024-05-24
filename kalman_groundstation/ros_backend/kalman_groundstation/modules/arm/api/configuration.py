# import rospy

from fastapi import APIRouter
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter


# from moveit_msgs.srv import ChangeControlDimensions, ChangeControlDimensionsRequest
# from std_srvs.srv import Empty

configuration_router = APIRouter(prefix="/configuration", tags=["configuration"])
node = rclpy.create_node("configuration_node")
servo_param_client =  node.create_client(SetParameters, "servo_node/set_parameters")


# change_control_dimensions = rospy.ServiceProxy(
#     "/servo_server/change_control_dimensions", ChangeControlDimensions
# )
# reset_servo_status = rospy.ServiceProxy("/servo_server/reset_servo_status", Empty)


# @configuration_router.put(
#     "/control_dimensions",
#     name="Sets control dimensions of 6dof arm",
#     response_model=bool,
# )
# async def put(
#     control_x_translation: bool,
#     control_y_translation: bool,
#     control_z_translation: bool,
#     control_x_rotation: bool,
#     control_y_rotation: bool,
#     control_z_rotation: bool,
# ):
#     msg = ChangeControlDimensionsRequest(
#         control_x_translation,
#         control_y_translation,
#         control_z_translation,
#         control_x_rotation,
#         control_y_rotation,
#         control_z_rotation,
#     )
#     change_control_dimensions(msg)
#     return True


# @configuration_router.put(
#     "/reset_servo_status", name="Resets status of servo server", response_model=bool
# )
# async def put():
#     reset_servo_status()


@configuration_router.put(
    "/linear_vel_scale", name="Sets linear velocity multiplier", response_model=bool
)
async def put(scale: float):
    while not servo_param_client.wait_for_service(timeout_sec=0.5):
        node.get_logger().info("service not available, waiting again...")
    
    request = SetParameters.Request()
    
    param = Parameter()
    param.name = "moveit_servo.scale.linear"
    param.value = scale
    
    request.parameters = [param]
    
    servo_param_client.call_async(request)
    return True


@configuration_router.put(
    "/angular_vel_scale", name="Sets angular velocity multiplier", response_model=bool
)
async def put(scale: float):
    while not servo_param_client.wait_for_service(timeout_sec=0.5):
        node.get_logger().info("service not available, waiting again...")
    
    request = SetParameters.Request()
    
    param = Parameter()
    param.name = "moveit_servo.scale.rotational"
    param.value = scale
    
    request.parameters = [param]
    
    servo_param_client.call_async(request)
    return True


# @configuration_router.put(
#     "/twist_command_frame",
#     name="Sets control dimensions of 6dof arm",
#     response_model=bool,
# )
# async def put(frame: str):
#     client = dynamic_reconfigure.client.Client(
#         "spacenav_to_twist", timeout=30, config_callback=None
#     )
#     client.update_configuration({"twist_command_frame": frame})
#     return True
