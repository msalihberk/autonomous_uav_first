from sensor_msgs.msg import Imu
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import Coordinate
from uav.topic_names import TopicNames
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from uav.math_funcs import MathFuncs
import numpy as np
import math

class UAVCalculation(Node):
    def __init__(self):
        super().__init__("UAVCalculation")

        self.coordinate_service = self.create_service(Coordinate, TopicNames.COORDINATE_CAL_TOPIC, self.calculate_coordinate)
        self.angle_service = self.create_service(Coordinate, TopicNames.ANGLE_CAL_TOPIC, self.calculate_angle)
    
        self.fov_horizontal = 1.047
        self.fov_vertical = 1.047 # Temporary value

        self.pos_subscripter = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.save_local_pos,
            qos_profile=qos_profile_sensor_data
        )

        self.imu_subscripter = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.save_imu_data,
            qos_profile=qos_profile_sensor_data
        )

        self.imu_data = Imu()
        self.local_pos = PoseStamped()
        self.default_alt = 5.0
        self.distance = 5.0

    def save_local_pos(self, msg):
        self.local_pos = msg

    def save_imu_data(self, msg):
        self.imu_data = msg

    

    def calculate_angle(self, request, response):
        try:
            pixel_error = (request.center_x) - (request.img_width / 2.0)

            angle = pixel_error / request.img_width * self.fov_horizontal
            # For ardupilot axis rule (Left + ,Right -)
            angle *= -1

            response.success = True
            response.angle = angle

        except:
            pass

        return response

    def calculate_coordinate(self, request, response):
        aspect_ratio = request.img_width / request.img_height
        self.fov_vertical = 2.0 * np.atan(np.tan(self.fov_horizontal / 2.0) / aspect_ratio)
        
        fx = (request.img_width / 2.0) / np.tan(self.fov_horizontal / 2.0)
        fy = (request.img_height / 2.0) / np.tan(self.fov_vertical / 2.0)

        x_cam = (request.center_x - (request.img_width / 2.0)) / fx
        y_cam = (request.center_y - (request.img_height / 2.0)) / fy
        z_cam = 1.0

        vector_raw = np.array([z_cam, x_cam, y_cam])
        ray_cam = vector_raw / np.linalg.norm(vector_raw)

        drone_q = self.imu_data.orientation
        r = MathFuncs.quaternion_to_rotation_matrix(drone_q)
        ray_world = np.dot(r, ray_cam)

        if ray_world[2] >= 0:
            response.success = False
            return response

        pos = self.local_pos.pose.position
        
        t = -pos.z / ray_world[2]

        if t < 0:
            response.success = False
            return response


        target_x = pos.x + float(t * ray_world[0])
        target_y = pos.y + float(t * ray_world[1])
        target_z = self.default_alt

        hypotenuse = math.sqrt(target_x ** 2 + target_y ** 2)
        yaw = MathFuncs.get_yaw_from_quaternion(self.imu_data.orientation)
        target_x = hypotenuse * math.cos(yaw)
        target_y = hypotenuse * math.sin(yaw)

        response.target_point.x = target_x
        response.target_point.y = target_y
        response.target_point.z = target_z
        response.success = True

        self.get_logger().info(f"Calculated Target Coordinates: x={target_x}, y={target_y}, z={target_z}")

        return response

def main(args=None):
    rclpy.init(args=args)

    node = UAVCalculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
