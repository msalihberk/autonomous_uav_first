from sympy import Quaternion

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import CamData
from uav.topic_names import TopicNames
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math

class UAVCalculation(Node):
    def __init__(self):
        super().__init__("UAVCalculation")

        self.coordinate_service = self.create_service(CamData, TopicNames.COORDINATE_CAL_TOPIC, 
                                                      self.calculate_coordinate, qos_profile_sensor_data)

        self.fov_horizontal = 1.047
        self.fov_vertical = 1.047 # Temporary value

    @staticmethod
    def quaternion_to_rotation_matrix(q: Quaternion):
        x, y, z, w = q.x, q.y, q.z, q.w
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        R = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)]
        ])

        return R

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
        r = self.quaternion_to_rotation_matrix(drone_q)
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
        target_z = 0.0

        hypotenuse = math.sqrt(target_x ** 2 + target_y ** 2)
        yaw = self.get_yaw_from_quaternion(self.imu_data.orientation)
        target_x = hypotenuse * math.cos(yaw)
        target_y = hypotenuse * math.sin(yaw)


        response.target_point.x = target_x
        response.target_point.y = target_y
        response.target_point.z = target_z
        response.success = True

        return response

def main(args=None):
    rclpy.init(args)

    node = UAVCalculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
