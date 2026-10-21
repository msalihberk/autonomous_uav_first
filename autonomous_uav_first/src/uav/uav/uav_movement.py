import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from custom_interfaces.srv import MovePoint
from uav.topic_names import TopicNames

class UAVMovement(Node):
    def __init__(self):
        super().__init__("UAVMovement")

        self.pos_publisher = self.create_publisher(
            PoseStamped, 
            "/mavros/setpoint_position/local", 
            10
        )

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

        self.move_server = self.create_service(MovePoint, TopicNames.MOVEMENT_TOPIC, self.move_to_target)
        self.timer_period = 0.1
        self.move_timer = self.create_timer(self.timer_period, self.move_callback)
        self.distance = 5.0

        self.end_point = PoseStamped()
        self.local_pos = PoseStamped()
        self.imu_data = Imu()

        self.is_task_completed = True

    def complete_task(self):
        self.is_task_completed = True

    def start_task(self):
        self.is_task_completed = False

    def is_pose_empty(self, pose: PoseStamped):
        x_val = pose.pose.position.x <= 0.1 and pose.pose.position.x >= -0.1
        y_val = pose.pose.position.y <= 0.1 and pose.pose.position.y >= -0.1
        z_val = pose.pose.position.z <= 0.1 and pose.pose.position.z >= -0.1
        return not x_val and not y_val and not z_val

    def is_local_pos_initialized(self):
        return not self.is_pose_empty(self.local_pos)

    def is_ready_to_move(self):
        return self.is_local_pos_initialized() and not self.is_pose_empty(self.end_point) and not self.is_task_completed

    def move_callback(self):
        if not self.is_ready_to_move():
            return
        
        self.end_point.header.stamp = self.get_clock().now().to_msg()

        self.pos_publisher.publish(self.end_point)

    def save_local_pos(self, msg):
        self.local_pos = msg

    def save_imu_data(self, msg):
        self.imu_data = msg

    def calculate_differance(self, vector1, vector2):
        x = abs(vector1.x - vector2.x)
        y = abs(vector1.y - vector2.y)
        z = abs(vector1.z - vector2.z)
        result = [x, y, z]

        return result

    def calculate_distance(self, vector1, vector2):
        values = self.calculate_differance(vector1, vector2)
        xq = values[0] ** 2
        yq = values[1] ** 2
        zq = values[2] ** 2

        sum = xq + yq + zq

        return sum ** (1/3)

    def reached_target(self):
        return self.calculate_distance(self.local_pos.pose.position, self.end_point.pose.position) <= self.distance and self.is_ready_to_move()

    def move_to_target(self, request, response):
        response.success = True
        if self.is_task_completed:
            if self.reached_target():
                self.get_logger().info(f"Reached target: {self.end_point.pose.position.x}, {self.end_point.pose.position.y}, {self.end_point.pose.position.z}")
                self.complete_task()
                return response

            self.start_task()
            self.move(request.point)

        return response

    def move(self, point: Point):
        self.end_point.pose.position.x = point.x
        self.end_point.pose.position.y = point.y
        self.end_point.pose.position.z = point.z
        
        self.get_logger().info(f"Moving to target: {self.end_point.pose.position.x}, {self.end_point.pose.position.y}, {self.end_point.pose.position.z}")

def main(args=None):
    rclpy.init()

    node = UAVMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()