import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from my_interfaces.msg import TargetPixels
from std_msgs.msg import Bool
from rclpy.time import Time
from uav.topic_names import TopicNames
import math
import time

class UAVIdle(Node):
    def __init__(self):
        super().__init__("UAVIdle")
        
        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('radius_x', 6.0)
        self.declare_parameter('radius_y', 4.0)
        self.declare_parameter('default_altitude', 5.0)

        self.timer_period = self.get_parameter('timer_period').value
        self.speed = self.get_parameter('speed').value
        self.Rx = self.get_parameter('radius_x').value
        self.Ry = self.get_parameter('radius_y').value
        self.alt = self.get_parameter('default_altitude').value

        self.timeout_sec = 1.0
        
        self.started = False
        self.start_time = 0.0

        self.pos_publisher = self.create_publisher(
            PoseStamped, 
            "/mavros/setpoint_position/local", 
            10
        )

        self.pos_subscriber = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.save_local_pos,
            qos_profile=qos_profile_sensor_data
        )

        self.target_subscriber = self.create_subscription(
            TargetPixels,
            "/uav/target/coordinates/pixel",
            self.control_target,
            10
        )

        self.mode_subscriber = self.create_subscription(Bool, TopicNames.IDLE_MODE_TOPIC, self.control_mode, qos_profile_sensor_data)

        self.local_pos = PoseStamped()
        self.local_pos_is_initialized = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.prev_yaw = 0.0

        self.move_timer = self.create_timer(self.timer_period, self.infinite_shape)
        self.move_timer.cancel()
        
        self.get_logger().info("UAVIdle node started. Waiting for goals...")

    def control_mode(self, data: Bool):
        if data.data:
            self.start_goal()
        else:
            self.cancel()

    def start_goal(self):
        if self.started:
            return

        self.get_logger().info("Goal received...")
        
        self.speed = self.get_parameter('speed').value
        self.Rx = self.get_parameter('radius_x').value
        self.Ry = self.get_parameter('radius_y').value

        if self.local_pos_is_initialized:
            self.start_x = self.local_pos.pose.position.x
            self.start_y = self.local_pos.pose.position.y
            self.alt = self.local_pos.pose.position.z
        else:
            self.start_x = 0.0
            self.start_y = 0.0

        self.start_time = time.time()
        self.prev_yaw = 0.0
        self.started = True
        self.move_timer.reset()

    def control_target(self, msg):
        msg_time = Time.from_msg(msg.header.stamp)
        current_time = self.get_clock().now()
        
        elapsed_sec = (current_time - msg_time).nanoseconds / 1e9
        
        if elapsed_sec < self.timeout_sec and self.started and msg.width > 0.2:
            self.cancel()

    def cancel(self):
        if self.started:
            self.get_logger().info("Goal cancellation requested.")
            self.pos_publisher.publish(self.local_pos)
            self.move_timer.cancel()
            self.started = False
            self.prev_yaw = 0.0

    def infinite_shape(self):
        elapsed_time = time.time() - self.start_time
        
        t = elapsed_time * (self.speed / 3.0) 

        pos = PoseStamped()
        pos.header.stamp = self.get_clock().now().to_msg()
        pos.header.frame_id = "map"

        pos.pose.position.x = self.start_x + self.Rx * math.sin(t)
        pos.pose.position.y = self.start_y + self.Ry * math.sin(t) * math.cos(t)
        pos.pose.position.z = self.alt

        dx = self.Rx * math.cos(t)
        dy = self.Ry * (math.cos(t)**2 - math.sin(t)**2)
        
        raw_yaw = math.atan2(dy, dx)

        d_yaw = raw_yaw - self.prev_yaw
        while d_yaw > math.pi:
            raw_yaw -= 2.0 * math.pi
            d_yaw = raw_yaw - self.prev_yaw
        while d_yaw < -math.pi:
            raw_yaw += 2.0 * math.pi
            d_yaw = raw_yaw - self.prev_yaw

        yaw = 0.8 * self.prev_yaw + 0.2 * raw_yaw
        self.prev_yaw = yaw

        pos.pose.orientation.w = math.cos(yaw * 0.5)
        pos.pose.orientation.x = 0.0
        pos.pose.orientation.y = 0.0
        pos.pose.orientation.z = math.sin(yaw * 0.5)

        self.pos_publisher.publish(pos)

    def save_local_pos(self, msg):
        self.local_pos = msg
        if not self.local_pos_is_initialized:
            self.local_pos_is_initialized = True

def main(args=None):
    rclpy.init(args=args)
    node = UAVIdle()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
