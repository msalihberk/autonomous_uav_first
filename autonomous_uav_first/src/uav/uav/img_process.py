import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from uav.topic_names import TopicNames
from rclpy.qos import qos_profile_sensor_data
from custom_interfaces.msg import CamData
import cv_bridge
import cv2
import numpy as np

class UAVImageProcessor(Node):
    def __init__(self):
        super().__init__('UAVImageProcessor')

        self.get_logger().info(f"Listening... {TopicNames.CAM_TOPIC}")
        self.img_subscriber = self.create_subscription(Image, TopicNames.CAM_TOPIC, self.process, qos_profile_sensor_data)
        self.data_publisher = self.create_publisher(CamData, TopicNames.CAM_DATA_TOPIC, qos_profile_sensor_data)

        self.lower1 = np.array([0, 60, 60])
        self.upper1 = np.array([10, 255, 255])

        self.lower2 = np.array([170, 60, 60])
        self.upper2 = np.array([180, 255, 255])

        self.area_treshold = 2.0

    def get_data(self, center_x: float, center_y: float, width: float, 
                 height: float, img_height: float, img_width: float):
        msg = CamData()
        msg.center_x = center_x
        msg.center_y = center_y
        msg.width = width
        msg.height = height
        msg.img_width = img_width
        msg.img_height = img_height
        msg.header.stamp = self.get_clock().now().to_msg()

        return msg

    def empty_data(self):
        msg = self.get_data(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        msg.header.stamp = Time().to_msg()

        return msg

    def process(self, msg):
        frame = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)

        mask = cv2.bitwise_or(mask1, mask2)

        cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones([5, 5], np.uint8))
        contours, hierarchy = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        data_msg = self.empty_data()

        img_height, img_width, _ = frame.shape

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            if area > self.area_treshold:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)

                data_msg = self.get_data(x + w / 2, y - h / 2, w, h, img_height, img_width)

        self.data_publisher.publish(data_msg)

        cv2.imshow("Cam", frame)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            exit()

def main(args=None):
    rclpy.init(args=args)

    node = UAVImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()