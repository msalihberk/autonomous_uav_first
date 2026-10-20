import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from topic_names import TopicNames
import cv_bridge
import cv2
import numpy as np

class UAVImageProcessor(Node):
    def __init__(self):
        super().__init__('UAVImageProcessor')

        self.img_subscriber = self.create_subscription(Image, TopicNames.CAM_TOPIC, self.process)

        self.lower1 = np.array([0, 60, 60])
        self.upper1 = np.array([10, 255, 255])

        self.lower2 = np.array([170, 60, 60])
        self.upper2 = np.array([180, 255, 255])

        self.area_treshold = 2.0

    def process(self, msg):
        frame = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)

        mask = cv2.bitwise_or(mask1, mask2)

        cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones([5, 5], np.uint8))
        contours, hierarchy = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            if area > self.area_treshold:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 0.5)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            exit()

cv2.destroyAllWindows()