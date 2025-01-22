#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image

class PixelInspector:
    def __init__(self):
        rospy.init_node('pixel_inspector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.cv_image = None

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Click to Inspect Pixels", self.cv_image)
            cv2.setMouseCallback("Click to Inspect Pixels", self.mouse_callback)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to process image: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.cv_image is not None:
            pixel_value = self.cv_image[y, x]  # OpenCV uses (row, column)
            rospy.loginfo(f"Pixel clicked: ({x}, {y}) - Value: {pixel_value}")

if __name__ == "__main__":
    try:
        PixelInspector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
