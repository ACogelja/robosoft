#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from robosoft.msg import Float32MultiArrayStamped
import os

class Optical_Measuring:
    def __init__(self):
        rospy.init_node('intensity_publisher', anonymous=True)

        os.system("v4l2-ctl -c exposure_auto=1")
        os.system("v4l2-ctl -c exposure_absolute=5")

        self. circle_positions = [(230, 288, 30), (355, 360, 30), (425, 230, 30), (300, 160, 30), (327, 258, 30)]  # (x, y, radius)
        self.bridge = CvBridge()

        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.test_pub_ = rospy.Publisher("/optical_sensor_test", Image, queue_size = 1)
        self.intensity_pub_ = rospy.Publisher("/optical_sensor", Float32MultiArrayStamped, queue_size = 1)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            current_intensities = []
            for (x, y, radius) in self.circle_positions:
                mask = np.zeros_like(gray_image, dtype=np.uint8)
                cv2.circle(mask, (x, y), radius, 255, -1)
                cv2.circle(cv_image, (x, y), radius, 255, -1)

                masked_region = cv2.bitwise_and(gray_image, gray_image, mask=mask)
                avg_intensity = cv2.mean(masked_region, mask=mask)[0]
                current_intensities.append(avg_intensity)

            # new_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
            # self.test_pub_.publish(new_msg)
            # print("Published!")

            intensity_msg = Float32MultiArrayStamped()
            intensity_msg.header.stamp = rospy.Time.now()
            intensity_msg.data = current_intensities
            self.intensity_pub_.publish(intensity_msg)
            # print(current_intensities)
            # print(sum(current_intensities)/5)

        except CvBridgeError as e:
            rospy.logerr(f"Failed to process image: {e}")


if __name__ == "__main__":
    try:
        optical_measurement = Optical_Measuring()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass