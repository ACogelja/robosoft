#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class HsvConverter:
    def __init__(self):
        rospy.init_node('hsv_converter_node', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # Publishers
        self.hsv_pub = rospy.Publisher('hsv_picture', Image, queue_size=10)
        self.red_circle_pub = rospy.Publisher('red_circles_detected', Image, queue_size=5)

    def image_callback(self, msg):
        try:
            #The image that we recieve has to be transformed to hsv
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # The ranges for the red color in the HSV picture
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            # Creating the red mas for the pic
            mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            red_mask = mask1 | mask2

            # Apply the mask to get only red areas only
            red_image = cv2.bitwise_and(cv_image,cv_image, mask=red_mask)

            # Detecting the circles in the masked red image
            gray_red = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)
            blurred_gray = cv2.GaussianBlur(gray_red, (9, 9), 2, 2)
            circles = cv2.HoughCircles(blurred_gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=100)

            # Draw circles on the original image after we find them on the hsv pic
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, r) in circles:
                    cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                    cv2.circle(cv_image, (x, y), 2, (0, 255, 255), 3)
            
            #---------------------
            #Classic publish on the topic red_circles_msg and topic hsv_picture
            # Convert the result image with red circles to ROS Image message
            red_circles_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.red_circle_pub.publish(red_circles_msg)
            #---------------------
            #the image needs to be converted from HSV image back to a ROS image message
            hsv_msg = self.bridge.cv2_to_imgmsg(hsv_image, "bgr8")
            self.hsv_pub.publish(hsv_msg)
            #---------------------

        except CvBridgeError as e:
            #If it fails to convert
            rospy.logerr(f"Failed to convert image: {e}")

if __name__ == '__main__':
    try:
        #classic main
        converter = HsvConverter()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    