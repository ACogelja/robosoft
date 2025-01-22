#!/usr/bin/env python3

import matplotlib.pyplot as plt
from collections import deque
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image

intensity_data = deque(maxlen=100)  # Shared data between the image callback and plotting
circle_positions = [(311, 355, 30), (225, 230, 30), (374, 158, 30), (433, 272, 30)]  # (x, y, radius)

class Optical_Measuring:
    def __init__(self):
        rospy.init_node('bending_optical_measurement', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        global intensity_data  #global data plotting
        try:
            #OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            current_intensities = []
            for (x, y, radius) in circle_positions:
                mask = np.zeros_like(gray_image, dtype=np.uint8)
                cv2.circle(mask, (x, y), radius, 255, -1)
                masked_region = cv2.bitwise_and(gray_image, gray_image, mask=mask)
                avg_intensity = cv2.mean(masked_region, mask=mask)[0]
                current_intensities.append(avg_intensity)

            #For plotting appending the data
            intensity_data.append(current_intensities)

        except CvBridgeError as e:
            rospy.logerr(f"Failed to process image: {e}")

def plot_data():
    """Plot intensity data dynamically in the main thread."""
    fig, ax = plt.subplots()
    lines = []
    for i in range(len(circle_positions)):
        line, = ax.plot([], [], label=f"Circle {i + 1}")
        lines.append(line)

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 255)
    ax.set_xlabel("Frame")
    ax.set_ylabel("Average Intensity")
    ax.legend()

    plt.ion()  # Turn on interactive mode
    plt.show()

    while not rospy.is_shutdown():
        # Update the plot with the latest data
        for i, line in enumerate(lines):
            intensities = [frame[i] for frame in intensity_data]
            line.set_ydata(intensities)
            line.set_xdata(range(len(intensities)))
        ax.relim()
        ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

if __name__ == "__main__":
    try:
        optical_measurement = Optical_Measuring()
        plot_data()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass