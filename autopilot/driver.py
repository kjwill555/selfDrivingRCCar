#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import cv_bridge as bridge
import cv2
from actuators.msg import Normalized_PWM
from sensor_msgs.msg import Image
from autopilot import AutoPilot


"""
Callback for a new image.
Converts the image to grayscale, makes a prediction, and publishes 
the predicted values

Parameters:
msg (sensor_msgs.msg.Image) : The received message
"""
def image_recvd(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    grayscale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    img_array = np.atleast_3d(grayscale)
    steering, throttle = pilot.predict(img_array)

    drive_msg = Normalized_PWM()
    drive_msg.steering = steering;
    drive_msg.throttle = throttle;
    pub.publish(drive_msg)

if __name__ == "__main__":
    rospy.init_node("autopilot")
    pilot = AutoPilot(sys.argv[1])
    pub = rospy.Publisher("norm_PWM", Normalized_PWM, queue_size=20)
    rospy.Subscriber("decompressed_image", Image, image_recvd)
    ros.spin()
