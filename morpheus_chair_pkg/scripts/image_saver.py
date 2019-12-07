#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.
# edits by Muhammad Sohaib Arif

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

from datetime import datetime
from os.path import expanduser

# Instantiate CvBridge
bridge = CvBridge()

#set folder to save images


def image_callback(msg):
    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "passthrough")
    except CvBridgeError, e:
        print(e)
    else:
        #get time to add to image name
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite(str(expanduser("~") + '/images/') + 'camera_image'+str(datetime.now()) + '.png', cv2_img)

def main():
    

    rospy.init_node('image_saver')
    # Define your image topic
    image_topic = "/raspicam_node/image"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()