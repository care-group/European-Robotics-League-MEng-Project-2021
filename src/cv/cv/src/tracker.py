#!/usr/bin/python
# coding=utf-8

# Very scuffed script to test cvbridge/opencv

from cv_bridge import CvBridge
import cv2
import sys
import rospy
from sensor_msgs.msg import Image

def img_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')    
    print(cv_image.shape)


# Set up tracker.
# Instead of MIL, you can also use
(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

print(cv2.__version__)
bridge = CvBridge()
img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,img_callback)

tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
tracker_type = tracker_types[2]

if tracker_type == 'BOOSTING':
    tracker = cv2.TrackerBoosting_create()
if tracker_type == 'MIL':
    tracker = cv2.TrackerMIL_create()
if tracker_type == 'KCF':
    tracker = cv2.TrackerKCF_create()
if tracker_type == 'TLD':
    tracker = cv2.TrackerTLD_create()
if tracker_type == 'MEDIANFLOW':
    tracker = cv2.TrackerMedianFlow_create()
if tracker_type == 'GOTURN':
    tracker = cv2.TrackerGOTURN_create()
if tracker_type == 'MOSSE':
    tracker = cv2.TrackerMOSSE_create()
if tracker_type == "CSRT":
    tracker = cv2.TrackerCSRT_create()

rospy.init_node('Tracker')

rospy.spin()