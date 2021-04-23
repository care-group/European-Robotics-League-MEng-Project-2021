#!/usr/bin/python

# Adapted from https://automaticaddison.com/working-with-ros-and-opencv-in-ros-noetic/

# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time

def publish_message():

    # Node is publishing to the video_frames topic using 
    # the message type Image
    pub = rospy.Publisher('/webcam', Image, queue_size=10)
        
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('video_pub_py', anonymous=True)
        
    rate = rospy.Rate(10) # 10hz

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    cap = cv2.VideoCapture(0)
        
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    
    # While ROS is still running.
    while not rospy.is_shutdown():
        
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()
            
        if ret == True:
                
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS image message
            pub.publish(br.cv2_to_imgmsg(frame))
            time.sleep(0.5) #using time because rate.sleep isn't terminating


         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass