#!/usr/bin/python

# YOLO code initially adapted from https://www.codespeedy.com/yolo-object-detection-from-image-with-opencv-and-python/

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import json
from cv.srv import LocalizePoint
from yolo import Yolo

# Commanded by /nav/semantic_labelling to look for all objects from the robot's camera feed using YOLO.
# Once objects are found, the labels and coordinates are published /semantic_labels, and a debugging image with bounding boxes is published to /semantic_labels/img
class Semantic_Labelling:
    def __init__(self):
        rospy.init_node('Semantic_Labelling')
        self.yolo = Yolo()
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('/semantic_labels/img', Image, queue_size=1)
        self.nav_pub = rospy.Publisher('/nav/somenavtopic', String, queue_size=1)

    def subscribe_jason(self):
        nav_subscriber = rospy.Subscriber('/nav/semantic_labelling', String,self.nav_callback)
        info_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo,self.info_callback)

    # Sets the target object and subscribes to the camera feed
    def nav_callback(self, msg):
        self.target=msg.data
        self.img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback,buff_size=1, queue_size=1)
    
    # Gets camera info
    def info_callback(self, msg):
        self.cam_info = msg

    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        objects,img = self.yolo.search_for_objects(cv_image)
        
        if(len(objects)!=0):

            #Pub image with bounding boxes to debug
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='passthrough'))

            for obj in objects:
                model = PinholeCameraModel()
                model.fromCameraInfo(self.cam_info)
                print(obj)
                xyz = model.projectPixelTo3dRay((obj["Point"][0],obj["Point"][1]))
                
                stampedPoint = PointStamped()
                stampedPoint.header=msg.header
                stampedPoint.point.x=xyz[0]
                stampedPoint.point.y=0
                stampedPoint.point.z=xyz[1]
                
                rospy.wait_for_service('get_3d_position')
                get_3d_points =rospy.ServiceProxy('get_3d_position',LocalizePoint)
                resp = get_3d_points(stampedPoint)

                threeDPoint = resp.localizedPointMap
                dictMsg={}
                dictMsg["name"]=obj["Label"]
                dictMsg["type"]="object"
                dictMsg["coords"]=[threeDPoint.point.x,threeDPoint.point.y,threeDPoint.point.z]
                dictMsg["others"]={}
                self.nav_pub.publish(json.dumps(dictMsg))

        else:
            print("Object not found.")


semantic_labelling = Semantic_Labelling()
semantic_labelling.subscribe_jason()
rospy.spin()