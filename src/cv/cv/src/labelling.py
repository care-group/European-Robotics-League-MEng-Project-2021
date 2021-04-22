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
from get_depth import Depth_Finder
# Commanded by /nav/semantic_labelling to look for all objects from the robot's camera feed using YOLO.
# Once objects are found, the labels and coordinates are published /semantic_labels, and a debugging image with bounding boxes is published to /semantic_labels/img
class Semantic_Labelling:
    def __init__(self):
        rospy.init_node('Semantic_Labelling')
        self.yolo = Yolo()
        self.bridge = CvBridge()
        self.depth_finder = Depth_Finder()
        self.img_pub = rospy.Publisher('/semantic_labels/img', Image, queue_size=10)
        self.nav_pub = rospy.Publisher('/nav/somenavtopic', String, queue_size=10)

        info_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo,self.info_callback,queue_size=None)
        img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback,queue_size=1, buff_size=52428800)

    
    # Gets camera info
    def info_callback(self, msg):
        self.cam_info = msg
    
    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')        

        #Respond to attribute error if subscribers haven't ran yet
        try:
            objects,img = self.yolo.search_for_objects(cv_image)
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='passthrough'))
            model = PinholeCameraModel()
            model.fromCameraInfo(self.cam_info)
        except AttributeError:
            print("waiting")
            return


        if(len(objects)!=0):
            for obj in objects:
                print(obj)

                xy = obj["Point"]
                dist = self.depth_finder.get_depth(int(xy[0]),int(xy[1]))
                depth = dist[0]

                vect = model.projectPixelTo3dRay((xy[0],xy[1])) 
                xyz = [el / vect[2] for el in vect]

                stampedPoint = PointStamped()
                stampedPoint.header.frame_id="head_rgbd_sensor_rgb_frame"
                stampedPoint.point.x=xyz[0]*depth
                stampedPoint.point.y=xyz[1]*depth
                stampedPoint.point.z=depth

                rospy.wait_for_service('transform_point')
                get_3d_points =rospy.ServiceProxy('transform_point',LocalizePoint)
                resp = get_3d_points(stampedPoint)
                #print(resp.localizedPointMap)
                threeDPoint = resp.localizedPointMap
                dictMsg={}
                dictMsg["name"]=obj["Label"]
                dictMsg["type"]="object"
                dictMsg["coords"]=[threeDPoint.point.x,threeDPoint.point.y,threeDPoint.point.z]
                dictMsg["others"]={}
                self.nav_pub.publish(json.dumps(dictMsg))

        else:
            print("No objects found.")


semantic_labelling = Semantic_Labelling()
rospy.spin()