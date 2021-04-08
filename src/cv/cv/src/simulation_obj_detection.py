#!/usr/bin/python

# YOLO code initially adapted from https://www.codespeedy.com/yolo-object-detection-from-image-with-opencv-and-python/

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import time
import numpy as np
import rospkg
from image_geometry import PinholeCameraModel, StereoCameraModel
from sensor_msgs.msg import CameraInfo
from cv.srv import LocalizePoint
import json
from yolo import Yolo
from get_depth import Depth_Finder

# Commanded by the jason agent /jason/detect_object to look for a given object from the robot's camera feed using YOLO.
# Once found, the coordinates are published /yolo/<target>, and a debugging image with bounding boxes is published to /yolo/<target>/img
class Object_Detection:
    def __init__(self):
        rospy.init_node('Object_Detection')
        self.target = ""
        self.yolo = Yolo()
        self.bridge = CvBridge()
        self.depth_finder = Depth_Finder()

        self.TIMEOUT = 15
        self.coord_pub_map = rospy.Publisher('/cv/detected_obj/coords/map', PointStamped, queue_size=10,latch=True)
        self.coord_pub_odom = rospy.Publisher('/cv/detected_obj/coords/odom', PointStamped, queue_size=10,latch=True)
        self.coord_pub_json = rospy.Publisher('/cv/detected_obj/coords/json', String, queue_size=10,latch=True)
        self.img_pub = rospy.Publisher('/yolo/img', Image, queue_size=1,latch=True)
        
        img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback)
        info_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo,self.info_callback)


    def subscribe(self):
        jason_subscriber = rospy.Subscriber('/jason/detect_object', String,self.jason_callback)

    def info_callback(self, msg):
        self.cam_info = msg

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')    

    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def jason_callback(self, msg):
        self.target=msg.data
        startingTime=time.time()
        running = True

        while(running):

            #Respond to attribute error if subscribers haven't ran yet
            try:
                objects,img = self.yolo.search_for_objects(self.cv_image)
                model = PinholeCameraModel()
                model.fromCameraInfo(self.cam_info)
            except AttributeError:
                continue
            
            target_obj = self.get_target_obj(objects)

            duration = time.time()-startingTime
            if(len(objects)!=0 and target_obj!=None):
                
                obj_coords = target_obj["Point"]
                print(obj_coords)
                
                #Pub image with bounding boxes debug
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='passthrough'))
        
                dist = self.depth_finder.get_depth(int(obj_coords[0]),int(obj_coords[1]))
                depth = dist[0]

                vect = model.projectPixelTo3dRay((obj_coords[0],obj_coords[1])) 
                xyz = [el / vect[2] for el in vect]

                stampedPoint = PointStamped()
                stampedPoint.header.frame_id="head_rgbd_sensor_rgb_frame"
                stampedPoint.point.x=xyz[0]*depth
                stampedPoint.point.y=xyz[1]*depth
                stampedPoint.point.z=depth

                rospy.wait_for_service('transform_point')
                get_3d_points =rospy.ServiceProxy('transform_point',LocalizePoint)
                resp = get_3d_points(stampedPoint)

                self.coord_pub_map.publish(resp.localizedPointMap)
                self.coord_pub_odom.publish(resp.localizedPointOdom)
                
                print(resp.localizedPointOdom)
                dictMsg={}
                dictMsg["x"]=resp.localizedPointMap.point.x
                dictMsg["y"]=resp.localizedPointMap.point.y
                dictMsg["z"]=resp.localizedPointMap.point.z
                self.coord_pub_json.publish(json.dumps(dictMsg))
                running = False
            elif(duration <self.TIMEOUT):
                print("Object not found.")
            else:
                print("Timeout.")
                running=False

    # Returns the dict (label, point) that's label corresponds to the target
    def get_target_obj(self,objects):
        if len(objects) == 0:
            return None
        
        for obj in objects:
            if obj["Label"]==self.target:
                return obj
        return None

obj_detection = Object_Detection()
obj_detection.subscribe()
rospy.spin()