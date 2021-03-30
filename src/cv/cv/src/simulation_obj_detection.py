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

# Commanded by the jason agent /jason/detect_object to look for a given object from the robot's camera feed using YOLO.
# Once found, the coordinates are published /yolo/<target>, and a debugging image with bounding boxes is published to /yolo/<target>/img
class Object_Detection:
    def __init__(self):
        rospy.init_node('Object_Detection')
        self.target = ""
        self.yolo = Yolo()
        self.bridge = CvBridge()
        self.TIMEOUT = 15
        self.coord_pub_map = rospy.Publisher('/cv/detected_obj/coords/map', PointStamped, queue_size=10,latch=True)
        self.coord_pub_odom = rospy.Publisher('/cv/detected_obj/coords/odom', PointStamped, queue_size=10,latch=True)
        self.coord_pub_json = rospy.Publisher('/cv/detected_obj/coords/json', String, queue_size=10,latch=True)
        self.img_pub = rospy.Publisher('/yolo/img', Image, queue_size=1,latch=True)

    def subscribe(self):
        jason_subscriber = rospy.Subscriber('/jason/detect_object', String,self.jason_callback)
        info_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo,self.info_callback)

    def info_callback(self, msg):
        self.cam_info = msg

    # Sets the target object and subscribes to the camera feed
    def jason_callback(self, msg):
        self.target=msg.data
        startingTime=time.time()
        self.img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback,[startingTime],buff_size=1, queue_size=1)
    
    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg,args):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')    
        objects,img = self.yolo.search_for_objects(cv_image)
        
        target_obj = self.get_target_obj(objects)

        duration = time.time()-args[0]
        if(len(objects)!=0 and target_obj!=None):
            
            obj_coords = target_obj["Point"]
            print(obj_coords)
            
            #Pub image with bounding boxes debug
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='passthrough'))
    
            model = PinholeCameraModel()
            model.fromCameraInfo(self.cam_info)
            xyz = model.projectPixelTo3dRay((obj_coords[0],obj_coords[1])) 
            
            stampedPoint = PointStamped()
            stampedPoint.header=msg.header
            stampedPoint.point.x=xyz[0]
            stampedPoint.point.y=0
            stampedPoint.point.z=xyz[1]
            
            rospy.wait_for_service('get_3d_position')
            get_3d_points =rospy.ServiceProxy('get_3d_position',LocalizePoint)
            resp = get_3d_points(stampedPoint)

            self.coord_pub_map.publish(resp.localizedPointMap)
            self.coord_pub_odom.publish(resp.localizedPointOdom)

            dictMsg={}
            dictMsg["x"]=resp.localizedPointMap.point.x
            dictMsg["y"]=resp.localizedPointMap.point.y
            dictMsg["z"]=resp.localizedPointMap.point.z
            self.coord_pub_json.publish(json.dumps(dictMsg))

            #Unregister to prevent continuously subscribing to camera feed.
            self.img_subscriber.unregister()
        elif(duration <self.TIMEOUT):
            print("Object not found.")
        else:
            print("Timeout.")
            #Unregister to prevent continuously subscribing to camera feed.
            self.img_subscriber.unregister()

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