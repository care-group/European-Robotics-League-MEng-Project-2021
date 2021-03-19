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


# Commanded by the jason agent /jason/detect_object to look for a given object from the robot's camera feed using YOLO.
# Once found, the coordinates are published /yolo/<target>, and a debugging image with bounding boxes is published to /yolo/<target>/img
class Object_Detection:
    def __init__(self):
        rospy.init_node('Object_Detection')
        self.yolo_path = rospkg.RosPack().get_path('cv')+"/yolo/"
        self.init_yolo()
        self.target = ""
        self.bridge = CvBridge()
        self.TIMEOUT = 60
        self.coord_pub = rospy.Publisher('/cv/obj_2d_position', PointStamped, queue_size=10,latch=True)
        self.img_pub = rospy.Publisher('/yolo/img', Image, queue_size=1,latch=True)

    def subscribe_jason(self):
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
        obj_coords,img = self.search_for_objects(cv_image)
        duration = time.time()-args[0]

        if(duration >self.TIMEOUT or len(obj_coords)!=0):

            print(obj_coords)
            
            #Pub image with bounding boxes debug
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='passthrough'))
    
            model = PinholeCameraModel()
            model.fromCameraInfo(self.cam_info)
            xyz = model.projectPixelTo3dRay((obj_coords[0][0],obj_coords[0][1]))  # Has potential to return multiple objects, so use [0] element
            
            stampedPoint = PointStamped()
            stampedPoint.header=msg.header
            stampedPoint.point.x=xyz[0]
            stampedPoint.point.y=0
            stampedPoint.point.z=xyz[1]
            
            self.coord_pub.publish(stampedPoint) 

            #Unregister to prevent continuously subscribing to camera feed.
            self.img_subscriber.unregister()
        else:
            print("Object not found.")

    def init_yolo(self):
        #Load YOLO Algorithm
        self.net=cv2.dnn.readNet(self.yolo_path+"yolov3.weights",self.yolo_path+"yolov3.cfg")
        #To load all objects that have to be detected
        self.classes=[]
        with open(self.yolo_path+"coco.names","r") as f:
            read=f.readlines()
        for i in range(len(read)):
            self.classes.append(read[i].strip("\n"))
        #Defining layer names
        self.layer_names=self.net.getLayerNames()
        self.output_layers=[]
        for i in self.net.getUnconnectedOutLayers():
            self.output_layers.append(self.layer_names[i[0]-1])

    def search_for_objects(self,img):
        height,width,channels=img.shape
        #Extracting features to detect objects
        blob=cv2.dnn.blobFromImage(img,0.00392,(416,416),(0,0,0),True,crop=False)
                                                                #Inverting blue with red
                                                                #bgr->rgb
        #We need to pass the img_blob to the algorithm
        self.net.setInput(blob)
        outs=self.net.forward(self.output_layers)
        #print(outs)
        #Displaying informations on the screen
        class_ids=[]
        confidences=[]
        boxes=[]
        for output in outs:
            for detection in output:
                #Detecting confidence in 3 steps
                scores=detection[5:]                #1
                class_id=np.argmax(scores)          #2
                confidence =scores[class_id]        #3
                if confidence >0.5: #Means if the object is detected
                    center_x=int(detection[0]*width)
                    center_y=int(detection[1]*height)
                    w=int(detection[2]*width)
                    h=int(detection[3]*height)
                    #Drawing a rectangle
                    x=int(center_x-w/2) # top left value
                    y=int(center_y-h/2) # top left value
                    boxes.append([x,y,w,h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        #Removing Double Boxes
        indexes=cv2.dnn.NMSBoxes(boxes,confidences,0.3,0.4)
        target_obj_positions=[]
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = self.classes[class_ids[i]]  # name of the objects
                if(self.is_target_obj(label)):
                    img = self.draw_obj(img,label,x,y,w,h,(0,255,0))
                    target_obj_positions.append(self.get_center_coords(x,y,w,h))
        return target_obj_positions,img

    def get_center_coords(self,x,y,w,h):
        return int(x+0.5*w),int(y+0.5*h)

    def draw_obj(self,img,label, x,y,w,h,colour):
        cv2.rectangle(img, (x, y), (x + w, y + h), colour, 2)
        cv2.putText(img, label, (x, y), cv2.FONT_HERSHEY_PLAIN, 1, colour, 2)
        
        # Draw center marker
        center_x,center_y = self.get_center_coords(x,y,w,h)
        cv2.circle(img, (center_x,center_y), radius=5, color=(0, 0, 255), thickness=-1)

        return img

    def is_target_obj(self,obj):
        return obj == self.target

obj_detection = Object_Detection()
obj_detection.subscribe_jason()
rospy.spin()