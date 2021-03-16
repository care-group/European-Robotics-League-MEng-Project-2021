#!/usr/bin/python

# YOLO code initially adapted from https://www.codespeedy.com/yolo-object-detection-from-image-with-opencv-and-python/

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv.msg import SemanticLabel
from cv_bridge import CvBridge
import time
import numpy as np
import rospkg

# Commanded by /nav/semantic_labelling to look for all objects from the robot's camera feed using YOLO.
# Once objects are found, the labels and coordinates are published /semantic_labels, and a debugging image with bounding boxes is published to /semantic_labels/img
class Semantic_Labelling:
    def __init__(self):
        rospy.init_node('Semantic_Labelling')
        self.yolo_path = rospkg.RosPack().get_path('cv')+"/yolo/"
        self.init_yolo()
        self.bridge = CvBridge()

    def subscribe_jason(self):
        nav_subscriber = rospy.Subscriber('/nav/semantic_labelling', String,self.nav_callback)
    
    # Sets the target object and subscribes to the camera feed
    def nav_callback(self, msg):
        self.target=msg.data
        startingTime=time.time()
        self.img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback)
    
    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        semantic_labels,img = self.search_for_objects(cv_image)


        if(len(semantic_labels)!=0):

            #Pub to debug
            img_pub = rospy.Publisher('/semantic_labels/img', Image, queue_size=10)
            img_pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='passthrough'))
            
            coord_pub = rospy.Publisher('/semantic_labels', SemanticLabel, queue_size=10)
            for obj in semantic_labels:
                print(obj)
                coord_pub.publish(obj)            
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
        semantic_labels=[]
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = self.classes[class_ids[i]]  # name of the objects
                img = self.draw_obj(img,label,x,y,w,h,(0,255,0))
                
                semantic_label = SemanticLabel()
                semantic_label.label = label

                centerX,centerZ = self.get_center_coords(x,y,w,h)
                semantic_label.point.x = centerX
                semantic_label.point.y = 0
                semantic_label.point.z = centerZ

                semantic_labels.append(semantic_label)
        return semantic_labels,img

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

semantic_labelling = Semantic_Labelling()
semantic_labelling.subscribe_jason()
rospy.spin()