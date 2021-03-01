import cv2
import numpy as np
import pandas as pd
import rospy
from deepface import DeepFace
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Commanded by the jason agent /jason/detect_face to look for a given face from the robot's camera feed using YOLO.
class Face_Detection:
    def __init__(self):
        rospy.init_node('Face_Detection')
        self.bridge = CvBridge()
        self.results_publisher = rospy.Publisher('/cv/face/personIs', String, queue_size=10)

    def subscribe_jason(self):
        jason_subscriber = rospy.Subscriber('/jason/detect_face', String,self.jason_callback)
    
    # Sets the target object and subscribes to the camera feed
    def jason_callback(self, msg):
        self.target = msg.data
        self.img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback)
    
    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # faces = "kimble","postman","deliman","plumber"
        person = self.get_person_from_face(cv_image)
        self.publish_results(person)
        self.img_subscriber.unregister()

    def get_person_from_face(self,img):
        faces = "samuel","bezos","simulated"
        for face in faces:
            try:
                same = self.search_for_face_db(img,"images/faces/"+face)
                if same==True:
                    print(face + "'s face detected!")
                    return face
                else:
                    print("Not " + face)
             # No face detected, return immediately    
            except ValueError:
                return None

        return None

    def search_for_face_db(self,img,path):
        result  = DeepFace.find(img,db_path=path)
        return (result.empty == False)

    def publish_results(self,person):

        if(person!=None):
            print("publishing that " +person+" is found")
            self.results_publisher.publish(person)
        else:
            self.results_publisher.publish("unknown")


face_detection = Face_Detection()
face_detection.subscribe_jason()
rospy.spin()