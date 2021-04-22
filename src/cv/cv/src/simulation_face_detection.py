#!/usr/bin/python

import rospy
from deepface import DeepFace
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import rospkg
import time

# Commanded by the jason agent /jason/detect_face, looks for a given face from the robot's camera feed using deepface.
# NB. It repeatedly looks for a face until either a face is found, or the timeout is reached (specified by TIMEOUT)
class Face_Detection:
    def __init__(self):
        rospy.init_node('Face_Detection')
        self.bridge = CvBridge()
        self.img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback)
        self.results_publisher = rospy.Publisher('/cv/face/personIs', String)
        self.db_base_path = rospkg.RosPack().get_path('cv')
        self.TIMEOUT = 60

    def subscribe_jason(self):
        jason_subscriber = rospy.Subscriber('/jason/detect_face', String,self.jason_callback)
    
    # Sets the target object and subscribes to the camera feed
    def jason_callback(self, msg):        
        startingTime=time.time()
        
        image_connected = False
        while not image_connected:
            try:
                self.cv_image.shape #If subscriber hasn't received image, accessing cv_image returns AttributeError
                image_connected = True
            except AttributeError:
                print("Waiting for camera feed.")

        
        person = self.get_person_from_face(self.cv_image)

        duration = time.time()-startingTime

        if(duration >self.TIMEOUT or person!=None):
            self.publish_results(person)
            self.img_subscriber.unregister()


    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        

    def get_person_from_face(self,img):
        faces = "samuel","bezos","simulated"
        for face in faces:
            try:
                same = self.search_for_face_db(img,self.db_base_path+"/images/faces/"+face)
                if same==True:
                    print(face + "'s face detected!")
                    return face
                else:
                    print("Not " + face)
             # No face detected, return immediately    
            except ValueError:
                print("No face detected.")
                return None
        return None

    def search_for_face_db(self,img,path):
        result  = DeepFace.find(img,db_path=path)
        return (result.empty == False)

    def publish_results(self,person):
        if(person!=None):
            self.results_publisher.publish(person)
        else:
            self.results_publisher.publish("unknown")

face_detection = Face_Detection()
face_detection.subscribe_jason()
rospy.spin()