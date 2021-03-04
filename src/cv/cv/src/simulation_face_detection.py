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
        self.results_publisher = rospy.Publisher('/cv/face/personIs', String, queue_size=10)
        self.img_pub = rospy.Publisher('/cv/image', Image, queue_size=10)
        self.db_base_path = rospkg.RosPack().get_path('cv')

        self.TIMEOUT = 60

    def subscribe_jason(self):
        jason_subscriber = rospy.Subscriber('/jason/detect_face', String,self.jason_callback)
    
    # Sets the target object and subscribes to the camera feed
    def jason_callback(self, msg):        
        startingTime=time.time()
        self.img_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image,self.img_callback,[startingTime])

    # Takes the current image from the camera feed and searches for the target object, returning coordinates 
    def img_callback(self, msg,args):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.img_pub.publish(msg) #Publish what the component sees for debugging (as there is a delay due to system performance)

        person = self.get_person_from_face(cv_image)

        duration = time.time()-args[0]

        if(duration >self.TIMEOUT or person!=None):
            self.publish_results(person)
            self.img_subscriber.unregister()

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