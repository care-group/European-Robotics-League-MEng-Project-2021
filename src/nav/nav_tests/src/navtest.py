#! /usr/bin/env python

print("Starting navtest.py")

import rospy
import math
import tf
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion

def quaternion_from_euler(roll, pitch, yaw):
    '''
    From HSR's utils.py
    '''
    q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                 pitch / 180.0 * math.pi,
                                                 yaw / 180.0 * math.pi, 'rxyz')
    
    return Quaternion(q[0], q[1], q[2], q[3])


class SimpleMoveBase():
    ''' 
    This class is adapted from theconstructsim.com ROS Basics in 5 Days course - Using Python Classes in ROS
    It implements a pseudo action server to move the HSR to coordinate nav goals 
    provided through the /azm_nav/coord_goal_listener topic
    '''

    def __init__(self, openListenerTopic=True):
        rospy.loginfo("Initiating basic_coordinate_goal_nav_node")
        rospy.init_node('basic_semantic_nav_node')
        self.move_base_simple_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.goal = PoseStamped()
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        if openListenerTopic:
            #start_topic_listener()
            self.goal_listener = rospy.Subscriber('/azm_nav/coord_goal_listener', Float64MultiArray, self.goal_callback)
    
    def publish_once(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        rospy.loginfo("Attempting to publish goal")
        while not self.ctrl_c:
            connections = self.move_base_simple_publisher.get_num_connections()
            if connections > 0:
                self.move_base_simple_publisher.publish(self.goal)
                rospy.loginfo("Goal Published")
                break
            else:
                self.rate.sleep()

    def simple_move_base_goal(self, x=0, y=0, theta=0):
        # Set message variables
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.orientation = quaternion_from_euler(0, 0, theta)

        # Publish
        self.publish_once()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def goal_callback(self, msg):
        ''' Listens for coord goals to move to '''
        if len(msg.data) != 3:
            rospy.loginfo("Goal Message received without exactly 3 inputs, ignoring.")
        else:
            rospy.loginfo("Goal position received, moving?")
            self.simple_move_base_goal(msg.data[0], msg.data[1], msg.data[2])


def run_move_to_coords(obj, x, y, w):
    print("sending goal command")
    obj.simple_move_base_goal(x, y, w)

if __name__ == '__main__':
    rospy.loginfo("executing navtest.py as main")
    print("executing navtest.py as main")
    print("Creating SimpleMoveBase obj")
    simple_move_obj = SimpleMoveBase()
    #run_move_to_coords(simple_move_obj, 0, 0, 0)
    rospy.loginfo("navtest.py is spinning")
    rospy.spin()