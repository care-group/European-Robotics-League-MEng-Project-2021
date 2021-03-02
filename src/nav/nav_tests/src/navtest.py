#! /usr/bin/env python

print("Starting navtest.py")

import rospy
import math
import tf
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult

def quaternion_from_euler(roll, pitch, yaw):
    '''
    From HSR's utils.py
    '''
    q = tf.transformations.quaternion_from_euler(roll / 180.0 * math.pi,
                                                 pitch / 180.0 * math.pi,
                                                 yaw / 180.0 * math.pi, 'rxyz')
    
    return Quaternion(q[0], q[1], q[2], q[3])


class SimpleMoveBase():
    """
    This class is adapted from theconstructsim.com ROS Basics in 5 Days course - Using Python Classes in ROS
    It implements a pseudo action server to move the HSR to coordinate nav goals 
    provided through the /azm_nav/coord_goal_listener topic
    
    Gives simple result feeback thru /azm_nav/goal_result
    """

    def __init__(self, openListenerTopic=True):
        rospy.loginfo("Initiating basic_coordinate_goal_nav_node")
        rospy.init_node('basic_semantic_nav_node')
        self.move_base_simple_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.goal = PoseStamped()
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        self.move_base_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_result_cb)
        self.goal_result_pub = rospy.Publisher('/azm_nav/goal_result', String, queue_size=1)
        self.result_fb = String()
        self.result_flag = False # Toggles on/off listening to goal result messages
        if openListenerTopic:
            #start_topic_listener()
            self.goal_listener = rospy.Subscriber('/azm_nav/coord_goal_listener', Float64MultiArray, self.goal_callback)
    
    # TODO make it stop trying to send the message after x number of attempts
    # TODO add error checking if msg doesnt match topic type
    # TODO DEBUG maybe make an ID to link attempt to success?
    def publish_once(self, topic, msg, content="message"):
        """
        Adapted from theconstruct's ros in 5 days course
        will keep retrying to publish the message if the publisher has no connections

        Args:
            topic (rospy publisher object): topic object to publish to
            msg (rospy message object): message object to publish 
            content (String): very short description of message for debug purposes
        """
        rospy.loginfo("Attempting to publish {} to {}".format(content, topic.name))
        while not self.ctrl_c:
            connections = topic.get_num_connections()
            if connections > 0:
                topic.publish(msg)
                rospy.loginfo("Message published to {}".format(topic.name))
                break
            else:
                rospy.loginfo("No subscribers on {}, sleeping.".format(topic.name))
                self.rate.sleep()

    def simple_move_base_goal(self, x=0, y=0, theta=0):
        # Set message variables
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.orientation = quaternion_from_euler(0, 0, theta)

        # Toggles on listening to goal result messages
        #rospy.sleep(2)
        rospy.loginfo("Started listening for goal result")
        self.result_flag = True
        # Publish
        self.publish_once(self.move_base_simple_publisher, self.goal, "goal")

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

    # TODO maybe change the message to include more information (like what the goal was and/or coords)
    # TODO make it react to all possible results "rosmsg info MoveBaseActionResult"
    # might want to use goal ids in case there are unexpected overlaps
    # like sending two goals at the same time or something
    def goal_result_cb(self, msg):
        ''' Listens for /move_base/status messages '''
        if self.result_flag:
            if msg.status.status == 3:
                self.result_fb.data = 'success'
            else:
                self.result_fb.data = msg.status.text
            self.result_flag = False
            self.publish_once(self.goal_result_pub, self.result_fb, "result_feedback")
            rospy.loginfo("Stopped listening to goal result")

def run_move_to_coords(obj, x, y, w):
    print("sending goal command")
    obj.simple_move_base_goal(x, y, w)

if __name__ == '__main__':
    print("executing navtest.py as main")
    print("Creating SimpleMoveBase obj")
    simple_move_obj = SimpleMoveBase()
    #run_move_to_coords(simple_move_obj, 0, 0, 0)
    rospy.loginfo("navtest.py is spinning")
    rospy.spin()