#! /usr/bin/env python

print("Starting tempnode.py")

import rospy
from std_msgs.msg import String
from azmutils import dynamic_euclid_dist, str_to_obj, obj_to_str
import json

class tempnode():
    """
    This class is adapted from theconstructsim.com ROS Basics in 5 Days course - Using Python Classes in ROS
    It implements a pseudo action server to move the HSR to coordinate nav goals 
    provided through the /azm_nav/coord_goal_listener topic
    
    Gives simple result feeback thru /azm_nav/goal_result
    """

    def __init__(self):
        # Base node inits
        rospy.loginfo("Initiating tempnode")
        rospy.init_node('tempnode')
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        rospy.on_shutdown(self.shutdownhook)
        # Goal publishing inits
        self.pub = rospy.Publisher('/azm_nav/semantic_label_additions', String, queue_size=1, latch=True)

    def publish_once(self, topic, msg, content="message"):
        rospy.loginfo("Attempting to publish {} to {}".format(content, topic.name))
        while not self.ctrl_c:
            connections = topic.get_num_connections()
            if connections > 0:
                topic.publish(msg)
                rospy.loginfo("Message published to {}".format(topic.name))
                break
            else:
                rospy.loginfo("No subscribers on {}, sleeping.".format(topic.name))

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True


if __name__ == '__main__':
    print("executing tempnode.py as main")
    print("Creating tempnode obj")
    tempnode = tempnode()
    topic = tempnode.pub
    msg = String()

    t = {"name":"stuff1","type":"object","coords":[2.7,0,30],"others":{}}
    msg.data = obj_to_str(t)
    print(msg)
    tempnode.publish_once(topic, msg)