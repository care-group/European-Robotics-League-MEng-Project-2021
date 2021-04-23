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
        self.sub = rospy.Subscriber('/azm_nav/semantic_manual_add', String, self.cb)
        self.semantic_goal = rospy.Publisher('/azm_nav/semantic_goal_listener', String, queue_size=1)
        self.goal_sub = rospy.Subscriber('/azm_nav/goal_result', String, self.goal_cb)
        self.reached = True

    def publish_once(self, topic, msg, content="message"):
        rospy.loginfo("Attempting to publish {} to {}".format(content, topic.name))
        while not self.ctrl_c:
            connections = topic.get_num_connections()
            if connections > 0:
                topic.publish(msg)
                rospy.loginfo("Message published to {}".format(topic.name))
                break
            else:
                #rospy.loginfo("No subscribers on {}, sleeping.".format(topic.name))
                pass

    def cb(self, msg):
        _t = {"name":msg.data,"type":"test","coords":[1,2,3],"others":{}}
        _msg = String()
        _msg.data = obj_to_str(_t)
        self.publish_once(self.pub, _msg)

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def do_nav_example(self):
        goals = ["shelves", "grannyAnnie", "exit"]
        stage = 0
        stage_done = 0
        while not stage_done == len(goals):
            if self.reached:
                self.reached = False
                stage += 1
            if stage_done < stage:
                msg = String()
                msg.data = goals[stage_done]
                print("directing robot to {}".format(goals[stage_done]))
                self.publish_once(self.semantic_goal, msg, "goal")
                stage_done += 1
            rospy.sleep(0.5)
        print("all goals sent")
    
    def goal_cb(self, msg):
        if msg.data == 'success':
            print("reached goal")
            self.reached = True
        else:
            print("something went wrong with navigation")
            print(msg.data)


if __name__ == '__main__':
    print("executing tempnode.py as main")
    print("Creating tempnode obj")
    tempnode = tempnode()
    tempnode.do_nav_example()
    print("tempnode.py is spinning")
    rospy.spin()
