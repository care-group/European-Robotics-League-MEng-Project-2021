#! /usr/bin/env python

import math
import moveit_commander
import rospy
import tf
from utils2 import *
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from geometry_msgs.msg import Pose
rospy.init_node("whole_body")



def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
    
#get the position of the object from a topic
def graspObject(pose):
    transformed_pose = transform_pose(my_pose, "fixture", "world")

    whole_body.set_pose_target(p)
    whole_body.set_planning_time(100)
    whole_body.set_planner_id("RRTConfigDefault")
    plan = whole_body.plan()
    whole_body.execute(plan)
    print(whole_body.get_current_pose().pose)