#! /usr/bin/env python2.7

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_geometry_msgs 
#import utils
import time
import math
import numpy as np
from std_msgs.msg import String
import json 
#baseMover = nav.SimpleMoveBase()
#rospy.init_node('move_group_python', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
base_vel_pub = rospy.Publisher ('/hsrb/command_velocity', Twist, queue_size=1)
completed = False
completed1 = False
completed2 = False
completed3 = False
pointCloud = None
graspPose = None
rospy.init_node('Test')
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

while(not completed2):
    try:
        groupGripper = moveit_commander.MoveGroupCommander("gripper")
        print("Connected to commander gripper")
        completed2 = True
        time.sleep(1)
    except: 
        print("Error Connecting to gripper commander")
        time.sleep(1)
while(not completed1):
    try:
        groupWholeBody = moveit_commander.MoveGroupCommander("whole_body")
        completed1 = True
        print("Connected to commander whole_body")
    except: 
        print("Error Connecting to whole body commander")
        time.sleep(1)
while(not completed3):
    try:
        groupArm = moveit_commander.MoveGroupCommander("arm")
        completed3 = True
        print("Connected to commander arm")
    except: 
        print("Error Connecting to arm commander")
        time.sleep(1)

def initListenerToPointCloud():
    subscriber = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',PointCloud2 ,definePointCloud)    

def initListenerToGraspingTarget():
    subscriber = rospy.Subscriber('/graspingTarget',String,graspMotion)

def definePointCloud(msg):
        global pointCloud 
        pointCloud = msg

def _transform_pose(self,value, from_frame, to_frame):
        listener = tf.TransformListener()
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0),rospy.Duration(4.0))
        point=PointStamped()
        point.header.frame_id = from_frame
        point.header.stamp =rospy.Time(0)
        point.point.x=value[0]
        point.point.y=value[1]
        point.point.z=value[2]
        point=listener.transformPoint(to_frame,point)
        return point

def mapPoseToRobotPose(pose):
    transform = tf_buffer.lookup_transform("odom",
                                       pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
    return pose_transformed

def robotPoseToMapPose(pose):
    transform = tf_buffer.lookup_transform("map",
                                       pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
    return pose_transformed

def pointcloudToPlanningScene(msg):
    global pointCloud
    global completed
    msg = pointCloud
    if not completed:
        try:
            trans = tf_buffer.lookup_transform("map", msg.header.frame_id,
                                            rospy.Time(0),
                                            rospy.Duration(4))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(msg, trans)
        rospy.sleep(2)
        scene.remove_world_object()
        rospy.sleep(2)
        data = pc2.read_points(cloud_out, field_names = ("x", "y", "z", "rgb"), skip_nans=True)
        counter = 0
        limitCounter = 0
        limit = 3
        for value in data:
            if limitCounter == limit:
                limitCounter = 0
                p = PoseStamped()
                p.header.frame_id = robot.get_planning_frame()
                p.pose.position.x = value[0]
                p.pose.position.y = value[1]
                p.pose.position.z = value[2]
                scene.add_box("point"+str(counter), p, (0.01, 0.01, 0.01))
                counter = counter + 1
                completed = True
            limitCounter = limitCounter + 1
        print("completed scene") 

def graspMotion(msg):
    #pointcloudToPlanningScene()
    groupGripper.set_joint_value_target("hand_motor_joint", 0.0)
    groupGripper.go()
    groupArm.set_named_target('neutral')
    groupArm.go()
    pose = groupWholeBody.get_current_pose()
    print(pose)
    graspPose = robotPoseToMapPose(pose)
    groupGripper.set_joint_value_target("hand_motor_joint", 1.0)
    groupGripper.go()
    groupWholeBody.set_planning_time(20)
    groupWholeBody.set_workspace([-3.0, -3.0, 3.0, 3.0])
    #groupWholeBody.set_planner_id("TRAC_IKKConfigDefault")
    groupWholeBody.set_planner_id("RRTConnectkConfigDefault")
    completed = False

    #my_dict=json.loads(msg.data)
    #print(my_dict)
    p = PoseStamped()
    p.header.frame_id = "map"
    #p.pose.position.x = float(my_dict["x"])
    #p.pose.position.y = float(my_dict["y"])
    #p.pose.position.z = float(my_dict["z"])+0.02
    p.pose.position.x = 1.62
    p.pose.position.y = 0.3
    p.pose.position.z = 0.43
    p.pose.orientation.x =graspPose.pose.orientation.x
    p.pose.orientation.y =graspPose.pose.orientation.y
    p.pose.orientation.z =graspPose.pose.orientation.z
    p.pose.orientation.w =graspPose.pose.orientation.w
    groupWholeBody.clear_pose_targets()
    groupWholeBody.set_pose_target(mapPoseToRobotPose(p))
    groupWholeBody.set_goal_tolerance(0.01)
    plan= groupWholeBody.plan()
    groupWholeBody.execute(plan)
    end_effector_value = groupWholeBody.get_current_pose()
    print(robotPoseToMapPose(end_effector_value)) 
    groupGripper.set_joint_value_target("hand_motor_joint", 0.3)
    groupGripper.go()
    pubFeedback = rospy.Publisher('/feedbackOnGrasping', String, queue_size=10)
    pubFeedback.publish("True") 

def graspMotion1():
    #pointcloudToPlanningScene()
    groupGripper.set_joint_value_target("hand_motor_joint", 0.0)
    groupGripper.go()
    groupArm.set_named_target('neutral')
    groupArm.go()
    pose = groupWholeBody.get_current_pose()
    print(pose)
    graspPose = robotPoseToMapPose(pose)
    groupGripper.set_joint_value_target("hand_motor_joint", 1.0)
    groupGripper.go()
    groupWholeBody.set_planning_time(20)
    groupWholeBody.set_workspace([-3.0, -3.0, 3.0, 3.0])
    #groupWholeBody.set_planner_id("TRAC_IKKConfigDefault")
    groupWholeBody.set_planner_id("RRTConnectkConfigDefault")
    completed = False

    #my_dict=json.loads(msg.data)
    #print(my_dict)
    p = PoseStamped()
    p.header.frame_id = "map"
    #p.pose.position.x = float(my_dict["x"])
    #p.pose.position.y = float(my_dict["y"])
    #p.pose.position.z = float(my_dict["z"])+0.02
    p.pose.position.x = 1.64
    p.pose.position.y = -0.04
    p.pose.position.z = 0.44
    p.pose.orientation.x =graspPose.pose.orientation.x
    p.pose.orientation.y =graspPose.pose.orientation.y
    p.pose.orientation.z =graspPose.pose.orientation.z
    p.pose.orientation.w =graspPose.pose.orientation.w
    groupWholeBody.clear_pose_targets()
    groupWholeBody.set_pose_target(mapPoseToRobotPose(p))
    groupWholeBody.set_goal_tolerance(0.01)
    plan= groupWholeBody.plan()
    groupWholeBody.execute(plan)
    end_effector_value = groupWholeBody.get_current_pose()
    print(robotPoseToMapPose(end_effector_value)) 
    groupGripper.set_joint_value_target("hand_motor_joint", 0.3)
    groupGripper.go()
    groupArm.set_named_target('neutral')
    pubFeedback = rospy.Publisher('/feedbackOnGrasping', String, queue_size=10)
    pubFeedback.publish("True") 

def placeMotion1():
    #pointcloudToPlanningScene()
    pose = groupWholeBody.get_current_pose()
    print(pose)
    graspPose = robotPoseToMapPose(pose)
    groupGripper.set_joint_value_target("hand_motor_joint", 1.0)
    groupGripper.go()
    groupWholeBody.set_planning_time(20)
    groupWholeBody.set_workspace([-3.0, -3.0, 3.0, 3.0])
    #groupWholeBody.set_planner_id("TRAC_IKKConfigDefault")
    groupWholeBody.set_planner_id("RRTConnectkConfigDefault")
    completed = False

    #my_dict=json.loads(msg.data)
    #print(my_dict)
    p = PoseStamped()
    p.header.frame_id = "map"
    #p.pose.position.x = float(my_dict["x"])
    #p.pose.position.y = float(my_dict["y"])
    #p.pose.position.z = float(my_dict["z"])+0.02
    p.pose.position.x = 1.83
    p.pose.position.y = 0.423
    p.pose.position.z = 0.44
    p.pose.orientation.x =graspPose.pose.orientation.x
    p.pose.orientation.y =graspPose.pose.orientation.y
    p.pose.orientation.z =graspPose.pose.orientation.z
    p.pose.orientation.w =graspPose.pose.orientation.w
    groupWholeBody.clear_pose_targets()
    groupWholeBody.set_pose_target(mapPoseToRobotPose(p))
    groupWholeBody.set_goal_tolerance(0.01)
    plan= groupWholeBody.plan()
    groupWholeBody.execute(plan)
    end_effector_value = groupWholeBody.get_current_pose()
    print(robotPoseToMapPose(end_effector_value)) 
    groupGripper.set_joint_value_target("hand_motor_joint", 1.0)
    groupGripper.go()
    groupArm.set_named_target('neutral')
    pubFeedback = rospy.Publisher('/feedbackOnPlacing', String, queue_size=10)
    pubFeedback.publish("True") 

if __name__ == '__main__':
    #initListenerToPointCloud()
    #initListenerToGraspingTarget()
    graspMotion1()
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)
    