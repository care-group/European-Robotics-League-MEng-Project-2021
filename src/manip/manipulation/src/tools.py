#! /usr/bin/env python

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
#sys.path.insert(1, '/home/developer/workspace/src/nav/nav_tests/src')
#import navtest as nav
moveit_commander.roscpp_initialize(sys.argv)
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


finish = False
def armInitialState():
    joint_goal = groupArm.get_current_joint_values()
    joint_goal[0]=0.27
    joint_goal[1]=-1.56
    joint_goal[2]=-1.55
    joint_goal[3]=0.10
    joint_goal[4]=1.50
    joint_goal[5]=0.00
    try: 
        groupArm.go(joint_goal, wait=True)
    except:
        print("Out of bounds")

def armGraspedState():
    joint_goal = groupArm.get_current_joint_values()
    joint_goal[0]=0.27
    joint_goal[1]=-1.56
    joint_goal[2]=-1.55
    joint_goal[3]=0.10
    joint_goal[4]=1.50
    joint_goal[5]=0.00
    try: 
        groupArm.go(joint_goal, wait=True)
    except:
        print("Out of bounds")

def moveJoint(joint,direction):
    print(joint)
    print(direction)
    joint_goal = groupGripper.get_current_joint_values()
    print(joint_goal)
    if(direction == '1'):
        joint_goal[int(joint)-1] = joint_goal[int(joint)-1]+0.1
    if(direction == '0'):
        joint_goal[int(joint)-1] = joint_goal[int(joint)-1]-0.1
    try: 
        groupGripper.go(joint_goal, wait=True)
    except:
        print("Out of bounds")
        return False
    return True
def getEndEffectorPose():
    end_effector_value = groupArm.get_current_pose().pose
    return end_effector_value

def closeGripper():
    joint_goal = groupGripper.get_current_joint_values()
    joint_goal[0]=0.04
    joint_goal[1]=-0.278
    joint_goal[2]=0.19
    joint_goal[3]=0.043
    joint_goal[4]=-0.2788
    try: 
        groupGripper.go(joint_goal, wait=True)
    except:
        print("Out of bounds")

def openGripper():
    joint_goal = groupGripper.get_current_joint_values()
    joint_goal[0]=0.014
    joint_goal[1]=-0.75
    joint_goal[2]=0.666
    joint_goal[3]=0.0018
    joint_goal[4]=-0.75
    try: 
        groupGripper.go(joint_goal, wait=True)
    except:
        print("Out of bounds")

def move_base_vel (vx, vy, vw):
    # Set velocity command values
    twist = Twist ()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw / 180.0 * math.pi # Convert from "degree" to "radian"
    base_vel_pub.publish (twist) # Publish velocity command
    rate = rospy.Rate(1) # 1hz
    rate.sleep()
    twist = Twist ()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0 / 180.0 * math.pi # Convert from "degree" to "radian"
    base_vel_pub.publish (twist) # Publish velocity command

def initListenerToPointCloud():
    subscriber = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',PointCloud2 ,definePointCloud)    

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
        limit = 5
        for value in data:
            if limitCounter == limit:
                limitCounter = 0
                p = PointStamped()
                p.header.frame_id = robot.get_planning_frame()
                p.pose.position.x = value[0]
                p.pose.position.y = value[1]
                p.pose.position.z = value[2]
                scene.add_box("point"+str(counter), p, (0.01, 0.01, 0.01))
                counter = counter + 1
                completed = True
            limitCounter = limitCounter + 1
        print("completed scene") 

if __name__ == '__main__':
    groupGripper.set_joint_value_target("hand_motor_joint", 0.0)
    groupGripper.go()
    groupArm.set_named_target('neutral')
    groupArm.go()
    pose = groupWholeBody.get_current_pose()
    print(pose)
    graspPose = robotPoseToMapPose(pose)
    groupGripper.set_joint_value_target("hand_motor_joint", 1.0)
    groupGripper.go()
    #initListenerToPointCloud()
    groupWholeBody.set_planning_time(20)
    groupWholeBody.set_workspace([-3.0, -3.0, 3.0, 3.0])
    #groupWholeBody.set_planner_id("TRAC_IKKConfigDefault")
    groupWholeBody.set_planner_id("RRTConnectkConfigDefault")
    completed = False
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x = 2.14
    p.pose.position.y = 4.72
    #p.pose.position.x = 2.44
    #p.pose.position.y = 0.8
    p.pose.position.z = 0.538
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
    
    '''
    end_effector_value = groupArm.get_current_pose().pose
    end_effector_value.position.z = end_effector_value.position.z + 0.1
    groupArm.clear_pose_targets()
    groupArm.set_pose_target(end_effector_value)
    groupArm.go()
    '''
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)
    
    '''
    groupWholeBody.set_planning_time(20)
    groupWholeBody.set_workspace([-3.0, -3.0, 3.0, 3.0])
    #groupWholeBody.set_planner_id("TRAC_IKKConfigDefault")
    groupWholeBody.set_planner_id("RRTConnectkConfigDefault")
    completed = False
    end_effector_value = groupWholeBody.get_current_pose().pose
    print(end_effector_value)
    end_effector_value.position.x = 0.4
    end_effector_value.position.y = 1.50
    end_effector_value.position.z = 0.6
    end_effector_value.orientation.w = 0.5
    groupWholeBody.clear_pose_targets()
    groupWholeBody.set_pose_target(end_effector_value)
    groupWholeBody.set_goal_tolerance(0.1)
    plan= groupWholeBody.plan(end_effector_value)
    groupWholeBody.execute(plan)
    end_effector_value = groupWholeBody.get_current_pose().pose
    print(end_effector_value)    
    '''
    '''
    waypoints = []

    # start with the current pose
    waypoints.append(groupArm.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x + 1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    # third move to the side
    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))
    groupArm.set_planning_time(10000)
    (plan3, fraction) = groupArm.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.1,        # eef_step
                             0.1)         # jump_threshold
    print(plan3)
    count = 1
    for point in plan3.joint_trajectory.points:
        print(point)
        point.time_from_start.secs = count * 5
        count=count+1
    try:
        groupArm.execute(plan3,wait=True)
    except:
        print("got error")
    groupArm.clear_pose_targets()
    
    while not finish:
        value = str(raw_input("Enter the number of the joint to move followed by a 1 or a 0 to move in respective direction. Type exit to close the node or close or open for the gripper and initial to move the arm to the initial state"))
        try:
            if value == 'exit':
                break
            elif value == 'open':
                openGripper()
            elif value == 'close':
                closeGripper()
            elif value == 'initial':
                armInitialState()
            elif value == 'move':
                x=input("X: ")
                y=input("Y: ")
                w=input("W: ")
                move_base_vel(x,y,w)
            else:
                moveJoint(value[0],value[1])
        except Exception as e:
            print(e)
            print("invalid")
    '''

'''
end_effector_value = group.get_current_pose().pose
print(end_effector_value)
end_effector_value.position.x = 0.1
end_effector_value.position.y = -0.04
end_effector_value.position.z = 0.65
end_effector_value.orientation.x = 0.50
end_effector_value.orientation.y = -0.50
end_effector_value.orientation.z = 0.51
end_effector_value.orientation.w = 0.50


group.set_pose_target(end_effector_value)
group.set_planning_time(10000)
group.set_planner_id("RRTstarkConfigDefault")
plan= group.plan()
group.execute(plan,wait=True)

waypoints = []
scale=1
wpose = group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)  
group.execute(plan,wait=True)

end_effector_value = group.get_current_pose().pose
print(end_effector_value)
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
'''