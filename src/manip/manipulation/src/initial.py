#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
completed = False
while(not completed):
    try:
        groupArm = moveit_commander.MoveGroupCommander("arm")
        groupGripper = moveit_commander.MoveGroupCommander("gripper")
        completed = True
    except: 
        print("Error Connecting to Commander")

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

def moveJoint(joint,direction):
    print(joint)
    print(direction)
    joint_goal = groupGripper.get_current_joint_values()
    print(joint_goal)
    if(direction == '1'):
        joint_goal[int(joint)-1] = joint_goal[int(joint)-1]+0.1
    if(direction == '0'):
        joint_goal[int(joint)-1] = joint_goal[int(joint)-1]-0.1
    #joint_goal[3] = -1
    try: 
        groupGripper.go(joint_goal, wait=True)
    except:
        print("Out of bounds")

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
        else:
            moveJoint(value[0],value[1])
    except Exception as e:
        print(e)
        print("invalid")
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