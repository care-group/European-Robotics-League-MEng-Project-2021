#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from getting3DPosition import Get3DPosition
from geometry_msgs.msg import PointStamped
import tf
from tools import armInitialState, closeGripper, openGripper, move_base_vel, getEndEffectorPose, moveJoint

class graspingAction():
    def __init__(self):
        self._subscriber = rospy.Subscriber(
                '/TargetPoint',PointStamped ,self._grasp)
        print("Initiated target point listener")

    def _get3D(self):
        depthTool =Get3DPosition()
        print(depthTool._get3DPointMap(-0.149,0.9,"map"))

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
        #self._pointPublisher(point)

    def _grasp(self,msg):
        error = 0.01
        getEndEffectorPose()
        print("Point received for grasping")
        armInitialState()
        openGripper()
        pose = getEndEffectorPose()
        print("End effector pose")
        pose= self._transform_pose([pose.position.x,pose.position.y,pose.position.z],"odom","map")
        print(pose.point.z, msg.point.z)  
        print(pose)
        if pose.point.z > msg.point.z:
            while not (msg.point.z-error < pose.point.z < msg.point.z-error):
                if not moveJoint('1','0'):
                    break
                pose = getEndEffectorPose()
                pose= self._transform_pose([pose.position.x,pose.position.y,pose.position.z],"odom","map")
                print(pose.point.z, msg.point.z)
        if pose.point.z < msg.point.z:
            while not (msg.point.z-error < msg.point.z < msg.point.z-error):
                if not moveJoint('1','1'):
                    break
                pose = getEndEffectorPose()
                pose= self._transform_pose([pose.position.x,pose.position.y,pose.position.z],"odom","map")
                print(pose.point.z, msg.point.z)
        print("Alignment completed")
        print(pose.point.z, msg.point.z)    
        
        objectPose = self._transform_pose([msg.point.x,msg.point.y,msg.point.z],"map","base_link")
        if(objectPose.point.y<0-error):
            while not(-error<objectPose.point.y<error):
                move_base_vel(0,0,-5)
                objectPose = self._transform_pose([msg.point.x,msg.point.y,msg.point.z],"map","base_link")
                print(objectPose.point.y)
        if(objectPose.point.y>0+error):
            while not(-error<objectPose.point.y<error):
                move_base_vel(0,0,5)
                objectPose = self._transform_pose([msg.point.x,msg.point.y,msg.point.z],"map","base_link")
                print(objectPose.point.y)
        print("Point in base link frame")
        print(objectPose.point.y)
        pose = getEndEffectorPose()
        pose= self._transform_pose([pose.position.x,pose.position.y,pose.position.z],"odom","map")
        while((pose.x-msg.point.x)+(pose.z-msg.point.z))
        
        




def main():
    rospy.init_node('GraspObject')
    node = graspingAction()
    node._get3D()
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)

if __name__ == '__main__':
    main()