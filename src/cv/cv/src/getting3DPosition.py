#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import PointStamped
from cv.srv import LocalizePoint

class Get3DPosition(object):
    def __init__(self):
        s = rospy.Service('get_3d_position',LocalizePoint,self._get3DPointMapFromTopic)

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


    def _get3DPointMapFromTopic(self,msg):
        xyz = [msg.unlocalizedPoint.point.x,msg.unlocalizedPoint.point.y,msg.unlocalizedPoint.point.z]
        
        mapP = self._transform_pose(xyz,"head_rgbd_sensor_rgb_frame", "map")
        odomP = self._transform_pose(xyz,"head_rgbd_sensor_rgb_frame", "odom")

        return {'localizedPointMap':mapP, 'localizedPointOdom':odomP}


def main():
    rospy.init_node('get_3d_position')
    node = Get3DPosition()
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)

if __name__ == '__main__':
    main()