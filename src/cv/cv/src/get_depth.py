#!/usr/bin/python

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import rospy

class Depth_Finder:
    def __init__(self):
        depth_subscriber = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points',PointCloud2,self.pc_callback,queue_size=None)

    def pc_callback(self, msg):
        self.pc = msg

    def get_depth(self,x, y):
        gen = pc2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[(x, y)])
        return next(gen)
