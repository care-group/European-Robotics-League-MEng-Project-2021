#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image
import numpy as np
import codecs
import sys
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes
done = False
#np.set_printoptions(threshold=sys.maxsize)
class Get3DPosition(object):

    def __init__(self):
        self._subscriber = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',PointCloud2 ,self._showPoints)

    def _showPoints(self, msg):
        global done
        if(not(done)):
            print("Header: ",msg.header)
            print("Height: ",msg.height)
            print("Width: ",msg.width)
            print("Point Step: ",msg.point_step)
            print("Row Step: ",msg.row_step)
            print("Point Fields: ",msg.fields)
            data = pc2.read_points(msg, field_names = ("x", "y", "z", "rgb"), skip_nans=True)
        
            counter = 0
            for value in data:
                if 0.30<=round(value[0],2)<=0.37 and 0.5<=round(value[1],2)<=0.6 and 1.7<=round(value[2],2)<=1.8:
                    print " x : %f  y: %f  z: %f " %(value[0],value[1],value[2])
                    counter = counter +1
                    test = value[3] 
                    # cast float32 to int so that bitwise operations are possible
                    s = struct.pack('>f' ,test)
                    i = struct.unpack('>l',s)[0]
                    # you can get back the float value by the inverse operations
                    pack = ctypes.c_uint32(i).value
                    r = (pack & 0x00FF0000)>> 16
                    g = (pack & 0x0000FF00)>> 8
                    b = (pack & 0x000000FF)
                    #print(r,g,b) 
                
            print(counter)
            '''
            for value in data:
                print " x : %f  y: %f  z: %f rgb: %f" %(value[0],value[1],value[2],value[3])
                print(value[3])
            done=True
            '''
    
def main():
    rospy.init_node('get_3d_position')
    node = Get3DPosition()
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)

if __name__ == '__main__':
    main()