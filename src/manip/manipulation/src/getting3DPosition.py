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
import tf
import time
from geometry_msgs.msg import PointStamped
done = False
pointCloud = None
#np.set_printoptions(threshold=sys.maxsize)

# Class for obtaining the depth component of a 2D detected object
# Once the class is instantiated the point cloud will be updated continously
# In order to get the 3D resulting point which include the depth component, 
# call the _get3DPointMap giving the X and Z coordinates and the initial reference frame.
# It will return the point referenced to the Map coordinate frame
class Get3DPosition(object):
    def __init__(self):
        self._subscriber = rospy.Subscriber(
            '/hsrb/head_rgbd_sensor/depth_registered/rectified_points',PointCloud2 ,self._definePointCloud)
        self._subscriber = rospy.Subscriber(
            '/cv/obj_2d_position',PointStamped ,self._get3DPointMapFromTopic)

    def _pointPublisher(self,point):
        pub = rospy.Publisher('TargetPoint', PointStamped, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():  
            pub.publish(point) 
            rate.sleep()

    def _pointPublisherCV(self,point):
        pub = rospy.Publisher('3DLocatedPoint', PointStamped, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():  
            pub.publish(point) 
            rate.sleep()

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

    def _inRange(self,data,error,targetX,targetY):
        for value in data:
            #print " x : %f  y: %f  z: %f " %(value[0],value[1],value[2])
            if ((targetX-error)<=round(value[0],2)<=(targetX+error) and (targetY-error)<=(round(value[1],2))<=(targetY+error)):
                print " x : %f  y: %f  z: %f " %(value[0],value[1],value[2]) 
                # cast float32 to int so that bitwise operations are possible
                #s = struct.pack('>f' ,test)
                #i = struct.unpack('>l',s)[0]
                # you can get back the float value by the inverse operations
                #pack = ctypes.c_uint32(i).value
                #r = (pack & 0x00FF0000)>> 16
                #g = (pack & 0x0000FF00)>> 8
                #b = (pack & 0x000000FF)
                #detected = True
                target = value
                transformed_point = self._transform_pose(value,"head_rgbd_sensor_rgb_frame", "map")
                
                return transformed_point
        return None                               

    def _definePointCloud(self, msg):
        global pointCloud 
        pointCloud = msg

    def _get3DPointMap(self,coordinateX,coordinateZ,referenceFrame):
        global done
        global pointCloud
        if(not(done)):
            done=True
            #valueX=-0.5
            #valueZ=1.04
            completed = False
            depthSensorFrame = None
            while(not completed):
                try:
                    depthSensorFrame = self._transform_pose([coordinateX,0,coordinateZ],referenceFrame,"head_rgbd_sensor_rgb_frame")
                    print("Point transformed correctly")
                    print(depthSensorFrame)
                    completed = True

                    time.sleep(1)
                except: 
                    print("Error transforming given point to head rgbd sensor frame")
                    time.sleep(1)
            #self._pointPublisher(depthSensorFrame)
            valueX=depthSensorFrame.point.x+0.11
            valueY=depthSensorFrame.point.y
            data = pc2.read_points(pointCloud, field_names = ("x", "y", "z", "rgb"), skip_nans=True)

            error=0.005
            detected = False
            target = None
            print("-------- NEW DATA ----- Error "+str(error))
            transformed_point = self._inRange(data,error,valueX,valueY)
            if transformed_point is None:
                print("Not correlated point detected")
                #return None
            else:
                print("Found point")
                self._pointPublisher(transformed_point)
                #return transformed_point   
    
    def _get3DPointMapFromTopic(self,msg):
        global done
        global pointCloud
        coordinateX = msg.point.x
        coordinateZ = msg.point.z
        referenceFrame = msg.header.frame_id
        if(not(done)):
            done=True
            #valueX=-0.5
            #valueZ=1.04
            completed = False
            depthSensorFrame = None
            while(not completed):
                try:
                    depthSensorFrame = self._transform_pose([coordinateX,0,coordinateZ],referenceFrame,"head_rgbd_sensor_rgb_frame")
                    print("Point transformed correctly")
                    print(depthSensorFrame)
                    completed = True

                    time.sleep(1)
                except: 
                    print("Error transforming given point to head rgbd sensor frame")
                    time.sleep(1)
            #self._pointPublisher(depthSensorFrame)
            valueX=depthSensorFrame.point.x+0.11
            valueY=depthSensorFrame.point.y
            data = pc2.read_points(pointCloud, field_names = ("x", "y", "z", "rgb"), skip_nans=True)

            error=0.005
            detected = False
            target = None
            print("-------- NEW DATA ----- Error "+str(error))
            transformed_point = self._inRange(data,error,valueX,valueY)
            if transformed_point is None:
                print("Not correlated point detected")
                #return None
            else:
                print("Found point")
                self._pointPublisherCV(transformed_point)
                #return transformed_point   
    
def main():
    rospy.init_node('get_3d_position')
    node = Get3DPosition()
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)

if __name__ == '__main__':
    main()