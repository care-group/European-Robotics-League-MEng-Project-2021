#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

def callback(msg):
    base_publisher.publish(twist)
    timer = 0
    while (timer<msg.data):
        print("Rotating for"+str(msg.data)+"seconds!")
        rate.sleep()
        timer+=1
        print(timer)


rospy.init_node('Rotate')
base_publisher = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
jason_sub = rospy.Subscriber('/jason/rotate', Int16, callback)

rate = rospy.Rate(1)
twist= Twist()
twist.angular.z = 5

while not rospy.is_shutdown():
    rate.sleep()