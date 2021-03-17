#! /usr/bin/env python

import math
import rospy
import time
import sys
sys.path.insert(0, '/hsr_stuff/')
from utils import *

rospy.init_node('base_and_sensor')

print("Hi")

laser = Laser()
while True:
    print("Waiting for  laser scan data.")
    scan_data = laser.get_data()
    try:
        a = scan_data.ranges[360]
        print("Got laserdata, moving on.")
        break
    except AttributeError:
        print("No laser scan data yet, sleeping for 500ms.")
        rospy.sleep(0.5)
    except:
        print("Something went wrong when trying to get laser scan data, sleeping for 1s.")
        rospy.sleep(1)

start_time = get_current_time_sec()
while True:
    scan_data = laser.get_data()
    if scan_data.ranges[360] < 0.3:
        break
    print("Moving forwards.")
    print(scan_data.ranges[360])
    move_base_vel(2, 0, 0)

move_base_vel(0, 0, 0)
print("Succesfully reached the wall")