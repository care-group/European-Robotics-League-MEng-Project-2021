#! /usr/bin/env python

import math
import rospy
import time
from utils import *

rospy.init_node('base_and_sensor')
print("Hi")

move_base_vel(1, 0, 0)


