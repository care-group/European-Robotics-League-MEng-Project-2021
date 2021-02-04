#! /usr/bin/env python

import rospy
from utils import *

rospy.init_node('navigation')

print("Hi Nav")

semantic_coords = {
    # coordinates in [x, y, w] 
    # where w is the direction it is facing in degrees
    "door" : [-0.416, 0, 0],
    "drawers" : [0.1, 0, 0],
    "full desk" : [1.5, 0, 0],
    "trash" : [2.7, 0, 0],
    "empty desk" : [0, 1.25, 0],
    "passage" : [2.65, 2.2, 0],
    "dude" : [0.4, 2.8, 0],
    "shelves" : [2.25, 4.4, 0]
}

def send_semantic_goal(name, semantics):
    if name not in semantics:
        print("That label does not exist in the semantic map")
        return False
    t = semantics[name]
    move_base_goal(t[0], t[1], t[2])

move_base_goal(2.5, 0.8, 0)

print("Moved?")