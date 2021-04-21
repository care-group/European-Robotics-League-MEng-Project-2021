#!/usr/bin/python2.7

import moveit_commander
import sys

print(sys.executable)

def move_head_tilt(v):
    head.set_joint_value_target("head_tilt_joint", v)
    return head.go()

head = moveit_commander.MoveGroupCommander("head")
move_head_tilt(-0.5)