#! /usr/bin/env python3
from py_trees.composites import Sequence
from tactick.My_movetoPoint import my_moveToPoint
from py_trees.common import Status
from obj.My_robot import my_robot
import rospy

if __name__ == "__main__":
    robot1 = my_robot(1)
    