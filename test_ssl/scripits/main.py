#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager

from action import MoveToBallAction

if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(5, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    action: Behaviour = MoveToBallAction(robotManager, 1)

    while not rospy.is_shutdown() and action.status not in [Status.SUCCESS,Status.FAILURE]:
        action.tick_once()
    else:
        print("Success.")