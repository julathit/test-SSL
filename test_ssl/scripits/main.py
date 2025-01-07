#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager


from action import MoveToBallAction, MoveToRandomPointAction, MoveToPointAction
from typing import List

if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(6, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    gotoPoint: Behaviour = MoveToPointAction(robotManager, 1, (0,0))
    action: Behaviour = MoveToBallAction(robotManager, 1)

    while not rospy.is_shutdown() and action.status not in [Status.SUCCESS,Status.FAILURE]:
        
        action.tick_once()
    else:
        print("Success.")
