#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager

from action import MoveToBallAction, GoToLineUpAction
from typing import List

def crateTree(robotManager: RobotManager) -> Sequence:
    root : Sequence = Sequence(name = "goToLineUp" , memory=True)

    goToL: Behaviour = GoToLineUpAction(robotManager, 1, 0)
    goTB : Behaviour = MoveToBallAction(robotManager, 1)

    root.add_child(goToL)
    root.add_child(goTB)

    return root


if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(5, ball)

    root: Sequence = crateTree(robotManager)
    tree: py_trees.trees.BehaviourTree = py_trees.trees.BehaviourTree(root)

    while not rospy.is_shutdown() and tree.root.status not in  [Status.SUCCESS,Status.FAILURE]:
        tree.tick()
    else:
        print("Success.")