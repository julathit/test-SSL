#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager

from action import MoveToBallAction, MoveToRandomPointAction, MoveRRTAction
from typing import List

if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(5, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # action: Behaviour = MoveRandomAction(robotManager, 0)

    # while not rospy.is_shutdown() and action.status not in [Status.SUCCESS,Status.FAILURE]:
    #     action.tick_once()
    # else:
    #     print("Success.")

    parallel_node: Parallel = Parallel(name="yeeee Tree!", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
    [parallel_node.add_child(MoveToRandomPointAction(robotManager, i)) for i in range(4)]
    parallel_node.add_child(MoveRRTAction(robotManager, 4))

    tree = py_trees.trees.BehaviourTree(parallel_node)

    while not rospy.is_shutdown() and tree.root.status not in [Status.FAILURE]:
        tree.tick()
    else:
        print("Success.")
