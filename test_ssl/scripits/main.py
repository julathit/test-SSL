#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager

from action import MoveToBallAction

# def crateTree() -> Sequence:
#     root : Sequence = Sequence(name = "goToPoint" , memory=True)

#     goToP: my_moveToPoint = my_moveToPoint(0,(0,0))

#     root.add_child(my_setUpState())
#     root.add_child(goToP)

#     return root


if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(5, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # root = crateTree()
    # tree = py_trees.trees.BehaviourTree(root)

    # py_trees.display.render_dot_tree(tree.root)

    action: Behaviour = MoveToBallAction(robotManager, 1)

    #while not rospy.is_shutdown() and tree.root.status not in  [Status.SUCCESS,Status.FAILURE]:
    #while not rospy.is_shutdown():
    while not rospy.is_shutdown() and action.status not in [Status.SUCCESS,Status.FAILURE]:
        action.tick_once()
    else:
        print("Success.")