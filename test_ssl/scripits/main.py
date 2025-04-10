#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager


from action import MoveToBallAction, KickAction, passBallAction
from typing import List

# def crateTree(robotManager: RobotManager):
#     root : Sequence = Sequence(name = "goToPoint" , memory=True)

#     goTB : MoveToBallAction = MoveToBallAction(robotManager,1)
#     kcikBall: KickAction = KickAction(robotManager,1)

#     root.add_child(goTB)
#     root.add_child(kcikBall)



#     return root

if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(6, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    pass1to2 = passBallAction(robotManager,1,2)
    pass2to3 = passBallAction(robotManager,2,3)
    pass3to1 = passBallAction(robotManager,3,1)
    root.add_child(pass1to2)
    root.add_child(pass2to3)
    root.add_child(pass3to1)
    tree = py_trees.trees.BehaviourTree(root)

    while not rospy.is_shutdown() and tree.root.status not in [Status.SUCCESS,Status.FAILURE]:
        
        tree.tick()
    else:
        print("Success.")
