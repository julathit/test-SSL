#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager


from action import MoveToBallAction, MoveToRandomPointAction, MoveToPointAction,Dribbler
from typing import List

def crateTree(robotManager: RobotManager):
    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    goTB : MoveToBallAction = MoveToBallAction(robotManager,1)
    drib: Dribbler = Dribbler(robotManager,1)
    gotP: MoveToPointAction = MoveToPointAction(robotManager,1,(0,0))
    gotP1: MoveToBallAction = MoveToPointAction(robotManager,1,(-1000,0))

    root.add_child(drib)
    root.add_child(gotP)
    root.add_child(goTB)
    root.add_child(gotP1)


    return root

if __name__ == "__main__":

    # initializing
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(6, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = crateTree(robotManager)
    tree = py_trees.trees.BehaviourTree(root)

    while not rospy.is_shutdown() and tree.root.status not in [Status.SUCCESS,Status.FAILURE]:
        
        tree.tick()
    else:
        print("Success.")
