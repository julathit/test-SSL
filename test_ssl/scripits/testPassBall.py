#! /usr/bin/env python3
import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

from component.Ball import Ball
from component.RobotManager import RobotManager


from action import MoveToBallAction, KickAction,DribblerAction, AimingAction, FaceToBallAction, waitAction
from simpleBehavior import passBallBehavior
from typing import List

# def crateTree(robotManager: RobotManager):
#     root : Sequence = Sequence(name = "goToPoint" , memory=True)

#     goTB : MoveToBallAction = MoveToBallAction(robotManager,1)
#     kcikBall: KickAction = KickAction(robotManager,1)

#     root.add_child(goTB)
#     root.add_child(kcikBall)



#     return root

if __name__ == "__main__":

    # initializings
    ball: Ball = Ball()
    robotManager: RobotManager = RobotManager(6, ball)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    passBall1 = passBallBehavior(robotManager,1,2)
    passBall2 = passBallBehavior(robotManager,2,3)
    passBall3 = passBallBehavior(robotManager,3,1)
    gotoball1 = MoveToBallAction(robotManager,1)
    catch = DribblerAction(robotManager, 1,2)

    root.add_children([
        passBall1,
        passBall2,
        passBall3,
        gotoball1,
        catch
    ])


    tree = py_trees.trees.BehaviourTree(root)

    while True:
        
        tree.tick()
    else:
        print("Success.")
