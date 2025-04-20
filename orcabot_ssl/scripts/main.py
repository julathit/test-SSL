#! /usr/bin/env python3
from utils.orcabot import *

enable_debug()

# RobotBlackBoard.printAllInfo()

def main():
    root: Sequence = Sequence(name = "goToPoint", memory=True)

    passBall1 = passBallBehavior(1, 2)
    passBall2 = passBallBehavior(2, 3)
    passBall3 = passBallBehavior(3, 1)
    gotoball1 = MoveToBallAction(1)
    catch = DribblerAction(1, 2)

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

main()