#! /usr/bin/env python3
from py_trees.composites import Sequence
from tactic.My_goToLineUp import my_goToLineUp
from py_trees.common import Status
from tactic.My_moveToBall import my_moveToBall

import time
import py_trees
import rospy
    
def crateTree():
    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    goToL: my_goToLineUp = my_goToLineUp(1,0)
    goTB : my_moveToBall = my_moveToBall(1)

    root.add_child(goToL)
    root.add_child(goTB)

    return root


if __name__ == "__main__":
    root = crateTree()
    tree = py_trees.trees.BehaviourTree(root)

    while not rospy.is_shutdown() and tree.root.status not in  [Status.SUCCESS,Status.FAILURE]:
        tree.tick()
    else:
        print("Success.")