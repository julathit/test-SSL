#! /usr/bin/env python3
from py_trees.composites import Sequence
from tactick.My_movetoPoint import my_moveToPoint
from py_trees.common import Status
from tactick.My_setUpState import my_setUpState

import time
import py_trees
import rospy
    
def crateTree():
    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    goToP: my_moveToPoint = my_moveToPoint(0,(0,0))

    root.add_child(my_setUpState())
    root.add_child(goToP)

    return root


if __name__ == "__main__":
    root = crateTree()
    tree = py_trees.trees.BehaviourTree(root)

    while not rospy.is_shutdown() and tree.root.status not in  [Status.SUCCESS,Status.FAILURE]:
        start = time.time()
        tree.tick()
        end = time.time()
        print(f"{(end - start)*1,000,000} s")
    else:
        print("Success.")