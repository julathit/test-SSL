from py_trees.common import Status
from py_trees.composites import Parallel
import py_trees
from .My_movetoPoint import my_moveToPoint

def my_setUpState():
    root : Parallel = Parallel(name="Set Up tree",policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    numberOfRobot : int = 11
    [root.add_child(my_moveToPoint(i,(-3273 + 500*i,-4464))) for i in range(numberOfRobot)]
    
    return root