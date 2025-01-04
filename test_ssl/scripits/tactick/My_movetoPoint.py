from py_trees.behaviour import Behaviour
from py_trees.common import Status
from obj.My_robot import my_robot

import time

class my_moveToPoint(Behaviour):
    def __init__(self, robot_ID : int, point : tuple) -> None:
        super(my_moveToPoint,self).__init__(f"robot {robot_ID}")
        self.point: tuple = point
        self.robot_ID: int = robot_ID
        self.robot: my_robot = my_robot(self.robot_ID)

    def update(self):
        # if self.robot.nearPoint(self.point):
        #     return Status.SUCCESS
        # elif self.robot.rayHit():
        #     return Status.RUNNING
        # else:
        #     self.robot.goToPoint(self.point)
        #     return Status.RUNNING
        
        if self.robot.nearPoint(self.point):
            return Status.SUCCESS
        else:
            self.robot.goToPoint(self.point)
            return Status.RUNNING
            