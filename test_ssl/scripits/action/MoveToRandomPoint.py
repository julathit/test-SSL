from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

import random

class MoveToRandomPoint(Behaviour):
    def __init__(self, robotManager: RobotManager, robot_ID : int):
        super(MoveToRandomPoint, self).__init__(f"robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)

    def initialise(self):
        self.randomPosition: tuple[int, int] = (random.randint(-4500, 4500), random.randint(-3000, 3000))
        return super().initialise()

    def update(self):
        if self.robot.nearPoint(self.randomPosition):
            return Status.SUCCESS
        else:
            self.robot.goToPoint(self.randomPosition)
            return Status.RUNNING
