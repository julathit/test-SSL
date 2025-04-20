from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB

import random

class MoveToRandomPoint(Behaviour):
    def __init__(self, robot_ID: int):
        super(MoveToRandomPoint, self).__init__(f"Move To Random Point Robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)

    def initialise(self):
        self.randomPosition: Position = Position(random.randint(-4500, 4500), random.randint(-3000, 3000))
        return super().initialise()

    def update(self):
        if self.robot.nearPoint(self.randomPosition):
            return Status.SUCCESS
        else:
            self.robot.goToPoint(self.randomPosition)
            return Status.RUNNING
