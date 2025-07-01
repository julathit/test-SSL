from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB
from component.area import Zone, ZoneManager

import random

class MoveToRandomPoint(Behaviour):
    def __init__(self, robot_ID: int):
        super(MoveToRandomPoint, self).__init__(f"Move To Random Point Robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)

    def initialise(self):
        zone = ZoneManager.getZoneBoundary(ZoneManager.getZoneFromRole(self.robot.getRole()))
        min_x = int(zone["x_min"])
        max_x = int(zone["x_max"])
        min_y = int(zone["y_min"])
        max_y = int(zone["y_max"])
        self.randomPosition: Position = Position(random.randint(min_x, max_x), random.randint(min_y, max_y))
        return super().initialise()

    def update(self):
        if self.robot.nearPoint(self.randomPosition):
            return Status.SUCCESS
        else:
            self.robot.goToPoint(self.randomPosition)
            return Status.RUNNING
