from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager


class faceToBall(Behaviour):
    def __init__(self, robotManager: RobotManager, robot_ID : int):
        super(faceToBall, self).__init__(f"robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)

    def update(self):
        if self.robot.faceToBall():
            return Status.SUCCESS
        else:
            return Status.RUNNING