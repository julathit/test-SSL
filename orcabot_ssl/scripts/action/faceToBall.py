from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB


class faceToBall(Behaviour):
    def __init__(self, robot_ID: int):
        super(faceToBall, self).__init__(f"Face to Ball Robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)

    def update(self):
        if self.robot.faceToBall():
            return Status.SUCCESS
        else:
            return Status.RUNNING