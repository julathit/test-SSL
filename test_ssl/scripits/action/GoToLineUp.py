from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.Ball import Ball
from component.RobotManager import RobotManager

from numpy import sqrt

class GoToLineUp(Behaviour):

    def __init__(self, robotManager: RobotManager, robot_ID: int, robot_target_ID: int, distance: int = 200):
        super(GoToLineUp, self).__init__(f"make line Up robot_ID {robot_ID}")
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)
        self.robot_ID: int = robot_ID
        self.robot_target_ID: int = robot_target_ID
        self.robotManager: RobotManager = robotManager
        self.distance: int = distance

        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)
        self.target_robot: Robot = robotManager.getRobotByID(self.robot_target_ID)

        self.ball: Ball = robotManager.getBall()

    def __calculate_lineup_point(self) -> tuple[float, float]:
        L_vec: tuple[int, int] = (self.ball.getPosition()[0] - self.target_robot.getPosition()[0], self.ball.getPosition()[1] - self.target_robot.getPosition()[1])
        L_vec_size: float = sqrt(L_vec[0]**2 + L_vec[1]**2)
        L_vec_norm: tuple[float, float] = L_vec/L_vec_size
        return L_vec_norm * self.distance + self.ball.getPosition()

    def update(self):
        # if self.robot.nearPoint(self.point):
        #     return Status.SUCCESS
        # elif self.robot.rayHit():
        #     return Status.RUNNING
        # else:
        #     self.robot.goToPoint(self.point)
        #     return Status.RUNNING
        line_up_p = self.__calculate_lineup_point()

        if self.robot.nearPoint(line_up_p):
            return Status.SUCCESS
        else:
            self.robot.goToPoint(line_up_p)
            return Status.RUNNING