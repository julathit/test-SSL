from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB

from numpy import sqrt

#need fix

class GoToLineUp(Behaviour):

    def __init__(self, robot_ID: int, robot_target_ID: int, avoidBall = False, distance: int = 300):
        super(GoToLineUp, self).__init__(f"make line Up robot_ID {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)
        self.target_robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_target_ID)
        self.distance: int = distance

        self.ball_position: Position = RBB.getBallPosition()

    def __calculate_lineup_point(self) -> Position:
        L_vec: tuple[float, float] = (self.ball_position.x - self.target_robot.getPosition().x, self.ball_position.y - self.target_robot.getPosition().y)
        L_vec_size: float = sqrt(L_vec[0]**2 + L_vec[1]**2)
        L_vec_norm: tuple[float, float] = (L_vec[0]/L_vec_size, L_vec[1]/L_vec_size)
        return Position(L_vec_norm[0] * self.distance + self.ball_position.x, L_vec_norm[1] * self.distance + self.ball_position.y)

    def update(self):
        # if self.robot.nearPoint(self.point):
        #     return Status.SUCCESS
        # elif self.robot.rayHit():
        #     return Status.RUNNING
        # else:
        #     self.robot.goToPoint(self.point)
        #     return Status.RUNNING
        line_up_p: Position = self.__calculate_lineup_point()

        if self.robot.nearPoint(line_up_p):
            return Status.SUCCESS
        else:
            self.robot.bestMoveModify(line_up_p)
            return Status.RUNNING