from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from utils.blackboard import RobotBlackBoard as RBB

class MoveToBall(Behaviour):
    def __init__(self, robot_ID: int):
        super(MoveToBall, self).__init__(f"Move To Ball Robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)

    def update(self):
        if self.robot.nearBall() != True:
            self.robot.MoveToBallModify()
            return Status.RUNNING
        else:
            self.feedback_message = ("I'm at the ball now!")
            self.logger.debug("\n%s.update()[%s -> %s][%s]" % (self.__class__.__name__, self.status, Status.SUCCESS, self.feedback_message))
            return Status.SUCCESS
