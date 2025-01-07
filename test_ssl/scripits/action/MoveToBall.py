from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

class MoveToBall(Behaviour):
    def __init__(self, robotManager: RobotManager, robot_ID : int):
        super(MoveToBall, self).__init__(f"robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)

    def update(self):
        if self.robot.nearBall() != True:
            self.robot.MoveToBallModify()
            return Status.RUNNING
        else:
            self.feedback_message = ("I'm at the ball now!")
            self.logger.debug("\n%s.update()[%s -> %s][%s]" % (self.__class__.__name__, self.status, Status.SUCCESS, self.feedback_message))
            return Status.SUCCESS
