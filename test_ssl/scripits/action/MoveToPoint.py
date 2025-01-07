from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

class MoveToPoint(Behaviour):
    def __init__(self, robotManager: RobotManager, robot_ID : int,point : tuple):
        super(MoveToPoint, self).__init__(f"robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)
        self.point: tuple = point

    def update(self):
        if not self.robot.nearPoint(self.point) :
            self.robot.bestMoveModify(self.point)
            return Status.RUNNING
        else:
            print("suc")
            self.feedback_message = ("I'm at the point now!")
            return Status.SUCCESS