from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB

class MoveToPoint(Behaviour):
    def __init__(self, robot_ID: int, point: Position):
        super(MoveToPoint, self).__init__(f"Move To Point Robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)
        self.point: Position = point

    def update(self):
        if not self.robot.nearPoint(self.point) :
            self.robot.bestMoveModify(self.point)
            return Status.RUNNING
        else:
            print("suc")
            self.feedback_message = ("I'm at the point now!")
            return Status.SUCCESS