from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from utils.blackboard import RobotBlackBoard as RBB
from component.area import ZoneManager

class MoveToBallAndSendSignal(Behaviour):
    def __init__(self, robot_ID: int, robot_target_ID: int):
        super(MoveToBallAndSendSignal,self).__init__(f"Move To Ball And Send Signal To Robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)
        self.target_robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_target_ID)

    def update(self):
        RBB.setRobotReceiver(self.target_robot.id)
        isInZone = ZoneManager.isInZone(RBB.getBallPosition(), ZoneManager.getZoneFromRole(self.robot.getRole()))
        if not isInZone:
            return Status.FAILURE
        elif self.robot.nearBall() != True:
            self.robot.MoveToBallModify()
            return Status.RUNNING
        else:
            self.feedback_message = ("I'm at the ball now!")
            self.logger.debug("\n%s.update()[%s -> %s][%s]" % (self.__class__.__name__, self.status, Status.SUCCESS, self.feedback_message))
            return Status.SUCCESS
