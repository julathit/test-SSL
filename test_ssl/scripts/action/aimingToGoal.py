from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB
from component.area import ZoneManager

class aimingToGoal(Behaviour):
    def __init__(self, robot_ID: int):
        super(aimingToGoal, self).__init__(f"Robot {robot_ID} Aiming at Goal")
        self.robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)
        self.ball_position = RBB.getBallPosition()

    def update(self):
        isInZone = ZoneManager.isInZone(RBB.getBallPosition(), ZoneManager.getZoneFromRole(self.robot.getRole()))
        if not isInZone:
            return Status.FAILURE
        if self.robot.aiming(self.ball_position, ZoneManager.getNearestPointOnGoalEntrance(self.ball_position, 100)) :
            return Status.SUCCESS
        else:
            return Status.RUNNING