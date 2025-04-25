from py_trees.behaviour import Behaviour
from typing import List, Tuple

from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from component.robot import Robot
from component.misc import Role
from component.area import ZoneManager, Zone

from action import MoveToBallAction, MoveToPointAction, KickAction, DribblerAction, AimingToGoalAction, FaceToBallAction, waitAction, MoveToRandomPointAction

class FielderCenterBT(Selector):
    def __init__(self, robot: Robot):
        super(FielderCenterBT, self).__init__("FielderCenter BT", False)
        self.robot = robot
        self.add_children([
            FielderCenterReceivePassBT(robot),
            FielderCenterIfCanShootBT(robot),
            MoveToRandomPointAction(robot.id),
        ])

__all__ = ['FielderCenterBT']

# Recieve Pass
class FielderCenterReceivePassBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderCenterReceivePassBT, self).__init__("FielderCenter Receive Pass BT", True)
        self.robot = robot
        self.add_children([checkIfShotReceiver(robot), FielderCenterRobotReceivePass(robot)])

class checkIfShotReceiver(Behaviour):
    def __init__(self, robot: Robot):
        super(checkIfShotReceiver, self).__init__("check If Shot Receiver")
        self.robot: Robot = robot

    def update(self):
        receiverB, _ = RobotBlackBoard.getIsRobotReceiver(self.robot.id)
        if receiverB:
            RobotBlackBoard.setRobotReceiver(None)
            return Status.SUCCESS
        else:
            return Status.FAILURE

class FielderCenterRobotReceivePass(Sequence):
    def __init__(self, robot: Robot):
        super(FielderCenterRobotReceivePass, self).__init__("FielderCenterRobot Receive Pass", True)

        self.add_children([
            FaceToBallAction(robot.id),
            # waitAction(robot.id, 7),
            # MoveToBallAction(robot.id),
            DribblerAction(robot.id, 5)
        ])

# Shoot
class FielderCenterIfCanShootBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderCenterIfCanShootBT, self).__init__("Fielder Center Selector", True)
        self.add_children([checkIfBallInZone(robot), FielderCenterRobotShootToGoal(robot)])

class checkIfBallInZone(Behaviour):
    def __init__(self, robot: Robot):
        super(checkIfBallInZone, self).__init__("check If Ball is in Center Zone")
        self.robot: Robot = robot

    def update(self):
        role = self.robot.getRole()
        if RobotBlackBoard.getPreviousBallPossession() > 0 and RobotBlackBoard.getPreviousBallPossession() != self.robot.id:
            return Status.FAILURE
        if ZoneManager.getZoneFromPosition(RobotBlackBoard.getBallPosition()) == ZoneManager.getZoneFromRole(role):
            return Status.SUCCESS
        else: return Status.FAILURE

class FielderCenterRobotShootToGoal(Sequence):
    def __init__(self, robot: Robot):
        super(FielderCenterRobotShootToGoal, self).__init__("FielderCenterRobot Shoot To Goal", True)

        RobotBlackBoard.setRobotShooterID(robot.id)

        faceToBall1 = FaceToBallAction(robot.id)
        gotoBall1 = MoveToBallAction(robot.id)
        dribling1 = DribblerAction(robot.id)
        aiming1 = AimingToGoalAction(robot.id)
        gotoBall2 = MoveToBallAction(robot.id)
        kick1 = KickAction(robot.id)

        self.add_children([
            faceToBall1,
            gotoBall1,
            dribling1,
            aiming1,
            gotoBall2,
            kick1
        ])