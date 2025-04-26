from py_trees.behaviour import Behaviour
from typing import List, Tuple

from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from component.robot import Robot
from component.misc import Role
from component.area import ZoneManager, Zone

from action import MoveToBallAction, MoveToPointAction, KickAction, DribblerAction, AimingAction, FaceToBallAction, waitAction, MoveToRandomPointAction

class FielderCenterBT(Selector):
    def __init__(self, robot: Robot):
        super(FielderCenterBT, self).__init__("FielderCenter BT", False)
        self.robot = robot
        self.add_children([
            FielderCenterReceivePassBT(robot),
            MoveToRandomPointAction(robot.id),
        ])

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
            exit()
            return Status.SUCCESS
        else:
            return Status.FAILURE

class FielderCenterRobotReceivePass(Sequence):
    def __init__(self, robot: Robot):
        super(FielderCenterRobotReceivePass, self).__init__("FielderCenterRobot Receive Pass", True)

        self.add_children([
            FaceToBallAction(robot.id),
            MoveToBallAction(robot.id),
            DribblerAction(robot.id, 1.5)
        ])

__all__ = ['FielderCenterBT']