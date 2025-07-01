# from py_trees.behaviour import Behaviour
from utils.mockBehaviour import Behaviour

from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from component.robot import Robot
from component.misc import Role
from component.area import ZoneManager, Zone

# from action import MoveToBallAction, MoveToPointAction, KickAction, DribblerAction, AimingAction, FaceToBallAction, waitAction, MoveToRandomPointAction
from role.FielderDefenderBT import FielderDefenderBT
from role.FielderCenterBT import FielderCenterBT

class CentralBT(Selector):
    def __init__(self, robot: Robot):
        super(CentralBT, self).__init__("Central BT", True)
        self.robot = robot

        if robot.getRole() in [Role.DEFENSIVE_LEFT, Role.DEFENSIVE_RIGHT]:
            self.add_children([FielderDefenderBT(robot),
                            #FielderRandomWalk(robot)
                            ])

        elif robot.getRole() in [Role.CENTER_LEFT, Role.CENTER_RIGHT]:
            self.add_children([FielderCenterBT(robot),
                            #FielderRandomWalk(robot)
                            ])

        # elif robot.getRole() in [Role.ATTACKER]:
        #     self.add_children([FielderDefenderBT(robot),
        #                     #FielderRandomWalk(robot)
        #                     ])

        # elif robot.getRole() in [Role.GOALKEEPER]:
        #     self.add_children([FielderDefenderBT(robot),
        #                     #FielderRandomWalk(robot)
        #                     ])