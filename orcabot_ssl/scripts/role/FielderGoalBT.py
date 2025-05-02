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

class FielderGoalBT(Selector):
    def __init__(self, robot: Robot):
        super(FielderGoalBT, self).__init__("Fielder Goal BT", False)
        self.robot = robot
        self.add_children([FielderDefenderIfCanShootBT(robot),
                        FielderRandomWalk(robot)
                        ])

class FielderRandomWalk(Sequence):
    def __init__(self, robot: Robot):
        super(FielderRandomWalk, self).__init__("Fielder Random Walk", True)

        self.add_children([
            MoveToRandomPointAction(robot.id),
        ])

class FielderDefenderIfCanShootBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderDefenderIfCanShootBT, self).__init__("Fielder Goal Selector", True)
        self.add_children([checkIfBallInZone(robot), FielderDefenderRobotShootToFront(robot)])

class checkIfBallInZone(Behaviour):
    def __init__(self, robot: Robot):
        super(checkIfBallInZone, self).__init__("check If Ball is in Goal Zone")
        self.robot: Robot = robot

    def update(self):
        role = self.robot.getRole()
        if RobotBlackBoard.getPreviousBallPossession() > 0 and RobotBlackBoard.getPreviousBallPossession() != self.robot.id:
            return Status.FAILURE
        if ZoneManager.getZoneFromPosition(RobotBlackBoard.getBallPosition()) == ZoneManager.getZoneFromRole(role):
            return Status.SUCCESS
        else: return Status.FAILURE

class FielderDefenderRobotShootToFront(Sequence):
    def __init__(self, robot: Robot):
        super(FielderDefenderRobotShootToFront, self).__init__("FielderDefenderRobot Shoot To Front", True)

        RobotBlackBoard.setRobotShooterID(robot.id)
        catcher_Id = RobotBlackBoard.getConfig("myteam", "attacker_id")

        def getNearestCenterRobot() -> int:
            robot_list = RobotBlackBoard.getRobotList(RobotBlackBoard.getMyTeam())
            distances: List[Tuple[int, int]] = []
            for r in robot_list:
                if r.getRole() in [Role.CENTER_LEFT, Role.CENTER_RIGHT]:
                    distances.append((r.id, r.getPosition().distanceTo(robot.getPosition())))

            rid, dist = distances[0]
            if dist < distances[1][1]:
                return rid
            return distances[1][0]

        catcher_Id = getNearestCenterRobot()

        faceToBall1 = FaceToBallAction(robot.id)
        gotoBall1 = MoveToBallAction(robot.id)
        dribling1 = DribblerAction(robot.id)
        aiming1 = AimingAction(robot.id, catcher_Id)
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

# class RobotClosestToBallBT(Sequence):
#     def __init__(self, robot: Robot):
#         super(RobotClosestToBallBT, self).__init__("Robot Closest To Ball BT", True)

#         # simulate behavior / implement out of this scope

#         move_toward_ball = Behaviour("Move towards ball")

#         self.add_children([RobotClosestToBall(robot), move_toward_ball])

# class RobotClosestToBall(Behaviour):
#     def __init__(self, robot: Robot):
#         super(RobotClosestToBall, self).__init__("Robot Closest To Ball?")

#     def update(self):
#         # need to implement
#         if RobotBlackBoard.getRole(self.robot) == Role.ATTACKER:
#             return Status.SUCCESS
#         else: return Status.FAILURE

# class CoverPassingOption(Sequence):
#     def __init__(self, robot: Robot):
#         super(CoverPassingOption, self).__init__("Robot Cover Passing Option BT", True)

#         # simulate behavior / implement out of this scope
#         identify_foe_to_guard = Behaviour("Identify Foe To Guard")
#         move_towards_toe = Behaviour("Move Towards For")

#         self.add_children([identify_foe_to_guard, move_towards_toe])