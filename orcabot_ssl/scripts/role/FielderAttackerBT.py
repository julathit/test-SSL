# from py_trees.behaviour import Behaviour
from utils.mockBehaviour import Behaviour

from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from interface.robot import Robot, Role

# 1
class FielderAttackerBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderAttackerBT, self).__init__("Fielder Attacker BT", True)
        self.add_children([checkIfAttackerRole(robot), FielderAttackerHasPossessionSelector(robot)])

# 1.1
class checkIfAttackerRole(Behaviour):
    def __init__(self, robot: Robot):
        super(checkIfAttackerRole, self).__init__("check If Attacker Role")
        self.robot: Robot = robot

    def update(self):
        if RobotBlackBoard.getRole(self.robot) == Role.ATTACKER:
            return Status.SUCCESS
        else: return Status.FAILURE

class FielderAttackerHasPossessionSelector(Selector):
    def __init__(self, robot: Robot):
        super(FielderAttackerHasPossessionSelector, self).__init__("FielderAction HasPossession Condition Selector", True)
        self.add_children([RobotHasPossesionBT(robot), FielderAttackerHasOpenShotSelector(robot)])

class RobotHasPossesionBT(Sequence):
    def __init__(self, robot: Robot):
        super(RobotHasPossesionBT, self).__init__("Robot has possesion BT", True)

        # simulate behavior / implement out of this scope
        position_self = py_trees.composites.Sequence("Position self", True)

        determine_location_to_position = Behaviour("Determine location to position")
        move_to_position = Behaviour("Move to position")

        position_self.add_children([determine_location_to_position, move_to_position])

        self.add_children([RobotHasPossesion(robot), position_self])

class RobotHasPossesion(Behaviour):
    def __init__(self, robot: Robot):
        super(RobotHasPossesion, self).__init__("Robot has possesion?")

    def update(self):
        # need to implement
        if RobotBlackBoard.getRole(self.robot) == Role.ATTACKER:
            return Status.SUCCESS
        else: return Status.FAILURE

class FielderAttackerHasOpenShotSelector(Selector):
    def __init__(self, robot: Robot):
        super(FielderAttackerHasOpenShotSelector, self).__init__("FielderAction HasOpenShot Condition Selector", True)
        self.add_children([RobotHasOpenShotBT(robot), FielderAttackerSpaceInFrontSelector(robot)])

class RobotHasOpenShotBT(Sequence):
    def __init__(self, robot: Robot):
        super(RobotHasOpenShotBT, self).__init__("Robot has possesion BT", True)

        # simulate behavior / implement out of this scope
        shoot = py_trees.composites.Sequence("Shoot", True)

        determine_optimal_shot = Behaviour("Determine optimal shot")
        kick_ball_quickly_at_target = Behaviour("Kick ball quickly at target")

        shoot.add_children([determine_optimal_shot, kick_ball_quickly_at_target])

        self.add_children([RobotHasOpenShot(robot), shoot])

class RobotHasOpenShot(Behaviour):
    def __init__(self, robot: Robot):
        super(RobotHasOpenShot, self).__init__("Robot has open shot?")

    def update(self):
        # not implement yet
        if True:
            return Status.SUCCESS
        else: return Status.FAILURE

class FielderAttackerSpaceInFrontSelector(Selector):
    def __init__(self, robot: Robot):
        super(FielderAttackerSpaceInFrontSelector, self).__init__("FielderAction Space in front of ballholder Condition Selector", True)

        # simulate behavior / implement out of this scope
        coordinated_pass = py_trees.composites.Sequence("Coordinated Pass", True)

        choose_ally = Behaviour("Choose Ally")
        determine_if_chip_needed = Behaviour("Determine if chip needed")
        execute_pass = Behaviour("Execute pass")

        coordinated_pass.add_children([choose_ally, determine_if_chip_needed, execute_pass])

        self.add_children([RobotSpaceInFrontBT(robot), coordinated_pass])

class RobotSpaceInFrontBT(Sequence):
    def __init__(self, robot: Robot):
        super(RobotSpaceInFrontBT, self).__init__("Robot has space in front of ballholder BT", True)

        # simulate behavior / implement out of this scope
        dribble = Behaviour("Dribble")

        self.add_children([RobotSpaceInFront(robot), dribble])

class RobotSpaceInFront(Behaviour):
    def __init__(self, robot: Robot):
        super(RobotSpaceInFront, self).__init__("Robot has space in front of ball holder?")

    def update(self):
        # not implement yet
        if True:
            return Status.SUCCESS
        else: return Status.FAILURE
