# from py_trees.behaviour import Behaviour
from utils.mockBehaviour import Behaviour

from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from interface.robot import Robot, Role

# 1
class FielderDefenderBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderDefenderBT, self).__init__("Fielder Defender BT", True)
        self.add_children([checkIfDefenderRole(robot), FielderDefenderRobotClosestToBallSelector(robot)])

# 1.1
class checkIfDefenderRole(Behaviour):
    def __init__(self, robot: Robot):
        super(checkIfDefenderRole, self).__init__("check If Defende Role")
        self.robot: Robot = robot

    def update(self):
        if RobotBlackBoard.getRole(self.robot) == Role.DEFENDER:
            return Status.SUCCESS
        else: return Status.FAILURE

class FielderDefenderRobotClosestToBallSelector(Selector):
    def __init__(self, robot: Robot):
        super(FielderDefenderRobotClosestToBallSelector, self).__init__("FielderAction Robot Closest To Ball Selector Condition Selector", True)
        self.add_children([RobotClosestToBallBT(robot), CoverPassingOption(robot)])

class RobotClosestToBallBT(Sequence):
    def __init__(self, robot: Robot):
        super(RobotClosestToBallBT, self).__init__("Robot Closest To Ball BT", True)

        # simulate behavior / implement out of this scope

        move_toward_ball = Behaviour("Move towards ball")

        self.add_children([RobotClosestToBall(robot), move_toward_ball])

class RobotClosestToBall(Behaviour):
    def __init__(self, robot: Robot):
        super(RobotClosestToBall, self).__init__("Robot Closest To Ball?")

    def update(self):
        # need to implement
        if RobotBlackBoard.getRole(self.robot) == Role.ATTACKER:
            return Status.SUCCESS
        else: return Status.FAILURE

class CoverPassingOption(Sequence):
    def __init__(self, robot: Robot):
        super(CoverPassingOption, self).__init__("Robot Cover Passing Option BT", True)

        # simulate behavior / implement out of this scope
        identify_foe_to_guard = Behaviour("Identify Foe To Guard")
        move_towards_toe = Behaviour("Move Towards For")

        self.add_children([identify_foe_to_guard, move_towards_toe])