# from py_trees.behaviour import Behaviour
from utils.mockBehaviour import Behaviour

from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from interface.robot import Robot, Role

# from action.RefereeCommandBT import Re

# 1
class GoalKeeperBT(Sequence):
    def __init__(self, robot: Robot):
        super(GoalKeeperBT, self).__init__("GoalKeeper BT", True)
        self.add_children([ \
            #RefereeCommandBT(robot), 
            GoalKeeperHasBallSelector(robot)])

class GoalKeeperHasBallSelector(Selector):
    def __init__(self, robot: Robot):
        super(GoalKeeperHasBallSelector, self).__init__("GoalKeeperAction HasBall Condition Selector", True)
        self.add_children([GoalKeeperOffenseBT(robot), GoalKeeperDefenseBT(robot)])

class GoalKeeperOffenseBT(Sequence):
    def __init__(self, robot: Robot):
        super(GoalKeeperOffenseBT, self).__init__("GoalKeeper Offense BT", True)

        self.add_children([RobotHasPossesionSelector(robot)])

class RobotHasPossesionSelector(Selector):
    def __init__(self, robot: Robot):
        super(RobotHasPossesionSelector, self).__init__("GoalKeeperAction HasPossession Condition Selector", True)
        self.add_children([RobotHasPossesionBT(robot), MoveToLocationWherePassingOptionBT(robot)])

class RobotHasPossesionBT(Sequence):
    def __init__(self, robot: Robot):
        super(RobotHasPossesionBT, self).__init__("Robot has possesion BT", True)

        # simulate behavior / implement out of this scope
        CoordinatedPassAction: Behaviour = Behaviour("Coordinated pass")

        self.add_children([RobotHasPossesion(robot), CoordinatedPassAction])

class RobotHasPossesion(Behaviour):
    def __init__(self, robot: Robot):
        super(RobotHasPossesion, self).__init__("Robot has possesion?")
        self.robot = robot

    def update(self):
        # need to implement
        if RobotBlackBoard.getRole(self.robot) == Role.ATTACKER:
            return Status.SUCCESS
        else: return Status.FAILURE

class MoveToLocationWherePassingOptionBT(Sequence):
    def __init__(self, robot: Robot):
        super(MoveToLocationWherePassingOptionBT, self).__init__("Robot has possesion BT", True)

        # simulate behavior / implement out of this scope
        DetermineOptimalPositionAction: Behaviour = Behaviour("Determine Optimal Position")
        MoveToPositionAction: Behaviour = Behaviour("Move To Position")

        self.add_children([DetermineOptimalPositionAction(robot), MoveToPositionAction(robot)])

class MoveToLocationWherePassingOptionBT(Sequence):
    def __init__(self, robot: Robot):
        super(MoveToLocationWherePassingOptionBT, self).__init__("Robot has possesion BT", True)

        # simulate behavior / implement out of this scope
        DetermineOptimalPositionAction: Behaviour = Behaviour("Determine Optimal Position")
        MoveToPositionAction: Behaviour = Behaviour("Move To Position")

        self.add_children([DetermineOptimalPositionAction, MoveToPositionAction])

class GoalKeeperDefenseBT(Sequence):
    def __init__(self, robot: Robot):
        super(GoalKeeperDefenseBT, self).__init__("GoalKeeper Defense BT", True)

        MoveToOptimalBlockingPositionAction: Behaviour = Behaviour("Move To Optimal Blocking Position Action")

        self.add_children([MoveToOptimalBlockingPositionAction])