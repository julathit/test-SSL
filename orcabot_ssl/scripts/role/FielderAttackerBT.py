from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Selector, Sequence
import py_trees

from utils.blackboard import RobotBlackBoard
from interface.robot import Robot, Role

# 1
class FielderAttackerBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderAttackerBT, self).__init__("Fielder Attacker BT")
        self.add_children([checkIfAttackerRole(robot)])

# 1.1
class checkIfAttackerRole(Behaviour):
    def __init__(self, robot: Robot):
        super(checkIfAttackerRole, self).__init__("check If Attacker Role")
        self.robot: Robot = robot

    def setup(self):
        if RobotBlackBoard.getRole(self.robot) == Role.ATTACKER:
            return Status.SUCCESS
        else: return Status.FAILURE

# 1.2
class RobotHasPossesionBT(Sequence):
    def __init__(self, robot: Robot):
        super(FielderAttackerBT, self).__init__("Robot has possesion BT")
        self.add_children([]) # import Action

robot_has_possesion = py_trees.composites.Selector("Robot has possesion?")

position_self = py_trees.composites.Sequence("Position self")

determine_location_to_position = Behaviour("Determine location to position")
move_to_position = Behaviour("Move to position")

position_self.add_children([determine_location_to_position, move_to_position])
py_trees.composites.Selector().add_child()

has_open_shoot = py_trees.composites.Selector("Has Open Shot?")

shoot = py_trees.composites.Sequence("Shoot")

determine_optimal_shot = Behaviour("Determine optimal shot")
kick_ball_quickly_at_target = Behaviour("Kick ball quickly at target")

shoot.add_children([determine_optimal_shot, kick_ball_quickly_at_target])

space_in_front_of_ballholder = py_trees.composites.Selector("Space in front of ballholder?")

dribble = Behaviour("Dribble")

coordinated_pass = py_trees.composites.Sequence("Coordinated Pass")

choose_ally = Behaviour("Choose Ally")
determine_if_chip_needed = Behaviour("Determine if chip needed")
execute_pass = Behaviour("Execute pass")

coordinated_pass.add_children([choose_ally, determine_if_chip_needed, execute_pass])

space_in_front_of_ballholder.add_children([shoot, space_in_front_of_ballholder])

robot_has_possesion.add_children([position_self, has_open_shoot])