from py_trees.behaviour import Behaviour
import py_trees

robot_has_possesion = py_trees.composites.Selector("Robot has possesion?")

position_self = py_trees.composites.Sequence("Position self")

determine_location_to_position = Behaviour("Determine location to position")
move_to_position = Behaviour("Move to position")

position_self.add_children([determine_location_to_position, move_to_position])

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