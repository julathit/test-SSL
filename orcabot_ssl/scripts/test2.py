#! /usr/bin/env python3
import py_trees

from role.FielderAttackerBT import FielderAttackerBT
from role.FielderDefenderBT import FielderDefenderBT
from role.GoalKeeperBT import GoalKeeperBT
from interface.robot import Robot, Position

# py_trees.display.render_dot_tree(FielderAttackerBT(Robot(0, Position(0, 0), 0)))
py_trees.display.render_dot_tree(FielderDefenderBT(Robot(0, Position(0, 0), 0)))
py_trees.display.render_dot_tree(GoalKeeperBT(Robot(0, Position(0, 0), 0)))