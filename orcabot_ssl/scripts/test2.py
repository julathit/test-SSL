#! /usr/bin/env python3
from utils.orcabot import *
from CentralBT import CentralBT
# py_trees.display.render_dot_tree(FielderAttackerBT(Robot(0, Position(0, 0), 0)))
robot = Robot("blue", 0, Position(0, 0), 0)
robot.role = Role.CENTER_LEFT
py_trees.display.render_dot_tree(CentralBT(robot))
# py_trees.display.render_dot_tree(GoalKeeperBT(Robot(0, Position(0, 0), 0)))