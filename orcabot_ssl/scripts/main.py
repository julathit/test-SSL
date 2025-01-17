#! /usr/bin/env python3

from utils.blackboard import RobotBlackBoard
from utils.Initializer import Initializer

#initialize
if not Initializer.init_all():
    exit()

RobotBlackBoard.printAllInfo()

print(RobotBlackBoard.getRobots())
print(RobotBlackBoard.getRobots("blue"))
print(RobotBlackBoard.getRobots("blue", 2))
