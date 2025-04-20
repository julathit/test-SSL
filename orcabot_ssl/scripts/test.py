#! /usr/bin/env python3

from utils.blackboard import RobotBlackBoard
from utils.Initializer import Initializer

#initialize
if not Initializer.init_all():
    exit()

RobotBlackBoard.printAllInfo()

# print(RobotBlackBoard.getRobots())
# print(RobotBlackBoard.getRobots("blue"))
# print(RobotBlackBoard.getRobots("blue", 2))

# print(RobotBlackBoard.getConfig("field", "field_size"))

# print(RobotBlackBoard.getBallPosition())

#test area
from component.area import ZoneManager

# from utils.debugger.render_field_in_pygame import render
# render(ZoneManager.getAllZones())
# render(zone_manager.getAllOpponentZones())

# for robot in RobotBlackBoard.getRobotList("blue"):
#     print(robot.id, ZoneManager.getZoneFromPosition(robot.position))

print("ball: ", ZoneManager.getZoneFromPosition(RobotBlackBoard.getBallPosition()))

robot = RobotBlackBoard.getRobot("blue", 0)
ball = RobotBlackBoard.getBallPosition()

print(robot.getPosition())

while not robot.nearPoint(ball):
    robot.goToBall()
# from utils.debugger.render_field_in_pygame import render
# render(ZoneManager.getAllZones())

