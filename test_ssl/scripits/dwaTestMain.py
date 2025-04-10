#!/usr/bin/env python3

from component.RobotManager import RobotManager
from component.Ball import Ball

from component.RobotManager import RobotManager
from component.Robot import Robot

if __name__=='__main__':

    robotManager: RobotManager = RobotManager(6, Ball())

    robot: Robot = robotManager.getRobotByID(1)

    while True:
        # robot.bestMoveModify((0,0))
        print(robot.aiming((0,0), (0,-1)))