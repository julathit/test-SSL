#!/usr/bin/env python3


from gui.ManualControl import ManualControl
from component.RobotManager import RobotManager
from component.Ball import Ball

from component.RobotManager import RobotManager
from component.Robot import Robot

if __name__=='__main__':

    robotManager: RobotManager = RobotManager(6, Ball())

    key_control = ManualControl(robotManager, 1)
    key_control.debug = False
    robot: Robot = robotManager.getRobotByID(1)

    while True:
        key_control.update()