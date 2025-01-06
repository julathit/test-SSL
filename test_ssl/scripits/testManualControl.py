from gui.ManualControl import ManualControl
from component.RobotManager import RobotManager
from component.Ball import Ball


if __name__=='__main__':

    robotManager: RobotManager = RobotManager(5, Ball())

    key_control = ManualControl(robotManager, 1)
    key_control.debug = True

    while True:
        key_control.update()