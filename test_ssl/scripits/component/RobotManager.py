from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

import numpy as np
import rospy
import math

from .Ball import Ball
from .Robot import Robot

class RobotManager:
    def __init__(self, nor: int, ball: Ball):

        self.nor = nor

        def __updateRobotsData(data):
            for i in range(0, len(data.robots_blue)):
                id_robots = data.robots_blue[i].robot_id
                for j in range(nor):
                    if id_robots == j:
                        self._robotDict[j] = data.robots_blue[i]

        self._robotDict: dict = {i: SSL_DetectionRobot() for i in range(nor)}
        self.sub = rospy.Subscriber("/vision", SSL_DetectionFrame, __updateRobotsData)

        def __getUpdateRobotsData() -> dict:
            return self._robotDict

        self.ball: Ball = ball
        self.robots: dict[int, Robot] = {i: Robot(i, ball = ball, allRobotsDataUpdataFunction = __getUpdateRobotsData) for i in range(nor)}

    def getRobotByID(self, ID: int) -> Robot:
        return self.robots[ID]

    def getBall(self) -> Ball:
        return self.ball