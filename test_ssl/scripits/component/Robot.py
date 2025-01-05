# import ROS communition message protocol
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

import numpy as np
import rospy
import math
from collections.abc import Callable

from .Ball import Ball

class Robot:
    def __init__(self, robot_ID: int, team: int = 1, ball: Ball = None, allRobotsDataUpdataFunction: Callable = None):
        self.robot_ID: int = robot_ID
        self.team: int = team
        self.ball: Ball = ball
        self.updateRobotsData: Callable = allRobotsDataUpdataFunction

        self.robotsData: dict[int, object] = self.updateRobotsData()
        self.robot = self.robotsData[self.robot_ID]

        self.ssl_msg: SSL = SSL()
        self.pub: rospy.Publisher = rospy.Publisher(f'/robot_blue_{robot_ID}/cmd', SSL, queue_size=10)

        rospy.init_node("detect", anonymous=False)

    def __updateRobotsData(self):
        self.robotsData = self.updateRobotsData()
        self.robot = self.robotsData[self.robot_ID]

    def __getPosition(self) -> tuple:
        self.__updateRobotsData()
        return (self.robotsData[self.robot_ID].x, self.robotsData[self.robot_ID].y)

    def __getOrintation(self) -> float:
        self.__updateRobotsData()
        return self.robotsData[self.robot_ID].orientation

    def __distanceToPoint(self, point: tuple) -> float:
        self.__updateRobotsData()
        return math.sqrt((point[1] - self.robotsData[self.robot_ID].y)**2 + (point[0] - self.robotsData[self.robot_ID].x)**2)

    def __angToPoint(self, point: tuple) -> float:
        self.__updateRobotsData()
        return math.atan2(point[1] - self.robotsData[self.robot_ID].y,point[0] - self.robotsData[self.robot_ID].x)

    def __distance(self, origins : tuple , points : list) -> list:
        x, y = origins
        return [np.sqrt((x - points[i][0])**2 + (y - points[i][1])**2) for i in range(len(points))]

    def __raycast(self, points: list, origin: tuple, angle: float, dt: int, number_of_step=2) -> bool:
        x = origin[0] + dt * np.cos(angle)
        y = origin[1] + dt * np.sin(angle)
        for _ in range(number_of_step):
            distances = self.__distance((x,y), points)
            print(distances)
            for dis in distances:
                if dis < dt/2:
                    return True

            x += dt * np.cos(angle)
            y += dt * np.sin(angle)

        return False


    def rayHit(self) -> bool:
        self.__updateRobotsData()
        num_of_robot = 11
        points = [(self.robotsData[i].x, self.robotsData[i].y) for i in range(num_of_robot) if i != self.robot_ID]
        return self.__raycast(points,self.__myRobotPosition(),self.__myRobotOrintation(),300)

    def __sendCommand(self, x: float, y: float, z: float, kickPower: bool):

        self.ssl_msg.cmd_vel.angular.z = z
        self.ssl_msg.cmd_vel.linear.x = x
        self.ssl_msg.cmd_vel.linear.y = y
        self.ssl_msg.kicker = kickPower
        self.pub.publish(self.ssl_msg)

    def goToPoint(self, point : tuple) -> None:
        headingAngToBall = self.__angToPoint(point) - self.robot.orientation

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if self.__distanceToPoint(point) < 20:
            self.__sendCommand(0,0,0,False)
        elif abs(headingAngToBall) < 0.1:
            self.__sendCommand(min(0.25*self.__distanceToPoint(point)+0.25,20),0,0,False)
        elif abs(headingAngToBall) >= 0.1:
            self.__sendCommand(0,0,3*headingAngToBall,False)

    def nearPoint(self, point : tuple) -> bool:
        if self.__distanceToPoint(point) < 130:
            return True
        return False

    def faceToPoint(self, point : tuple) -> None:
        headingAngToBall = self.__angToPoint(point) - self.robot.orientation

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if abs(headingAngToBall) >= 0.1:
            self.__sendCommand(0,0,3*headingAngToBall,False)

    def goToBall(self) -> None:
        self.goToPoint(self.ball.getPosition())

    def faceToBall(self) -> None:
        self.faceToPoint(self.ball.getPosition())

    def nearBall(self)-> bool:
        return self.nearPoint(self.ball.getPosition())

