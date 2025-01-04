import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import numpy as np

from .My_ball import my_ball

import math

#number of robot
n = 11

#init ssl_msg
ssl_msg = {i: SSL() for i in range(n)}

#make dict of robot
robot = {i: SSL_DetectionRobot() for i in range(n)}

def recibir_datos(data):

    for i in range(0, len(data.robots_blue)):
        id_robots = data.robots_blue[i].robot_id
        for j in range(n):
            if id_robots == j:
                robot[j] = data.robots_blue[i]

#initPub
rospy.init_node("detect", anonymous=False)

#initSubscriber
sub = rospy.Subscriber("/vision", SSL_DetectionFrame, recibir_datos)

#initpub
pub = {i: rospy.Publisher(f'/robot_blue_{i}/cmd', SSL, queue_size=10) for i in range(n)}

class my_robot:
    def __init__(self,robot_ID : int,team = 1 ):
        self.robot_ID = robot_ID
        self.team = team
        self.ball = my_ball()

    def __myRobotPosition(self) -> tuple:
        return (robot[self.robot_ID].x,robot[self.robot_ID].y)

    def __myRobotOrintation(self) -> float:
        return robot[self.robot_ID].orientation

    def __distanceToPoint(self, point: tuple) -> None:
        return math.sqrt((point[1] - robot[self.robot_ID].y)**2 + (point[0] - robot[self.robot_ID].x)**2) 

    def __angToPoint(self, point: tuple) -> None:
        return math.atan2(point[1] - robot[self.robot_ID].y,point[0] - robot[self.robot_ID].x)
    
    def __distance(self,origins : tuple , points : list) -> list:
        x, y = origins
        return [np.sqrt((x-points[i][0])**2 + (y-points[i][1])**2) for i in range(len(points))]
    
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
        num_of_robot = 11
        points = [(robot[i].x,robot[i].y) for i in range(num_of_robot) if i != self.robot_ID]
        return self.__raycast(points,self.__myRobotPosition(),self.__myRobotOrintation(),300)
    
    def __sendCommand(self,x: float, y: float, z: float, kickPower: bool):
        
        ssl_msg[self.robot_ID].cmd_vel.angular.z = z
        ssl_msg[self.robot_ID].cmd_vel.linear.x = x
        ssl_msg[self.robot_ID].cmd_vel.linear.y = y
        ssl_msg[self.robot_ID].kicker = kickPower
        pub[self.robot_ID].publish(ssl_msg[self.robot_ID])

    def goToPoint(self,point : tuple) -> None:
        headingAngToBall = self.__angToPoint(point) - robot[self.robot_ID].orientation

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
        
    def nearPoint(self,point : tuple) -> bool:
        if self.__distanceToPoint(point) < 40:
            return True
        return False

    def faceToPoint(self,point : tuple) -> None:
        headingAngToBall = self.__angToPoint(point) - robot[self.robot_ID].orientation

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if abs(headingAngToBall) >= 0.1:
            self.__sendCommand(0,0,3*headingAngToBall,False)

    def goToBall(self) -> None:
        self.goToPoint(self.ball.ballPosition())

    def faceToBall(self) -> None:
        self.faceToPoint(self.ball.ballPosition())

    def nearBall(self)-> bool:
        self.nearPoint(self.ball.ballPosition())

