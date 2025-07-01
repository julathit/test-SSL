# import ROS communition message protocol
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

import numpy as np
import rospy
import math
from utils import SToVCal, PIDcal

import control as ct

from component.misc import Position, Role
import time

class Robot:
    def __init__(self, team:str, rid: int, position: Position, orientation: float):
        # Preventing circular import
        from utils.blackboard import RobotBlackBoard

        self.id: int = rid
        self.position: Position = position
        self.orientation: float = orientation
        self.role: Role = None

        self.RobotBlackBoard = RobotBlackBoard
        self.nor = 0
        self.team = team

        self.ssl_msg: SSL = SSL()
        self.pub = None
        
        #pass data
        self.oldx = 0
        self.oldy = 0
        self.oldPosition = np.array([self.position.x,self.position.y,self.orientation])
        self.oldTime = time.time()
        self.pid = PIDcal(Kp=0.01,Ki=0,Kd=3)
        # Kp=0.01,Ki=0,Kd=3.05
        #end pass data

        rospy.init_node("detect", anonymous=False)

    def __str__(self):
        return str("Robot: {} -> {}".format(self.id, self.position))

    def patch(self, x: float, y: float, orientation: float):
        self.position.x = x
        self.position.y = y
        self.orientation = orientation

    def updatePub(self):
        self.pub = rospy.Publisher(f'/robot_{self.team}_{self.id}/cmd', SSL, queue_size=10)


    def getRole(self) -> Role:
        return self.role

    def getPosition(self) -> Position:
        return self.position

    def getOrientation(self) -> float:
        return self.orientation

    def __distanceToPoint(self, point: Position) -> float:
        return math.sqrt((point.y - self.position.y)**2 + (point.x - self.position.x)**2)

    def __angToPoint(self, point: Position) -> float:
        return math.atan2(point.y - self.position.y, point.x - self.position.x)

    def __distance(self, origins: Position , points: list) -> list:
        x, y = origins.x, origins.y
        return [np.sqrt((x - points[i][0])**2 + (y - points[i][1])**2) for i in range(len(points))]

    def __raycast(self, points: list, origin: Position, angle: float, dt: int, number_of_step=2) -> bool:
        x = origin.x + dt * np.cos(angle)
        y = origin.y + dt * np.sin(angle)
        for _ in range(number_of_step):
            distances = self.__distance(Position(x, y), points)
            print(distances)
            for dis in distances:
                if dis < dt/2:
                    return True

            x += dt * np.cos(angle)
            y += dt * np.sin(angle)

        return False

    #robot command
    def rayHit(self) -> bool:
        points = [(self.RobotBlackBoard.getRobot(self.team, i).getPosition().x, self.RobotBlackBoard.getRobot(self.team, i).getPosition().y) for i in range(self.nor) if i != self.id]
        return self.__raycast(points, self.getPosition(), self.getOrientation(), 300)

    def sendCommand(self, x: float, y: float, z: float, kickPower = False, dribbler = False):
        self.ssl_msg.cmd_vel.angular.z = z
        self.ssl_msg.cmd_vel.linear.x = x
        self.ssl_msg.cmd_vel.linear.y = y
        self.ssl_msg.kicker = kickPower
        self.ssl_msg.dribbler = dribbler
        # print(self.ssl_msg)
        self.pub.publish(self.ssl_msg)

        # rospy.Publisher('/robot_blue_0/cmd', SSL, queue_size=10).publish(self.ssl_msg)

    def goToPoint(self, point: Position, speed = 1.0) -> None:
        maxSpeed = 2
        headingAngToBall = self.__angToPoint(point) - self.getOrientation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi


        if self.__distanceToPoint(point) < 20:
            self.sendCommand(0, 0, 0, False)
        elif abs(headingAngToBall) < 0.1:
            self.sendCommand(min(0.25 * self.__distanceToPoint(point) + 0.25, maxSpeed) * speed, 0, 0, False)
        elif abs(headingAngToBall) >= 0.1:
            self.sendCommand(0, 0, 3 * headingAngToBall, False)

    def nearPoint(self, point: Position, threshold: int = 20) -> bool:
        # print("dis to", point, "is", self.__distanceToPoint(point))
        if self.__distanceToPoint(point) < threshold:
            return True
        return False

    def faceToPoint(self, point: Position) -> bool:
        headingAngToBall = self.__angToPoint(point) - self.getOrientation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if abs(headingAngToBall) >= 0.1:
            self.sendCommand(0, 0, 3 * headingAngToBall, False)
            return False
        else:
            return True

    def kick(self):
        self.sendCommand(0, 0, 0, True, 0)

    def dribbler(self):
        self.sendCommand(0, 0, 0, 0, True)

    def stop(self):
        self.sendCommand(0, 0, 0, False, False)

    def goToBall(self, speed = 1.0) -> None:
        self.goToPoint(self.RobotBlackBoard.getBallPosition(), speed)

    def faceToBall(self) -> bool:
        return self.faceToPoint(self.RobotBlackBoard.getBallPosition())

    def nearBall(self) -> bool:
        return self.nearPoint(self.RobotBlackBoard.getBallPosition(), 125)

    #testing Method dwa

    def getV(self):
        x, y = self.getPosition().to_list()
        vx = x - self.oldx
        vy = y - self.oldy
        v = np.sqrt(vx**2+vy**2)
        self.oldx, self.oldy = x, y
        return v


    def aiming(self, point: Position, target: Position, r = 250, speed = 1.0):
        headingAngToBall = self.__angToPoint(point) - self.getOrientation()
        distance = self.__distanceToPoint(point)
        headingAngle_to_target = math.atan2( target.y - point.y ,target.x - point.x ) - self.__angToPoint(point)
        maxSpeed = 0.5
        err = 50

        #compute for headingAngle
        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi
        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        #compute for robotAngle
        if headingAngle_to_target > math.pi:
            headingAngle_to_target -= 2 * math.pi
        elif headingAngle_to_target < -math.pi:
            headingAngle_to_target += 2 * math.pi

        if abs(headingAngToBall) >= 0.2:
            self.sendCommand(0 ,0, 2*headingAngToBall, False)
        elif abs(headingAngToBall) < 0.2:
            xspeed = 0

            if abs(distance) < r - err:
                    xspeed = -min(0.25*self.__distanceToPoint(point)+0.25, maxSpeed) * speed
            elif abs(distance) > r + err:
                    xspeed = min(0.25*self.__distanceToPoint(point)+0.25, maxSpeed) * speed


            if abs(headingAngle_to_target) >= 0.01 or abs(headingAngToBall) >= 0.1:
                self.sendCommand(xspeed, -headingAngle_to_target*0.5, (2*headingAngToBall+headingAngle_to_target), False)
            else:

                self.sendCommand(0, 0, 0, False)
                return True
        return False
    
    def moveToPointMotionMapping(self,point: Position):
        x, y = self.getPosition().to_list()
        dx = point.x - x 
        dy = point.y - y 
        o = self.getOrientation()
        dom = self.__angToPoint(point) - o
        S = np.array([dx,dy,dom])
        V = SToVCal(S,o)/5
        vx, vy, yaw = V
        self.sendCommand(vx,vy,yaw*10,False,False)

    def MotionMapping(self,point: Position, angle = None) -> np.array:
        x, y = self.getPosition().to_list()
        dx = point.x - x 
        dy = point.y - y 
        o = self.getOrientation()
        if angle == None:
            dom = self.__angToPoint(point) - o
        else:
            dom = angle*np.pi/180 - o

        if dom > math.pi:
            dom -= 2 * math.pi

        elif dom < -math.pi:
            dom += 2 * math.pi

        S = np.array([dx,dy,dom])
        print
        V = SToVCal(S,o)/5
        vx, vy, yaw = V
        return np.array([vx,vy,yaw*20])

    def messureSpeed(self) -> np.array:
        currentPos = np.array([self.getPosition().x,self.getPosition().y,self.getOrientation()])
        currentTime = time.time()
        delS = currentPos - self.oldPosition
        dt = currentTime - self.oldTime
        v = delS/dt
        self.oldPosition = currentPos
        self.oldTime = currentTime
        return v, dt
    
    def MoveToPointWithPID(self,point: Position,angle = None):
        messure, dt = self.messureSpeed()
        print(point)
        setpoint = self.MotionMapping(point,angle)
        output = self.pid.compute(setpoint,messure,dt)
        vx, vy, vz = output
        vxs, vys, vzs = setpoint

        if self.nearPoint(point):
            self.sendCommand(0,0,0)
        else:
            self.sendCommand(vx,vy,vzs)
        
    def MoveToBallPID(self):
        BallPosition = self.RobotBlackBoard.getBallPosition()
        print(BallPosition)
        if not self.nearBall():
            self.MoveToPointWithPID(BallPosition)
        return