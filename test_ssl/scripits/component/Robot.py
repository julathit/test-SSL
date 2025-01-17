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
        self.updateRobotsData: Callable = allRobotsDataUpdataFunction

        self.robotsData: dict[int, object] = self.updateRobotsData()
        self.robot = self.robotsData[self.robot_ID]

        self.ssl_msg: SSL = SSL()
        self.pub: rospy.Publisher = rospy.Publisher(f'/robot_blue_{robot_ID}/cmd', SSL, queue_size=10)
        self.ball: Ball = ball

        self.num_of_robot = 6

        #this for dwa

        # Robot specifications
        self.robot_radius = 200
        self.max_speed = 1000
        self.min_speed = -500
        self.max_yaw_rate = 40.0 * np.pi / 180.0
        self.max_accel = 90
        self.max_delta_yaw_rate = 10.0 * np.pi / 180.0
        self.velocity_resolution = 10
        self.yaw_rate_resolution = 2.5 * np.pi / 180.0
        self.dt = 1  # Time step
        self.predict_time = 5.0  # Prediction horizon

        # Cost function weights
        self.heading_cost_gain = 0.1
        self.distance_cost_gain = 40.0
        self.velocity_cost_gain = 1.0

        self.oldx = self.robotsData[self.robot_ID].x
        self.oldy = self.robotsData[self.robot_ID].y

        #end dwa

        rospy.init_node("detect", anonymous=False)

    # Define the motion model
    def motion(self,state: np.array, control_input: list, dt: float) -> np.array:
        x, y, yaw, v, omega = state
        v_new, omega_new = control_input
        x += v_new * np.cos(yaw) * dt
        y += v_new * np.sin(yaw) * dt
        yaw += omega_new * dt
        v = v_new
        omega = omega_new
        return np.array([x, y, yaw, v, omega])

    def calculate_dynamic_window(self,state: np.array) -> tuple:
        v, omega = state[3], state[4]
        vs = [max(self.min_speed, v - self.max_accel * self.dt),
            min(self.max_speed, v + self.max_accel * self.dt)]
        ws = [max(-self.max_yaw_rate, omega - self.max_delta_yaw_rate * self.dt),
            min(self.max_yaw_rate, omega + self.max_delta_yaw_rate * self.dt)]
        return vs, ws

    def predict_trajectory(self,state: np.array, v: float, omega: float) -> np.array:
        trajectory = [state]
        time = 0
        while time <= self.predict_time:
            state = self.motion(state, [v, omega], self.dt)
            trajectory.append(state)
            time += self.dt
        return np.array(trajectory)

    def calc_to_goal_cost(self,trajectory: list, goal: list) -> float:
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        return np.hypot(dx, dy)

    def calc_obstacle_cost(self,trajectory, obstacles):
        min_distance = float('inf')
        for ob in obstacles:
            distances = np.hypot(trajectory[:, 0] - ob[0], trajectory[:, 1] - ob[1])
            min_distance = min(min_distance, min(distances))
        if min_distance < self.robot_radius:
            return float('inf')  # Collision
        return 1.0 / min_distance

    def dwa(self,state, goal, obstacles):
        dynamic_window = self.calculate_dynamic_window(state)
        best_u = [0.0, 0.0]
        best_trajectory = np.array([state])
        min_cost = float('inf')

        for v in np.arange(dynamic_window[0][0], dynamic_window[0][1], self.velocity_resolution):
            for omega in np.arange(dynamic_window[1][0], dynamic_window[1][1], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(state, v, omega)
                to_goal_cost = self.heading_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                obstacle_cost = self.distance_cost_gain * self.calc_obstacle_cost(trajectory, obstacles)
                speed_cost = self.velocity_cost_gain * (self.max_speed - trajectory[-1, 3])
                total_cost = to_goal_cost + obstacle_cost + speed_cost

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [v, omega]
                    best_trajectory = trajectory

        return best_u, best_trajectory

    def __updateRobotsData(self):
        self.robotsData = self.updateRobotsData()
        self.robot = self.robotsData[self.robot_ID]
        self.ball.getPosition()

    def getPosition(self) -> tuple:
        self.__updateRobotsData()
        return (self.robotsData[self.robot_ID].x, self.robotsData[self.robot_ID].y)

    def getOrintation(self) -> float:
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

    #robot command
    def rayHit(self) -> bool:
        self.__updateRobotsData()
        num_of_robot = self.num_of_robot
        points = [(self.robotsData[i].x, self.robotsData[i].y) for i in range(num_of_robot) if i != self.robot_ID]
        return self.__raycast(points, self.__myRobotPosition(), self.__myRobotOrintation(), 300)

    def sendCommand(self, x: float, y: float, z: float, kickPower = False, dribbler = False):

        self.ssl_msg.cmd_vel.angular.z = z
        self.ssl_msg.cmd_vel.linear.x = x
        self.ssl_msg.cmd_vel.linear.y = y
        self.ssl_msg.kicker = kickPower
        self.ssl_msg.dribbler = dribbler
        self.pub.publish(self.ssl_msg)

    def goToPoint(self, point : tuple,speed = 1.0) -> None:
        maxSpeed = 2
        headingAngToBall = self.__angToPoint(point) - self.getOrintation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if self.__distanceToPoint(point) < 20:
            self.sendCommand(0,0,0,False)
        elif abs(headingAngToBall) < 0.1:
            self.sendCommand(min(0.25*self.__distanceToPoint(point)+0.25,maxSpeed)*speed,0,0,False)
        elif abs(headingAngToBall) >= 0.1:
            self.sendCommand(0,0,3*headingAngToBall,False)

    def nearPoint(self, point : tuple, threshold: int = 40) -> bool:
        if self.__distanceToPoint(point) < threshold:
            return True
        return False

    def faceToPoint(self, point : tuple) -> None:
        headingAngToBall = self.__angToPoint(point) - self.getOrintation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if abs(headingAngToBall) >= 0.1:
            self.sendCommand(0,0,3*headingAngToBall,False)

    def kick(self):
        self.__updateRobotsData()
        self.sendCommand(0,0,0,True,0)

    def dribbler(self):
        self.__updateRobotsData()
        self.sendCommand(0,0,0,0, True)

    def stop(self):
        self.__updateRobotsData()
        self.sendCommand(0,0,0,False,False)

    def goToBall(self,speed = 1.0) -> None:
        self.__updateRobotsData()
        self.goToPoint(self.ball.getPosition(),speed)

    def faceToBall(self) -> None:
        self.faceToPoint(self.ball.getPosition())

    def nearBall(self)-> bool:
        return self.nearPoint(self.ball.getPosition(), 145)

    #testing Method dwa

    def getV(self):
        x,y = self.getPosition()
        vx = x - self.oldx
        vy = y - self.oldy
        v = np.sqrt(vx**2+vy**2)
        self.oldx, self.oldy = x, y
        return v

    def bestMove(self,point: tuple) -> list:
        self.__updateRobotsData()
        goal = np.array(point)

        obstacles = [(self.robotsData[i].x, self.robotsData[i].y) for i in range(self.num_of_robot) if i != self.robot_ID]
        x,y = self.getPosition()
        z = self.getOrintation()
        state = np.array([x, y, z, self.getV(), 0.0])
        u,_ = self.dwa(state,goal,obstacles)
        vx, ome = u
        self.sendCommand(vx/90 + 1,0,ome*10,False)

    def bestMoveModify(self, point: tuple):
        headingAngToBall = self.__angToPoint(point) - self.getOrintation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if self.__distanceToPoint(point) < 300:
            self.goToPoint(point)
        elif abs(headingAngToBall) < np.pi/4:
            self.bestMove(point)
        elif abs(headingAngToBall) >= np.pi/4:
            self.sendCommand(0,0,3*headingAngToBall,False)

    def MoveToBallModify(self):
        self.__updateRobotsData()
        point = self.ball.getPosition()
        headingAngToBall = self.__angToPoint(point) - self.getOrintation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if self.__distanceToPoint(point) < 550:
            self.goToBall(0.25)
        elif abs(headingAngToBall) < np.pi/4:
            self.bestMove(point)
        elif abs(headingAngToBall) >= np.pi/4:
            self.sendCommand(0,0,3*headingAngToBall,False)
