# import ROS communition message protocol
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

import numpy as np
import rospy
import math

from component.misc import Position, Role

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

        self.oldx = 0
        self.oldy = 0

        #end dwa

        rospy.init_node("detect", anonymous=False)

    def __str__(self):
        return str("Robot: {} -> {}".format(self.id, self.position))

    def patch(self, x: float, y: float, orientation: float):
        self.position.x = x
        self.position.y = y
        self.orientation = orientation

    def updatePub(self):
        self.pub = rospy.Publisher(f'/robot_{self.team}_{self.id}/cmd', SSL, queue_size=10)

    # Define the motion model
    def motion(self, state: np.array, control_input: list, dt: float) -> np.array:
        x, y, yaw, v, omega = state
        v_new, omega_new = control_input
        x += v_new * np.cos(yaw) * dt
        y += v_new * np.sin(yaw) * dt
        yaw += omega_new * dt
        v = v_new
        omega = omega_new
        return np.array([x, y, yaw, v, omega])

    def calculate_dynamic_window(self, state: np.array) -> tuple:
        v, omega = state[3], state[4]
        vs = [max(self.min_speed, v - self.max_accel * self.dt),
            min(self.max_speed, v + self.max_accel * self.dt)]
        ws = [max(-self.max_yaw_rate, omega - self.max_delta_yaw_rate * self.dt),
            min(self.max_yaw_rate, omega + self.max_delta_yaw_rate * self.dt)]
        return vs, ws

    def predict_trajectory(self, state: np.array, v: float, omega: float) -> np.array:
        trajectory = [state]
        time = 0
        while time <= self.predict_time:
            state = self.motion(state, [v, omega], self.dt)
            trajectory.append(state)
            time += self.dt
        return np.array(trajectory)

    def calc_to_goal_cost(self, trajectory: list, goal: Position) -> float:
        dx = goal.x - trajectory[-1, 0]
        dy = goal.y - trajectory[-1, 1]
        return np.hypot(dx, dy)

    def calc_obstacle_cost(self, trajectory, obstacles):
        min_distance = float('inf')
        for ob in obstacles:
            distances = np.hypot(trajectory[:, 0] - ob[0], trajectory[:, 1] - ob[1])
            min_distance = min(min_distance, min(distances))
        if min_distance < self.robot_radius:
            return float('inf')  # Collision
        return 1.0 / min_distance

    def dwa(self, state, goal: Position, obstacles):
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
        return self.nearPoint(self.RobotBlackBoard.getBallPosition(), 115)

    #testing Method dwa

    def getV(self):
        x, y = self.getPosition().to_list()
        vx = x - self.oldx
        vy = y - self.oldy
        v = np.sqrt(vx**2+vy**2)
        self.oldx, self.oldy = x, y
        return v

    def bestMove(self, point: Position, speed = 1.0, avoidBall = False) -> list:

        obstacles = [
            self.RobotBlackBoard.getRobot(self.team, i).getPosition().to_list()
            for i in range(self.nor) if i != self.id
        ]
        if avoidBall:
            obstacles.append(self.RobotBlackBoard.getBallPosition().to_list)
        x, y = self.getPosition().to_list()
        z = self.getOrientation()
        state = np.array([x, y, z, self.getV(), 0.0])
        u, _ = self.dwa(state, point, obstacles)
        vx, ome = u
        self.sendCommand((vx/90  + 1)*speed, 0, ome*10, False)

    def bestMoveModify(self, point: Position, avoidBall = False):
        headingAngToBall = self.__angToPoint(point) - self.getOrientation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if self.__distanceToPoint(point) < 250:
            self.goToPoint(point, 0.6)
        elif self.__distanceToPoint(point) < 400:
            self.goToPoint(point, 0.4)
        elif abs(headingAngToBall) < np.pi/4:
            self.bestMove(point, 0.6, avoidBall)
        elif abs(headingAngToBall) >= np.pi/4:
            self.sendCommand(0, 0, 3*headingAngToBall, False)

    def MoveToBallModify(self):
        point = self.RobotBlackBoard.getBallPosition()
        headingAngToBall = self.__angToPoint(point) - self.getOrientation()

        if headingAngToBall > math.pi:
            headingAngToBall -= 2 * math.pi

        elif headingAngToBall < -math.pi:
            headingAngToBall += 2 * math.pi

        if self.__distanceToPoint(point) < 250:
            self.goToBall(0.3)
        elif self.__distanceToPoint(point) < 400:
            self.goToBall(0.7)
        elif abs(headingAngToBall) < np.pi/4:
            self.bestMove(point)
        elif abs(headingAngToBall) >= 0.1:
            self.sendCommand(0, 0, 3*headingAngToBall, False)


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