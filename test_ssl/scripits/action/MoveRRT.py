from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

from .utils.RRTPlanner import rrt

class MoveRRT(Behaviour):
    def __init__(self, robotManager: RobotManager, robot_ID : int):
        super(MoveRRT, self).__init__(f"robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)
        self.robotManager: RobotManager = robotManager

    def update(self):
        if self.robot.nearBall() != True:

            obstrucle_list = []

            for i in range(self.robotManager.nor):
                if i != self.robot_ID:
                    obstrucle_list.append((self.robot.robotsData[i].x, self.robot.robotsData[i].y))

            path = rrt(self.robot.getPosition(), self.robotManager.ball.getPosition(), obstrucle_list, 50, (-4500, 4500), (-6000, 6000), 50, 150)

            while path == None:
                print("finding path")
                path = rrt(self.robot.getPosition(), self.robotManager.ball.getPosition(), obstrucle_list, 50, (-4500, 4500), (-6000, 6000), 50, 150)
            self.robot.goToPoint((path[1][0]*10, path[1][1]*10))
            return Status.RUNNING
        else:
            self.feedback_message = ("I'm at the ball now!")
            self.logger.debug("\n%s.update()[%s -> %s][%s]" % (self.__class__.__name__, self.status, Status.SUCCESS, self.feedback_message))
            return Status.SUCCESS

# start = (0, 0)
# goal = (9, 9)
# obstacle_coords = [(3, 3), (5, 5), (7, 7)]
# obstacle_radius = 0.8
# x_bounds = (0, 10)
# y_bounds = (0, 10)
# max_iters = 10000
# step_size = 0.05

# path = rrt(start, goal, obstacle_coords, obstacle_radius, x_bounds, y_bounds, max_iters, step_size)