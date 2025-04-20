from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from component.misc import Position
from utils.blackboard import RobotBlackBoard as RBB
import time

class dribbler(Behaviour):
    def __init__(self, robot_ID: int, delay = 1.0):
        super(dribbler,self).__init__(f"dribbler robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)
        self.delay = delay

    def initialise(self):
        self.time = self.delay
        self.start_time = time.time()

    def update(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if elapsed_time< self.time:
            self.robot.dribbler()
            return Status.RUNNING
        else:
            self.robot.stop()
            return Status.SUCCESS