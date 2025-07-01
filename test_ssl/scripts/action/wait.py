from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
import time

from utils.blackboard import RobotBlackBoard as RBB

class wait(Behaviour):
    def __init__(self, robot_ID: int, delay = 1.0):
        super(wait, self).__init__(f"Wait on robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)
        self.delay = delay

    def initialise(self):
        self.time = self.delay
        self.start_time = time.time()

    def update(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if elapsed_time < self.time:
            return Status.RUNNING
        else:
            self.robot.stop()
            return Status.SUCCESS