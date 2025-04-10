from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager
import time

class wait(Behaviour):
    def __init__(self,robotManager: RobotManager ,robot_ID: int, delay = 1.0):
        super(wait,self).__init__(f"wait robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)
        self.delay = delay

    def initialise(self):
        self.time = self.delay
        self.start_time = time.time()

    def update(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if elapsed_time< self.time:
            return Status.RUNNING
        else:
            self.robot.stop()
            return Status.SUCCESS