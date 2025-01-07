from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

class dribbler(Behaviour):
    def __init__(self,robotManager: RobotManager ,robot_ID: int):
        super(dribbler,self).__init__(f"dribbler robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)

    def initialise(self):
        self.time = 10
        self.dt = 0

    def update(self):
        if self.dt < self.time:
            self.robot.dribbler()
            self.dt += 1
            return Status.RUNNING
        else:
            self.robot.stop()
            return Status.SUCCESS