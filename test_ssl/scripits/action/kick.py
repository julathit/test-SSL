from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

class kick(Behaviour):
    def __init__(self,robotManager: RobotManager ,robot_ID: int):
        super(kick,self).__init__(f"kick ball robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot: Robot = robotManager.getRobotByID(self.robot_ID)

    def initialise(self):
        self.time = 10
        self.dt = 0

    def update(self):
        if self.dt < self.time:
            self.robot.kick()
            self.dt += 0.001
            print(self.dt)
            return Status.RUNNING
        else:
            self.robot.stop()
            return Status.SUCCESS