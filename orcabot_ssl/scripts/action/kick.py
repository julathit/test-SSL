from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.robot import Robot
from utils.blackboard import RobotBlackBoard as RBB

class kick(Behaviour):
    def __init__(self, robot_ID: int):
        super(kick,self).__init__(f"kick ball robot {robot_ID}")
        self.robot: Robot = RBB.getRobot(RBB.getMyTeam(), robot_ID)

    def initialise(self):
        self.time = 2
        self.dt = 0

        RBB.setRobotShooterID(self.robot.id)

    def update(self):
        if self.dt < self.time:
            RBB.setPreviousBallPossession(self.robot.id)
            self.robot.kick()
            self.dt += 0.001
            print(self.dt)
            return Status.RUNNING
        else:
            RBB.setPreviousBallPossession(-99)
            self.robot.stop()
            return Status.SUCCESS