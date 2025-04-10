from py_trees.behaviour import Behaviour
from py_trees.common import Status
from component.Robot import Robot
from component.RobotManager import RobotManager

class aiming(Behaviour):
    def __init__(self, robotManager: RobotManager, robot_ID: int, robot_target_ID: int):
        super(aiming,self).__init__(f"{robot_ID} aiming at {robot_target_ID}")
        #get class of robot and ball
        self.robot = robotManager.getRobotByID(robot_ID)
        self.target_robot = robotManager.getRobotByID(robot_target_ID)
        self.ball = robotManager.getBall()
        #get robot by id
        self.robot_ID = robot_ID
        self.robot_target_ID = robot_target_ID
    
    def update(self):
        if self.robot.aiming(self.ball.getPosition(),self.target_robot.getPosition()) :
            return Status.SUCCESS
        else:
            return Status.RUNNING