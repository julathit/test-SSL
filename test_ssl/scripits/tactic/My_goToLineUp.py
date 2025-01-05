from py_trees.behaviour import Behaviour
from py_trees.common import Status
from obj.My_robot import my_robot

from obj.My_robot import my_robot
from obj.My_ball import my_ball

from numpy import sqrt

class my_goToLineUp(Behaviour):

    def __init__(self, robotId : int,robot_target : int,distance = 200):
        super(my_goToLineUp,self).__init__(f"make line Up robotId {robotId}")
        self.robotId = robotId
        self.robot_target = robot_target
        self.distance = distance
        self.robot = my_robot(self.robotId)
        self.robot_T = my_robot(self.robot_target)
        self.ball = my_ball()

    def calculate_lineup_p(self) -> tuple:
        L_vec : tuple= (self.ball.ballPosition()[0] - self.robot_T.myRobotPosition()[0],self.ball.ballPosition()[1] - self.robot_T.myRobotPosition()[1])
        L_vec_size : float = sqrt(L_vec[0]**2 + L_vec[1]**2)
        L_vec_norm : tuple= L_vec/L_vec_size
        return L_vec_norm*self.distance + self.ball.ballPosition()
    
    def update(self):
        # if self.robot.nearPoint(self.point):
        #     return Status.SUCCESS
        # elif self.robot.rayHit():
        #     return Status.RUNNING
        # else:
        #     self.robot.goToPoint(self.point)
        #     return Status.RUNNING
        line_up_p = self.calculate_lineup_p()
        
        if self.robot.nearPoint(line_up_p):
            return Status.SUCCESS
        else:
            self.robot.goToPoint(line_up_p)
            return Status.RUNNING