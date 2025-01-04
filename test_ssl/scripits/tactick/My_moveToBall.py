from py_trees.behaviour import Behaviour
from py_trees.common import Status
from obj.My_robot import my_robot

class my_moveToBall(Behaviour):
    def __init__(self, robot_ID : int, point : tuple) -> None:
        super(my_moveToBall,self).__init__(f"robot {robot_ID}")
        self.point: tuple = point
        self.robot_ID: int = robot_ID
        self.robot1: my_robot = my_robot(self.robot_ID)

    def update(self):
        if not self.robot1.nearBall():
            self.robot1.goToBall()
            return Status.RUNNING
        else:
            return Status.SUCCESS
        