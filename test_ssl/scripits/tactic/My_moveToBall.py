from py_trees.behaviour import Behaviour
from py_trees.common import Status
from obj.My_robot import my_robot

class my_moveToBall(Behaviour):
    def __init__(self, robot_ID : int) -> None:
        super(my_moveToBall,self).__init__(f"robot {robot_ID}")
        self.robot_ID: int = robot_ID
        self.robot1: my_robot = my_robot(self.robot_ID)

    def update(self):
        if self.robot1.nearBall():
            return Status.SUCCESS
        else:
            print(self.robot1.nearBall())
            self.robot1.goToBall()
            return Status.RUNNING
        