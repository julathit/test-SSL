from component.robot import Robot
from component.misc import Position
from utils.colors import Colors

class RobotList(list):
    def __init__(self, team: str, size: int):
        super(RobotList, self).__init__()
        self.size = size
        self.team = team
        for i in range(size):
            self.append(Robot(team, i, Position(0, 0), 0))

    # for debugging
    def __str__(self):
        ret_str = ""
        for robot in self:
            ret_str += Colors.MYTH + " robotID: {}".format(robot.id) + "\n" + Colors.OKBLUE + "\t- Position: {{{}}}\n".format(robot.position)
        return ret_str + Colors.RESET

    def patch(self, newdatalist):
        for i in range(self.size):
            self[i].patch(newdatalist[i].x, newdatalist[i].y, newdatalist[i].orientation)

class RobotDict(dict):
    def __str__(self):
        ret_str = ""
        for key in self.keys():
            ret_str += Colors.OKBLUE + " team: " + Colors.OKGREEN + key + Colors.OKBLUE + ", number of robots: " + Colors.OKGREEN + "{}\n".format(str(len(self[key])))
        return ret_str[:-1] + Colors.RESET