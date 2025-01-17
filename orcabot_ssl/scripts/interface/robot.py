from enum import Enum
from utils.colors import colors

class Role(Enum):
    ATTACKER = 1
    DEFENDER = 2
    GOALKEEPER = 3

class Position(object):
    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y

    def __str__(self):
        return str("Position ({}, {})".format(self.x, self.y))

class Robot(object):
    def __init__(self, id: int, position: Position, orientation: float):
        self.id: int = id
        self.position: Position = position
        self.orientation: float = orientation
        self.role: Role = None

    def __str__(self):
        return str("Robot: {} -> {}".format(self.id, self.position))

    def patch(self, x: float, y: float, orientation: float):
        self.position.x = x
        self.position.y = y
        self.orientation = orientation

class RobotList(list):
    def __init__(self, size: int):
        super(RobotList, self).__init__()
        self.size = size
        for i in range(size):
            self.append(Robot(i, Position(0, 0), 0))

    # for debugging
    def __str__(self):
        ret_str = ""
        for robot in self:
            ret_str += colors.MYTH + " robotID: {}".format(robot.id) + "\n" + colors.OKBLUE + "\t- Position: {{{}}}\n".format(robot.position)
        return ret_str + colors.RESET

    def patch(self, newdatalist):
        for i in range(self.size):
            self[i].patch(newdatalist[i].x, newdatalist[i].y, newdatalist[i].orientation)

class RobotDict(dict):
    def __str__(self):
        ret_str = ""
        for key in self.keys():
            ret_str += colors.OKBLUE + " team: " + colors.OKGREEN + key + colors.OKBLUE + ", number of robots: " + colors.OKGREEN + "{}\n".format(str(len(self[key])))
        return ret_str[:-1] + colors.RESET
