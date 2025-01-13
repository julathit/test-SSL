from enum import Enum

class Role(Enum):
    ATTACKER = 1
    DEFENDER = 2
    GOALKEEPER = 3

class Position(object):
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y

    def __str__(self):
        return str("Position {}, {}".format(self.x, self.y))

class Robot(object):
    def __init__(self, id: int, team: str, position: Position, orientation: float):
        self.id: int = id
        self.team: str = team
        self.position: Position = position
        self.orientation: float = orientation
        self.role: Role = None

    def __str__(self):
        return str("Robot: {}, {}".format(self.id, self.position))

class RobotList(list):
    def __str__(self):
        return str("[" + ", ".join([str(self[i]) for i in range(len(self))]) + "]")