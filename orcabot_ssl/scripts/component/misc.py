from enum import Enum

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

    def to_list(self):
        return [self.x, self.y]

    def to_tuple(self):
        return (self.x, self.y)
