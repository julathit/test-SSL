from enum import Enum

class Role(Enum):
    GOALKEEPER = 1
    DEFENSIVE_LEFT = 2
    DEFENSIVE_RIGHT = 3
    CENTER_LEFT = 4
    CENTER_RIGHT = 5
    ATTACKING = 6

class Position(object):
    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y

    def __str__(self):
        return str("Position ({}, {})".format(self.x, self.y))

    def distanceTo(self, other: "Position") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def to_list(self):
        return [self.x, self.y]

    def to_tuple(self):
        return (self.x, self.y)
