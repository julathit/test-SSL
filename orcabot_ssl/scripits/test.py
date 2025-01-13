#! /usr/bin/env python3
import py_trees

class Position(object):
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y

    def __str__(self):
        return str("Position {}, {}".format(self.x, self.y))

class Robot(object):
    def __init__(self, id: int, position: Position):
        self.id: int = id
        self.position: Position = position

    def __str__(self):
        return str("Robot: {}, {}".format(self.id, self.position))

class RobotList(list):
    def __str__(self):
        return str("[" + ", ".join([str(self[i]) for i in range(len(self))]) + "]")

class RobotDict(object):
    def __str__(self):
        return str(list(self.__dict__.keys()))


writer = py_trees.blackboard.Client(name="Writer")
writer.register_key(key="robots", access=py_trees.common.Access.WRITE)
reader = py_trees.blackboard.Client(name="Reader")
reader.register_key(key="robots", access=py_trees.common.Access.READ)

writer.robots = RobotDict()
writer.robots.ra = Robot(0, Position(50, 50))
writer.robots.rb = Robot(1, Position(100, 100))

print(reader)

print(reader.robots)

print(reader.robots.ra.position)