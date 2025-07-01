from component.robot import Robot
from component.misc import Position
from utils.colors import Colors

from collections.abc import MutableSequence
from typing import MutableSequence, List, TypeVar, Generic, Iterator

class RobotList(MutableSequence[Robot]):
    def __init__(self, team: str, size: int):
        super().__init__()
        self._items: List[Robot] = []
        self.size = size
        self.team = team
        for i in range(size):
            self.append(Robot(team, i, Position(0, 0), 0))

    def __getitem__(self, index: int) -> Robot:
        return self._items[index]

    def __setitem__(self, index: int, value: Robot) -> None:
        self._items[index] = value

    def __delitem__(self, index: int) -> None:
        del self._items[index]

    def __len__(self) -> int:
        return len(self._items)

    def insert(self, index: int, value: Robot) -> None:
        self._items.insert(index, value)

    def __iter__(self) -> Iterator[Robot]:
        return iter(self._items)

    # for debugging
    def __str__(self):
        ret_str = ""
        for robot in self:
            ret_str += Colors.MYTH + f" robotID: {robot.id}\n" + Colors.OKBLUE + f"\t- Position: {{{robot.position}}}\n"
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