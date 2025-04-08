from py_trees.blackboard import Client
from py_trees.common import Access

from component.robot import Robot
from component.misc import Position, Role
from component.robotEx import RobotDict, RobotList

from utils.colors import Colors

import traceback

class RobotBlackBoard():
    _initialized = False
    _bb_manager = None

    def __new__(cls, *args, **kwargs):
        if not cls._initialized:
            cls._initialized = True
            cls._bb_manager = Client(name = "Main Robot blackboard")

            # Robot
            cls._bb_manager.register_key(key="robots", access=Access.WRITE)
            cls._bb_manager.register_key(key="robots", access=Access.READ)
            cls._bb_manager.robots = RobotDict()
            cls._bb_manager.robots["yellow"] = RobotList(6)
            cls._bb_manager.robots["blue"] = RobotList(6)

            # Ball
            cls._bb_manager.register_key(key="ball", access=Access.WRITE)
            cls._bb_manager.register_key(key="ball", access=Access.READ)
            cls._bb_manager.ball = Position(0, 0)

            # cls._bb_manager.register_key(key="team_variable", access=Access.WRITE)
            # cls._bb_manager.register_key(key="team_variable", access=Access.READ)
            # cls._bb_manager.team_variable = dict()

            # cls._bb_manager.register_key(key="gamestate", access=Access.WRITE)
            # cls._bb_manager.register_key(key="gamestate", access=Access.READ)
            # cls._bb_manager.gamestate = dict()

            cls._bb_param = Client(name = "Config Parameter blackboard")

            cls._bb_param.register_key(key="parameters", access=Access.WRITE)
            cls._bb_param.register_key(key="parameters", access=Access.READ)
            cls._bb_param.parameters = dict()

        return super().__new__(cls)

    @staticmethod
    def setConfig(config):
        RobotBlackBoard._bb_param.parameters = config

    @staticmethod
    def getConfig(*keys):
        config = RobotBlackBoard._bb_param.parameters
        try:
            for key in keys:
                config = config[key]
            return config
        except KeyError as e:
            print(f"{Colors.FAIL}[blackboard] -> Error: Missing key: {e} when involving getConfig for {keys}.\n[blackboard] -> The full traceback is shown below")
            traceback.print_stack()
            print(f"{Colors.FAIL}[blackboard] -> To prevent any further unexpected behavior, the program will be terminated")
            exit(1)
        except Exception as e:
            print(f"{Colors.FAIL}[blackboard] -> Unexpected error: {e}.\n[blackboard] -> The full traceback is shown below")
            traceback.print_stack()
            print(f"{Colors.FAIL}[blackboard] -> To prevent any further unexpected behavior, the program will be terminated")
            exit(1)

    @staticmethod
    def printAllInfo():
        print(RobotBlackBoard._bb_manager)
        print(RobotBlackBoard._bb_param)

    @staticmethod
    def getRobot(team: str = "", id: int = -1) -> Robot:
        return RobotBlackBoard._bb_manager.robots[team][id]

    @staticmethod
    def getRobotList(team: str = "") -> RobotList:
        return RobotBlackBoard._bb_manager.robots[team]

    @staticmethod
    def getRobotDict() -> RobotDict:
        return RobotBlackBoard._bb_manager.robots

    # @staticmethod
    # def getRobotRole(id: int) -> Role:
    #     return RobotBlackBoard._bb_manager.robo

    @staticmethod
    def getBallPosition() -> Position:
        return RobotBlackBoard._bb_manager.ball

    @staticmethod
    def patchRobotData(team: str, newdatalist):
        if team in RobotBlackBoard._bb_param.parameters["match"]["teams"]:
            for i in range(RobotBlackBoard._bb_manager.robots[team].size):
                RobotBlackBoard._bb_manager.robots[team][i].patch(newdatalist[i].x, newdatalist[i].y, newdatalist[i].orientation)

    @staticmethod
    def patchBallData(newdata):
        RobotBlackBoard._bb_manager.ball.x = newdata.x
        RobotBlackBoard._bb_manager.ball.y = newdata.y

    @staticmethod
    def getRole(robot: Robot) -> Role:
        return robot.role


RobotBlackBoard()