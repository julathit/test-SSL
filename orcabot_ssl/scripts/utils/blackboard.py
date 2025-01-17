from py_trees.blackboard import Client
from py_trees.common import Access

from interface.robot import RobotList, RobotDict, Robot, Position, Role

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

    def setConfig(config):
        RobotBlackBoard._bb_param.parameters = config

    @staticmethod
    def printAllInfo():
        print(RobotBlackBoard._bb_manager)
        print(RobotBlackBoard._bb_param)

    @staticmethod
    def getRobots(team: str = "", id: int = -1): # may return RobotDict, RobotList, Robot
        if team in RobotBlackBoard._bb_param.parameters["match"]["teams"]:
            if id in range(RobotBlackBoard._bb_param.parameters["match"]["nor"]):
                return RobotBlackBoard._bb_manager.robots[team][id]
            return RobotBlackBoard._bb_manager.robots[team]
        return RobotBlackBoard._bb_manager.robots

    # @staticmethod
    # def getRobotRole(id: int) -> Role:
    #     return RobotBlackBoard._bb_manager.robo

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