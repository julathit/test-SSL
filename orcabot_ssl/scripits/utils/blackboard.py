from py_trees.blackboard import Client
from py_trees.common import Access

from ctype.robot import RobotList, Robot, Position

class RobotBlackBoard():
    _initialized = False
    _bb_manager = None

    def __new__(cls, *args, **kwargs):
        if not cls._initialized:
            cls._initialized = True
            cls._bb_manager = Client(name = "Main Robot blackboard")

            cls._bb_manager.register_key(key="robot", access=Access.WRITE)
            cls._bb_manager.register_key(key="robot", access=Access.READ)
            cls._bb_manager.robot = RobotList()

            cls._bb_manager.register_key(key="team_variable", access=Access.WRITE)
            cls._bb_manager.register_key(key="team_variable", access=Access.READ)
            cls._bb_manager.team_variable = dict()

            cls._bb_manager.register_key(key="gamestate", access=Access.WRITE)
            cls._bb_manager.register_key(key="gamestate", access=Access.READ)
            cls._bb_manager.gamestate = dict()

            cls._bb_manager.register_key(key="parameters", access=Access.WRITE)
            cls._bb_manager.register_key(key="parameters", access=Access.READ)
            cls._bb_manager.parameters = dict()

        return super().__new__(cls)

    @staticmethod
    def getInfo():
        print(RobotBlackBoard._bb_manager)

    @staticmethod
    def getRobots() -> RobotList:
        return RobotBlackBoard._bb_manager.robot

RobotBlackBoard()