from py_trees.blackboard import Client
from py_trees.common import Access

from component.robot import Robot
from component.misc import Position, Role
from component.robotEx import RobotDict, RobotList

from utils.colors import Colors
from typing import Tuple, List

import traceback

class RobotBlackBoard():
    _initialized = False
    _bb_manager = None

    def __new__(cls, *args, **kwargs):
        if not cls._initialized:
            cls._initialized = True

            cls._bb_param = Client(name = "Config Parameter blackboard")

            cls._bb_param.register_key(key="parameters", access=Access.WRITE)
            cls._bb_param.register_key(key="parameters", access=Access.READ)
            cls._bb_param.parameters = dict()

            cls._bb_manager = Client(name = "Main Robot blackboard")

            # Robot
            cls._bb_manager.register_key(key="robots", access=Access.WRITE)
            cls._bb_manager.register_key(key="robots", access=Access.READ)
            cls._bb_manager.robots = RobotDict()
            cls._bb_manager.robots["yellow"] = RobotList("yellow", 6)
            cls._bb_manager.robots["blue"] = RobotList("blue", 6)

            # Ball
            cls._bb_manager.register_key(key="ball", access=Access.WRITE)
            cls._bb_manager.register_key(key="ball", access=Access.READ)
            cls._bb_manager.ball = Position(0, 0)

            # cls._bb_manager.register_key(key="team_variable", access=Access.WRITE)
            # cls._bb_manager.register_key(key="team_variable", access=Access.READ)
            # cls._bb_manager.team_variable = dict()

            cls._bb_manager.register_key(key="gamestate", access=Access.WRITE)
            cls._bb_manager.register_key(key="gamestate", access=Access.READ)
            cls._bb_manager.gamestate = dict()

        return super().__new__(cls)

    @staticmethod
    def setConfig(config):
        RobotBlackBoard._bb_param.parameters = config
        RobotBlackBoard.initConfig()

    @staticmethod
    def initConfig():
        for team in RobotBlackBoard.getRobotDict().keys():
            for robot in RobotBlackBoard.getRobotList(team):
                robot.nor = RobotBlackBoard.getConfig("match", "nor")
                robot.team = RobotBlackBoard.getConfig("match", "our_team")
                robot.updatePub()

        RobotBlackBoard.initRole()
        RobotBlackBoard._bb_manager.gamestate = {
            "shooter_id": None,
            "receive_shot_id": None,
            "team_has_ball": None,
            "opponent_has_ball": False,
            "prev_ball_possesion": None,
        }

    @staticmethod
    def initRole():
        gollie = RobotBlackBoard.getConfig("myteam", "goallie_id")
        RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), gollie).role = Role.GOALKEEPER

        defenders = RobotBlackBoard.getConfig("myteam", "defender_id")
        RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), defenders[0]).role = Role.DEFENSIVE_LEFT
        RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), defenders[1]).role = Role.DEFENSIVE_RIGHT

        centers = RobotBlackBoard.getConfig("myteam", "center_id")
        RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), centers[0]).role = Role.CENTER_LEFT
        RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), centers[1]).role = Role.CENTER_RIGHT

        attacker = RobotBlackBoard.getConfig("myteam", "attacker_id")
        RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), attacker).role = Role.ATTACKING

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
    def getMyTeam() -> str:
        return RobotBlackBoard.getConfig("match", "our_team")

    @staticmethod
    def getOpTeam() -> str:
        return RobotBlackBoard.getConfig("match", "opponent_team")

    @staticmethod
    def printAllInfo() -> None:
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

    @staticmethod
    def getRobotRole(id: int) -> Role:
        return RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), id).role

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

# ===========================================================================================================
# █▀█ █▀█ █▄▄ █▀█ ▀█▀   █▄▄ █▀▀ █░█ ▄▀█ █░█ █ █▀█ █▀█   █▀▀ █▀█ █▀▄▀█ █▀▄▀█ █░█ █▄░█ █ █▀▀ ▄▀█ ▀█▀ █ █▀█ █▄░█
# █▀▄ █▄█ █▄█ █▄█ ░█░   █▄█ ██▄ █▀█ █▀█ ▀▄▀ █ █▄█ █▀▄   █▄▄ █▄█ █░▀░█ █░▀░█ █▄█ █░▀█ █ █▄▄ █▀█ ░█░ █ █▄█ █░▀█
# ===========================================================================================================

    @staticmethod
    def getRobotShooterID() -> int:
        shooter_id = RobotBlackBoard._bb_manager.gamestate["shooter_id"]
        return RobotBlackBoard.getRobot(RobotBlackBoard.getMyTeam(), shooter_id).id if shooter_id != None else None

    @staticmethod
    def setRobotShooterID(rid: int) -> None:
        RobotBlackBoard._bb_manager.gamestate["shooter_id"] = rid

    @staticmethod
    def getIsRobotReceiver(rid: int) -> Tuple[bool, int]:
        receiver_id = RobotBlackBoard._bb_manager.gamestate["receive_shot_id"]
        if receiver_id == None:
            return (False, None)
        return (receiver_id == rid, RobotBlackBoard.getRobotShooterID())

    def setRobotReceiver(rid: int) -> None:
        RobotBlackBoard._bb_manager.gamestate["receive_shot_id"] = rid

    @staticmethod
    def getBallPossession() -> int:
        """
        Returns the id of the robot that has possession of the ball.
        If no robot has possession, returns None.
        """
        rid = RobotBlackBoard._bb_manager.gamestate["team_has_ball"]
        if rid != None:
            return rid
        elif RobotBlackBoard._bb_manager.gamestate["opponent_has_ball"]:
            return -1
        else:
            return None

    @staticmethod
    def getPreviousBallPossession() -> int:
        """
        Returns the id of the robot that had possession of the ball before the current one.
        If no robot had possession, returns None.
        """
        rid = RobotBlackBoard._bb_manager.gamestate["prev_ball_possesion"]
        if rid != None:
            return rid
        else:
            return -99

    @staticmethod
    def setPreviousBallPossession(rid: int) -> None:
        RobotBlackBoard._bb_manager.gamestate["prev_ball_possesion"] = rid

RobotBlackBoard()