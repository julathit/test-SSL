__all__ = ['py_trees', 'rospy', 'Behaviour', 'Sequence', 'Parallel', 'Status', 'RobotBlackBoard']

import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

# initializing
from utils.blackboard import RobotBlackBoard
from utils.Initializer import Initializer
from utils.colors import Colors

if not Initializer.init_all():
    exit()

from action import MoveToBallAction, MoveToPointAction, KickAction, DribblerAction, AimingAction, FaceToBallAction, waitAction
__all__ = __all__ + ['MoveToBallAction', 'MoveToPointAction', 'KickAction', 'DribblerAction', 'AimingAction', 'FaceToBallAction', 'waitAction']

from simpleBehavior import passBallBehavior
__all__ = __all__ + ['passBallBehavior']

# from role.FielderDefenderBT import FielderDefenderBT
# __all__ = __all__ + ['FielderDefenderBT']

def enable_debug():
    global debug
    debug = True

    py_trees.logging.level = py_trees.logging.Level.DEBUG

__all__ = __all__ + ['enable_debug']

from component.area import ZoneManager, Zone
__all__ = __all__ + ['ZoneManager', 'Zone']

from component.robot import Robot
__all__ = __all__ + ['Robot']

from component.misc import Role, Position
__all__ = __all__ + ['Role', 'Position']

from component.robotEx import RobotDict, RobotList
__all__ = __all__ + ['RobotDict', 'RobotList']

def updateState() -> None:
    """
    Update the state of the blackboard.
    """
    updateGameState()

__all__ = __all__ + ['updateState']

def updateGameState() -> None:
    team = RobotBlackBoard.getMyTeam()
    for robot in RobotBlackBoard.getRobotList(team):
        if robot.nearBall():
            print(f"Our Robot {robot.id} has possession of the ball")
            RobotBlackBoard._bb_manager.gamestate["team_has_ball"] = robot.id
            RobotBlackBoard._bb_manager.gamestate["opponent_has_ball"] = False
            return None

    op_team = RobotBlackBoard.getConfig("match", "opponent_team")
    for robot in RobotBlackBoard.getRobotList(op_team):
        if robot.nearBall():
            print(f"Op Robot {robot.id} has possession of the ball")
            RobotBlackBoard._bb_manager.gamestate["team_has_ball"] = None
            RobotBlackBoard._bb_manager.gamestate["opponent_has_ball"] = True
            RobotBlackBoard._bb_manager.gamestate["prev_ball_possesion"] = -robot.id
            return None
    else:
        RobotBlackBoard._bb_manager.gamestate["team_has_ball"] = None
        RobotBlackBoard._bb_manager.gamestate["opponent_has_ball"] = False



# end of file
import main
main.main()
print(Colors.OKGREEN + "[Orcabot SSL] The script has been executed successfully.")