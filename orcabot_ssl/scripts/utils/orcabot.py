__all__ = ['py_trees', 'rospy', 'Behaviour', 'Sequence', 'Parallel', 'Status', 'RobotBlackBoard']

import py_trees
import rospy

from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel
from py_trees.common import Status

# initializing
from utils.blackboard import RobotBlackBoard
from utils.Initializer import Initializer

if not Initializer.init_all():
    exit()

from action import MoveToBallAction, KickAction, DribblerAction, AimingAction, FaceToBallAction, waitAction
__all__ = __all__ + ['MoveToBallAction', 'KickAction', 'DribblerAction', 'AimingAction', 'FaceToBallAction', 'waitAction']

from simpleBehavior import passBallBehavior
__all__ = __all__ + ['passBallBehavior']

def enable_debug():
    global debug
    debug = True

    py_trees.logging.level = py_trees.logging.Level.DEBUG

__all__ = __all__ + ['enable_debug']