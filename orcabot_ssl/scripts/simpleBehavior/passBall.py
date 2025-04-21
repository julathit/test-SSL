from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel

from utils.blackboard import RobotBlackBoard

from action import MoveToBallAction, KickAction, GoToLineUpAction, FaceToBallAction, DribblerAction,AimingAction, waitAction

def passBall(passer_Id: int, catcher_Id: int):
    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    faceToBall1 = FaceToBallAction(passer_Id)
    gotoBall1 = MoveToBallAction(passer_Id)
    dribling1 = DribblerAction(passer_Id)
    aiming1 = AimingAction(passer_Id,catcher_Id)
    gotoBall2 = MoveToBallAction(passer_Id)
    faceToBallcatch = FaceToBallAction(catcher_Id)
    kick1 = KickAction(passer_Id)
    catching = DribblerAction(catcher_Id, 1.5)

    root.add_children([
        faceToBall1,
        gotoBall1,
        dribling1,
        aiming1,
        faceToBallcatch,
        gotoBall2,
        kick1,
        catching
    ])

    return root