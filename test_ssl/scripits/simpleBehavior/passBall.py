from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence, Parallel

from component.RobotManager import RobotManager

from action import MoveToBallAction, KickAction, GoToLineUpAction, FaceToBallAction, DribblerAction,AimingAction,waitAction

def passBall(robotManager: RobotManager,passer_Id, catcher_Id):
    root : Sequence = Sequence(name = "goToPoint" , memory=True)

    faceToBall1 = FaceToBallAction(robotManager,passer_Id)
    gotoBall1 = MoveToBallAction(robotManager,passer_Id)
    dribling1 = DribblerAction(robotManager,passer_Id)
    aiming1 = AimingAction(robotManager,passer_Id,catcher_Id)
    gotoBall2 = MoveToBallAction(robotManager,passer_Id)
    faceToBallcatch = FaceToBallAction(robotManager,catcher_Id)
    kick1 = KickAction(robotManager,passer_Id)
    catching = DribblerAction(robotManager,catcher_Id,1.5)

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