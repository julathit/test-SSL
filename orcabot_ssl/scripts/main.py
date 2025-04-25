#! /usr/bin/env python3
from utils.orcabot import *

enable_debug()

# RobotBlackBoard.printAllInfo()

def main():
    # root: Sequence = Sequence(name = "go to center of zone", memory=True)

    # moveacs = []
    # for robot in RobotBlackBoard.getRobotList("blue"):
    #     zone = ZoneManager.getZoneFromRole(robot.getRole())
    #     position = ZoneManager.getCenterOfZone(zone)
    #     moveacs.append(MoveToPointAction(robot.id, position))

    # root.add_children(moveacs)

    # pre_root: Parallel = Parallel(name = "B Central Behavior", policy = py_trees.common.ParallelPolicy.SuccessOnOne())

    from CentralBT import FielderDefenderBT, CentralBT

    # print(ZoneManager.opponent_zone_position["goal"])
    # print(ZoneManager.getNearestPointOnGoalEntrance(RobotBlackBoard.getBallPosition(), 20))

    # pre_root.add_children([
    #     # FielderDefenderBT(RobotBlackBoard.getRobot("blue", 0)),
    #     # FielderDefenderBT(RobotBlackBoard.getRobot("blue", 1)),
    #     # FielderDefenderBT(RobotBlackBoard.getRobot("blue", 2)),
    #     # FielderDefenderBT(RobotBlackBoard.getRobot("blue", 4)),
    #     # FielderDefenderBT(RobotBlackBoard.getRobot("blue", 5)),

    #     # MoveToPointAction(RobotBlackBoard.getRobot("blue", 0).id,
    #     #                 ZoneManager.getNearestPointOnGoalEntrance(RobotBlackBoard.getBallPosition(), 20)
    #     #                 ),
    # ])

    # root: Sequence = Sequence(name = "test walk and kick", memory = True, children = [pre_root])

    tree0 = py_trees.trees.BehaviourTree(CentralBT(RobotBlackBoard.getRobot("blue", 0)))
    tree1 = py_trees.trees.BehaviourTree(CentralBT(RobotBlackBoard.getRobot("blue", 1)))
    tree2 = py_trees.trees.BehaviourTree(FielderDefenderBT(RobotBlackBoard.getRobot("blue", 2)))
    tree4 = py_trees.trees.BehaviourTree(FielderDefenderBT(RobotBlackBoard.getRobot("blue", 4)))
    # tree5 = py_trees.trees.BehaviourTree(FielderDefenderBT(RobotBlackBoard.getRobot("blue", 5)))

    while True:
        updateState()
        tree0.tick()
        tree1.tick()
        tree2.tick()
        tree4.tick()
        # updateRole()
    # else:
    #     print("Success.")

    # import time
    # while 1:
    #     updateState()
    #     print(RobotBlackBoard.getBallPossession())
    #     time.sleep(1)


def updateRole():
    for robot in RobotBlackBoard.getRobotList("blue"):
        print(robot.getRole())
        pass