from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

import rospy
import time
import tomli
import signal
import sys

from utils.blackboard import RobotBlackBoard

class Initializer():
    @staticmethod
    def initConfig() -> bool:
        print("[initConfig] initializing configuration file")
        try:
            with open("./src/test-SSL/orcabot_ssl/scripts/config/match.toml", mode="rb") as file:
                config = tomli.load(file)
                RobotBlackBoard.setConfig(config)
                print("[initConfig] Success!")
                return True
        except FileNotFoundError:
            print("[initConfig] config file not found? template file creation will be implemented soon")

    @staticmethod
    def initSubscriber() -> bool:
        print("[initSubscriber] initializing rospy subscriber")
        _robotDict_blue: dict = {i: SSL_DetectionRobot() for i in range(6)} # type: ignore
        _robotDict_yellow: dict = {i: SSL_DetectionRobot() for i in range(6)} # type: ignore
        Initializer.sub_data_received = False

        def __updateRobotsData(data):
            for i in range(0, len(data.robots_blue)):
                id_robots = data.robots_blue[i].robot_id
                for j in range(6):
                    if id_robots == j:
                        _robotDict_blue[j] = data.robots_blue[i]

            RobotBlackBoard.patchRobotData("blue", _robotDict_blue)

            for i in range(0, len(data.robots_yellow)):
                id_robots = data.robots_yellow[i].robot_id
                for j in range(6):
                    if id_robots == j:
                        _robotDict_yellow[j] = data.robots_yellow[i]

            RobotBlackBoard.patchRobotData("yellow", _robotDict_yellow)
            try:
                RobotBlackBoard.patchBallData(data.balls[0])
            except: pass

            Initializer.sub_data_received = True

        sub = rospy.Subscriber("/vision", SSL_DetectionFrame, __updateRobotsData) # type: ignore
        rospy.init_node("detect", anonymous=False)

        retry = True
        timeout = 3
        time.sleep(0.1)
        while retry and timeout > 0:
            if Initializer.sub_data_received:
                print("[initSubscriber] Success!")
                return True
            else:
                print("[initSubscriber] retry in 2 second")
                timeout -= 1
                time.sleep(2)
        print("[initSubscriber] falied. Please check if your simulation is running")
        return False

    @staticmethod
    def initZoneManager() -> bool:
        print("[initZoneManager] initializing zone manager")
        try:
            from component.area import ZoneManager
            print("[initZoneManager] Success!")
            return True
        except Exception as e:
            print(f"[initZoneManager] failed: {e}")
            return False

    @staticmethod
    def init_all() -> bool:
        print("[Initializer] initializing all components")

        def handler(signum, frame):
            print("\n[!] Interrupt received, stopping...")
            sys.exit(0)

        signal.signal(signal.SIGINT, handler)

        if not Initializer.initConfig():
            return False
        if not Initializer.initSubscriber():
            return False
        if not Initializer.initZoneManager():
            return False
        print("[Initializer] All initializations completed successfully!")

        return True