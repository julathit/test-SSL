import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

class Ball:
    def __init__(self) -> None:

        def __updateBallData(data):
            self.ball = data.balls

        self.sub = rospy.Subscriber("/vision", SSL_DetectionFrame, __updateBallData ) # type: ignore
        self.ball = Pose()
        self.position = (1000,0)

    def getPosition(self) -> tuple:
        try:
            self.position = (self.ball[0].x, self.ball[0].y)
            # print('return')
        except:
            # print('except')
            pass
        return self.position
