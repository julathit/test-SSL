import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *

ball = Pose()

p_ball = (0,0)



def recibir_datos(data):
    global ball
    ball = data.balls

sub = rospy.Subscriber("/vision", SSL_DetectionFrame, recibir_datos)

def save_ball():

    global p_ball

    try:
        p_ball = ((ball[0].x), (ball[0].y))
        # print('return')
        return (p_ball)

    except:
        # print('except')
        pass

class my_ball:
    def ballPosition(self) -> tuple:
        save_ball()
        return p_ball    
        