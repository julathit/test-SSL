#! /usr/bin/env python3
import py_trees

class colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    WHITE = '\033[38;5;30m'
    MYTH = '\033[38;5;30m'

class Position(object):
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y

    def __str__(self):
        return str("Position: ({}, {})".format(self.x, self.y))

class Robot(object):
    def __init__(self, id: int, position: Position):
        self.id: int = id
        self.position: Position = position
        self.orientationL: int = 0

    def __str__(self):
        return str("Robot: {}, {}".format(self.id, self.position))

    def patch(self, x, y, ori):
        self.position = Position(x, y)
        self.orientation = ori

class RobotList(list):
    def __init__(self, size: int):
        super(RobotList, self).__init__()
        self.size = size
        for i in range(size):
            self.append(Robot(i, Position(0, 0)))

    def __str__(self):
        ret_str = ""
        for robot in self:
            ret_str += colors.MYTH + " robotID: {}".format(robot.id) + "\n" + colors.OKBLUE + "\t- Position: {{{}}}\n".format(robot.position)
        return ret_str

    def patch(self, newdatalist):
        for i in range(self.size):
            self[i].patch(newdatalist[i].x, newdatalist[i].y, newdatalist[i].orientation)

# class RobotDictEx(dict):
#     def __str__(self):
#         ret_str = ""
#         for key in self.keys():
#             ret_str += colors.MYTH + " robotID: {}".format(key) + "\n" + colors.OKBLUE + "\t- value: {{{}}}\n".format(self[key])
#         return ret_str

class RobotDict(dict):
    # def __init__(self):
    #     super(RobotDict, self).__init__()
    #     self.__dict__: RobotDictEx = RobotDictEx()

    def __str__(self):
        ret_str = ""
        for key in self.keys():
            ret_str += colors.OKBLUE + " team: " + colors.OKGREEN + key + colors.OKBLUE + ", number of robots: " + colors.OKGREEN + "{}\n".format(str(len(self[key])))
        return ret_str


writer = py_trees.blackboard.Client(name="Writer")
writer.register_key(key="robots", access=py_trees.common.Access.WRITE)
reader = py_trees.blackboard.Client(name="Reader")
reader.register_key(key="robots", access=py_trees.common.Access.READ)

writer.robots = RobotDict()
writer.robots['yellow'] = RobotList(6)
writer.robots['blue'] = RobotList(6)

print(writer)
print(writer.robots)
print(writer.robots.keys())
print(writer.robots["yellow"])
print(writer.robots["yellow"][0])


# from geometry_msgs.msg import *
# from gazebo_msgs.msg import *
# from grsim_ros_bridge_msgs.msg import *
# from krssg_ssl_msgs.msg import *

# import numpy as np
# import rospy

# _robotDict: dict = {i: SSL_DetectionRobot() for i in range(6)} # type: ignore

# def __updateRobotsData(data):
#     for i in range(0, len(data.robots_blue)):
#         id_robots = data.robots_blue[i].robot_id
#         for j in range(6):
#             if id_robots == j:
#                 global _robotDict
#                 _robotDict[j] = data.robots_blue[i]

# sub = rospy.Subscriber("/vision", SSL_DetectionFrame, __updateRobotsData) # type: ignore
# rospy.init_node("detect", anonymous=False)

# # Sample data for robots
# team1_data = [
#     {"id": 1, "x": 100, "y": 200, "orientation": 90},
#     {"id": 2, "x": 150, "y": 250, "orientation": 180},
#     {"id": 3, "x": 200, "y": 300, "orientation": 45},
#     {"id": 4, "x": 250, "y": 350, "orientation": 60},
#     {"id": 5, "x": 300, "y": 400, "orientation": 30},
#     {"id": 6, "x": 350, "y": 450, "orientation": 120},
# ]

# team2_data = [
#     {"id": 1, "x": 400, "y": 200, "orientation": 270},
#     {"id": 2, "x": 450, "y": 250, "orientation": 90},
#     {"id": 3, "x": 500, "y": 300, "orientation": 135},
#     {"id": 4, "x": 550, "y": 350, "orientation": 210},
#     {"id": 5, "x": 600, "y": 400, "orientation": 300},
#     {"id": 6, "x": 650, "y": 450, "orientation": 15},
# ]

# import pygame

# # Initialize Pygame
# pygame.init()

# # Screen dimensions
# screen_width = 800
# screen_height = 600

# # Colors
# white = (255, 255, 255)
# black = (0, 0, 0)
# blue = (0, 0, 255)
# red = (255, 0, 0)

# # Set up the screen
# screen = pygame.display.set_mode((screen_width, screen_height))
# pygame.display.set_caption("Robot Positions and Orientations")

# # Font for displaying text
# font = pygame.font.Font(None, 36)

# # Main loop
# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     # Clear the screen
#     screen.fill(white)

#     # Display team 1 data
#     y_offset = 10
#     text = font.render("Team 1 (Blue):", True, blue)
#     screen.blit(text, (10, y_offset))
#     y_offset += 30

#     for robot in team1_data:
#         text = font.render(
#             f"Robot {robot['id']}: Position ({robot['x']}, {robot['y']}), Orientation {robot['orientation']}°",
#             True,
#             blue,
#         )
#         screen.blit(text, (10, y_offset))
#         y_offset += 30

#     # Display team 2 data
#     y_offset += 20
#     text = font.render("Team 2 (Red):", True, red)
#     screen.blit(text, (10, y_offset))
#     y_offset += 30

#     for robot in team2_data:
#         text = font.render(
#             f"Robot {robot['id']}: Position ({robot['x']}, {robot['y']}), Orientation {robot['orientation']}°",
#             True,
#             red,
#         )
#         screen.blit(text, (10, y_offset))
#         y_offset += 30

#     # Update the display
#     pygame.display.flip()

# # Quit Pygame
# pygame.quit()