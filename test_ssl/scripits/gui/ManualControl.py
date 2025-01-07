#!/usr/bin/env python3

import pygame
import sys

from component.Robot import Robot
from component.RobotManager import RobotManager

class ManualControl:
    def __init__(self, robotManager: RobotManager, robot_ID: int):
        self.robot: Robot = robotManager.getRobotByID(robot_ID)
        self.debug = False

        #pygame initialize
        pygame.init()

        # Set up the display
        width, height = 110, 5

        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Keyboard Input")

        # Set up the clock
        self.clock = pygame.time.Clock()

        # Variables to track if a key is currently pressed
        self.keys_pressed = set()

        self.moveSpeed = 2
        self.rotationalSpeed = 5

    def update(self):
        self.isDebugMode()
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                # Check if the key is not already being tracked
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                elif event.key not in self.keys_pressed:
                    self.keys_pressed.add(event.key)
                if event.key == pygame.K_SPACE:
                    self.robot.sendCommand(0,0,0,True)
            elif event.type == pygame.KEYUP:
                # Check if the released key was being tracked
                if event.key in self.keys_pressed:
                    self.keys_pressed.remove(event.key)
                    self.robot.sendCommand(0,0,0,False)

        # Check if the 'W' key is currently pressed
        if pygame.K_w in self.keys_pressed:
            self.robot.sendCommand(self.moveSpeed, 0, 0, False)
        if pygame.K_a in self.keys_pressed:
            self.robot.sendCommand(0, self.moveSpeed, 0, False)
        if pygame.K_s in self.keys_pressed:
            self.robot.sendCommand(-self.moveSpeed, 0, 0, False)
        if pygame.K_d in self.keys_pressed:
            self.robot.sendCommand(0, -self.moveSpeed, 0, False)
        if pygame.K_k in self.keys_pressed:
            self.robot.sendCommand(0, 0, self.rotationalSpeed, False)
        if pygame.K_l in self.keys_pressed:
            self.robot.sendCommand(0, 0, -self.rotationalSpeed, False)
        if len(self.keys_pressed) == 0:
            self.robot.sendCommand(0, 0, 0, False)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        self.clock.tick(60)

    def isDebugMode(self):
        if self.debug:
            print(self.robot.getPosition())
