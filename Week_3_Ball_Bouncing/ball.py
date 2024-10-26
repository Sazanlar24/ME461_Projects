import math
import numpy as np
import pyautogui

HEIGHT_MAX = 1060
WIDTH_MAX = 1900

class Ball:
    def __init__(self):

        self.initial_x = 950
        self.initial_y = 100

        self.pos_x = self.initial_x
        self.pos_y = self.initial_y

        self.velocity_x = 0
        self.velocity_y = 250

        self.radius = int(1900 / 40)

        self.points = [(self.pos_x + self.radius*math.cos(angle), self.pos_y + self.radius*math.sin(angle)) for angle in np.linspace(0, 2*math.pi, 100)]

    def calculateNewPos(self, time_step):

        screen_width, screen_height = pyautogui.size()
        
        self.pos_x += self.velocity_x * time_step
        self.pos_y += self.velocity_y * time_step
        self.radius = int(1900 / 40)

        self.pos_x = max(min(self.pos_x, screen_width - self.radius), self.radius)
        self.pos_y = max(min(self.pos_y, screen_height - self.radius), self.radius)

        
