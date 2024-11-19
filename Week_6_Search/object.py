from math import pi, cos, sin
import random

class Object:

    SHAPES = ["circle", "triangle", "pentagon"]
    COLORS = [(128, 0, 128), (0, 128, 0), (255, 0, 0)]

    def __init__(self, nRow, nHeight):
        self.row = nRow
        self.column = nHeight

        self.GRID_SIZE = 50
            
        self.orta_x = self.column * self.GRID_SIZE + self.GRID_SIZE / 2
        self.orta_y = self.row * self.GRID_SIZE + self.GRID_SIZE / 2
        self.orta_coord = (self.orta_x, self.orta_y)

        self.shape = random.choice(self.SHAPES)
        self.color = random.choice(self.COLORS)

        self.triangle_coord  = []
        self.pentagon_coord  = []
        if self.shape == "triangle":
            self.set_triangle_points()
        elif self.shape == "pentagon":
            self.set_pentagon_points()

    def set_triangle_points(self):

        point1 = (self.orta_x - self.GRID_SIZE/10*4, self.orta_y + self.GRID_SIZE/10*4)
        point2 = (self.orta_x + self.GRID_SIZE/10*4, self.orta_y + self.GRID_SIZE/10*4)
        point3 = (self.orta_x, self.orta_y - self.GRID_SIZE/10*4)

        self.triangle_coord = [point1, point2, point3]

    def set_pentagon_points(self):

        for i in range(5):
            angle = 2 * pi * i / 5  # Angle in radians
            x = self.orta_x + self.GRID_SIZE/10*4 * cos(angle)
            y = self.orta_y + self.GRID_SIZE/10*4 * sin(angle)      
            self.pentagon_coord.append((x, y))