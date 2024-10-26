from math import sqrt
import random

class Target:

    def __init__(self):
        
        self.pos_x = int(1900/2/10/2)-50
        self.pos_y = int(1900/2/10)
        self.radius = int(1900/2/10)  # half of the width of the screen divided by 10

        self.allPossibleLocations = [(int(1900/2/10/2)-50, self.radius), (int(1900/2/10/2)-50, self.radius + 200), (int(1900/2/10/2)-50, self.radius + 400), 
                                  (1900-int(1900/2/10/2)+50, self.radius), (1900-int(1900/2/10/2)+50, self.radius+200), (1900-int(1900/2/10/2)+50, self.radius+400)]

        self.score = 0

    def checkCollision(self, ball_x, ball_y, ball_radius):
        dist = sqrt((ball_x - self.pos_x)**2 + (ball_y - self.pos_y)**2)
        
        if dist <= self.radius + ball_radius:
            self.changePlace(self.pos_x, self.pos_y)

    def changePlace(self, old_x, old_y):

        possible_locations = self.allPossibleLocations.copy()
        possible_locations.remove((old_x, old_y))
        new_pos = random.choice(possible_locations)
        self.pos_x = new_pos[0]
        self.pos_y = new_pos[1]

        self.increaseScore()

    def increaseScore(self):
        self.score += 1