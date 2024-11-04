import random

target_colors = {'+1':(128, 128, 0), '+3':(255, 255, 0), 
                 '-1':(0, 128, 128), '-3':(0, 255, 255),
                 'freeze':(0, 0, 255), 'untouchable':(255, 255, 255)
                 }

class Target:
    def __init__(self, grid):
        self.grid_inside = grid
        self.row    = None
        self.column = None
        self.feature = None
        self.color = None
        self.alive = False   
        self.spawn_random()        

    def spawn_random(self):
        self.row    = random.randint(0, self.grid_inside.rows - 1)
        self.column = random.randint(0, self.grid_inside.columns - 1)
        self.feature, self.color = random.choice(list(target_colors.items()))
        self.alive = True

    def check_eaten(self, snake):
        if snake.location_list[0] == (self.row, self.column):
            self.alive = False
            # Apply features of target to the snake
            if self.feature in ['+1', '-1','+3','-3']: 
                self.point = int(self.feature)
                snake.applyTargetChange(self.point) 
            elif self.feature == 'freeze':
                snake.wait = 5
            elif self.feature == 'untouchable':
                snake.nontouch = 10
