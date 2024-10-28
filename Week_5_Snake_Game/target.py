import random

class Target:
    def __init__(self, grid):
        self.grid_inside = grid
        self.row    = None
        self.column = None
        self.alive = True   
        self.spawn_random()        

    def spawn_random(self):
        self.row    = random.randint(0, self.grid_inside.rows - 1)
        self.column = random.randint(0, self.grid_inside.columns - 1)
        self.alive = True

    def check_eaten(self, snake):
        if snake.location_list[0] == (self.row, self.column):
            self.alive = False
            snake.applyTargetChange() # özel özelliği varsa buna parametre olarak verilebilir
