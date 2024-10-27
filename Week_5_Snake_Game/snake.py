class Snake:
    def __init__(self, player_index):
        self.alive = True
        self.number = player_index
        self.points = 0
        self.head_location = (0, 0)
        self.location = [self.head_location]
        
    def move():
        pass

    def die(self):
        self.alive = False