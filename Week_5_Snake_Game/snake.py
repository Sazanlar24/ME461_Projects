COLORS = [
    (0, 255, 0),    # Greens
    (255, 0, 0),    # Red
    (255, 255, 0),  # Yellow    
    (0, 0, 255)     # Blue
]

class Snake:
    def __init__(self, player_index, grid):
        self.alive = True
        self.number = player_index
        self.points = 0
        
        self.heading = "down"
        self.head_location = (6, 12)
        
        self.location_list = [self.head_location]
        self.length = len(self.location_list)
        
        self.head_color = (0, 0, 0) # black
        self.body_color = COLORS[player_index]

        self.grid_inside = grid
        
    def move(self):
        if self.heading == "right":
            self.head_location = (self.head_location[0], self.head_location[1] + 1) # column is increased by 1
        elif self.heading == "left":
            self.head_location = (self.head_location[0], self.head_location[1] - 1) # column is decreased by 1
        elif self.heading == "up":
            self.head_location = (self.head_location[0] - 1, self.head_location[1]) # row is decreased by 1
        elif self.heading == "down":
            self.head_location = (self.head_location[0] + 1, self.head_location[1]) # row is increased by 1

        # sol taraftan çıkan yılan sağ tarafta devam ediyor
        if self.head_location[1] < 0:
            self.head_location = (self.head_location[0], self.grid_inside.columns - 1)
        # sağ -> sol
        elif self.head_location[1] > self.grid_inside.columns - 1:
            self.head_location = (self.head_location[0], 0)

        # alt -> üst
        if self.head_location[0] < 0:
            self.head_location = (self.grid_inside.rows - 1, self.head_location[1])
        # üst -> alt
        elif self.head_location[0] > self.grid_inside.rows - 1:
            self.head_location = (0, self.head_location[1])

        # yeni lokasyon listesi
        self.location_list = [self.head_location] + self.location_list[:len(self.location_list) - 1]    

    def applyTargetChange(self):
        self.points += 1    # yenen targeta göre değişebilir
        self.length += 1

        tail = self.location_list[-1]
        self.location_list.append(tail)  # Adds a new part to the end of the snake's body
        # uzunluğu nasıl artacak

    def die(self):
        self.alive = False

    def checkEatenByItself(self):

        for location in self.location_list[1:]:
            if location == self.head_location:
                print("kendini yedi")
                self.alive = False
                break