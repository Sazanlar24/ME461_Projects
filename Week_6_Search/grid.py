import pygame
import random
from object import Object

class Grid:

    def __init__(self, nWidth, nHeight, nObjects):
        # Initialize pygame
        pygame.init()

        # Size of a cell
        self.CELL_SIZE = 50

        # Screen dimensions
        self.WIDTH = nWidth * self.CELL_SIZE
        self.HEIGHT = nHeight * self.CELL_SIZE

        self.object_number = nObjects
        self.object_list = []

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GRAY = (200, 200, 200)
        self.BLUE = (0, 0, 255)

        self.start_square  = None
        self.target_square = None

        # Initialize the screen
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Search Application")

        # Create a 2D list to track selected cells
        self.grid = [["empty" for _ in range(nWidth)] for _ in range(nHeight)]
        self.create_objects()

        self.ready_for_search = False

    def draw_grid(self):
        # Draw the grid with colored squares if they are selected
        for row in range(len(self.grid)):  # Loop over the rows
            for column in range(len(self.grid[row])):  # Loop over the columns
                rect = pygame.Rect(column * self.CELL_SIZE, row * self.CELL_SIZE, self.CELL_SIZE, self.CELL_SIZE)

                cell_content = self.grid[row][column]
                if cell_content == "empty":
                    color = self.WHITE
                elif cell_content == "start" or cell_content == "target":
                    color = self.BLUE
                else:
                    color = self.WHITE

                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, self.GRAY, rect, 1)  # Border of the cell

    def get_grid_pos(self, mouse_pos):
        # Convert mouse position to grid coordinates
        x, y = mouse_pos
        row = y // self.CELL_SIZE
        col = x // self.CELL_SIZE
        return row, col
    
    def create_objects(self):

        total_cells = len(self.grid) * len(self.grid[0])
        if self.object_number > total_cells:
            raise ValueError("Object number is higher than the total number of cells in the grid.")
        
        selected_positions = set()

        while len(selected_positions) < self.object_number:
            row_id = random.randint(0, len(self.grid)-1)
            col_id = random.randint(0, len(self.grid[0])-1)
            
            # If the position is not already selected, change its value to 1
            if (row_id, col_id) not in selected_positions:
        
                selected_positions.add((row_id, col_id))

                newObj = Object(row_id, col_id)
                self.grid[row_id][col_id] = newObj

                self.object_list.append(newObj)
    
    def draw_objects(self):

        for object in self.object_list:

            if object.shape == "circle":
                pygame.draw.circle(self.screen, object.color, object.orta_coord, 20)

            elif object.shape == "triangle":
                pygame.draw.polygon(self.screen, object.color, object.triangle_coord)
            
            elif object.shape == "pentagon":
                pygame.draw.polygon(self.screen, object.color, object.pentagon_coord)

    def event_handler(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle mouse click            

                if self.start_square == None:
                    mouse_pos = pygame.mouse.get_pos()
                    row, col = self.get_grid_pos(mouse_pos)

                    # Toggle the selected state of the cell
                    if 0 <= row < self.HEIGHT and 0 <= col < self.WIDTH and self.grid[row][col] == "empty":
                        self.grid[row][col] = "start"
                        self.start_square = (row, col)
                    else:
                        print("you cannot choose there")

                elif self.target_square == None:

                    mouse_pos = pygame.mouse.get_pos()
                    row, col = self.get_grid_pos(mouse_pos)

                    # Toggle the selected state of the cell
                    if 0 <= row < self.HEIGHT and 0 <= col < self.WIDTH and self.grid[row][col] == "empty":
                        #self.grid[row][col] = not self.grid[row][col]
                        self.grid[row][col] = "target"
                        self.target_square = (row, col)
                        self.ready_for_search = True
                    else:
                        print("you cannot choose there")

                else:
                    print(f"Start: {self.start_square}, Target: {self.target_square}")
                    print("you cannot choose extra points")
                    return True