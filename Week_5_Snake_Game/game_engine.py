import pygame
import sys

from grid import Grid
from snake import Snake

class Engine():
    def __init__(self):
        
        # Initialize pygame
        pygame.init()

        self.gameGrid = Grid(row_number=10, column_number=20)
        self.snake = Snake(0, self.gameGrid)

        # Create the window
        self.screen = pygame.display.set_mode((self.gameGrid.screen_width, self.gameGrid.screen_height))
        pygame.display.set_caption("Snake Game Grid")
        
    def draw_food(self, row, column):
        red_color = (255, 0, 0)
        pygame.draw.rect(
            self.screen, 
            red_color, 
            (column * self.gameGrid.grid_size, row * self.gameGrid.grid_size, self.gameGrid.grid_size, self.gameGrid.grid_size)
            )

    def draw_grid(self):
        for x in range(0, self.gameGrid.screen_width, self.gameGrid.grid_size):
            for y in range(0, self.gameGrid.screen_height, self.gameGrid.grid_size):
                rect = pygame.Rect(x, y, self.gameGrid.grid_size, self.gameGrid.grid_size)
                pygame.draw.rect(self.screen, self.gameGrid.grid_color, rect, 1)

    def draw_snake(self):
        
        # HEAD
        pygame.draw.rect(
            self.screen, 
            self.snake.head_color, 
            (self.snake.location_list[0][1] * self.gameGrid.grid_size, 
             self.snake.location_list[0][0] * self.gameGrid.grid_size, 
             self.gameGrid.grid_size, 
             self.gameGrid.grid_size)
            )

        # BODY
        for location in self.snake.location_list[1:]:
            pygame.draw.rect(
                self.screen, 
                self.snake.body_color, 
                (location[1] * self.gameGrid.grid_size, 
                 location[0] * self.gameGrid.grid_size, 
                 self.gameGrid.grid_size, 
                 self.gameGrid.grid_size)
                )