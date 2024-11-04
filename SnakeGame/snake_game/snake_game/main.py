#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pygame, sys, time
from game_engine import Engine

food_position  = (1, 1)  # row=2, column=5
seconds = 0

class GameNode(Node):

    def __init__(self):
       super().__init__("first_player")

    def SnakeGame():
        global seconds
        gameEngine = Engine()

        gameEngine.gameGrid.grid_cells[food_position[0]][food_position[1]] = 1  # Mark food in the grid (1 means food)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    # Change snake heading based on arrow key pressed
                    if event.key == pygame.K_UP:
                        if gameEngine.snake.heading != "down":
                            gameEngine.snake.heading = "up"  
                    elif event.key == pygame.K_DOWN:
                        if gameEngine.snake.heading != "up":
                            gameEngine.snake.heading = "down"  
                    elif event.key == pygame.K_LEFT:
                        if gameEngine.snake.heading != "right":
                            gameEngine.snake.heading = "left"  
                    elif event.key == pygame.K_RIGHT:
                        if gameEngine.snake.heading != "left":
                            gameEngine.snake.heading = "right"

            # Fill the background
            gameEngine.screen.fill(gameEngine.gameGrid.background_color)

            
            gameEngine.draw_grid()
            gameEngine.snake.move()
            gameEngine.snake.checkEatenByItself()
            gameEngine.draw_target()
            gameEngine.draw_snake()        

            # Update the display
            pygame.display.flip()

            time.sleep(0.1) # 2 hz

def main(args=None):
    rclpy.init(args=args)
    game = GameNode()
    rclpy.spin(game)
    rclpy.shutdown

        
if __name__ == "__main__":
    main()
