import pygame, sys, time
from game_engine import Engine

food_position  = (2, 5)  # row=2, column=5
seconds = 0

def main():
    global seconds
    gameEngine = Engine()

    gameEngine.gameGrid.grid_cells[food_position[0]][food_position[1]] = 1  # Mark food in the grid (1 means food)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Fill the background
        gameEngine.screen.fill(gameEngine.gameGrid.background_color)

        # Draw the grid
        gameEngine.draw_grid()

        # Draw food
        gameEngine.draw_food(food_position[0],  food_position[1])

        # Update the display
        pygame.display.flip()

        time.sleep(1)
        seconds += 1
        print(f"Seconds: {seconds}")
        
if __name__ == "__main__":
    main()