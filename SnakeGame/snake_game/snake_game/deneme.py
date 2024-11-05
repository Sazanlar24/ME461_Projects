#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pygame
import sys
import random
import math
import time
from tutorial_interfaces.msg import Position, Direction, Target

class Grid:
    def __init__(self, row_number, column_number):
        self.rows = row_number
        self.columns = column_number
        self.grid_size = 50
        self.screen_width = self.grid_size * self.columns
        self.screen_height = self.grid_size * self.rows
        self.grid_color = (100, 100, 100)  # Darker gray color
        self.background_color = (173, 216, 230)  # Light blue color

class Snake:
    COLORS = [
        (0, 255, 0),    # Green
        (255, 0, 0),    # Red
        (255, 255, 0),  # Yellow    
        (0, 0, 255)     # Blue
    ]
    
    def __init__(self, player_id, grid):
        self.grid = grid
        self.alive = True
        self.points = 0
        self.length = 1
        self.heading = "right"
        self.head_location = (6, 12)
        self.location_list = [self.head_location]
        self.body_color = self.COLORS[player_id]
        self.head_color = self.COLORS[player_id]
        self.wait = 0  # Initialize wait time
        self.nontouch = 0  # Initialize untouchable time

    def move(self):
        # Check if the snake is freezing
        if self.wait > 0: 
            self.wait -= 1
            return
        elif self.heading == "right":
            self.head_location = (self.head_location[0], self.head_location[1] + 1)  # Move right
        elif self.heading == "left":
            self.head_location = (self.head_location[0], self.head_location[1] - 1)  # Move left
        elif self.heading == "up":
            self.head_location = (self.head_location[0] - 1, self.head_location[1])  # Move up
        elif self.heading == "down":
            self.head_location = (self.head_location[0] + 1, self.head_location[1])  # Move down

        # Handle wrapping around the grid
        self.head_location = (
            self.head_location[0] % self.grid.rows,  # Wrap row
            self.head_location[1] % self.grid.columns  # Wrap column
        )

        # Update the location list
        self.location_list = [self.head_location] + self.location_list[:-1]

    def apply_target_change(self, point):     
        self.points += point
        self.length += point

        tail = self.location_list[-1]
        if point == 1:
            self.location_list.append(tail)  # Adds a new part to the end of the snake's body
        elif point == 3:
            self.location_list.append(tail)  # Adds a new part to the end of the snake's body
            self.location_list.append(tail)  # Adds a new part to the end of the snake's body
            self.location_list.append(tail)  # Adds a new part to the end of the snake's body
        if not len(tail) == True:
            if point == -1:
                self.location_list.pop()
        if len(tail) > 3:
            if point == -3:
                self.location_list.pop()
                self.location_list.pop()
                self.location_list.pop()

    def check_eaten_by_itself(self):
        if self.nontouch > 0: 
            self.nontouch -= 1
            return
        
        for location in self.location_list[1:]:
            if location == self.head_location:
                print("The snake ate itself")
                self.alive = False
                break

class Target:
    target_colors = {
        '+1': (128, 128, 0),  # olive
        '+3': (255, 255, 0),  # yellow
        '-1': (0, 128, 128),  # teal
        '-3': (128, 0, 128)   # purple
    }

    joker_colors = {
        'freeze': (0, 0, 255), 
        'untouchable': (255, 255, 255)
    }

    shapes = {
        "circle",
        "triangle",
        "pentagon"
    }
    
    def __init__(self, grid):
        self.grid = grid
        self.feature = None
        self.color = None
        self.row = None
        self.column = None
        self.featuret = None
        self.colort = None
        self.rowt = None
        self.columnt = None
        self.alive = False   
        self.alivet = False
        self.special_spawn_time = None
        self.food_spawn_time = None
        self.spawn_random() 
        self.spawn_joker()        

    def spawn_random(self):
        self.row    = random.randint(0, self.grid.rows - 1)
        self.column = random.randint(0, self.grid.columns - 1)
        self.feature, self.color = random.choice(list(self.target_colors.items()))
        self.alive = True
        self.shape = random.choice(list(self.shapes))
        self.food_spawn_time = time.time()

    def spawn_joker(self):
        self.rowt    = random.randint(0, self.grid.rows - 1)
        self.columnt = random.randint(0, self.grid.columns - 1)
        self.featuret, self.colort = random.choice(list(self.joker_colors.items()))
        self.alivet = True
        self.special_spawn_time = time.time()

    def check_eaten(self, snake):
        if snake.head_location == (self.row, self.column):
            self.alive = False
            if self.feature in ['+1', '-1', '+3', '-3']: 
                point = int(self.feature)
                snake.apply_target_change(point) 
            self.spawn_random()  # Spawn a new target
        if (self.rowt, self.columnt) != (self.row, self.column):
            if snake.head_location == (self.rowt, self.columnt):
                self.alivet = False
                if self.featuret == 'freeze':
                    snake.wait = 10
                elif self.featuret == 'untouchable':
                    snake.nontouch = 20
                self.spawn_joker()
        if (time.time() - self.special_spawn_time) > 5:
            self.alivet = False
        if (time.time() - self.food_spawn_time) > 10:
            self.alive = False
            
class GameEngine:
    def __init__(self, screen):
        # Initialize pygame
        pygame.init()
        self.grid = Grid(row_number=10, column_number=20)
        self.snake = Snake(0, self.grid)
        self.target = Target(self.grid)
        self.screen = screen

        self.special_spawn_time = None

    def draw_target(self):
        if self.target.alive:
            
            orta_x = self.target.column * self.grid.grid_size + self.grid.grid_size / 2
            orta_y = self.target.row * self.grid.grid_size + self.grid.grid_size / 2

            if self.target.shape == "circle":
                pygame.draw.circle(self.screen, self.target.color, (orta_x, orta_y), 20)

            elif self.target.shape == "triangle":
                point1 = (orta_x - 15, orta_y - 20)
                point2 = (orta_x - 15, orta_y + 20)
                point3 = (orta_x + 20, orta_y)
                pygame.draw.polygon(self.screen, self.target.color, [point1, point2, point3])
            
            elif self.target.shape == "pentagon":
                corners = []
                for i in range(5):
                    angle = 2 * math.pi * i / 5  # Angle in radians
                    x = orta_x + 20 * math.cos(angle)
                    y = orta_y + 20 * math.sin(angle)
                    corners.append((x, y))
                pygame.draw.polygon(self.screen, self.target.color, corners)

            self.target.check_eaten(self.snake)
        else:
            self.target.spawn_random()

        if self.target.alivet:
            pygame.draw.rect(
                self.screen, 
                self.target.colort, 
                (self.target.columnt * self.grid.grid_size, 
                 self.target.rowt * self.grid.grid_size, 
                 self.grid.grid_size, 
                 self.grid.grid_size)
            )
        else: 
            self.target.spawn_joker()

    def draw_grid(self):
        for x in range(0, self.grid.screen_width, self.grid.grid_size):
            for y in range(0, self.grid.screen_height, self.grid.grid_size):
                rect = pygame.Rect(x, y, self.grid.grid_size, self.grid.grid_size)
                pygame.draw.rect(self.screen, self.grid.grid_color, rect, 1)

    def draw_snake(self):
        if self.snake.alive:
            # Draw head
            pygame.draw.rect(
                self.screen, 
                self.snake.head_color, 
                (self.snake.location_list[0][1] * self.grid.grid_size, 
                 self.snake.location_list[0][0] * self.grid.grid_size, 
                 self.grid.grid_size, 
                 self.grid.grid_size)
            )

            # Draw body
            for location in self.snake.location_list[1:]:
                pygame.draw.rect(
                    self.screen, 
                    self.snake.body_color, 
                    (location[1] * self.grid.grid_size, 
                     location[0] * self.grid.grid_size, 
                     self.grid.grid_size, 
                     self.grid.grid_size)
                )

class PlayerNode(Node):
    def __init__(self, player_id, screen):
        super().__init__(f'player_{player_id}_node')
        self.player_id = player_id
        self.engine = GameEngine(screen)
        
        # Publisher and Subscriber setup
        self.position_pub = self.create_publisher(Position, f'/player_{player_id}_position', 10)
        self.direction_pub = self.create_publisher(Direction, f'/player_{player_id}_direction', 10)
        
        self.position_sub = self.create_subscription(Position, f'/player_{3 - player_id}_position', self.update_opponent_position, 10)
        self.direction_sub = self.create_subscription(Direction, f'/player_{3 - player_id}_direction', self.update_opponent_direction, 10)
        
        # Game over flag
        self.game_over = False
        
        # Timer for game updates
        self.create_timer(0.1, self.game_loop)

    def game_loop(self):
        if self.game_over:
            return  # Stop executing if game is over

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP and self.engine.snake.heading != "down":
                    self.engine.snake.heading = "up"
                elif event.key == pygame.K_DOWN and self.engine.snake.heading != "up":
                    self.engine.snake.heading = "down"
                elif event.key == pygame.K_LEFT and self.engine.snake.heading != "right":
                    self.engine.snake.heading = "left"
                elif event.key == pygame.K_RIGHT and self.engine.snake.heading != "left":
                    self.engine.snake.heading = "right"

        self.engine.snake.move()
        self.engine.snake.check_eaten_by_itself()

        # Check if snake is alive
        if not self.engine.snake.alive:
            print("Game Over: Snake is dead")  # Debugging game over
            self.game_over = True

        # Update the screen after processing input
        self.update_screen()  # Ensure the screen is updated before checking alive status

        if self.engine.snake.alive:
            self.publish_positions()
            self.publish_direction()

    def publish_positions(self):
        position_msg = Position()
        position_msg.row, position_msg.column = self.engine.snake.head_location
        self.position_pub.publish(position_msg)

    def publish_direction(self):
        direction_msg = Direction()
        direction_msg.heading = self.engine.snake.heading
        self.direction_pub.publish(direction_msg)

    def update_opponent_position(self, msg):
        self.engine.snake.head_location = (msg.row, msg.column)
        # Update the opponent's snake body if needed (not implemented here)

    def update_opponent_direction(self, msg):
        # Update opponent direction if needed (not implemented here)
        pass

    def update_screen(self):
        self.engine.screen.fill(self.engine.grid.background_color)
        self.engine.draw_grid()
        self.engine.draw_snake()
        self.engine.draw_target()
        pygame.display.flip()

class GameNode(Node):
    def __init__(self):
        super().__init__("game_node")
        
        # Create the Pygame window here
        screen = pygame.display.set_mode((1000, 500))  # Example size
        pygame.display.set_caption("Snake Game")
        
        self.player1 = PlayerNode(1, screen)
        self.player2 = PlayerNode(2, screen)

def main(args=None):
    rclpy.init(args=args)
    
    screen = pygame.display.set_mode((1000, 500))
    
    player1_node = PlayerNode(player_id=1, screen=screen)
    player2_node = PlayerNode(player_id=2, screen=screen)

    try:
        rclpy.spin(player1_node)  # Spin for player 1
        rclpy.spin(player2_node)  # Spin for player 2
    except KeyboardInterrupt:
        pass
    finally:
        player1_node.destroy_node()
        player2_node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
