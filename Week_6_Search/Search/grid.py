import pygame
import random
from object import Object
from collections import deque
import heapq

class Grid:

    def __init__(self, nWidth, nHeight, nObjects):
        # Initialize pygame
        pygame.init()

        # Size of a cell
        self.CELL_SIZE = 50

        # Screen dimensions (reserve space on the left for the UI)  
        self.UI_WIDTH = 300  # Width of the UI panel
        self.WIDTH = nWidth * self.CELL_SIZE + self.UI_WIDTH  # Total width
        self.HEIGHT = nHeight * self.CELL_SIZE  # Height remains the same

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

        # Show the menu at the start
        self.choice = self.display_menu()

    def display_menu(self):
        """
        Display a menu to allow the user to choose a search algorithm.
        This menu will always be on the right side of the screen.
        """
        font = pygame.font.Font(None, 36)
        options = ["Depth-First Search", "Breadth-First Search", "Uniform-Cost Search",
                "Greedy Search", "A* Search"]

        selected = 0  # Initially select the first option

        while True:
            # Draw the UI background on the right
            pygame.draw.rect(self.screen, self.BLACK, (self.WIDTH - 300, 0, 300, self.HEIGHT))

            # Render menu options on the right side of the screen
            for i, option in enumerate(options):
                # Highlight the selected option in blue, others in white
                color = self.BLUE if i == selected else self.WHITE
                text = font.render(option, True, color)
                self.screen.blit(text, (self.WIDTH - 250, self.HEIGHT // 2 - len(options) * 20 + i * 40))

            # Handle menu input
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        selected = (selected - 1) % len(options)  # Move up in the list
                    elif event.key == pygame.K_DOWN:
                        selected = (selected + 1) % len(options)  # Move down in the list
                    elif event.key == pygame.K_RETURN:  # Confirm selection
                        return options[selected]

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_pos = pygame.mouse.get_pos()
                    for i, option in enumerate(options):
                        # Check if a menu option is clicked
                        text_rect = pygame.Rect(
                            self.WIDTH - 250,
                            self.HEIGHT // 2 - len(options) * 20 + i * 40,
                            200, 30
                        )
                        if text_rect.collidepoint(mouse_pos):
                            return options[i]
                        
            
            self.draw_grid()
            self.draw_objects()
            pygame.display.flip()
        
    def bfs(self, start, target):
        """
        Perform Breadth-First Search (BFS) to find a path from start to target.
        :param start: Tuple (row, col) starting position
        :param target: Tuple (row, col) target position
        """
        rows, cols = len(self.grid), len(self.grid[0])
        queue = deque([start])
        visited = set()
        parent = {} # To reconstruct the solution path

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Up, Down, Left, Right

        while queue:
            current = queue.popleft()  # Pop the first element (FIFO)

            if current in visited:
                continue
            visited.add(current)

            # If target is reached
            if current == target:
                print("Target reached!")
                self.mark_path(self.reconstruct_path(parent, start, target))
                return True

            # Explore neighbors
            x, y = current
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                    # Check if the cell is walkable
                    if self.grid[nx][ny] == "empty" or self.grid[nx][ny] == "target":
                        queue.append((nx, ny))
                        parent[(nx, ny)] = (x, y)

            # Visualization: mark the current cell as visited
            if self.grid[x][y] != "start" and self.grid[x][y] != "target":
                self.grid[x][y] = "visited"

            self.draw_grid()
            self.draw_objects()
            pygame.display.flip()

            # Add a small delay to visualize the process
            pygame.time.delay(100)
        print("No path found.")
        return

    def dfs(self, start, target):
        """
        Perform Depth-First Search (DFS) to find a path from start to target.
        :param start: Tuple (row, col) starting position
        :param target: Tuple (row, col) target position
        """
        rows, cols = len(self.grid), len(self.grid[0])
        stack = [start]
        visited = set()
        parent = {} # To reconstruct the solution path

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)

            # If target is reached
            if current == target:
                print("Target reached!")
                self.mark_path(self.reconstruct_path(parent, start, target))
                return True

            # Explore neighbors
            x, y = current
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                    # Check if the cell is walkable
                    if self.grid[nx][ny] == "empty" or self.grid[nx][ny] == "target":
                        stack.append((nx, ny))
                        parent[(nx, ny)] = (x, y)

            # Visualization: mark the current cell as visited
            if self.grid[x][y] != "start" and self.grid[x][y] != "target":
                self.grid[x][y] = "visited"

            self.draw_grid()
            self.draw_objects()
            pygame.display.flip()

            # Add a small delay to visualize the process
            pygame.time.delay(100)

        print("No path found")
        return False
    
    def ucs(self, start, target):
        """
        Perform Uniform-Cost Search (UCS) to find the least-cost path from start to target.
        :param start: Tuple (row, col) starting position
        :param target: Tuple (row, col) target position
        """
        rows, cols = len(self.grid), len(self.grid[0])
        pq = []  # Priority queue for UCS
        heapq.heappush(pq, (0, start))  # Push (cost, position)
        visited = set()
        parent = {}  # To reconstruct the solution path

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        
        while pq:
            cost, current = heapq.heappop(pq)

            if current in visited:
                continue
            visited.add(current)

            # If target is reached
            if current == target:
                print("Target reached!")
                self.mark_path(self.reconstruct_path(parent, start, target))
                return True

            # Explore neighbors
            x, y = current
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                    # Check if the cell is walkable
                    if self.grid[nx][ny] == "empty" or self.grid[nx][ny] == "target":
                        new_cost = cost + 1  # Assuming uniform cost for all moves
                        heapq.heappush(pq, (new_cost, (nx, ny)))
                        parent[(nx, ny)] = (x, y)

            # Visualization: mark the current cell as visited
            if self.grid[x][y] != "start" and self.grid[x][y] != "target":
                self.grid[x][y] = "visited"

            # Update the screen to show the current state
            self.draw_grid()
            self.draw_objects()
            pygame.display.flip()

            # Add a small delay to visualize the process
            pygame.time.delay(100)

        print("No path found")
        return False
    
    def greedy_search(self, start, target):
        """
        Perform Greedy Search to find a path from start to target.
        :param start: Tuple (row, col) starting position
        :param target: Tuple (row, col) target position
        """
        rows, cols = len(self.grid), len(self.grid[0])
        priority_queue = []  # Min-heap for greedy selection
        heapq.heappush(priority_queue, (0, start))  # (heuristic, position)
        visited = set()
        parent = {}  # To reconstruct the solution path

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        def heuristic(node):
            # Manhattan Distance
            return abs(node[0] - target[0]) + abs(node[1] - target[1])

        while priority_queue:
            _, current = heapq.heappop(priority_queue)
            if current in visited:
                continue
            visited.add(current)

            # If target is reached
            if current == target:
                print("Target reached!")
                self.mark_path(self.reconstruct_path(parent, start, target))
                return True

            # Explore neighbors
            x, y = current
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                    # Check if the cell is walkable
                    if self.grid[nx][ny] == "empty" or self.grid[nx][ny] == "target":
                        heapq.heappush(priority_queue, (heuristic((nx, ny)), (nx, ny)))
                        parent[(nx, ny)] = (x, y)

            # Visualization: mark the current cell as visited
            if self.grid[x][y] != "start" and self.grid[x][y] != "target":
                self.grid[x][y] = "visited"

            # Update the screen to show the current state
            self.draw_grid()
            self.draw_objects()
            pygame.display.flip()

            # Add a small delay to visualize the process
            pygame.time.delay(100)

        print("No path found")
        return False
    
    def a_star(self, start, target):
        """
        Perform A* Search to find a path from start to target.
        :param start: Tuple (row, col) starting position
        :param target: Tuple (row, col) target position
        """
        rows, cols = len(self.grid), len(self.grid[0])

        # Priority queue (frontier) initialized with the start node and its cost
        frontier = []
        heapq.heappush(frontier, (0, start))
        
        g_cost = {start: 0}  # Cost from start to the current node
        parent = {}  # To reconstruct the path

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        while frontier:
            # Pop the node with the lowest f-cost
            _, current = heapq.heappop(frontier)

            # If the target is reached
            if current == target:
                print("Target reached!")
                self.mark_path(self.reconstruct_path(parent, start, target))
                return True

            x, y = current
            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if 0 <= nx < rows and 0 <= ny < cols:
                    neighbor = (nx, ny)

                    # Skip non-walkable cells
                    if self.grid[nx][ny] != "empty" and self.grid[nx][ny] != "target":
                        continue

                    # Calculate the cost
                    new_g_cost = g_cost[current] + 1  # Assume uniform cost for moving
                    if neighbor not in g_cost or new_g_cost < g_cost[neighbor]:
                        g_cost[neighbor] = new_g_cost
                        f_cost = new_g_cost + self.heuristic(neighbor, target)
                        heapq.heappush(frontier, (f_cost, neighbor))
                        parent[neighbor] = current

            # Visualization: mark the current cell as visited
            if self.grid[x][y] != "start" and self.grid[x][y] != "target":
                self.grid[x][y] = "visited"

            # Update the screen to show the current state
            self.draw_grid()
            self.draw_objects()
            pygame.display.flip()

            # Add a small delay to visualize the process
            pygame.time.delay(100)

        print("No path found")
        return False
    
    def heuristic(self, node, target):
        """
        Heuristic function for A* (Manhattan distance).
        :param node: Current node as a tuple (row, col)
        :param target: Target node as a tuple (row, col)
        :return: Estimated cost to reach the target
        """
        return abs(node[0] - target[0]) + abs(node[1] - target[1])
    
    def reconstruct_path(self, parent, start, target):
        path = []
        current = target
        while current != start:
            path.append(current)
            current = parent[current]
        path.append(start)
        path.reverse()
        return path

    def mark_path(self, path):
        """
        Mark the path found by the search algorithm.
        :param path: List of tuples representing the path
        """
        for row, col in path:
            if self.grid[row][col] != "start" and self.grid[row][col] != "target":
                self.grid[row][col] = "path"

    def draw_grid(self):
        # Draw the grid with colored squares if they are selected
        for row in range(len(self.grid)):  # Loop over the rows
            for column in range(len(self.grid[row])):  # Loop over the columns
                rect = pygame.Rect(column * self.CELL_SIZE, row * self.CELL_SIZE, self.CELL_SIZE, self.CELL_SIZE)

                cell_content = self.grid[row][column]
                if cell_content == "empty":
                    color = self.WHITE
                elif cell_content == "start":
                    color = self.BLUE
                elif cell_content == "target":
                    color = (255, 0, 0)  # Red for target
                elif cell_content == "path":
                    color = (0, 255, 0)  # Green for path
                elif cell_content == "visited":
                    color = (255, 255, 0)  # Yellow for visited cells
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
        # Draw the objects in the right panel (grid area)
        for object in self.object_list:
            if object.shape == "circle":
                pygame.draw.circle(self.screen, object.color, object.orta_coord, 20)
            elif object.shape == "triangle":
                pygame.draw.polygon(self.screen, object.color, object.triangle_coord)
            elif object.shape == "pentagon":
                pygame.draw.polygon(self.screen, object.color, object.pentagon_coord)

    def start_search(self):
        """
        Starts the selected search algorithm if start and target squares are selected.
        """
        if not self.start_square or not self.target_square:
            print("Start and/or target square not set!")
            return

        # Proceed with the selected search algorithm
        if self.choice == "Depth-First Search":
            self.dfs(self.start_square, self.target_square)
        elif self.choice == "Breadth-First Search":
            self.bfs(self.start_square, self.target_square)
        elif self.choice == "Uniform-Cost Search":
            self.ucs(self.start_square, self.target_square)
        elif self.choice == "Greedy Search":
            self.greedy_search(self.start_square, self.target_square)
        elif self.choice == "A* Search":
            self.a_star(self.start_square, self.target_square)
        else:
            print("Invalid choice!")

        # Update the screen to show the current state
        self.draw_grid()
        self.draw_objects()
        pygame.display.flip()
        self.wait_for_reset()
        self.choice = self.display_menu()

    def reset_grid(self):
        """
        Reset the grid to its initial state.
        Clear all paths, visited cells, and reset the start and target positions.
        """
        # Reset grid cells to "empty" except for objects
        for row in range(len(self.grid)):
            for col in range(len(self.grid[row])):
                if isinstance(self.grid[row][col], str):  # If the cell is not an object
                    self.grid[row][col] = "empty"
        
        # Clear start and target positions
        self.start_square = None
        self.target_square = None
        self.ready_for_search = False

    def wait_for_reset(self):
        """
        After the search finishes, ask the user if they want to reset the grid.
        This function will display a prompt or listen for a key press to reset.
        """
        font = pygame.font.Font(None, 36)
        text = font.render("Press R to Reset Grid, Q to Quit", True, self.BLACK)
        self.screen.blit(text, (self.WIDTH // 2 - text.get_width() // 2, self.HEIGHT // 2))
        pygame.display.flip()

        waiting_for_input = True
        while waiting_for_input:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:  # Press 'R' to reset
                        self.reset_grid()
                        waiting_for_input = False
                    elif event.key == pygame.K_q:  # Press 'Q' to quit
                        pygame.quit()
                        exit()

    def event_handler(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Handle mouse click on the grid or UI
                if self.start_square == None:
                    mouse_pos = pygame.mouse.get_pos()
                    row, col = self.get_grid_pos(mouse_pos)

                    # Toggle the selected state of the cell
                    if 0 <= row < self.HEIGHT and 0 <= col < self.WIDTH - 300 and self.grid[row][col] == "empty":  # Prevent clicks on the UI area
                        self.grid[row][col] = "start"
                        self.start_square = (row, col)
                    else:
                        print("you cannot choose there")

                elif self.target_square == None:
                    mouse_pos = pygame.mouse.get_pos()
                    row, col = self.get_grid_pos(mouse_pos)

                    # Toggle the selected state of the cell
                    if 0 <= row < self.HEIGHT and 0 <= col < self.WIDTH - 300 and self.grid[row][col] == "empty":  # Prevent clicks on the UI area
                        self.grid[row][col] = "target"
                        self.target_square = (row, col)
                        self.ready_for_search = True
                    else:
                        print("you cannot choose there")

                else:
                    print(f"Start: {self.start_square}, Target: {self.target_square}")
                    print("you cannot choose extra points")
                    return True
