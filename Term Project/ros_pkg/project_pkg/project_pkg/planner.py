import heapq

# Constants for the grid dimensions
R = 8  # Number of columns
C = 5  # Number of rows

# Bonus points array
bonus_points = [
    10, 40, 17, 34, 2, 26, 7, 37, 
    18, 27, 22, 20, 19, 6, 5, 11, 
    21, 8, 24, 16, 29, 39, 14, 12, 
    28, 1, 31, 35, 30, 3, 13, 38, 
    25, 23, 36, 9, 4, 33, 32, 15
]

def manhattan_distance(pos1, pos2):
    """Calculate the Manhattan distance between two positions."""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def plan_path(bonus_points:list, start:int, target:int, max_moves:int):
    """
    Perform A* Search to find the path that maximizes points and reaches the target.
    :param start: Tuple (row, col) starting position
    :param target: Tuple (row, col) target position
    :param max_moves: Maximum number of moves allowed
    :return: (max_points, path) where max_points is the sum of points collected,
             and path is a list of positions visited.
    """
    target = int(target)

    start  = (start // R, start % R)
    target = (target // R, target % R)

    # Convert 1D bonus points to a 2D grid
    grid = [bonus_points[i * C:(i + 1) * C] for i in range(R)]

    # Directions for movement (Up, Down, Left, Right)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    # Priority queue for A* (min-heap)
    pq = []
    heapq.heappush(pq, (0, 0, start, [start], grid[start[0]][start[1]]))  
    # (priority, steps, current position, path, total points)
    
    visited = set()
    best_path = []
    max_points = 0

    while pq:
        priority, steps, current, path, points = heapq.heappop(pq)
        row, col = current

        # Stop if we've already visited this position with fewer steps
        if (row, col, steps) in visited:
            continue
        visited.add((row, col, steps))

        # Stop if we exceed max_moves
        if steps > max_moves:
            continue

        # If the target is reached, update the best result
        if current == target:
            if points > max_points:
                max_points = points
                best_path = path
            continue

        # Explore neighbors
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if 0 <= nr < C and 0 <= nc < R:  # Check grid boundaries
                new_points = points + grid[nr][nc]
                heuristic = manhattan_distance((nr, nc), target)  # Distance to target
                new_priority = -(new_points) + heuristic  # Combine points and heuristic
                heapq.heappush(pq, (new_priority, steps + 1, (nr, nc), path + [(nr, nc)], new_points))

    return max_points, best_path