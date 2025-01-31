import heapq

R = 5
C = 8
DIRECTIONS = [(-1, 0, "UP"), (1, 0, "DOWN"), (0, -1, "LEFT"), (0, 1, "RIGHT")]
MOVE_COST = 0.5  # Moving forward takes 1 second
TURN_COST = 0.34  # Turning takes 0.5 seconds

def find_max_points(grid: list, start: int, end: int, time_limit: float):
    start_row, start_col = (start - 1) // C, (start - 1) % C
    target_row, target_col = (end - 1) // C, (end - 1) % C
    start_pos, target_pos = (start_row, start_col), (target_row, target_col)

    pq = [(-grid[start_row][start_col], 0, start_row, start_col, None, [(start_row, start_col)])]
    max_score, best_path = 0, []

    while pq:
        current_points, elapsed_time, x, y, last_direction, path = heapq.heappop(pq)
        current_points = -current_points

        if (x, y) == target_pos and elapsed_time <= time_limit:
            if current_points > max_score:
                max_score, best_path = current_points, path
            continue

        for dx, dy, direction in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < R and 0 <= ny < C and (nx, ny) not in path:
                new_time = elapsed_time + MOVE_COST
                if last_direction and last_direction != direction:
                    new_time += TURN_COST
                
                if new_time <= time_limit:
                    new_points = current_points + grid[nx][ny]
                    heapq.heappush(pq, (-new_points, new_time, nx, ny, direction, path + [(nx, ny)]))
    
    return {"score": max_score, "path": best_path}

# Example Usage
grid = [
    10, 40, 17, 34, 2, 26, 7, 37,
    18, 27, 22, 20, 19, 6, 5, 11,
    21, 8, 24, 16, 29, 39, 14, 12,
    28, 1, 31, 35, 30, 3, 13, 38,
    25, 23, 36, 9, 4, 33, 32, 15
]
time_limit = 10  # Time limit in seconds
start = 1  # Start point (1-based index)
end = 32  # End point (1-based index)

result = find_max_points(grid, start, end, time_limit)
print(f"Max Points: {result['score']}")
print(f"Path: {result['path']}")
