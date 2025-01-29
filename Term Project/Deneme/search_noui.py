import random
import heapq

GRID_SIZE = 5
MOVES_LIMIT = 10

# Helper function to create a grid with random points
def create_grid():
    return [[random.randint(1, 10) for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

def find_max_points(grid, start, end):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    pq = [(-grid[start[0]][start[1]], start[0], start[1], MOVES_LIMIT, [(start[0], start[1])])]  # (negative points, x, y, remaining moves, path)
    max_score = 0
    best_path = []

    while pq:
        current_points, x, y, remaining_moves, path = heapq.heappop(pq)
        current_points = -current_points

        # If we reach the end point, check if it's the best score
        if (x, y) == end:
            if current_points > max_score:
                max_score = current_points
                best_path = path
            continue

        if remaining_moves == 0:
            continue

        # Explore neighbors
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in path:
                new_points = current_points + grid[nx][ny]
                heapq.heappush(pq, (-new_points, nx, ny, remaining_moves - 1, path + [(nx, ny)]))

    return {"score": max_score, "path": best_path}

if __name__ == "__main__":
    grid = create_grid()
    for row in grid:
        print("\t".join(map(str, row)))

    start = tuple(map(int, input("Enter start point (row, col): ").split(',')))
    end = tuple(map(int, input("Enter end point (row, col): ").split(',')))
    print(grid)

    result = find_max_points(grid, start, end)
    print(f"Maximum Points Collected: {result['score']}")
    print("Path Taken:", result['path'])
