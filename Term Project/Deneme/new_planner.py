import heapq
from math import sqrt

R = 5
C = 8

def find_max_points(grid:list, start:int, end:int):

    start_row = (start - 1) // C
    start_column = (start - 1) % C
    start = (start_row, start_column)

    target_row = (end - 1) // C
    target_column = (end - 1) % C
    target = (target_row, target_column)

    #print(f"end: {end}")

    #print(f"start point: {start}")
    #print(f"target point: {target}")

    MOVES_LIMIT = int(2 * sqrt((start_row - target_row)**2 + (start_column - target_column)**2)) + 1
    #print("Move limit: ", MOVES_LIMIT)

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    pq = [(-grid[start[0]][start[1]], start[0], start[1], MOVES_LIMIT, [(start[0], start[1])])]  # (negative points, x, y, remaining moves, path)
    max_score = 0
    best_path = []

    while pq:
        current_points, x, y, remaining_moves, path = heapq.heappop(pq)
        current_points = -current_points

        # If we reach the end point, check if it's the best score
        if (x, y) == target:
            if current_points > max_score:
                max_score = current_points
                best_path = path
            continue

        if remaining_moves == 0:
            continue

        # Explore neighbors
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < R and 0 <= ny < C and (nx, ny) not in path:
                new_points = current_points + grid[nx][ny]
                heapq.heappush(pq, (-new_points, nx, ny, remaining_moves - 1, path + [(nx, ny)]))

    return {"score": max_score, "path": best_path}

if __name__ == "__main__":

    start = 1
    end = 40

    cell_list = [10, 40, 17, 34, 2, 26, 7, 37, 
                18, 27, 22, 20, 19, 6, 5, 11, 
                21, 8, 24, 16, 29, 39, 14, 12, 
                28, 1, 31, 35, 30, 3, 13, 38, 
                25, 23, 36, 9, 4, 33, 32, 15]
    
    grid = [cell_list[i * C:(i + 1) * C] for i in range(R)]
    print(grid)

    for start in range(1, 40):
        for end in range(1, 40):

            result = find_max_points(grid, start, end)
            #print(f"Maximum Points Collected: {result['score']}")
            print("Path Taken:", result['path'])
            #print("Length of the path: ", len(result['path']))

            if (len(result['path']) == 0):
                print("patladi")