import tkinter as tk
from tkinter import messagebox
import random
import heapq

GRID_SIZE = 10
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

class PathfindingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Pathfinding Game")

        self.grid = create_grid()
        self.start = None
        self.end = None
        self.path = []
        self.max_points = 0

        self.buttons = [[None for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.create_ui()

    def create_ui(self):
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                btn = tk.Button(
                    self.root,
                    text=str(self.grid[row][col]),
                    width=5,
                    height=2,
                    command=lambda r=row, c=col: self.cell_click(r, c)
                )
                btn.grid(row=row, column=col, padx=2, pady=2)
                self.buttons[row][col] = btn

        self.find_path_btn = tk.Button(self.root, text="Find Path", command=self.find_path)
        self.find_path_btn.grid(row=GRID_SIZE, column=0, columnspan=GRID_SIZE // 2, pady=10)

        self.reset_btn = tk.Button(self.root, text="Reset", command=self.reset)
        self.reset_btn.grid(row=GRID_SIZE, column=GRID_SIZE // 2, columnspan=GRID_SIZE // 2, pady=10)

    def cell_click(self, row, col):
        if not self.start:
            self.start = (row, col)
            self.buttons[row][col].config(bg="green")
        elif not self.end:
            self.end = (row, col)
            self.buttons[row][col].config(bg="red")

    def find_path(self):
        if not self.start or not self.end:
            messagebox.showwarning("Warning", "Please select both start and end points.")
            return

        result = find_max_points(self.grid, self.start, self.end)
        self.max_points = result["score"]
        self.path = result["path"]

        for row, col in self.path:
            if (row, col) != self.start and (row, col) != self.end:
                self.buttons[row][col].config(bg="blue")

        messagebox.showinfo("Result", f"Maximum Points Collected: {self.max_points}")

    def reset(self):
        self.grid = create_grid()
        self.start = None
        self.end = None
        self.path = []
        self.max_points = 0

        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                btn = self.buttons[row][col]
                btn.config(text=str(self.grid[row][col]), bg="SystemButtonFace")

if __name__ == "__main__":
    root = tk.Tk()
    app = PathfindingApp(root)
    root.mainloop()
