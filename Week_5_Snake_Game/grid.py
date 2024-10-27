class Grid:
    def __init__(self, row_number, column_number):
        self.rows = row_number
        self.columns = column_number
        
        self.grid_size = 50
        self.screen_width  = self.grid_size * self.columns
        self.screen_height = self.grid_size * self.rows

        self.grid_color       = (100, 100, 100)  # Darker gray color
        self.background_color = (173, 216, 230)  # Light blue color

        self.grid_cells = [[0 for _ in range(self.columns)] for _ in range(self.rows)]  # 0 means empty