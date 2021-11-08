import numpy

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = numpy.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.width = width
        self.height = height
        
    def cell_at(self, x, y):
        return self.grid[y, x]
    
    def set(self, x, y, num):
        self.grid[y,x] = num
    
    def as_list(self):
        return numpy.reshape(self.grid, (1, self.width * self.height))[0]
    
    def __str__(self):
        return str(self.grid)
    