import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import rospy

def calculate_distance(x1, y1, x2, y2):
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

def find_midpoint(point1, point2):
  """
  Calculates the midpoint between two 2D points.

  Args:
    point1: A tuple or list representing the first point (x1, y1).
    point2: A tuple or list representing the second point (x2, y2).

  Returns:
    A tuple representing the midpoint (mid_x, mid_y).
  """
  x1, y1 = point1
  x2, y2 = point2

  mid_x = (x1 + x2) / 2
  mid_y = (y1 + y2) / 2

  return (mid_x, mid_y)

class DiscreteGrid:
    def __init__(self, corner1, corner2, rows, cols, offset=None, row_offset=0):
        self.corner1 = np.array(corner1, dtype=float)
        self.corner2 = np.array(corner2, dtype=float)
        self.rows = rows
        self.cols = cols

        self.min_corner = np.minimum(self.corner1, self.corner2)
        self.max_corner = np.maximum(self.corner1, self.corner2)
        self.grid_size = self.max_corner - self.min_corner

        self.cell_width = self.grid_size[0] / self.cols
        self.cell_height = self.grid_size[1] / self.rows

        self.dict=None
        self.offset = offset
        self.row_offset=row_offset
        self.center = find_midpoint(corner1, corner2)

    def get_cell_center(self, row, col):
        x = self.max_corner[0] - (col + 0.5) * self.cell_width
        y = self.max_corner[1] - (row + 0.5) * self.cell_height
        return (x, y)

    def get_cell_index(self, position, clamp=True):
        """
        Returns (row, col) index of a point in the grid.
        If clamp=True, clamps to the nearest valid cell.
        If clamp=False, returns None if the position is out of bounds.
        """
        x, y = position
        col = int((self.max_corner[0]-x) / self.cell_width)
        row = int((self.max_corner[1]-y) / self.cell_height)

        if clamp:
            col = max(0, min(self.cols - 1, col))
            row = max(0, min(self.rows - 1, row))
            return (row, col+self.row_offset)

        if 0 <= row < self.rows and 0 <= col < self.cols:
            return (row, col+self.row_offset)
        return None

    def get_linear_index(self, row, col):
        """
        Returns the linear index from a row-col pair
        """
        if self.offset is None:
            return col * self.rows + row 
        else:
            return col*self.rows + row + self.offset 

    def get_index_from_position(self, position, clamp=True):
        """
        Returns linear index from a physical position.
        If clamp=True, clamps to nearest cell.
        If clamp=False, returns None if out of bounds.
        """
        idx = self.get_cell_index(position, clamp=clamp)
        if idx:
            return self.get_linear_index(*idx)
        return None


    def to_dict(self, height=0.2):
        if self.dict==None:
            temp = {}
            for  i in range(self.rows):
                for j in range(self.cols):
                    temp[self.get_linear_index(i,j)] = self.get_cell_center(i,j) + (height,)
            self.dict = copy.deepcopy(temp) 
        return self.dict

    #def path_to_dict(self, path, height=0.2):


    def visualize(self, end_effector_pos=None, path_points=None):
        fig, ax = plt.subplots(figsize=(10, 5))

        # Draw grid and label each cell with its linear index
        for row in range(self.rows):
            for col in range(self.cols):
                (x,y) = self.get_cell_center(row, col)
                rect = plt.Rectangle((x, y), self.cell_width, self.cell_height,
                                    edgecolor='black', facecolor='none')
                ax.add_patch(rect)
                idx = self.get_linear_index(row, col)
                ax.text(x + self.cell_width / 2 , y + self.cell_height / 2,
                        str(idx), ha='center', va='center', fontsize=8, color='blue')

        # Plot path (linear indices)
        if path_points:
            path_xy = []
            for idx in path_points:
                row = idx // self.cols
                col = idx % self.cols
                center = self.get_cell_center(row, col)
                path_xy.append(center)
                if len(path_xy)==len(path_points):
                    color = 'b'
                elif len(path_xy)==1:
                    color = 'r'
                else:
                    color = 'm'
                ax.plot(center[0], center[1], 'go',color=color)  # plot node
                #ax.text(center[0], center[1] + 0.2, str(idx), color='green', ha='center', fontsize=8)

            # Draw arrows between path points
            for i in range(len(path_xy) - 1):
                x0, y0 = path_xy[i]
                x1, y1 = path_xy[i + 1]
                dx = x1 - x0
                dy = y1 - y0
                ax.arrow(x0, y0, dx * 0.85, dy * 0.85,
                        head_width=0.15, head_length=0.2,
                        fc='green', ec='green', length_includes_head=True)

        # Plot end effector if given
        if end_effector_pos:
            ax.plot(end_effector_pos[0], end_effector_pos[1], 'ro', label='End Effector')
            ax.text(end_effector_pos[0], end_effector_pos[1] + 0.2,
                    f'EE ({end_effector_pos[0]:.1f}, {end_effector_pos[1]:.1f})',
                    color='red', ha='center')
            idx = self.get_index_from_position(end_effector_pos)
            if idx is not None:
                ax.text(end_effector_pos[0], end_effector_pos[1] - 0.3,
                        f'Index: {idx}', color='darkred', ha='center')

        ax.set_xlim(self.min_corner[0], self.max_corner[0])
        ax.set_ylim(self.min_corner[1], self.max_corner[1])
        ax.set_aspect('equal')
        ax.set_title("Discrete Grid with Directional Path and Optional End Effector")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True)
        if end_effector_pos or path_points:
            ax.legend()
        plt.tight_layout()
        #plt.show()


    def __repr__(self):
        return f"<DiscreteGrid: {self.rows}x{self.cols} from {self.corner1.tolist()} to {self.corner2.tolist()}>"


class DualGrid(DiscreteGrid):
    def __init__(self, corner1, corner2, rows, cols, corner_one, corner_two, cols_two):
        super().__init__(corner1, corner2, rows, cols)
        self.grid_two = DiscreteGrid(corner_one, corner_two,rows, cols_two, offset=rows*cols,row_offset=2)

        #self.total_rows = self.grid1.rows + self.grid2.rows
    
    def checkDistance(self, x, y):
        if calculate_distance(self.center[0], self.center[1], x, y) < calculate_distance(self.grid_two.center[0], self.grid_two.center[1], x, y):
            return 0
        else:
            return 1

    def augmented_get_cell_center(self, row, col):
        if (0 <= col <self.cols):
            return self.get_cell_center(row, col)
        elif self.cols <= col:
            return self.grid_two.get_cell_center(row, col-self.cols)

    def augemented_get_cell_index(self, position, clamp=True):
        """
        Returns (row, col) index of a point in the grid.
        If clamp=True, clamps to the nearest valid cell.
        If clamp=False, returns None if the position is out of bounds.
        """
        #Check which Grid is closer!
        x, y = position
        #grid_1 = calculate_distance(self.center[0], self.center[1], x, y) 
        #grid_2 = calculate_distance(self.grid_two.center[0], self.grid_two.center[1], x, y)
        #print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #print(grid_1,grid_2)
        if calculate_distance(self.center[0], self.center[1], x, y) < calculate_distance(self.grid_two.center[0], self.grid_two.center[1], x, y):
            #print("CLOSER TO GRID 1")
            return self.get_cell_index(position, clamp=clamp)
        else:
            return self.grid_two.get_cell_index(position,clamp=clamp)

    def augemented_get_linear_index(self, row, col):
        """
        Returns the linear index from a row-col pair
        """
        if (0 <= col <self.cols):
            return self.get_linear_index(row,col)
        elif self.cols <= col:
            return self.grid_two.get_linear_index(row, col-self.cols)

    def augmented_get_index_from_position(self, position, clamp=True):
        """
        Returns linear index from a physical position.
        If clamp=True, clamps to nearest cell.
        If clamp=False, returns None if out of bounds.
        """
        idx = self.augemented_get_cell_index(position, clamp=clamp)
        #print(idx)
        if idx:
            return self.augemented_get_linear_index(*idx)
        return None
    
    def augemented_visualize(self, end_effector_pos=None, path_points=None):
        if calculate_distance(self.center[0], self.center[1], end_effector_pos[0], end_effector_pos[1]) < calculate_distance(self.grid_two.center[0], self.grid_two.center[1], end_effector_pos[0], end_effector_pos[1]):
            self.visualize(end_effector_pos, path_points)
        else:
            self.grid_two.visualize(end_effector_pos,path_points)
    
    def to_dict_fix(self, height=0.2):
        if self.dict==None:
            temp = {}
            for  i in range(self.rows):
                for j in range(self.cols+self.grid_two.cols):
                        temp[self.augemented_get_linear_index(i,j)] = self.augmented_get_cell_center(i,j) + (height,)
            self.dict = copy.deepcopy(temp) 
        return self.dict




    


# grid = DualGrid((0.22, -0.563),(0.55,0.255), 5, 2,(-0.55,0.6), (-0.26, -0.563), 3)
# print(grid.get_cell_center(0,0))
# print(grid.get_cell_index((0.517, 0.11866666666666667)))
# print(grid.get_linear_index(0,0))
# grid.visualize()
# grid.grid_two.visualize()
# plt.show()
#print(grid.to_dict_fix())
#grid.augemented_visualize([-1,-2])
# Slightly outside right and top
# pos1 = (10.2, 5.3)
# print(grid.get_cell_index(pos1, clamp=True))   # Clamped to (4, 9)
# print(grid.get_index_from_position(pos1, clamp=True))  # Clamped to index 49

# # Same input, no clamping
# print(grid.get_cell_index(pos1, clamp=False))  # Returns None

# grid.visualize(pos1)
