from test_grid import DiscreteGrid

class TraversalPaths(DiscreteGrid):
    def __init__(self, corner1, corner2, rows, cols):
        super().__init__(corner1, corner2, rows, cols)

    """
    Returns a list of linear indices traversing the grid row-by-row
    in a zigzag (serpentine) pattern from bottom to top.
    """
    def generate_zigzag_path(self):
        path = []
        for row in range(self.rows):
            if row % 2 == 0:
                # Left to right
                cols_range = range(self.cols)
            else:
                # Right to left
                cols_range = reversed(range(self.cols))
            for col in cols_range:
                idx = self.get_linear_index(row, col)
                path.append(idx)
        return path

    """
    Traverse column-wise in zigzag:
    Even cols bottom->top, odd cols top->bottom.
    """
    def generate_column_zigzag_path(self, start_from_left=True):

        path = []
        cols = range(self.cols) if start_from_left else range(self.cols-1, -1, -1)
        for col in cols:
            if col % 2 == 0:
                rows_range = range(self.rows)
            else:
                rows_range = reversed(range(self.rows))
            for row in rows_range:
                path.append(self.get_linear_index(row, col))
        return path

    def generate_spiral_path(self):
        """
        Returns linear indices traversing the grid in a clockwise spiral.
        """
        path = []
        top, bottom = 0, self.rows - 1
        left, right = 0, self.cols - 1

        while left <= right and top <= bottom:
            # Traverse from left to right
            for col in range(left, right + 1):
                path.append(self.get_linear_index(top, col))
            top += 1

            # Traverse down the right column
            for row in range(top, bottom + 1):
                path.append(self.get_linear_index(row, right))
            right -= 1

            if top <= bottom:
                # Traverse from right to left
                for col in range(right, left - 1, -1):
                    path.append(self.get_linear_index(bottom, col))
                bottom -= 1

            if left <= right:
                # Traverse up the left column
                for row in range(bottom, top - 1, -1):
                    path.append(self.get_linear_index(row, left))
                left += 1

        return path

    def generate_snake_center_out(self):
        """
        Traverses grid starting from the center row going outwards in a snake pattern.
        For grids with even rows, starts near the center.
        """
        path = []
        center_row = self.rows // 2
        directions = [1, -1]  # Alternate going down and up from center
        rows_visited = set()

        def zigzag_row(row):
            # Left to right on even offset from center, right to left otherwise
            if (row - center_row) % 2 == 0:
                cols_range = range(self.cols)
            else:
                cols_range = reversed(range(self.cols))
            return [self.get_linear_index(row, c) for c in cols_range]

        # Start from center row
        path.extend(zigzag_row(center_row))
        rows_visited.add(center_row)

        offset = 1
        while len(rows_visited) < self.rows:
            for d in directions:
                r = center_row + d * offset
                if 0 <= r < self.rows and r not in rows_visited:
                    path.extend(zigzag_row(r))
                    rows_visited.add(r)
            offset += 1

        return path


# class RobotTraversal(RobotGrid):
#     def __init

# grid = TraversalPaths((0, 0), (5, 5), rows=5, cols=5)
# path = grid.generate_column_zigzag_path()
# grid.visualize(path_points=path)
