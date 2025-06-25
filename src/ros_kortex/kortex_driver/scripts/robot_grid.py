from test_grid import DiscreteGrid
from collections import deque

class RobotGrid(DiscreteGrid):
    def __init__(self, corner1, corner2, rows, cols, robot_center=None, min_radius=0):
        super().__init__(corner1, corner2, rows, cols)
        self.robot_center = robot_center or self._default_robot_center()
        self.min_radius = min_radius

    def _default_robot_center(self):
        # Grid center in world coordinates
        return (
            (self.min_corner[0] + self.max_corner[0]) / 2,
            (self.min_corner[1] + self.max_corner[1]) / 2
        )

    def is_cell_reachable(self, row, col):
        center = self.get_cell_center(row, col)
        dx = center[0] - self.robot_center[0]
        dy = center[1] - self.robot_center[1]
        distance = (dx**2 + dy**2)**0.5
        return distance >= self.min_radius

    def get_reachable_cells(self):
        reachable = []
        for row in range(self.rows):
            for col in range(self.cols):
                if self.is_cell_reachable(row, col):
                    reachable.append(self.get_linear_index(row, col))
        return reachable

    def visualize(self, end_effector_pos=None, path_points=None):
        import matplotlib.pyplot as plt
        import numpy as np

        fig, ax = plt.subplots(figsize=(10, 5))

        for row in range(self.rows):
            for col in range(self.cols):
                center = self.get_cell_center(row, col)
                x = self.min_corner[0] + col * self.cell_width
                y = self.min_corner[1] + row * self.cell_height
                idx = self.get_linear_index(row, col)

                # Check reachability
                reachable = self.is_cell_reachable(row, col)
                face_color = 'white' if reachable else 'lightgray'

                rect = plt.Rectangle((x, y), self.cell_width, self.cell_height,
                                     edgecolor='black', facecolor=face_color, alpha=0.8)
                ax.add_patch(rect)
                ax.text(x + self.cell_width / 2, y + self.cell_height / 2,
                        str(idx), ha='center', va='center', fontsize=8,
                        color='black' if reachable else 'gray')

        # Draw robot center and reach radius
        if self.robot_center and self.min_radius > 0:
            ax.plot(*self.robot_center, 'kx', label='Robot Center')
            circle = plt.Circle(self.robot_center, self.min_radius, color='red', fill=False, linestyle='--')
            ax.add_patch(circle)

        # Path (same as before)
        if path_points:
            path_xy = []
            for idx in path_points:
                row = idx // self.cols
                col = idx % self.cols
                center = self.get_cell_center(row, col)
                path_xy.append(center)
                ax.plot(center[0], center[1], 'go')
                ax.text(center[0], center[1] + 0.2, str(idx), color='green', ha='center', fontsize=8)

            path_xy = np.array(path_xy)
            for i in range(len(path_xy) - 1):
                x0, y0 = path_xy[i]
                x1, y1 = path_xy[i + 1]
                dx, dy = x1 - x0, y1 - y0
                ax.arrow(x0, y0, dx * 0.85, dy * 0.85, head_width=0.15, head_length=0.2,
                         fc='green', ec='green', length_includes_head=True)

        # EE marker
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
        ax.set_title("Robot Grid with Unreachable Center Zone")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True)
        ax.legend()
        plt.tight_layout()
        plt.show()

class RobotTraversal(RobotGrid):
    def __init__(self, corner1, corner2, rows, cols, robot_center=None, min_radius=0):
        super().__init__(corner1, corner2, rows, cols, robot_center, min_radius)
    
    def generate_connected_zigzag_path(self):
        """
        Zigzag through reachable cells, changing direction when blocked by an unreachable cell.
        This respects walls and ensures path continuity.
        """
        visited = set()
        path = []

        def is_valid(r, c):
            return (0 <= r < self.rows and 0 <= c < self.cols and
                    self.is_cell_reachable(r, c) and (r, c) not in visited)

        # Find a valid starting point
        for r in range(self.rows):
            for c in range(self.cols):
                if self.is_cell_reachable(r, c):
                    start = (r, c)
                    break
            else:
                continue
            break
        else:
            return []  # No valid cell

        # Zigzag traversal starting from `start`
        queue = deque()
        queue.append(start)
        visited.add(start)

        while queue:
            r, c = queue.popleft()
            path.append(self.get_linear_index(r, c))

            # Determine horizontal direction (zigzag)
            direction = 1 if r % 2 == 0 else -1
            next_c = c + direction

            if is_valid(r, next_c):
                queue.append((r, next_c))
                visited.add((r, next_c))
            else:
                # Try to go to next row
                next_r = r + 1
                for dc in [0, direction]:  # Prefer same column or in direction
                    nr, nc = next_r, c + dc
                    if is_valid(nr, nc):
                        queue.append((nr, nc))
                        visited.add((nr, nc))
                        break

        return path

    def generate_full_reachable_path(self):
        """
        Generates a path that visits every reachable cell exactly once,
        never stepping through unreachable tiles.
        Returns a list of linear indices in a valid robot-walkable order.
        """
        visited = set()
        path = []

        def is_valid(r, c):
            return (0 <= r < self.rows and
                    0 <= c < self.cols and
                    self.is_cell_reachable(r, c) and
                    (r, c) not in visited)

        def dfs(r, c):
            visited.add((r, c))
            path.append(self.get_linear_index(r, c))

            # Prefer directions in a consistent order (could alternate for snake effect)
            for dr, dc in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # Right, Down, Left, Up
                nr, nc = r + dr, c + dc
                if is_valid(nr, nc):
                    dfs(nr, nc)

        # Find a reachable starting point (closest to robot center)
        min_dist = float('inf')
        start_cell = None
        for row in range(self.rows):
            for col in range(self.cols):
                if self.is_cell_reachable(row, col):
                    cx, cy = self.get_cell_center(row, col)
                    dist = ((cx - self.robot_center[0]) ** 2 + (cy - self.robot_center[1]) ** 2) ** 0.5
                    if dist < min_dist:
                        min_dist = dist
                        start_cell = (row, col)

        if start_cell:
            dfs(*start_cell)

        return path[:len(path)-2]


    def generate_reachable_spiral_path(self):
        """
        Traverses the grid in a spiral pattern, skipping unreachable cells.
        """
        path = []
        visited = set()
        top, bottom = 0, self.rows - 1
        left, right = 0, self.cols - 1

        while top <= bottom and left <= right:
            for col in range(left, right + 1):
                if self.is_cell_reachable(top, col):
                    path.append(self.get_linear_index(top, col))
            top += 1

            for row in range(top, bottom + 1):
                if self.is_cell_reachable(row, right):
                    path.append(self.get_linear_index(row, right))
            right -= 1

            if top <= bottom:
                for col in range(right, left - 1, -1):
                    if self.is_cell_reachable(bottom, col):
                        path.append(self.get_linear_index(bottom, col))
                bottom -= 1

            if left <= right:
                for row in range(bottom, top - 1, -1):
                    if self.is_cell_reachable(row, left):
                        path.append(self.get_linear_index(row, left))
                left += 1

        return path




grid = RobotTraversal((0, 0), (10, 10), rows=10, cols=10, min_radius=2.5)


path = grid.generate_full_reachable_path()
grid.visualize(path_points=path)


# grid = RobotGrid((0, 0), (2, 2), rows=6, cols=6, min_radius=0.3)

# print("Reachable cells:", grid.get_reachable_cells())
# grid.visualize()
