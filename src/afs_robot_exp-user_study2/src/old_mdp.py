import numpy as np

class ArmMDP:
    def __init__(self, grid_size=15, table_bounds=[[0.2, 0.8], [-0.3, 0.3]], goal=None, obstacles=set(), incomplete_reward=True):
        """
        Initialize the MDP with factored state representation <x, y, obstacle>.

        Parameters:
        - grid_size: Size of the discretized grid.
        - table_bounds: The physical boundaries of the table.
        - goal: The goal position as a tuple (x, y).
        - obstacles: A set of obstacle positions.
        """
        self.grid_size = grid_size
        self.table_bounds = table_bounds
        self.goal = goal if goal else (grid_size - 1, grid_size // 2)  # Default to top-center
        self.obstacles = obstacles
        self.incomplete_reward = incomplete_reward

        # Define actions: Left, Right, Up, Down
        self.actions = {
            0: (0, +1), # Down
            1: (0, -1), # Up
            2: (-1, 0), # Left
            3: (+1, 0)  # Right
        }

        self.step_size = (table_bounds[0][1] - table_bounds[0][0]) / grid_size

    def get_states(self):
        """Returns all possible states in the factored representation (x, y, obstacle)."""
        states = []
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                if (x, y) not in self.obstacles:
                    obstacle = 0
                else:
                    obstacle = 1
                states.append((x, y, obstacle))
        return states

    def get_reward(self, state):
        """Compute the reward for a given state <x, y, obstacle>."""
        x, y, obstacle = state
        if (x, y) == self.goal:
            return 100
        elif obstacle == 1:
            if self.incomplete_reward:
                return -1
            else:
                return -20
        else:
            return -1

    def transition_function(self, state, action):
        """
        Computes the next state given a state <x, y, obstacle> and an action.

        Parameters:
        - state: The current (x, y, obstacle) tuple.
        - action: The action index (0=Left, 1=Right, 2=Up, 3=Down).

        Returns:
        - next_state: The new (x, y, obstacle) tuple after taking the action.
        """
        x, y, _ = state  # Ignore the current obstacle value (recompute below)

        dx, dy = self.actions[action]
        next_x, next_y = x + dx, y + dy

        # Ensure next state is within bounds
        if 0 <= next_x < self.grid_size and 0 <= next_y < self.grid_size:
            obstacle = 1 if (next_x, next_y) in self.obstacles else 0  # Update obstacle presence
            return (next_x, next_y, obstacle)

        return (x, y, 0)  # Stay in place if the move is invalid

    def get_successors(self, state):
        """
        Returns all possible successor states from a given state.

        Parameters:
        - state: The current (x, y, obstacle) position.

        Returns:
        - List of tuples [(next_state, action, probability)].
        """
        successors = []
        for action in self.actions:
            next_state = self.transition_function(state, action)
            successors.append((next_state, action, 1.0))  # Deterministic MDP
        return successors

    def step(self, state, action):
        """
        Takes a step in the environment.

        Parameters:
        - state: The current (x, y, obstacle) position.
        - action: The action index.

        Returns:
        - next_state: The new (x, y, obstacle) position.
        - reward: The reward for the transition.
        - done: Whether the episode is finished.
        """
        next_state = self.transition_function(state, action)
        reward = self.get_reward(next_state)
        done = (next_state[0], next_state[1]) == self.goal
        return next_state, reward, done

    def continuous_position(self, grid_pos):
        """
        Convert discrete grid position back to continuous (x, y, z).
        The z-value is now fixed to match the object's height.
        """
        x = self.table_bounds[0][0] + grid_pos[0] * self.step_size
        y = self.table_bounds[1][0] + grid_pos[1] * self.step_size
        return [x, y, 0.7]  # Matching object height
