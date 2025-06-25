import numpy as np

class ArmMDP:
    # original
    # table_bounds=[[0.005, 0.657], [0.276, 0.793]]
    # initial setup
    # table_bounds=[[0.2, 0.8], [-0.3, 0.3]]
    def __init__(self, table_bounds=[[0.2, 0.8], [-0.3, 0.3]], grid=None, incomplete_reward=False):
        self.table_bounds = table_bounds
        self.incomplete_reward = incomplete_reward
        self.action_mapping = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
        self.actions = {
            0: (-1, 0), # Up
            1: (+1, 0), # Down
            2: (0, -1), # Left
            3: (0, +1),  # Right
        }
        self.num_actions = len(self.actions)
        self.grid = grid = np.asarray(grid, dtype='c')
        self.grid_list = [[c.decode('utf-8') for c in line] for line in self.grid.tolist()]
        self.n_rows = len(self.grid)
        self.n_cols = len(self.grid[0])
        self.col_step_size = (table_bounds[1][1] - table_bounds[1][0]) / self.n_cols  # Height step
        self.row_step_size = (table_bounds[0][1] - table_bounds[0][0]) / self.n_rows  # Width step
        self.num_states = self.n_rows * self.n_cols
        self.goal = ((np.where(self.grid == b'G')[0].item(), np.where(self.grid == b'G')[1].item(), 0, 0))
        self.agent_reward_cache = {}
        self.learned_reward_cache = {}
        self.oracle_demos = {}
        self.critical_state_preds = {}
        self.reward_cache = {}
        self.oracle_q_values = {}
        self.agent_q_values = {}
        self.is_oracle = None
        self.calibration_reward = None
        self.is_baseline = False
        self.get_all_reward = False
        self.start_state = ((np.where(self.grid == b'A')[0].item(), np.where(self.grid == b'A')[1].item(), 0, 0))
        self.all_states = self.get_states()
        self.domain = 'kitchen_arm'
        self.is_baseline = False
        self.reset()

    def reset(self):
        self.state = self.start_state

    def get_states(self):
        """
        Returns all possible states in factored representation.
        States can be represented as follows depending on the domain description:
            - Kitchen task:
                (x, y, object type, object height);
                object type: 0/1/2 (no object, mat, object with some height); object height: 0/1/2/3 (no height, negligible height, height 1, height 2);
                mild NSE: (x, y, 2, 2), severe NSE: (x, y, 2, 3), all other cases are no NSE.
        """
        states = []
        for y in range(self.n_rows):
            for x in range(self.n_cols):
                currState = self.grid_list[y][x]
                if currState == 'O':
                    obstacle = 1
                    height = 1
                elif currState == 'M':
                    obstacle = 2
                    height = 2
                elif currState == 'S':
                    obstacle = 2
                    height = 3
                elif currState == '#':
                    continue
                else:
                    obstacle = 0
                    height = 0
                states.append((y, x, obstacle, height))
        return states

    def get_reward(self, state, action, is_oracle=None, is_lrtdp=False):
        if is_oracle==True:
            self.incomplete_reward = False
        elif is_oracle==False:
            self.incomplete_reward = True

        if is_lrtdp and self.is_baseline==False:
            if state[0] == self.goal[0] and state[1] == self.goal[1]:
                return 0
            # Mild NSE (object with height 1)
            elif state[2] == 2 and state[3] == 2:
                if self.incomplete_reward:
                    return 1
                elif self.is_oracle:
                    return 5
            # Severe NSE (object with height 2)
            elif state[2] == 2 and state[3] == 3:
                if self.incomplete_reward:
                    return 1
                elif self.is_oracle:
                    return 10
            else:
                return 1
        elif self.is_baseline:
            x, y, obstacle, height = state
            if (x, y) == (self.goal[0], self.goal[1]):
                return 0
            else:
                if self.calibration_reward is not None:
                    r = self.calibration_reward
                rew = 1
                rew += (r[0]*obstacle) + (r[1]*height)
                return rew

        # else:
        #     x, y, obstacle, surface = state
        #     if (x, y) == (self.goal[0], self.goal[1]):
        #         return 100
        #     elif obstacle == 1 and surface == 0:
        #         if self.incomplete_reward:
        #             return -1
        #         else:
        #             return -5
        #     elif obstacle == 1 and surface == 1:
        #         if self.incomplete_reward:
        #             return -1
        #         else:
        #             return -10
        #     else:
        #         return -1

    # def transition_function(self, state, action):
    #     x, y, obstacle, surface = state  # Ignore the current obstacle value (recompute below)

    #     dx, dy = self.actions[action]
    #     next_x, next_y = x + dx, y + dy

    #     # Ensure next state is within bounds
    #     if 0 <= next_x < self.n_rows and 0 <= next_y < self.n_cols:
    #         if self.grid_list[next_x][next_y] == 'O':
    #             return (next_x, next_y, 1, 0)
    #         elif self.grid_list[next_x][next_y] == '$':
    #             return (next_x, next_y, 1, 1)
    #         else:
    #             return (next_x, next_y, 0, 0)

    #     return (x, y, obstacle, surface)  # Stay in place if the move is invalid

    def get_actions(self, state):
        return [0, 1, 2, 3]

    def is_boundary(self, state):
        x, y = state
        return (x < 0 or x > self.n_rows-1 or y < 0 or y > self.n_cols-1 )

    def is_goal(self, state):
        if state[0] == self.goal[0] and state[1] == self.goal[1]:
            return True
        return False

    def getActionFactorRep(self, a):
        if a == 0: # up
            return (-1, 0)
        elif a == 1: # down
            return (1, 0)
        elif a == 2: # left
            return (0, -1)
        else: # right
            return (0, 1)

    def move(self, currFactoredState, action):
        x, y, _, _ = currFactoredState
        new_state = tuple(x + y for (x, y) in zip((x, y), self.getActionFactorRep(action)))
        if self.is_boundary(new_state):
            # self.state = currFactoredState
            return currFactoredState, True
        else:
            if self.grid_list[new_state[0]][new_state[1]] == 'O':
                return (new_state[0], new_state[1], 1, 1), False
            elif self.grid_list[new_state[0]][new_state[1]] == 'M':
                return (new_state[0], new_state[1], 2, 2), False
            elif self.grid_list[new_state[0]][new_state[1]] == 'S':
                return (new_state[0], new_state[1], 2, 3), False
            else:
                return (new_state[0], new_state[1], 0, 0), False

    def get_side_states(self, state, action):
        side_states =[]
        for a in range(self.num_actions):
            if a != action:
                new_state, is_wall = self.move(state, a)
                if not is_wall:
                    side_states.append(new_state)
                elif is_wall:
                    side_states.append(state)
        return side_states

    def get_transition(self, curr_state, action, next_state):
        succ_factored_state, is_wall = self.move(curr_state, action)
        # print("\ncurr_state: {}, next_state: {}, succ_factored_state: {}".format(curr_state, next_state, succ_factored_state))
        sstates = self.get_side_states(curr_state, action)
        # print("sstates: {}".format(sstates))

        success_prob = 0.8
        fail_prob = 0.2/3

        if is_wall:
            # print("hit boundary")
            transition_probs = []
            for feature_idx in range(len(curr_state)):
                if (curr_state[feature_idx] == next_state[feature_idx]):
                    transition_probs.append(1)
                else:
                    transition_probs.append(0)
            return np.prod(transition_probs)

        elif not is_wall:
            # print("no boundary")
            transition_probs = []
            if (next_state[0]==succ_factored_state[0] and next_state[1]==succ_factored_state[1]):
                transition_probs.append(success_prob)
                if (next_state[2]==succ_factored_state[2]):
                    transition_probs.append(1)
                elif (next_state[2]!=succ_factored_state[2]):
                    transition_probs.append(0)
                return np.prod(transition_probs)

            # print('side states: ', sstates)
            for side_state in sstates:
                if (next_state[0]==side_state[0] and next_state[1]==side_state[1]):
                    # print(sstates)
                    # print("if condn {}".format(side_state))
                    state_count = sstates.count(next_state)
                    fail_prob *= state_count
                    transition_probs.append(fail_prob)
                    if (next_state[2]==side_state[2]):
                        transition_probs.append(1)
                    elif (next_state[2]!=side_state[2]):
                        transition_probs.append(0)
                    return np.prod(transition_probs)

        return 0

    def get_possible_next_states(self, state):
        possible_states = set()
        for action in range(self.num_actions):
            next_state, _ = self.move(state, action)
            possible_states.add(next_state)
        # print("state: {}, possible_states: {}".format(state, possible_states))
        return possible_states

    def get_successors(self, state, action):
        successors, succ_probabilities = [], []
        for next_state in self.get_possible_next_states(state):
            p = self.get_transition(state, action, next_state)
            if p > 0:
                successors.append(next_state)
                succ_probabilities.append(p)
        # print("state: {}, action: {}, successors: {}, succ_probabilities: {}".format(state, action, successors, succ_probabilities))
        return successors, succ_probabilities


    # def get_successors(self, state, action):
    #     successors = []
    #     next_state = self.transition_function(state, action)
    #     successors.append(next_state)  # Deterministic MDP
    #     return successors

    # def step(self, state, action):
    #     next_state = self.transition_function(state, action)
    #     reward = self.get_reward(next_state, action, is_oracle=self.is_oracle, is_lrtdp=True)
    #     done = (next_state[0], next_state[1]) == (self.goal[0], self.goal[1])
    #     return next_state, reward, done

    def step(self, state, action, evaluate=False):
        terminal = False
        successors, succ_probabilities = self.get_successors(state, action)
        # np.random.seed(self.seed)
        next_state_idx = np.random.choice(len(successors), p=succ_probabilities)
        next_state = successors[next_state_idx]
        if (next_state, action) in self.agent_reward_cache:
            reward = self.agent_reward_cache[(next_state, action)]
        else:
            reward = self.get_reward(next_state, action, self.is_oracle, is_lrtdp=True)
        if self.is_goal(next_state):
            terminal = True
        return successors[next_state_idx], reward, succ_probabilities[next_state_idx], terminal

    def continuous_position(self, grid_pos):
        """
        Convert discrete grid position back to continuous (x, y, z).
        The z-value is now fixed to match the object's height.
        """
        x = self.table_bounds[0][0] + (grid_pos[0] + 0.5) * self.row_step_size
        y = self.table_bounds[1][0] + (grid_pos[1]+ 0.5) * self.col_step_size
        # use 0.2 for the arm. In simulation, use 0.7
        return [x, y, 0.8]
