import numpy as np

def value_iteration(env):
    """
    Performs Value Iteration on the factored MDP environment.

    Parameters:
    - env: An instance of `ArmMDP`.

    Returns:
    - policy: A policy mapping (x, y) to optimal actions.
    """
    ACTIONS = env.actions
    states = env.get_states()
    V = np.zeros((env.n_rows, env.n_cols))  # Value function
    policy = np.zeros((env.n_rows, env.n_cols), dtype=int)  # Policy mapping (x, y) → action

    GAMMA = 0.9  # Discount factor
    THRESHOLD = 0.01  # Convergence threshold

    while True:
        delta = 0
        new_V = np.copy(V)

        for state in states:
            x, y, obstacle, surface = state  # Factored state representation
            # x, y, _ = state

            if (x, y) == (env.goal[0], env.goal[1]):
                continue  # Goal state is terminal

            max_value = float('-inf')
            best_action = 0

            for a, (dx, dy) in ACTIONS.items():
                next_state = env.transition_function(state, a)  # Get next state
                nx, ny, next_obstacle, next_surface = next_state
                # nx, ny, _ = next_state
                # input()
                if (tuple(state), a) in env.agent_reward_cache:
                    reward = env.agent_reward_cache[(tuple(state), a)]
                else:
                    reward = env.get_reward(state, a)  # Compute reward

                value = reward + GAMMA * V[nx, ny]  # Bellman update
                if value > max_value:
                    max_value = value
                    best_action = a

            new_V[x, y] = max_value  # Update value function
            policy[x, y] = best_action  # Store best action
            delta = max(delta, abs(V[x, y] - new_V[x, y]))

        V = new_V
        if delta < THRESHOLD:
            break

    return policy
