import numpy as np

def get_calibration_rewards(grid, n):
    unknown_theta = []
    sampled_theta = np.random.rand(n, 1)
    for i in range(len(sampled_theta)):
        if grid.domain=='vase' or grid.domain=='outdoor':
            theta_1 = sampled_theta[i][0]
            theta_2 = np.sqrt(1 - np.square(theta_1))
            theta = [theta_1, theta_2]
        else:
            theta_1 = np.random.rand()  # Generate a random value between 0 and 1
            theta_2 = np.random.rand() * np.sqrt(1 - theta_1**2)  # Generate a random value between 0 and sqrt(1 - theta_1^2)
            theta_3 = np.sqrt(1 - theta_1**2 - theta_2**2)  # Calculate theta_3
            theta = [theta_1, theta_2, theta_3]
        unknown_theta.append(theta)
    return unknown_theta

def get_feedback_methods():
    # feedback_methods = ['correction', 'approval', 'dam']
    feedback_methods = ['rank', 'approval', 'dam', 'correction', ]
    return feedback_methods

def get_preferred_state_action_pairs(all_state_action_labels):
    preferred_state_action_pairs = {}
    for (state, action), label in all_state_action_labels.items():
        if state not in preferred_state_action_pairs:
            preferred_state_action_pairs[state] = set()
        if (label==0) and (action not in preferred_state_action_pairs[state]):
            preferred_state_action_pairs[state].add(action)
    return preferred_state_action_pairs

# def get_random_theta(grid, n):
#     np.random.seed(42)
#     unknown_theta = []
#     for _ in range(n):
#         values = np.random.rand(4)
#         values[0] = 1
#         values[1] = 0.5
#         values[2] = np.random.uniform(0, 0.5)
#         values[3] = np.random.uniform(0, 0.5)


#         values /= np.linalg.norm(values)
#         unknown_theta.append(values.tolist())
#     return unknown_theta

def get_random_theta(grid, n):
    # np.random.seed(42)
    unknown_theta = []
    p_zero = 0.3
    n_samples = (n, 1)
    sampled_theta = np.where(np.random.rand(*n_samples) < p_zero, 0, np.random.uniform(0, 1, n_samples))
    for i in range(len(sampled_theta)):
        theta_1 = sampled_theta[i][0]
        theta_2 = np.sqrt(1-np.square(theta_1))
        theta = [theta_1, theta_2]
        unknown_theta.append(theta)
    return unknown_theta