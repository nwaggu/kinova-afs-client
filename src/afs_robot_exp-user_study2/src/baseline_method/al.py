import numpy as np
from src.lrtdp import lrtdp
from src.baseline_method.helper_functions import *
from src.baseline_method.feedback_methods import *

REGULARIZATION_THRESHOLD = 0.0001
REGULARIZATION_PENALTY = 0.001
def apply_regularization(grid, state, actions):
    actions_list = list(actions)
    q_values = [grid.agent_q_values[(state, a)] for a in actions_list]
    for i, q1 in enumerate(q_values):
        for j, q2 in enumerate(q_values):
            if i != j and abs(q1 - q2) < REGULARIZATION_THRESHOLD:
                grid.agent_q_values[(state, actions_list[i])] += REGULARIZATION_PENALTY
                grid.agent_q_values[(state, actions_list[j])] += REGULARIZATION_PENALTY


def softmax_probability(grid, state, actions, beta, t, unknown_theta):
    numerator = 0
    prob_sum = 0
    state_probs = []
    denominator = np.sum([np.exp(-beta * grid.agent_q_values[(state, a_prime)]) for a_prime in actions])
    for a in actions:
        numerator = np.exp(-beta * grid.agent_q_values[(state, a)])
        prob = numerator / denominator
        state_probs.append(prob)
        prob_sum += prob * np.log(prob / prior_distribution(t, len(unknown_theta)))

    return prob_sum, state_probs

# Define your prior distribution
def prior_distribution(theta, total_theta_values):
    prior = np.ones_like(theta) / total_theta_values
    return prior

# Calculate expected KL-divergence for a given feedback type x
def expected_kl_divergence(grid, unknown_theta, all_beta):
    max_ekl = float('-inf')
    total = 0.0
    for t_idx, t in enumerate(unknown_theta):

        all_sa_pairs = get_all_sa_pairs(grid)
    feedback_methods = get_feedback_methods()

    for i, method in enumerate(feedback_methods):
        beta = all_beta[i]
        if method=='correction':
            # state_action_labels = get_correction(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{t_idx}_correction.txt"
            if not os.path.exists(filename):
                print(f"File {filename} does not exist.")
                return {}
            state_action_labels = {}
            with open(filename, 'r') as file:
                lines = file.readlines()
                for line in lines[1:]:  # Skip first line (header)
                    parts = line.strip().split(', ')
                    if len(parts) == 3:
                        state, action, label = parts
                        state_action_labels[(state, action)] = label

        elif method=='approval':
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{t_idx}_approval.txt"
            if not os.path.exists(filename):
                print(f"File {filename} does not exist.")
                return {}
            state_action_labels = {}
            with open(filename, 'r') as file:
                lines = file.readlines()
                for line in lines[1:]:  # Skip first line (header)
                    parts = line.strip().split(', ')
                    if len(parts) == 3:
                        state, action, label = parts
                        state_action_labels[(state, action)] = label
        elif method=='dam':
            # state_action_labels = get_demo_action_mismatch(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{t_idx}_dam.txt"
            if not os.path.exists(filename):
                print(f"File {filename} does not exist.")
                return {}
            state_action_labels = {}
            with open(filename, 'r') as file:
                lines = file.readlines()
                for line in lines[1:]:  # Skip first line (header)
                    parts = line.strip().split(', ')
                    if len(parts) == 3:
                        state, action, label = parts
                        state_action_labels[(state, action)] = label
        elif method=='rank':
            # state_action_labels = get_rank(objectUids, kinova, grid, num_feedback, all_sa_pairs)
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{t_idx}_rank.txt"
            if not os.path.exists(filename):
                print(f"File {filename} does not exist.")
                return {}
            state_action_labels = {}
            with open(filename, 'r') as file:
                lines = file.readlines()
                for line in lines[1:]:  # Skip first line (header)
                    parts = line.strip().split(', ')
                    if len(parts) == 3:
                        state, action, label = parts
                        state_action_labels[(state, action)] = label
        state_action_pairs = get_preferred_state_action_pairs(state_action_labels)



        grid.calibration_reward = t
        grid.get_all_reward = True
        _ = lrtdp(grid, is_oracle=False)
        theta_prob_sum = 0.0
        for state, actions in state_action_pairs.items():
            # apply_regularization(grid, state, actions)
            prob_sum, _ = softmax_probability(grid, state, actions, beta, t, unknown_theta)
            theta_prob_sum += prob_sum
        total += theta_prob_sum
        ekl = total / len(unknown_theta)
        if ekl > max_ekl:
            max_ekl = ekl
            best_feedback = i
    return best_feedback

# Select the most informative feedback type
def active_feedback_selection(start_compliance, end_compliance, to_state, objectUids, kinova, grid, beta, oracle_policy, agent_policy, num_feedback, unknown_theta):
    best_feedback = None

    # all_sa_pairs = get_all_sa_pairs(grid)
    # feedback_methods = get_feedback_methods()

    # for i, method in enumerate(feedback_methods):
    #     if method=='correction':
    #         # state_action_labels = get_correction(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
    #         filename = f"ri_callibration_feedback/0_correction.txt"
    #         if not os.path.exists(filename):
    #             print(f"File {filename} does not exist.")
    #             return {}
    #         state_action_labels = {}
    #         with open(filename, 'r') as file:
    #             lines = file.readlines()
    #             for line in lines[1:]:  # Skip first line (header)
    #                 parts = line.strip().split(', ')
    #                 if len(parts) == 3:
    #                     state, action, label = parts
    #                     state_action_labels[(state, action)] = label

    #     elif method=='approval':
    #         filename = f"ri_callibration_feedback/0_approval.txt"
    #         if not os.path.exists(filename):
    #             print(f"File {filename} does not exist.")
    #             return {}
    #         state_action_labels = {}
    #         with open(filename, 'r') as file:
    #             lines = file.readlines()
    #             for line in lines[1:]:  # Skip first line (header)
    #                 parts = line.strip().split(', ')
    #                 if len(parts) == 3:
    #                     state, action, label = parts
    #                     state_action_labels[(state, action)] = label
    #     elif method=='dam':
    #         # state_action_labels = get_demo_action_mismatch(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
    #         filename = f"ri_callibration_feedback/0_dam.txt"
    #         if not os.path.exists(filename):
    #             print(f"File {filename} does not exist.")
    #             return {}
    #         state_action_labels = {}
    #         with open(filename, 'r') as file:
    #             lines = file.readlines()
    #             for line in lines[1:]:  # Skip first line (header)
    #                 parts = line.strip().split(', ')
    #                 if len(parts) == 3:
    #                     state, action, label = parts
    #                     state_action_labels[(state, action)] = label
    #     elif method=='rank':
    #         # state_action_labels = get_rank(objectUids, kinova, grid, num_feedback, all_sa_pairs)
    #         filename = f"ri_callibration_feedback/0_rank.txt"
    #         if not os.path.exists(filename):
    #             print(f"File {filename} does not exist.")
    #             return {}
    #         state_action_labels = {}
    #         with open(filename, 'r') as file:
    #             lines = file.readlines()
    #             for line in lines[1:]:  # Skip first line (header)
    #                 parts = line.strip().split(', ')
    #                 if len(parts) == 3:
    #                     state, action, label = parts
    #                     state_action_labels[(state, action)] = label
    #     state_action_pairs = get_preferred_state_action_pairs(state_action_labels)


    best_feedback = expected_kl_divergence(grid, unknown_theta, beta)
        # if ekl > max_ekl:
        #     max_ekl = ekl
        #     best_feedback = i
    return best_feedback
