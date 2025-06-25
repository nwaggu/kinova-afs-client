from src.baseline_method.helper_functions import *
from src.baseline_method.feedback_methods import *
from src.lrtdp import lrtdp
import numpy as np
from scipy.optimize import minimize
from scipy.special import logsumexp

# Define the MLE objective function
def mle_objective(grid, beta, state_action_labels):
    beta_est_val = 0

    state_action_pairs = get_preferred_state_action_pairs(state_action_labels)
    for state, actions in state_action_pairs.items():
        print('state: ', state)
        # Compute the normalization term for the current state
        all_actions_in_state = [action_val for (state_, action_val) in grid.agent_q_values.keys() if state_ == state]
        normalization_term = logsumexp([-beta * grid.agent_q_values[(state, a)] for a in all_actions_in_state])
        # Add the log-likelihood for each observed action in the state
        for a in actions:
            beta_est_val += -beta * grid.agent_q_values[(state, a)] - normalization_term
        print('beta est: ', beta_est_val)

    return -beta_est_val

def fit_beta(grid, state_action_labels, beta):
    # Use an optimizer to find the beta that maximizes the MLE objective
    result = minimize(lambda beta: mle_objective(grid, beta[0], state_action_labels), [beta], bounds=[(0, None)])
    beta = result.x[0]
    return beta

def get_oracle_feedback_and_fit_beta(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, beta, num_feedback, cal_index):
    feedback_methods = get_feedback_methods()
    all_sa_pairs = get_all_sa_pairs(grid)

    for i, method in enumerate(feedback_methods):
        state_action_labels = None
        if method=='correction':
            # state_action_labels = get_correction(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{i}_correction.txt"
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
            # state_action_labels = get_approval(objectUids, kinova, grid, num_feedback, all_sa_pairs)
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{i}_approval.txt"
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
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{i}_dam.txt"
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
            filename = f"stochastic_output/new_setup_ri_callibration_feedback/{i}_rank.txt"
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


        # if state_action_labels is not None:
        #     filename = f"stochastic_output/new_setup_ri_callibration_feedback/{cal_index}_{method}.txt"
        #     os.makedirs(os.path.dirname(filename), exist_ok=True)
        #     if not os.path.exists(filename):
        #         with open(filename, 'w') as f:
        #             f.write("{}, {}, {}\n".format('state', 'action', 'label'))
        #     with open(filename, 'a') as file:
        #         for key, value in state_action_labels.items():
        #             file.write(f"{key[0]}, {key[1]}, {value}\n")
        #     file.close()

        beta[i] = fit_beta(grid, state_action_labels, beta[i])
    return beta

def estimate_beta(start_compliance, end_compliance, to_state, objectUids, kinova, grid, num_feedback, calibration_rewards):
    # calibration_rewards = get_calibration_rewards(grid, n=4)
    feedback_beta_cap = np.zeros(len(get_feedback_methods()))
    # i = 4
    # reward = calibration_rewards[i]

    for i, reward in enumerate(calibration_rewards):
        print(f'---------New reward function ({i+1}/{len(calibration_rewards)})---------')
        grid.calibration_reward = reward
        calibration_oracle_policy = lrtdp(grid, is_oracle=True)
        oracle_q_values = grid.oracle_q_values
        calibration_agent_policy = lrtdp(grid, is_oracle=False)
        grid.oracle_q_values = oracle_q_values
        feedback_beta_cap = get_oracle_feedback_and_fit_beta(start_compliance, end_compliance, to_state, objectUids, kinova, grid, calibration_oracle_policy, calibration_agent_policy, feedback_beta_cap, num_feedback, cal_index=i)
    return feedback_beta_cap # should contain 3 values, 1 beta for each feedback type
