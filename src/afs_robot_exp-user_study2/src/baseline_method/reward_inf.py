from src.feedback.helper_functions import get_all_sa_pairs
from src.baseline_method.helper_functions import *
from src.baseline_method.feedback_methods import *
from src.baseline_method.al import softmax_probability, prior_distribution
from src.lrtdp import lrtdp
from decimal import Decimal


def update_beliefs_with_feedback(start_compliance, end_compliance, to_state, objectUids, kinova, grid, selected_feedback_id, beta, oracle_policy, agent_policy, num_feedback, unknown_theta):
    selected_feedback = get_feedback_methods()[selected_feedback_id]
    print('selected feedback: ', selected_feedback)

    log_likelihoods = []

    state_action_labels = None
    all_sa_pairs = get_all_sa_pairs(grid)
    if selected_feedback=='correction':
        # state_action_labels = get_correction(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
        filename = f"stochastic_output/new_setup_ri_callibration_feedback/0_correction.txt"
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

    elif selected_feedback=='approval':
        # state_action_labels = get_approval(objectUids, kinova, grid, num_feedback, all_sa_pairs)
        filename = f"stochastic_output/new_setup_ri_callibration_feedback/0_approval.txt"
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


    # elif method=='annotated_correction':
    #     state_action_labels = get_annotated_correction(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
    # elif method=='annotated_approval':
    #     state_action_labels = get_annotated_approval(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)

    elif selected_feedback=='dam':
        # state_action_labels = get_demo_action_mismatch(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
        filename = f"stochastic_output/new_setup_ri_callibration_feedback/0_dam.txt"
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

    elif selected_feedback=='rank':
        # state_action_labels = get_rank(objectUids, kinova, grid, num_feedback, all_sa_pairs)
        filename = f"stochastic_output/new_setup_ri_callibration_feedback/0_rank.txt"
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

    # Calculate likelihood for each potential theta value
    # print('unknown theta: ', unknown_theta)
    for t in unknown_theta:
        grid.calibration_reward = t
        grid.get_all_reward = True
        _ = lrtdp(grid, is_oracle=False)

        SMOOTHING_EPSILON = 1e-10

        log_likelihood = 0  # initialize log_likelihood
        for state, actions in state_action_pairs.items():
            _, state_probs = softmax_probability(grid, state, actions, beta[selected_feedback_id], t, unknown_theta)
            # print('state probs: ', state_probs)
            for sa_prob in state_probs:
                # Apply smoothing
                sa_prob = max(sa_prob, SMOOTHING_EPSILON)
                log_likelihood += np.log(sa_prob)

        log_likelihoods.append(log_likelihood)

    likelihoods = np.exp(log_likelihoods - np.max(log_likelihoods))  # Use log-sum-exp trick to avoid underflow
    posterior = likelihoods / np.sum(likelihoods)
    best_theta_value = unknown_theta[np.argmax(posterior)]

    return best_theta_value
