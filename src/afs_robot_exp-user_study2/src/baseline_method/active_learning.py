from feedback.helper_functions import get_all_sa_pairs
from baseline.helper_functions import *
from baseline.feedback_methods import *
from fmdp.lrtdp_new import lrtdp
import numpy as np
np.set_printoptions(precision=15)

# Define your prior distribution
def prior_distribution(theta, total_theta_values):
    print("prior_dist in active_learning")
    prior = 1.0 / total_theta_values
    return prior

def softmax_probability(grid, state, actions, beta, t, unknown_theta):
    print("softmax_probability in active_learning")
    log_prob_sum = 0
    state_probs = []
    # all_actions_in_state = [action_val for (state_, action_val) in grid.agent_q_values.keys() if state_ == state]
    denominator = np.sum([np.exp(-beta * grid.agent_q_values[(state, a_prime)]) for a_prime in actions])
    for a in actions:
        numerator = np.exp(-beta * grid.agent_q_values[(state, a)])
        prob = numerator / denominator
        state_probs.append(prob)
        log_prob_sum -= np.log(prob)

    return log_prob_sum, state_probs


# Calculate expected negative log likelihood for a given feedback type x
def expected_nll(grid, state_action_pairs, unknown_theta, beta):
    total = 0.0
    for t in unknown_theta:
        grid.calibration_reward = t
        grid.get_all_reward = True
        _ = lrtdp(grid, is_oracle=False)
        theta_nll_sum = 0.0
        for state, actions in state_action_pairs.items():
            nll_sum, _ = softmax_probability(grid, state, actions, beta, t, unknown_theta)
            theta_nll_sum += nll_sum
        total += theta_nll_sum
    return np.mean(total)

# Select the most informative feedback type
def active_feedback_selection(grid, beta, oracle_policy, agent_policy, num_feedback, unknown_theta):
    min_nll = float('inf')
    best_feedback = None

    all_sa_pairs = get_all_sa_pairs(grid)
    feedback_methods = get_feedback_methods()

    for i, method in enumerate(feedback_methods):
        if method=='correction':
            state_action_labels = get_corrections(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
        elif method=='approval':
            state_action_labels = get_approval(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
        elif method=='dam':
            state_action_labels = get_demo_action_mismatch(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
        state_action_pairs = get_preferred_state_action_pairs(state_action_labels)

        nll = expected_nll(grid, state_action_pairs, unknown_theta, beta[i])
        if nll < min_nll:
            min_nll = nll
            best_feedback = i

    return best_feedback
