from feedback.helper_functions import get_all_sa_pairs
from baseline.helper_functions import *
from baseline.feedback_methods import *
from baseline.active_learning import softmax_probability, prior_distribution
from fmdp.lrtdp_new import lrtdp

def update_beliefs_with_feedback(grid, selected_feedback_id, beta, oracle_policy, agent_policy, num_feedback, unknown_theta):
    selected_feedback = get_feedback_methods()[selected_feedback_id]
    print('selected feedback: ', selected_feedback)

    neg_log_likelihoods = []

    state_action_labels = None
    all_sa_pairs = get_all_sa_pairs(grid)
    if selected_feedback=='correction':
        state_action_labels = get_corrections(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
    elif selected_feedback=='approval':
        state_action_labels = get_approval(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
    elif selected_feedback=='dam':
        state_action_labels = get_demo_action_mismatch(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs)
    state_action_pairs = get_preferred_state_action_pairs(state_action_labels)

    # print('unknown theta: ', unknown_theta)
    for t in unknown_theta:
        grid.calibration_reward = t
        grid.get_all_reward = True
        _ = lrtdp(grid, is_oracle=False)

        nll = 0  # initialize negative log likelihood
        for state, actions in state_action_pairs.items():
            _, state_probs = softmax_probability(grid, state, actions, beta[selected_feedback_id], t, unknown_theta)
            for sa_prob in state_probs:
                nll -= np.log(sa_prob)

        neg_log_likelihoods.append(nll)

    log_prior = np.log(prior_distribution(unknown_theta, len(unknown_theta)))
    log_posterior = -np.array(neg_log_likelihoods) + log_prior
    print('log post: ', log_posterior)

    best_theta_value = unknown_theta[np.argmax(log_posterior)]

    return best_theta_value
