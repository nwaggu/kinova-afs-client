from src.lrtdp import lrtdp
import sys
from src.baseline_method.beta_estimation import estimate_beta
from src.baseline_method.baseline_performance import get_performance
from src.baseline_method.helper_functions import get_random_theta
from src.baseline_method.al import active_feedback_selection
from src.baseline_method.reward_inf import update_beliefs_with_feedback
from src.metrics import *

def ri(start_compliance, end_compliance, send_traj, to_state, pb, objectUids, kinova, train_grid, output_dir, baseline, test_map_names, filename):
    num_feedback = [5]
    unknown_theta = get_random_theta(train_grid, n=5)

    ri_unknown_theta = get_random_theta(train_grid, n=1)[0]

    train_grid.calibration_reward = ri_unknown_theta
    train_grid.is_baseline = False
    oracle_policy = lrtdp(train_grid, is_oracle=True)
    agent_policy = lrtdp(train_grid, is_oracle=False)
    pb.start()

    for n_feedback in num_feedback:
        beta_cap = estimate_beta(start_compliance, end_compliance, to_state, objectUids, kinova, train_grid, n_feedback, unknown_theta)
        selected_feedback_id = active_feedback_selection(start_compliance, end_compliance, to_state, objectUids, kinova, train_grid, beta_cap, oracle_policy, agent_policy, 3, unknown_theta)
        # print('feedback selected: ', selected_feedback_id)
        best_theta_value = update_beliefs_with_feedback(start_compliance, end_compliance, to_state, objectUids, kinova, train_grid, selected_feedback_id, beta_cap, oracle_policy, agent_policy, n_feedback, unknown_theta)
        # print('best theta: ', best_theta_value)
        get_performance(send_traj, best_theta_value, n_feedback, selected_feedback_id, output_dir, filename)
