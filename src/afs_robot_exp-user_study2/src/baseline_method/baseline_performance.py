import os
from src.lrtdp import lrtdp
from src.helper_functions import readMDPfile
from src.kitchen_mdp import ArmMDP
from src.baseline_method.helper_functions import *
from src.metrics import *

def get_performance(send_traj, best_theta, num_feedback, selected_feedback_id, output_dir, filename):

    for test_map_name in [1]:
        test_grid = readMDPfile(filename, test_map_name, is_oracle=False)
        test_grid = ArmMDP(grid=test_grid, incomplete_reward=True)
        test_grid.reset()
        test_grid.is_baseline = True
        all_states = test_grid.get_states()
        test_grid.is_oracle=False

        test_grid.calibration_reward = best_theta
        test_grid_agent_policy = lrtdp(test_grid, is_oracle=False, use_cache=True)
        # print(test_grid_agent_policy)
        # _, naive_agent_traj = simulate_trajectory(test_grid, test_grid_agent_policy)
        # traj_id = []
        # for s, a in naive_agent_traj:
        #     s_id = all_states.index(s) if s in all_states else 0
        #     traj_id.append(s_id)
        # send_traj(traj_id)

        selected_feedback = get_feedback_methods()[selected_feedback_id]

        num_severe_nse, num_mild_nse, _, _, mean_reward, reward_std, all_trajs = get_nse_encountered(test_grid, test_grid_agent_policy)

        os.makedirs(output_dir, exist_ok=True)

        to_write = ['map_name', 'num_feedback', 'selected_feedback', 'theta_est', 'avg_severe_nse', 'severe_nse_sd', 'avg_mild_nse', 'mild_nse_sd']
        output_file = os.path.join(output_dir, 'main_approach_test_grid.csv')

        if not os.path.exists(output_file):
            with open(output_file, 'w') as f:
                f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(*to_write))
        with open(output_file, 'a') as f:
            f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(
                test_map_name, num_feedback, selected_feedback, best_theta,
                np.mean(num_severe_nse), np.std(num_severe_nse),
                np.mean(num_mild_nse), np.std(num_mild_nse)
            ))
        f.close()

        with open(output_dir+'/all_trajectories_baseline.csv', 'a') as f:
            if os.stat(output_dir+'/all_trajectories_baseline.csv').st_size == 0:
                f.write("{},{}\n".format('env', 'iteration', 'traj'))
            for idx in range(len(all_trajs)):
                f.write("{},{}\n".format(test_map_name, all_trajs[idx]))
            f.close()

        to_write = ['map_name', 'num_feedback', 'selected_feedback', 'theta_est', 'mean_reward', 'reward_std']
        output_file = os.path.join(output_dir, 'all_methods_rewards.csv')
        if not os.path.exists(output_file):
            with open(output_file, 'w') as f:
                f.write("{}, {}, {}, {}, {}, {}\n".format(*to_write))
        with open(output_file, 'a') as f:
            f.write("{}, {}, {}, {}, {}, {}\n".format(
                test_map_name, num_feedback, selected_feedback, best_theta, mean_reward, reward_std)
            )
        f.close()
