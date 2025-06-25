from src.main_approach.reward_pred_model import get_state_action_successors
from src.feedback.helper_functions import *
from src.helper_functions import readMDPfile
from src.kitchen_mdp import ArmMDP
from src.main_approach.helper_functions import update_agent_policy
from src.metrics import *
import numpy as np, os

def pred_test_grid(send_traj, test_map_names, model, output_dir, filename, iterations):
    to_write = ['iterations', 'map_name', 'avg_severe_nse', 'severe_nse_sd', 'avg_mild_nse', 'mild_nse_sd']

    is_oracle = False
    incomplete_reward = True
    for test_map_name in test_map_names:
        print('Predicting for test grid', test_map_name, '...')
        grid = readMDPfile(filename, test_map_name, is_oracle=False)
        test_grid = ArmMDP(grid=grid, incomplete_reward=True)
        test_grid.reset()
        all_test_state_action_pairs = get_state_action_successors(test_grid)
        f1_calculation_data = []
        all_states = test_grid.get_states()

        state_pred, test_grid_agent_policy = update_agent_policy(test_grid, model)
        # print(test_grid_agent_policy)
        # for i in range(10):
        #     _, naive_agent_traj = simulate_trajectory(test_grid, test_grid_agent_policy)
        #     traj_id = []
        #     for s, a in naive_agent_traj:
        #         s_id = all_states.index(s) if s in all_states else 0
        #         traj_id.append(s_id)
        #     send_traj(traj_id)


        pred_fn = output_dir+'/test_grid_predictions_'+str(test_map_name)+'.csv'
        with open(pred_fn, 'a') as f:
            if os.stat(pred_fn).st_size == 0:
                f.write("{},{},{},{}\n".format('iteration','state', 'action', 'pred'))

            for i, sa_pair in enumerate(all_test_state_action_pairs):
                state, action = sa_pair[0], sa_pair[1]
                f1_calculation_data.append((iterations, 'our_approach', state, action, state_pred[i]))

                f.write("{},{},{},{}\n".format(iterations, state, action, state_pred[i]))
        f.close()

        num_severe_nse, num_mild_nse, trajectory_lengths, _, mean_reward, reward_std, all_trajectories = get_nse_encountered(test_grid, test_grid_agent_policy)
        f1_score, accuracy = get_f1_and_accuracy(test_grid, f1_calculation_data)

        with open(output_dir+'/main_approach_test_grid.csv', 'a') as f:
            if os.stat(output_dir+'/main_approach_test_grid.csv').st_size == 0:
                f.write("{},{},{},{},{},{}\n".format(*to_write))
            f.write("{},{},{},{},{},{}\n".format(iterations, test_map_name, np.mean(num_severe_nse), np.std(num_severe_nse), np.mean(num_mild_nse), np.std(num_mild_nse)))
            f.close()

        with open(output_dir+'/nse_count.csv', 'a') as f:
            if os.stat(output_dir+'/nse_count.csv').st_size == 0:
                f.write("{},{},{},{},{}\n".format('env', 'iteration', '#severe_nse', '#mild_nse', 'traj_len'))
            for idx in range(len(trajectory_lengths)):
                f.write("{},{},{},{},{}\n".format(test_map_name, iterations, num_severe_nse[idx], num_mild_nse[idx], trajectory_lengths[idx]))
            f.close()

        with open(output_dir+'/all_trajectories_'+str(iterations)+'.csv', 'a') as f:
            if os.stat(output_dir+'/all_trajectories_'+str(iterations)+'.csv').st_size == 0:
                f.write("{},{},{}\n".format('env', 'iteration', 'traj'))
            for idx in range(len(all_trajectories)):
                f.write("{},{},{}\n".format(test_map_name, iterations, all_trajectories[idx]))
            f.close()

        with open(output_dir+'/f1_and_accuracy.csv', 'a') as f1:
            if os.stat(output_dir+'/f1_and_accuracy.csv').st_size == 0:
                f1.write("{},{},{},{},{}\n".format('iteration', 'method', 'map_name', 'f1_score', 'accuracy'))
            f1.write("{},{},{},{},{}\n".format(iterations, 'our_approach', test_map_name, f1_score, accuracy))
            f1.close()

        with open(output_dir+'/all_methods_rewards.csv', 'a') as f:
            if os.stat(output_dir+'/all_methods_rewards.csv').st_size == 0:
                f.write('iterations, test_map_name, mean_reward, reward_std\n')
            f.write('{}, {}, {}, {}\n'.format(iterations, test_map_name, mean_reward, reward_std))
            f.close()

        print('Prediction on test grid', test_map_name, 'complete!')
