import numpy as np, os
from src.helper_functions import simulate_trajectory
from pathlib import Path

def get_oracle_performance(mdp_env, oracle_policy, naive_policy, output_dir):
    to_write = ['map_name', 'avg_severe_nse', 'severe_nse_sd', 'avg_mild_nse', 'mild_nse_sd']
    num_severe_nse, num_mild_nse, trajectory_lengths, _, mean_reward, reward_std, o_all_trajs = get_nse_encountered(mdp_env, oracle_policy)

    filename = Path(output_dir+'/oracle_performance.csv')
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(output_dir+'/oracle_performance.csv', 'a') as f:
        if os.stat(output_dir+'/oracle_performance.csv').st_size == 0:
            f.write("{},{},{},{},{}\n".format(*to_write))
        f.write("{},{},{},{},{}\n".format(0, np.mean(num_severe_nse), np.std(num_severe_nse), np.mean(num_mild_nse), np.std(num_mild_nse)))
        f.close()
    
    with open(output_dir+'/all_trajectories_oracle.csv', 'a') as f:
        if os.stat(output_dir+'/all_trajectories_oracle.csv').st_size == 0:
            f.write("{},{}\n".format('env', 'traj'))
        for idx in range(len(o_all_trajs)):
            f.write("{},{}\n".format(0, o_all_trajs[idx]))
        f.close()

    num_severe_nse, num_mild_nse, trajectory_lengths, _, mean_reward, reward_std, na_all_trajs = get_nse_encountered(mdp_env, naive_policy)
    filename = Path(output_dir+'/naive_agent_performance.csv')
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(output_dir+'/naive_agent_performance.csv', 'a') as f:
        if os.stat(output_dir+'/naive_agent_performance.csv').st_size == 0:
            f.write("{},{},{},{},{}\n".format(*to_write))
        f.write("{},{},{},{},{}\n".format(0, np.mean(num_severe_nse), np.std(num_severe_nse), np.mean(num_mild_nse), np.std(num_mild_nse)))
        f.close()

    with open(output_dir+'/all_trajectories_naive_agent.csv', 'a') as f:
        if os.stat(output_dir+'/all_trajectories_naive_agent.csv').st_size == 0:
            f.write("{},{}\n".format('env', 'traj'))
        for idx in range(len(na_all_trajs)):
            f.write("{},{}\n".format(0, na_all_trajs[idx]))
        f.close()

def get_nse_encountered(grid, policy, send_traj=None, num_trajectories=10):
    times_visited = np.zeros((grid.n_rows, grid.n_cols))
    severe_nse_per_trajectory, mild_nse_per_trajectory = [], []
    rewards = []
    trajectory_lengths = []
    all_states = grid.get_states()
    all_trajectories = []

    for _ in range(num_trajectories):
        num_severe_nse = 0
        num_mild_nse = 0
        trajectory = []
        traj_rewards, trajectory = simulate_trajectory(grid, policy)
        traj_id = []
        for s, a in trajectory:
            s_id = all_states.index(s) if s in all_states else 0
            traj_id.append(s_id)
        all_trajectories.append(traj_id)
        if send_traj!=None:
            send_traj(traj_id)
        trajectory_lengths.append(len(trajectory))

        rewards.append(traj_rewards)

        for step in range(len(trajectory)):
            state = trajectory[step][0]
            times_visited[state[0]][state[1]] += 1
            if state[2] not in (0, 1, 2):
                print('state encoded with True/False')
            elif state[2] not in (True, False):
                print('state encoded with 0/1/2')

            if state[2]==True and state[3]==False:
                num_mild_nse += 1
            elif state[2]==True and state[3]==True:
                num_severe_nse += 1
        severe_nse_per_trajectory.append(num_severe_nse)
        mild_nse_per_trajectory.append(num_mild_nse)
        # print('trajectory: ', trajectory, '\n# mild nse: ', mild_nse_per_trajectory, '\n# severe nse: ', severe_nse_per_trajectory)
    return severe_nse_per_trajectory, mild_nse_per_trajectory, trajectory_lengths, times_visited, np.mean(rewards), np.std(rewards), all_trajectories

def get_f1_and_accuracy(grid, prediction_entries):
    iteration = prediction_entries[0][0]
    method = prediction_entries[0][1]
    # print(iteration, method)
    domain = grid.domain
    tp = {'no-nse': 0, 'mild-nse': 0, 'severe-nse': 0}
    fp = {'no-nse': 0, 'mild-nse': 0, 'severe-nse': 0}
    fn = {'no-nse': 0, 'mild-nse': 0, 'severe-nse': 0}
    for i in range(len(prediction_entries)):
        state = prediction_entries[i][2]
        action = prediction_entries[i][3]
        state_pred = prediction_entries[i][4]

        # severe nse
        if state[2]==True and state[3]==True:
            if state_pred==2:
                tp['severe-nse'] += 1
            elif state_pred==1:
                fn['severe-nse'] += 1
                fp['mild-nse'] += 1
            elif state_pred==0:
                fn['severe-nse'] += 1
                fp['no-nse'] += 1
        # mild nse
        elif state[2]==True and state[3]==False:
            if state_pred==1:
                tp['mild-nse'] += 1
            elif state_pred==2:
                fn['mild-nse'] += 1
                fp['severe-nse'] += 1
            elif state_pred==0:
                fn['mild-nse'] += 1
                fp['no-nse'] += 1
        # no nse
        else:
            if state_pred==0:
                tp['no-nse'] += 1
            elif state_pred==1:
                fn['no-nse'] += 1
                fp['mild-nse'] += 1
            elif state_pred==2:
                fn['no-nse'] += 1
                fp['severe-nse'] += 1

    precision = {'no-nse': 0, 'mild-nse': 0, 'severe-nse': 0}
    recall = {'no-nse': 0, 'mild-nse': 0, 'severe-nse': 0}
    f1_score = {'no-nse': 0, 'mild-nse': 0, 'severe-nse': 0}
    nse_severity = ['no-nse', 'mild-nse', 'severe-nse']
    for nse in nse_severity:
        try:
            precision[nse] = tp[nse] / (tp[nse]+fp[nse])
        except ZeroDivisionError:
            precision[nse] = 0
        try:
            recall[nse] = tp[nse] / (tp[nse]+fn[nse])
        except ZeroDivisionError:
            recall[nse] = 0
        try:
            f1_score[nse] = (2*precision[nse]*recall[nse]) / (precision[nse]+recall[nse])
        except ZeroDivisionError:
            f1_score[nse] = 0
    all_correct = sum(tp.values())
    tot_samples = len(prediction_entries)
    accuracy = all_correct / tot_samples
    return f1_score, accuracy
