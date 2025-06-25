from src.main_approach.helper_functions import *
from src.main_approach.reward_pred_model import pred_cs_and_get_qcap, update_reward_model
from src.main_approach.test_grids_prediction import pred_test_grid
import numpy as np

def choose_feedback(feedback_types, t, epsilon, f_G, f_N, f_costs, baseline=None):

    fb_val = np.zeros(len(feedback_types))
    fb_probability = get_feedback_probs()
    for i, f in enumerate(feedback_types):
        probability = fb_probability[i]
        cost = f_costs[i]
        f_gain = f_G[i]
        if baseline!=None:
            f_gain = 1
            if baseline=='cost-sensitive':
                probability = 1
            elif baseline=='most-probable':
                cost = 1
        fb_val[i] = probability * (1/ (f_gain*cost))
        if baseline!=None:
            last_term = 0
            # last_term = np.sqrt(np.log(t)/(f_N[i] + epsilon))
        else:
            last_term = np.sqrt(np.log(t)/(f_N[i] + epsilon))
        fb_val[i] += last_term
        # print('probability: ', probability, 'cost: ', cost, 'last term: ', last_term, 'f_G: ', f_gain)
    feedback_ch = np.argmax(fb_val)
    return feedback_ch, fb_val

def learn(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, grid, oracle_policy, initial_agent_policy, labels, output_dir, n_clusters, tot_budget, baseline, epsilon=0.001):
    grid.reset()
    trained_model = None

    feedback_types = ['correction', 'approval', 'dam', 'ranking']
    f_N = np.zeros(len(feedback_types)) # number of times a feedback was used

    t = 1
    budget = tot_budget
    f_costs = get_feedback_costs()
    f_G = np.zeros(len(feedback_types)) + epsilon # initializing with default fb gain vals
    method_queried = ''
    method_received = ''
    all_fg, all_budget = [], []
    p_cap, q_cap = [], []
    all_fb_val = []
    critical_states = []
    m = get_agent_initial_sa_labels(grid)
    all_states = grid.get_states()


    while budget > 0:
        # critical_states = sample_critical_states(grid, t, p_cap, q_cap, m, critical_states, labels, n_clusters)
        critical_states = sample_random_critical_states(grid, t, n_clusters, is_random=True)

        # print('current budget: ', budget)
        all_budget.append(budget)
        all_fg.append(list(f_G))

        # 1. agent chooses a feedback method
        feedback_ch, fb_val = choose_feedback(feedback_types, t, epsilon, f_G, f_N, f_costs, baseline=baseline)
        # print('chosen feedback: ', feedback_types[feedback_ch])
        all_fb_val.append(list(fb_val))
        if feedback_types[feedback_ch] in ['approval', 'dam', 'ranking', 'correction', ]:
            method_queried += feedback_types[feedback_ch][0]+'-'
        elif feedback_types[feedback_ch] == 'annotated_correction':
            method_queried += 'ac-'
        else:
            method_queried += 'aa-'
        # 2. oracle provides feedback
        if get_chosen_feedback(feedback_ch, baseline=baseline)!=None:
            print('chosen feedback: ', feedback_types[feedback_ch])
            collect_oracle_feedback(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, grid, critical_states, feedback_ch, oracle_policy, initial_agent_policy)
            if feedback_types[feedback_ch] in ['correction', 'approval', 'dam', 'ranking']:
                method_received += feedback_types[feedback_ch][0]+'-'
            elif feedback_types[feedback_ch] == 'annotated_correction':
                method_received += 'ac-'
            else:
                method_received += 'aa-'

            # 3. agent approximates oracle's action labels (safe/unsafe)
            if p_cap!=[]:
                m = q_cap
            p_cap = approximate_p_cap(grid)

            # 4. train model -> agent learns from the approximation
            trained_model = update_reward_model(grid)

            # update variables
            f_N[feedback_ch] += 1
            q_cap = pred_cs_and_get_qcap(grid, critical_states, trained_model)
            f_G = update_feedback_gain(p_cap, q_cap, feedback_ch, f_G, critical_states)

        budget -= f_costs[feedback_ch]
        t += 1
    learned_reward_to_file(grid, output_dir+"/learned_reward_smartQ.csv", tot_budget)
    # learned_reward_to_file(grid, output_dir+"/cs_pred_reward.csv", tot_budget)
    write_val_to_file(all_fg, all_budget, output_dir+'/fg_budget_vals/', tot_budget)
    write_val_to_file(all_fb_val, all_budget, output_dir+'/fb_vals/', tot_budget)


    _, updated_agent_policy = update_agent_policy(grid, trained_model)
    print('Train grid agent policy:\n', updated_agent_policy)
    return trained_model

def afs(start_compliance, end_compliance, send_traj, to_state, pb, objectUids, kinova, train_grid, oracle_policy, naive_policy, output_dir, baseline, test_map_names, filename):
    n_clusters = 3
    iterations = [5, 10, 15, 20, 25]
    # iterations = [5]
    labels = cluster_states(train_grid, n_clusters, cluster_algo='kmeans')

    # start_pos = (np.where(train_grid.grid == b'A')[0].item(), np.where(train_grid.grid == b'A')[1].item())
    # goal = (np.where(train_grid.grid == b'G')[0].item(), np.where(train_grid.grid == b'G')[1].item())
    for i in iterations:

        # kinova.cmd(compute_ik(kinova, train_grid.continuous_position(start_pos[:2])))
        pb.start()
        print('---'*20)
        print('total budget: ', i)
        model = learn(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, train_grid, oracle_policy, naive_policy, labels, output_dir, n_clusters, tot_budget=i, baseline=baseline)

        pred_test_grid(send_traj, test_map_names, model, output_dir, filename, iterations=i)

    write_config(output_dir+"/config.csv", get_feedback_costs(), get_feedback_probs())
    pass
