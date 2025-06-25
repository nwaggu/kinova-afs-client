import numpy as np, random, math, itertools
from scipy.stats import entropy
from src.lrtdp import lrtdp
from src.feedback.feedback_formats import *
from src.main_approach.reward_pred_model import predict
# from feedback.traffic_accuracy_analysis import *
from src.feedback.helper_functions import *
from src.vi import value_iteration

from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
from sklearn.metrics import pairwise_distances

# np.random.seed(12236)
# random.seed(12236)

'''
0 : corrections
1: approval
2: dam
'''
def select_diverse_points(cluster_data, num_diverse_points):
    diverse_subset = [random.choice(cluster_data)]
    cluster_data = [x for x in cluster_data if not np.array_equal(x, diverse_subset[0])]

    # Greedy selection of diverse points
    while len(diverse_subset) < num_diverse_points:
        max_distance = -1
        selected_point = None
        for data_point in cluster_data:
            min_distance = min([np.linalg.norm(data_point - x) for x in diverse_subset])
            if min_distance > max_distance:
                max_distance = min_distance
                selected_point = data_point
        diverse_subset.append(selected_point)
        cluster_data = [x for x in cluster_data if not np.array_equal(x, selected_point)]
    return diverse_subset

def get_agent_initial_sa_labels(grid):
    m = []
    all_states = grid.all_states
    for state in all_states:
        state_m = []
        all_actions = range(grid.num_actions)
        if grid.domain=='bp':
            all_actions = grid.get_actions(state)
        for _ in all_actions:
            state_m.append(0)
        m.append(state_m)
    return m

def get_cluster_info_gain(clusters, all_states, p_cap, q_cap, m, prev_cs_id):
    prev_critical_states = {}
    for id in prev_cs_id:
        state = tuple(all_states[id])
        prev_critical_states[state] = [p_cap[id], q_cap[id], m[id]] # state : [oracle feedback, agent prediction, prev oracle feedback]

    cluster_p_q = {}
    for item in clusters:
        cluster_p_q[item] = []
        cluster_p_cap, cluster_q_cap, cluster_m_cap = [], [], []
        for i in clusters[item]:
            i = tuple(i)
            if i in prev_critical_states.keys():
                cluster_p_cap.append(prev_critical_states[i][0])
                cluster_q_cap.append(prev_critical_states[i][1])
                cluster_m_cap.append(prev_critical_states[i][2])
        cluster_p_q[item].append([cluster_p_cap, cluster_q_cap, cluster_m_cap])
    '''
    eg of cluster_p_q: {0: [[[a,b,c,d], [e,f,g,h]], # p_cap
                        [[r,t,y,e], [p,o,q,w]], # q_cap
                        [[], []] # m
                        ]}
    '''

    cluster_info_gain = []
    for item in cluster_p_q:
        cluster_p = cluster_p_q[item][0][0]
        cluster_q = cluster_p_q[item][0][1]
        cluster_m = cluster_p_q[item][0][2]

        all_labels = [0, 1, 2]
        cluster_p_dist = get_dist(cluster_p, all_labels)
        cluster_q_dist = get_dist(cluster_q, all_labels)
        cluster_m_dist = get_dist(cluster_m, all_labels)

        gain_val = 0.001
        eps = 1e-10
        for j in range(len(cluster_p_dist)):
            pk, qk = cluster_p_dist[j], cluster_m_dist[j]
            val_k = entropy(pk+eps, qk+eps)
            gain_val += val_k
        gain_val = gain_val/len(cluster_p_dist)
        cluster_info_gain.append(gain_val)
    return cluster_info_gain

def euclidean_distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

def jaccard_distance(a, b):
    intersection = sum(x == y for x, y in zip(a, b))
    union = len(a) + len(b) - intersection
    return 1 - intersection / union

# Assign points to the nearest centroid
def assign_points_to_centroids(data, centroids):
    clusters = {i: [] for i in range(len(centroids))}
    labels = []
    for point in data:
        distances = [jaccard_distance(point, centroid) for centroid in centroids]
        min_distance_index = distances.index(min(distances))
        clusters[min_distance_index].append(point)
        labels.append(min_distance_index)
    return clusters, labels

# Recompute centroids
def recompute_centroids(clusters):
    centroids = []
    for points in clusters.values():
        centroid = tuple(sum(col) / len(col) for col in zip(*points)) if points else None
        centroids.append(centroid)
    return centroids

def initialize_centroids_kmeans_plusplus(data, k):
    random.seed(42)
    centroids = [random.choice(data)]
    for _ in range(1, k):
        distances = [min(euclidean_distance(point, c) for c in centroids) for point in data]
        total = sum(distances)
        probabilities = [d / total for d in distances]
        cumulative_probabilities = list(itertools.accumulate(probabilities))
        r = random.random()
        for i, cp in enumerate(cumulative_probabilities):
            if r < cp:
                centroids.append(data[i])
                break
    return centroids

def handle_empty_clusters(clusters, centroids, data):
    random.seed(42)
    for i, centroid in enumerate(centroids):
        if centroid is None:
            centroids[i] = random.choice(data)
            clusters[i].append(centroids[i])
    return clusters, centroids

# K-means Algorithm
def k_means(data, k, max_iterations=100):
    centroids = initialize_centroids_kmeans_plusplus(data, k)
    for _ in range(max_iterations):
        clusters, labels = assign_points_to_centroids(data, centroids)
        clusters, centroids = handle_empty_clusters(clusters, centroids, data)
        new_centroids = recompute_centroids(clusters)
        if new_centroids == centroids:
            break
        centroids = new_centroids
    return clusters, centroids, labels


def k_centers(X, k):
    """
    K-Centers clustering algorithm.

    Parameters:
    - X: ndarray of shape (n_samples, n_features), input data points.
    - k: int, number of clusters.

    Returns:
    - centers: List of indices representing chosen cluster centers.
    - labels: ndarray of shape (n_samples,), cluster assignments for each point.
    """
    if not isinstance(X, np.ndarray):
        X = np.array(X)
    n_samples = X.shape[0]
    # Randomly select the first center
    centers = [np.random.choice(n_samples)]

    # Iteratively select the next center
    for _ in range(1, k):
        # Compute distances from each point to the nearest center
        distances = pairwise_distances(X, X[centers]).min(axis=1)
        # Select the point farthest from its nearest center
        new_center = np.argmax(distances)
        centers.append(new_center)

    # Assign points to the nearest center
    distances = pairwise_distances(X, X[centers])
    labels = np.argmin(distances, axis=1)

    return centers, labels

def cluster_states(grid, k, cluster_algo='kmeans'):
    all_states = grid.all_states
    formatted_data = [(int(bool1), int(bool2)) for x, y, bool1, bool2 in all_states] # Format all_states to all numeric

    # k = 3 # Number of clusters
    if cluster_algo=='kcenters':
        formatted_data_list = [list(tup) for tup in formatted_data]
        _, labels = k_centers(formatted_data_list, k)
    elif cluster_algo=='dbscan':
        formatted_data_list = [list(tup) for tup in formatted_data]
        db = DBSCAN(eps=0.3, min_samples=10).fit(formatted_data_list)
        labels = db.labels_
    elif cluster_algo=='kmeans':
        _, _, labels = k_means(formatted_data, k)
    return labels

def sample_critical_states(grid, t, p_cap, q_cap, m, prev_critical_states, labels, n_clusters):
    all_states = grid.all_states
    np.random.seed(t*14)
    k = n_clusters # Number of clusters
    n = int(0.05*len(all_states)) # Total number of states to sample
    n = int(0.05*len(all_states)) # Total number of states to sample
    # set n to 1, if 5% of the state space is less than 1
    if n<k:
        n=k

    # # K-means clustering with Euclidean distance
    # kmeans = KMeans(n_clusters=k, random_state=0, n_init=10, algorithm='lloyd')
    # kmeans.fit(formatted_data)
    # labels = kmeans.labels_

    # K-means clustering with Jaccard distance
    # _, _, labels = k_means(formatted_data, k)

    # Grouping states based on clustering results
    clusters = {}
    i = 0
    for item in labels:
        if item in clusters:
            # if i not in queried_points:
                clusters[item].append(all_states[i])
        else:
            # if i not in queried_points:
                clusters[item] = [all_states[i]]
        i +=1

    ####### Print clusters ###########
    # for item in clusters:
    #     print("Cluster ", item)
    #     print('num samples: ', len(clusters[item]))
    #     for i in clusters[item]:
    #         print(i)
    #     input()
    ##################################

    # setting the number of datapoints to select
    cluster_sampling_wts = []
    if p_cap==[] and q_cap==[]: # Equal random sampling
        for _ in clusters:
            wt = int(n/k)
            cluster_sampling_wts.append(wt)
            samples_from_each_cluster = cluster_sampling_wts
        i = 0
        while sum(samples_from_each_cluster) != n:
            if sum(samples_from_each_cluster) > n:
                samples_from_each_cluster[i] -= 1
            elif sum(samples_from_each_cluster) < n:
                samples_from_each_cluster[i] += 1
            i += 1
    elif p_cap!=[] and q_cap!=[]:
        clusters_info_gain = get_cluster_info_gain(clusters, all_states, p_cap, q_cap, m, prev_critical_states)
        # print(clusters_info_gain)
        clusters_info_gain = np.array(clusters_info_gain)
        weights = clusters_info_gain / clusters_info_gain.sum()
        cluster_sampling_wts = weights.tolist()

        samples_from_each_cluster = [1] * n_clusters # to sample at least 1 point from each cluster
        remaining_samples = n - sum(samples_from_each_cluster) # remaining points to samples
        for item in clusters:
            n_sample_from_each_cluster = math.ceil(cluster_sampling_wts[item]*remaining_samples)
            samples_from_each_cluster[item] += n_sample_from_each_cluster

        # for i in range(len(samples_from_each_cluster)):
        #     if samples_from_each_cluster[i]<1:
        #         samples_from_each_cluster[i]=1

        sorted_weights = np.argsort(weights)
        i = 0
        while sum(samples_from_each_cluster) != n:
            if sum(samples_from_each_cluster) > n:
                samples_from_each_cluster[sorted_weights[i]] -= 1
            elif sum(samples_from_each_cluster) < n:
                samples_from_each_cluster[sorted_weights[-i]] += 1
            i += 1
    # print('samples from each cluster: ', samples_from_each_cluster)
    sampled_state_indices = []
    for item in clusters:
        sampled_points = np.random.choice(len(clusters[item]), size=samples_from_each_cluster[item], replace=False)
        # sampled_states = select_diverse_points(clusters[item], samples_from_each_cluster[item])

        sampled_states = []
        for pt in sampled_points:
            state = clusters[item][pt]
            sampled_states.append(state)
        # print('num states sampled: ', sampled_states)
        for state in sampled_states:
            if samples_from_each_cluster[item]>0:
                # point_idx = np.where(np.all(all_states == state, axis=1))
                point_idx = all_states.index(state)
                sampled_state_indices.append(point_idx)
                samples_from_each_cluster[item] -= 1
            elif samples_from_each_cluster[item]==0:
                break

    return sampled_state_indices

def sample_random_critical_states(grid, t, num_clusters, is_random=True):
    all_states = grid.get_states()
    if is_random:
        np.random.seed(t)
        num_to_sample = num_clusters if int((0.05) * len(all_states))<num_clusters else int((0.05) * len(all_states))
        state_indices = [idx for idx, state in enumerate(all_states)]
        sampled_indices = random.sample(state_indices, num_to_sample)
    else:
        num_to_sample = int((0.05/3) * len(all_states))
        # Separate the indices into two lists based on the flag value of the state
        floor_indices = [idx for idx, state in enumerate(all_states) if (state[1] == True and state[2] == False)]
        carpet_indices = [idx for idx, state in enumerate(all_states) if (state[1] == True and state[2] == True)]
        free_indices = [idx for idx, state in enumerate(all_states) if (state[1] == False and state[2] == False)]

        # Randomly sample from each list
        sampled_floor_indices = random.sample(floor_indices, num_to_sample)
        sampled_carpet_indices = random.sample(carpet_indices, num_to_sample)
        sampled_free_indices = random.sample(free_indices, num_to_sample)

        # Combine the sampled lists of indices
        sampled_indices = sampled_floor_indices + sampled_carpet_indices + sampled_free_indices
    return sampled_indices

# def get_q_vals(grid, states):
#     all_states = grid.getStateFactorRep()
#     for s in states:
#         # print(grid.oracle_q_values)
#         # print(grid.agent_q_values)
#         state = tuple(all_states[s])
#         all_actions = range(grid.num_actions)
#         if grid.domain=='bp':
#             all_actions = grid.get_actions(state)
#         for action in all_actions:
#             action = int(action)
#             sa_pair = (state, action)
#             oracle_q_val = grid.oracle_q_values[sa_pair]
#             agent_q_val = grid.agent_q_values[sa_pair]
#             print(oracle_q_val, agent_q_val)
#     input('press enter')

def get_feedback_probs():
    # feedbacks: corr, app, dam, rank
    probs = [0.6, 0.6, 0.4, 0.4]
    return probs

def get_feedback_costs():
    # feedbacks: [corr, app, dam, rank]
    costs = [4, 2, 3, 2]
    return costs

def get_chosen_feedback(feedback_ch, baseline=None):
    probs = get_feedback_probs()
    if baseline=='costs-sensitive':
        return feedback_ch
    fb_prob = probs[feedback_ch]
    choice = np.random.choice([None, feedback_ch], 1, p=[(1-fb_prob), fb_prob])
    return choice

def update_agent_policy(grid, model):
    state_preds = predict(grid, model)
    # print(grid.agent_reward_cache)
    grid_agent_policy = lrtdp(grid, is_oracle=False, use_cache=True)
    # grid_agent_policy = value_iteration(grid)
    return state_preds, grid_agent_policy

def collect_oracle_feedback(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, grid, critical_states, feedback_ch, oracle_policy, initial_agent_policy):
    if feedback_ch==0: # Correction
        get_correction_feedback(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, grid, critical_states, initial_agent_policy, oracle_policy)
    elif feedback_ch==1: # Approval
        get_approval_feedback(kinova, objectUids, grid, critical_states, initial_agent_policy, oracle_policy)
    elif feedback_ch==2: # Demonstration
        get_dam_feedback(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, grid, critical_states, initial_agent_policy, oracle_policy)
    elif feedback_ch==3: # Ranking
        get_ranking_feedback(kinova, objectUids, grid, critical_states, initial_agent_policy, oracle_policy)

def approximate_p_cap(grid):
    all_states = grid.get_states()
    p_cap = []

    for state in all_states:
        state_vals = []
        state = tuple(state)
        all_actions = range(grid.num_actions)
        for action in all_actions:
            action = int(action)
            if (state, action) not in grid.learned_reward_cache:
                grid.learned_reward_cache[(state, action)] = 0
            state_vals.append(grid.learned_reward_cache[(state, action)])
        if state_vals!=[]:
            p_cap.append(state_vals)

    return p_cap

def get_dist(dist, all_labels):
    new_dist = []
    for state_vals in dist:
        unique_labels, label_counts = np.unique(state_vals, return_counts=True)
        label_prob = np.zeros(len(all_labels))
        label_prob[unique_labels] = label_counts/len(state_vals)
        new_dist.append(label_prob)
    return new_dist

def update_feedback_gain(p_cap, q_cap, feedback_ch, f_G, critical_states):
    cs_p_cap, cs_q_cap = [], []
    for id in critical_states:
        cs_p_cap.append(p_cap[id])
        cs_q_cap.append(q_cap[id])

    # print('p cap: ', p_cap)
    # print('q cap: ', q_cap)
    all_labels = [0, 1, 2]
    p_cap_dist = get_dist(cs_p_cap, all_labels)
    q_cap_dist = get_dist(cs_q_cap, all_labels)

    # print('p cap dist: ', p_cap_dist)
    # print('q cap dist: ', q_cap_dist)


    gain_val = 0.001
    eps = 1e-10
    for j in range(len(p_cap_dist)):
        pk, qk = p_cap_dist[j], q_cap_dist[j]
        val_k = entropy(pk+eps, qk+eps)
        gain_val += val_k
    gain_val = gain_val/len(p_cap_dist)
    f_G[feedback_ch] = gain_val
    # print(f_G)

    return f_G

def write_val_to_file(all_fg, all_budget, output_dir, tot_budget):
    filename = Path(output_dir+str(tot_budget)+'.csv')
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, 'a') as f1:
        if os.stat(filename).st_size == 0:
            f1.write("budget, fg\n")
        for i in range(len(all_budget)):
            f1.write('{},{}\n'.format(all_budget[i], all_fg[i]))
        f1.close()

def write_config(filename, f_costs, f_probs):
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, 'a') as f1:
        if os.stat(filename).st_size == 0:
            f1.write("variables, value\n")
        f1.write('feedback cost (corr, app, dam, rank), {}\n'.format(f_costs))
        f1.write('feedback probs (corr, app, dam, rank), {}\n'.format(f_probs))
        f1.close()
