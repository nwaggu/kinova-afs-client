from sklearn.ensemble import RandomForestRegressor, RandomForestClassifier
from sklearn.model_selection import ParameterSampler, StratifiedKFold
# from fmdp.lrtdp_new import lrtdp
from src.vi import value_iteration
from src.lrtdp import lrtdp
from feedback.helper_functions import *
from sklearn import metrics
import numpy as np
from copy import deepcopy
from src.main_approach.test_grids_prediction import pred_test_grid

def get_state_action_pairs(grid):
    state_action_pairs = []
    all_states = grid.get_states()
    all_actions = grid.actions.keys()
    for state in all_states:
        for action in all_actions:
            if grid.get_successors(state, action)!=[]:
                state_action_pairs.append((tuple(state), action))
                # state_action_pairs.append([state[0][0], state[0][1], 0 if state[1]==False else 1, action])
    return state_action_pairs


def generalize(train_grid, test_map_names, iteration, method, output_dir, filename):
    print('Generalizing...')
    seen_state_features = []
    seen_state_loc = []
    seen_penalty = []
    seen_state_action = []

    # Get seen states and actions for training from grid.learned_reward_cache
    for keys, value in train_grid.learned_reward_cache.items():
        seen_state_loc.append(keys[0][0])
        seen_state_action.append(keys[1])
        if train_grid.domain=='vase' or train_grid.domain=='outdoor':
            seen_state_features.append([0 if keys[0][1]==False else 1, 0 if keys[0][2]==False else 1, keys[1]])
        else:
            seen_state_features.append([0 if keys[0][1]==False else 1, 0 if keys[0][2]==False else 1, 0 if keys[0][3]==False else 1, keys[1]])
        seen_penalty.append(value)
    final_model = predict(seen_state_features, seen_penalty, model_type='classification')

    all_state_action_pairs = get_state_action_pairs(train_grid)
    unseen_state_feature = []
    unseen_state_loc = []
    unseen_state_action = []
    for sa_pair in all_state_action_pairs:
        unseen_state_loc.append((sa_pair[0][0]))
        if train_grid.domain=='vase' or train_grid.domain=='outdoor':
            unseen_state_feature.append([0 if sa_pair[0][1]==False else 1, 0 if sa_pair[0][2]==False else 1, sa_pair[1]])
        else:
            unseen_state_feature.append([0 if sa_pair[0][1]==False else 1, 0 if sa_pair[0][2]==False else 1, 0 if sa_pair[0][3]==False else 1, sa_pair[1]])
        unseen_state_action.append(sa_pair[1])

    state_pred = final_model.predict(np.array(unseen_state_feature))
    for i in range(len(unseen_state_loc)):
        if train_grid.domain=='bp':
            state = ((unseen_state_loc[i]), True if unseen_state_feature[i][0]==1 else False, True if unseen_state_feature[i][1]==1 else False, True if unseen_state_feature[i][2]==1 else False)
        else:
            state = ((unseen_state_loc[i]), True if unseen_state_feature[i][0]==1 else False, True if unseen_state_feature[i][1]==1 else False)
        action = unseen_state_action[i]
        train_grid.agent_reward_cache[(state, action)] = train_grid.get_reward(state, action, is_oracle=False) + (state_pred[i]*5)

    train_grid.incomplete_reward = True
    updated_agent_policy = lrtdp(train_grid, is_oracle=False, use_cache=True, val=100 if method=='DAM' else 42)
    # updated_agent_policy = value_iteration(train_grid)

    # print('updated agent policy for train grid:\n', updated_agent_policy)

    # Predict for test grid
    pred_test_grid(test_map_names, final_model, iteration, method, output_dir, filename)
    return updated_agent_policy, final_model

def train_folds(model,x_train_, y_train_, x_test_, y_test_):
    model.fit(x_train_,y_train_)
    test_label = model.predict(x_test_)
    mse = metrics.mean_squared_error(test_label, y_test_)
    return mse

def predict(X, y, model_type='regression'):
    print('Predicting...')

    zero_count = y.count(0)
    one_count = y.count(1)
    two_count = y.count(2)

    X = np.array(X)
    y = np.array(y)

    # ua-vase size 17
    # params = {
    #             "max_depth": [None, 2, 5, 10],
    #             "max_features":['sqrt', 'log2', None, 0.5, 0.75, 1.0],
    #             "min_samples_leaf": [1, 5, 10, 50],
    #             "max_leaf_nodes": [None, 5, 10, 20],
    #             "n_estimators": [20, 50, 100, 200, 250],
    #             "class_weight": ["balanced", None],
    #         }

    # bp size 17
    # params = {
    #             "max_depth": [None, 2, 5, 10],
    #             "max_features":['sqrt', 'log2', None],
    #             "min_samples_leaf": [1, 5, 10, 50],
    #             "max_leaf_nodes": [None, 5, 10, 20],
    #             "n_estimators": [100, 200, 300, 400, 500],
    #             "class_weight": ["balanced", None],
    #         }

    # outdoor
    params = {
                "max_depth": [None, 2, 5, 10],
                "max_features":['sqrt', 'log2', None],
                "min_samples_leaf": [1, 5, 10, 50],
                "max_leaf_nodes": [None, 5, 10, 20],
                "n_estimators": [50, 100, 200, 500, 900],
                "class_weight": ["balanced", None],
            }


    # params = {
    #             "max_depth": [None, 2, 5, 10],
    #             "max_features":['sqrt', 'log2', None, 0.5, 0.75, 1.0],
    #             "min_samples_leaf": [1, 5, 10, 50],
    #             "max_leaf_nodes": [None, 5, 10, 20],
    #             "n_estimators": [20, 50, 100, 200, 250],
    #             "random_state":[42],
    #             "n_jobs": [-1],
    #             # "class_weight": ["balanced"]
    #         }

    num_splits = 2
    if zero_count < 2 or one_count < 2 or two_count < 2:
        num_splits = 1
    best_score = -1
    best_params = {}
    y_pred = []

    candidate_params = list(ParameterSampler(param_distributions=params, n_iter=10, random_state=42))

    init_model = RandomForestRegressor()
    if model_type == 'classification':
        init_model = RandomForestClassifier()

    model = deepcopy(init_model)
    for parameters in candidate_params:
        model.set_params(**parameters)
        if(num_splits > 1):
            cv =  StratifiedKFold(n_splits=num_splits, random_state=42, shuffle=True)
            cv_scores  = []
            for train, test in cv.split(X, y[:]):
                x_train_, x_test_, y_train_, y_test_ =  X[train], X[test], y[train], y[test]
                score = train_folds(model, x_train_, y_train_, x_test_, y_test_)
                cv_scores.append(score)
            avg_score = float(sum(cv_scores))/len(cv_scores)
            if(avg_score > best_score):
                best_params = parameters
                best_score = avg_score
        else:
            best_params = parameters
            break

    final_model = deepcopy(model)
    final_model.set_params(**best_params)
    final_model.fit(X, y, sample_weight=None)

    return final_model
