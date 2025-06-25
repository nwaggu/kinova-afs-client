from src.feedback.helper_functions import *
from src.feedback.feedback_formats import slow_move
from src.helper_functions import *
import random


# def get_annotated_correction(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs):
#     agent_demos = get_demonstration(grid, num_trials=num_feedback, policy=agent_policy, is_oracle=False)
#     state_action_labels = {}

#     for sa_pair in all_sa_pairs:
#         state, action = sa_pair[0], sa_pair[1]
#         if (state, action) not in state_action_labels:
#             state_action_labels[(state, action)] = 0

#         if state in agent_demos:
#             agent_action = agent_demos[state]
#             if is_safe_action(grid, state, agent_action) == False:
#                 action_label = evaluate_action(grid, state, agent_action)
#                 safe_action = int(oracle_policy[state])
#                 for a in range(grid.num_actions):
#                     if a != safe_action:
#                         state_action_labels[(state, a)] = action_label
#     return state_action_labels

# def get_annotated_approval(grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs):
#     max_allowed_feedbacks = (grid.num_states-grid.num_walls)*grid.num_actions
#     if num_feedback > max_allowed_feedbacks:
#         num_feedback = max_allowed_feedbacks
#     random_sa_pairs = get_random_state_action_pairs(grid, num_feedback)
#     state_action_labels = {}

#     for sa_pair in all_sa_pairs:
#         state, action = sa_pair[0], sa_pair[1]
#         if (state, action) not in state_action_labels:
#             state_action_labels[(state, action)] = 0

#         if (state, action) in random_sa_pairs:
#             if is_safe_action(grid, state, action) == False:
#                 action_label = evaluate_action(grid, state, action)
#                 state_action_labels[(state, action)] = action_label
#     return state_action_labels

def get_demo_action_mismatch(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs):
    state_action_labels = {}
    all_states = grid.all_states
    fb_state_choices = []
    for s in all_states:
        if s!=grid.start_state:
           fb_state_choices.append(s) 
    feedback_states = random.sample(fb_state_choices, num_feedback)

    for sa_pair in all_sa_pairs:
        state, action = sa_pair[0], sa_pair[1]
        if (state, action) not in state_action_labels:
            state_action_labels[(state, action)] = 0

    for state in feedback_states:
        # state = tuple(state)
        state_xyz = grid.continuous_position(state[:2])
        to_state(state_xyz[0],state_xyz[1],state_xyz[2])

        print('Demonstrate the action the robot has to take in this state.')
        key_press = input("Press ENTER to continue...")
        if key_press == "":
            agent_action = int(agent_policy[state])
            oracle_action = get_feedback_input_from_arm(start_compliance, end_compliance, grid, kinova, state)
            # oracle_action = int(oracle_policy[state[0], state[1]])
            if agent_action!=oracle_action:
                state_action_labels[(state, agent_action)] = 2

    return state_action_labels


def get_correction(start_compliance, end_compliance, to_state, objectUids, kinova, grid, oracle_policy, agent_policy, num_feedback, all_sa_pairs):
    # agent_demos = get_demonstration(grid, num_trials=num_feedback, policy=agent_policy, is_oracle=False)
    state_action_labels = {}
    all_states = grid.all_states
    feedback_states = random.sample(all_states, num_feedback)

    for sa_pair in all_sa_pairs:
        state, action = sa_pair[0], sa_pair[1]
        if (state, action) not in state_action_labels:
            state_action_labels[(state, action)] = 0

    for state in feedback_states:
        action = agent_policy[state]
        next_state, reward,_, done = grid.step(state, action)
        state_xyz = grid.continuous_position(state[:2])
        print('State: ', state, 'Action: ', grid.action_mapping[action], 'Next state: ', next_state)

        # Show the robot's action
        new_xyz = grid.continuous_position(next_state[:2])
        new_joint_angles = compute_ik(kinova, new_xyz)
        time.sleep(1)
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(grid, objectUids, kinova, new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)

        correction = input("Is the robot action correct? (yes/no): ").strip()
        if correction == "no":
            to_state(state_xyz[0], state_xyz[1], state_xyz[2])
            print('Demonstrate the correct action the robot has to take in this state.')
            safe_action = int(get_feedback_input_from_arm(start_compliance, end_compliance, grid, kinova, state))
            # safe_action = oracle_policy[state[0], state[1]]
            key_press = input("Press ENTER to continue...")
            # if key_press == "":

            for a in range(grid.num_actions):
                if a != safe_action:
                    state_action_labels[(state, a)] = 2
    return state_action_labels

def get_approval(objectUids, kinova, grid, num_feedback, all_sa_pairs):
    max_allowed_feedbacks = grid.n_rows*grid.n_cols*grid.num_actions
    if num_feedback > max_allowed_feedbacks:
        num_feedback = max_allowed_feedbacks
    random_sa_pairs = get_random_state_action_pairs(grid, num_feedback)
    state_action_labels = {}

    for sa_pair in all_sa_pairs:
        state, action = sa_pair[0], sa_pair[1]
        if (state, action) not in state_action_labels:
            state_action_labels[(state, action)] = 0

    for (state, action) in random_sa_pairs:
        state_xyz = grid.continuous_position(state[:2])
        next_state, reward, _, done = grid.step(state, action)

        # Show the robot's action
        new_xyz = grid.continuous_position(next_state[:2])
        new_joint_angles = compute_ik(kinova, new_xyz)
        time.sleep(1)
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(grid, objectUids, kinova, new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)
        print('State: ', state, 'Action: ', grid.action_mapping[action])
        feedback = input("Do you approve this action? (yes/no): ").strip().lower()
        if feedback == "no":
            state_action_labels[(state, action)] = 2
        else:
            state_action_labels[(state, action)] = 0

    return state_action_labels

def get_rank(objectUids, kinova, grid, num_feedback, all_sa_pairs, SEED=42):
    max_allowed_feedbacks = grid.n_rows*grid.n_cols*grid.num_actions
    if num_feedback > max_allowed_feedbacks:
        num_feedback = max_allowed_feedbacks
    all_states = grid.all_states
    feedback_states = random.sample(range(len(all_states)), num_feedback)
    random_queries = get_random_ranking_queries(grid, feedback_states, SEED)
    state_action_labels = {}
    ranks = {}

    for sa_pair in all_sa_pairs:
        state, action = sa_pair[0], sa_pair[1]
        if (state, action) not in state_action_labels:
            state_action_labels[(state, action)] = 0

    for (state, a1, a2) in random_queries:
        state_xyz = grid.continuous_position(state[:2])
        next_state_a1, reward, _, done = grid.step(state, a1)
        a1_new_xyz = grid.continuous_position(next_state_a1[:2])
        a1_new_joint_angles = compute_ik(kinova, a1_new_xyz)
        time.sleep(1)
        print('State: ', state, 'Action 1: ', grid.action_mapping[a1])
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(grid, objectUids, kinova, a1_new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)

        next_state_a2, reward, _, done = grid.step(state, a2)
        a2_new_xyz = grid.continuous_position(next_state_a2[:2])
        a2_new_joint_angles = compute_ik(kinova, a2_new_xyz)
        time.sleep(1)
        print('State: ', state, 'Action 2: ', grid.action_mapping[a2])
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(grid, objectUids, kinova, a2_new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)

        feedback = input("Is action 1 or action 2 better? (1/2): ").strip()
        if feedback == "1":
            ranks[state] = [a1, a2]
        elif feedback == "2":
            ranks[state] = [a2, a1]
    state_action_labels = map_labels(state_action_labels, ranks)
    return state_action_labels

def map_labels(state_action_labels, ranks):
    for state, action_ranks in ranks.items():
        selected_a = action_ranks[0]
        unselected_a = action_ranks[1]
        state_action_labels[(state, selected_a)] = 0
        state_action_labels[(state, unselected_a)] = 2
    return state_action_labels
