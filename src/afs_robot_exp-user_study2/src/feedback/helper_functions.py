import time
import pybullet as p
import numpy as np
# import rospy

def replay_action(kinova, state, action, mdp_env, compute_ik):
    # This function has to be fixed
    print("\nReplaying action... Press ENTER to proceed with feedback.")

    # Store the previous joint angles before executing the action
    previous_joint_angles = kinova.get_joint_angles()
    q = np.deg2rad(previous_joint_angles)

    # Compute the next state based on the given action
    next_state = mdp_env.transition_function(state, action)
    new_xyz = mdp_env.continuous_position(next_state[:2])
    new_joint_angles = compute_ik(kinova, new_xyz)

    while True:
        replay = input("Replay action? (yes/no/reset): ").strip().lower()

        if replay == "yes":
            print("Resetting arm to previous state...")

            # Instantly reset to previous joint angles
            # for j, angle in enumerate(previous_joint_angles):
            #     p.reset(kinova._id, kinova._actuated_joints[j], angle)
            kinova.reset(q)
            time.sleep(1)  # Allow movement
            print("Replaying action...")
            # Move the arm to the next state (replay action)
            kinova.cmd(new_joint_angles)
            time.sleep(1)  # Allow movement

        elif replay == "no":
            print(f"Exiting replay.")
            break  # Exit the replay loop

        else:
            print("Invalid input")

def set_arm_to_end_effector_position(kinova, target_position):
    end_effector_index = 8
    joint_angles = p.calculateInverseKinematics(kinova._id, end_effector_index, target_position)

    # Instantly set the arm to the computed joint positions
    for j, angle in enumerate(joint_angles[:7]):  # Use only 7 DOF
        p.resetJointState(kinova._id, kinova._actuated_joints[j], angle)

def get_end_effector_position(start_compliance, end_compliance, kinova):
    rospy.loginfo("Switching to effort mode for correction feedback...")

    start_compliance()

    key_press = input("Press ENTER to continue...")
    if key_press == "":
        end_effector_pos = end_compliance()

    # rospy.loginfo("Waiting for user to move the arm to the corrected position...")
    # rospy.sleep(5)  # Wait for human input (or use a better event-based trigger)

    rospy.loginfo(f"New corrected position recorded: {end_effector_pos}")

    # Uncomment following lines if using pybullet
    # end_effector_index = 8
    # link_state = p.getLinkState(kinova._id, end_effector_index)
    # end_effector_pos = link_state[0]  # Extract position (x, y, z)
    return end_effector_pos

def discretize_position(mdp_env, pos):
    x, y, _ = pos  # Ignore z-coordinate
    grid_x = int(round((x - mdp_env.table_bounds[0][0]) / mdp_env.col_step_size))
    grid_y = int(round((y - mdp_env.table_bounds[1][0]) / mdp_env.row_step_size))

    # Ensure the indices stay within grid bounds
    grid_x = max(0, min(mdp_env.n_cols - 1, grid_x))
    grid_y = max(0, min(mdp_env.n_rows - 1, grid_y))

    return [grid_x, grid_y, 0.7]

def get_feedback_input_from_arm(start_compliance, end_compliance, env, kinova, state):
    GRID_POS = [[0.35000000000000003, -0.24, 0.8], [0.35000000000000003, -0.12, 0.8], [0.35000000000000003, 0.0, 0.8], [0.35000000000000003, 0.12, 0.8], [0.35000000000000003, 0.24000000000000005, 0.8], [0.6500000000000001, -0.24, 0.8], [0.6500000000000001, -0.12, 0.8], [0.6500000000000001, 0.0, 0.8], [0.6500000000000001, 0.12, 0.8], [0.6500000000000001, 0.24000000000000005, 0.8]]
    print(state)
    x, y, _, _ = state
    response = get_end_effector_position(start_compliance, end_compliance, kinova)
    new_x_cont, new_y_cont, new_z_cont = response.x_position, response.y_position, response.z_position
    new_state_cont = [new_x_cont, new_y_cont, new_z_cont]
    all_states = env.all_states
    new_state_id = GRID_POS.index(new_state_cont)
    new_state_discrete = all_states[new_state_id]
    new_x, new_y, _, _ = new_state_discrete
    # new_x, new_y, _ = discretize_position(env, (new_x_cont, new_y_cont, new_z_cont))
    print('new state: ', new_x, new_y)
    # Discretize the end effector position (if simulator is used)
    # new_x, new_y = discretize_position(env, end_effector_pos)

    delta_x = new_x - x
    delta_y = new_y - y
    action = None
    if (delta_x, delta_y)==(0, -1):
        action = 1 # up
    elif (delta_x, delta_y)==(0, 1):
        action = 0 # down
    elif (delta_x, delta_y)==(-1, 0):
        action = 2 # left
    elif (delta_x, delta_y)==(1, 0):
        action = 3 # right
    return action


def get_human_feedback(env, feedback_format, state, action, kinova, mdp_env, compute_ik):

    # Replay the action before collecting feedback
    # replay_action(kinova, state, action, mdp_env, compute_ik)

    if feedback_format == "approval":
        feedback = input("Do you approve this action? (yes/no): ").strip().lower()
        return {"feedback_type": "approval", "feedback": False if feedback=="no" else True}

    elif feedback_format == "correction":
        correction = input("Is the robot action correct? (yes/no): ").strip()
        if correction == "yes":
            feedback = action
        else:
            print('Demonstrate the correct action the robot has to take in this state.')
            key_press = input("Press ENTER to continue...")
            if key_press == "":
                feedback = get_feedback_input_from_arm(env, kinova, state)
        return {"feedback_type": "correction", "feedback": int(feedback)}

    # elif feedback_format == "ranking":
    #     ranking = input("Is action 1 better than action 2? (yes/no)").strip()
    #     return {"feedback_type": "ranking", "rank": feedback=="yes"}

    elif feedback_format == "dam":
        print('Demonstrate the action the robot has to take in this state.')
        key_press = input("Press ENTER to continue...")
        if key_press == "":
            feedback = get_feedback_input_from_arm(env, kinova, state)
        return {"feedback_type": "demonstration", "feedback": int(feedback)}

    else:
        print("Invalid feedback format. Please choose from ['approval', 'correction', 'ranking', 'demonstration'].")
        return None

import os
from pathlib import Path

def simulate_trajectory(grid, policy):
    trajectory = []
    trial_reward = 0
    terminal= False
    grid.reset()
    action = int(policy[grid.state])
    trajectory.append([tuple(grid.state), action])
    observation, reward, _, terminal = grid.step(grid.state, action)
    action = int(policy[observation[0], observation[1]])
    trajectory.append([tuple(observation), action])
    trial_reward += reward
    while not terminal:
        observation, reward, _, terminal = grid.step(observation, action)
        action = int(policy[observation[0], observation[1]])
        trajectory.append([tuple(observation), action])
        trial_reward += reward
    return trial_reward, trajectory


def get_demonstration(grid, num_trials, policy, is_oracle=True):
    agent_demos = {}
    for _ in range(num_trials):
        _, trajectory = simulate_trajectory(grid, policy)
        for step in range(len(trajectory)):
            state = trajectory[step][0]
            action = trajectory[step][1]
            if is_oracle:
                if state not in grid.oracle_demos:
                    grid.oracle_demos[state] = action
            elif not is_oracle:
                if state not in agent_demos:
                    agent_demos[state] = action
    return agent_demos

def is_safe_action(state, action):
    if state[1] == True:
        if action in [0,1,2,3]:
            return False
    return True

def get_random_state_action_pairs(grid, curr_budget):
    state_action_pairs = []
    rv = np.random.uniform(0, 1000)
    rv *= 100
    np.random.seed(int(rv))
    indices = np.random.choice(grid.num_states*grid.num_actions, int(curr_budget), replace=False)
    for idx in range(len(indices)):
        state_idx = indices[idx] // grid.num_actions
        random_state = grid.get_states()[state_idx]
        agent_action = indices[idx] % grid.num_actions
        state_action_pairs.append((tuple(random_state), agent_action))
    return state_action_pairs

def get_all_sa_pairs(grid):
    state_action_pairs = []
    for state in grid.get_states():
        for action in range(grid.num_actions):
            state_action_pairs.append([tuple(state), action])
    return state_action_pairs

def get_random_ranking_queries(grid, critical_states, SEED=42):
    # np.random.seed(SEED)
    all_states = grid.get_states()
    state_action_pairs = []
    for sa in critical_states:
        state = tuple(all_states[sa])
        all_actions = list(grid.actions.keys())
        random_actions = np.random.choice(all_actions, 2, replace=False)
        action_1, action_2 = random_actions[0], random_actions[1]
        state_action_pairs.append((state, action_1, action_2))
    return state_action_pairs

def map_to_labels(grid, ranks):
    for state, action_ranks in ranks.items():
        selected_a = action_ranks[0]
        unselected_a = action_ranks[1]
        grid.learned_reward_cache[(state, selected_a)] = 0
        grid.learned_reward_cache[(state, unselected_a)] = 2

def get_grid_traffic(grid):
    traffic = np.zeros((grid.rows, grid.cols))
    for state in grid.get_states():
        if state[1] == True:
            traffic[state[0][0]][state[0][1]] = 1
    return traffic

def learned_reward_to_file(grid, filename, curr_budget):
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)

    with open(filename, 'a') as f:
        if os.stat(filename).st_size == 0:
            f.write("iteration,(state,action),penalty\n")
        for key, value in grid.learned_reward_cache.items():
            f.write('{},{},{}\n'.format(curr_budget, key, value))
        f.close()

def get_random_sa_pairs(grid, critical_states):
    states = grid.get_states()
    # cs = []
    # for cs_id in critical_states:
    #     c_state = tuple(states[cs_id])
    #     cs.append(c_state)
    state_action_pairs = []
    # # np.random.seed(42)
    # num_states = len(critical_states)
    # random_pair_idx = np.random.choice(num_states*4, num_states, replace=False)
    # for i in range(len(random_pair_idx)):
    #     state_id = random_pair_idx[i] // 4
    #     state = tuple(cs[state_id])
    #     action = int(random_pair_idx[i] % 4)
    #     state_action_pairs.append((state, action))

    for cs_id in critical_states:
        c_state = tuple(states[cs_id])
        random_action = np.random.randint(0, 4)
        state_action_pairs.append((c_state, random_action))

    return state_action_pairs
