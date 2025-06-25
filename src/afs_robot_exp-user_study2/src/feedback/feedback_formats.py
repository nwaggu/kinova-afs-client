import time
from src.helper_functions import compute_ik
from src.feedback.helper_functions import get_feedback_input_from_arm, get_all_sa_pairs, get_random_sa_pairs, set_arm_to_end_effector_position, get_random_ranking_queries, map_to_labels
import pybullet as p
import numpy as np
from src.helper_functions import *

def slow_move(env, objectUids, kinova, target_angles, duration=2.0, steps=50):
    """
    Gradually moves the Kinova arm to target joint angles over a set duration.

    Parameters:
    - kinova: The Kinova robotic arm instance.
    - target_angles: The final joint angles to reach.
    - duration: Total time (in seconds) for the movement (default: 2 seconds).
    - steps: Number of intermediate steps (default: 50).
    """
    # # Get the current joint angles
    # current_angles = np.array([state[0] for state in p.getJointStates(kinova._id, kinova._actuated_joints)])
    # target_angles = np.array(target_angles)

    # # Create interpolation steps
    # for t in np.linspace(0, 1, steps):
    #     interpolated_angles = (1 - t) * current_angles + t * target_angles
    #     kinova.cmd(interpolated_angles)
    #     time.sleep(duration / steps)  # Adjust sleep to control speed
    # visualize_scene(env)

    # Store initial positions of objects
    initial_positions = {}
    for obj_id in objectUids:
        pos, _ = p.getBasePositionAndOrientation(obj_id)
        initial_positions[obj_id] = pos  # Store position only

    # Get the current joint angles
    current_angles = np.array([state[0] for state in p.getJointStates(kinova._id, kinova._actuated_joints)])
    target_angles = np.array(target_angles)

    # Move the arm gradually
    for t in np.linspace(0, 1, steps):
        interpolated_angles = (1 - t) * current_angles + t * target_angles
        kinova.cmd(interpolated_angles)
        time.sleep(duration / steps)  # Adjust sleep to control speed

    # Restore objects to their original positions
    for obj_id, pos in initial_positions.items():
        p.resetBasePositionAndOrientation(obj_id, pos, [0, 0, 0, 1])  # Reset position, keep orientation same
    p.stepSimulation()



def get_correction_feedback(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, env, critical_states, initial_agent_policy, oracle_policy):
    all_states = env.all_states
    all_sa_pairs = get_all_sa_pairs(env)

    for sa_pair in all_sa_pairs:
        state, action = sa_pair[0], sa_pair[1]
        state = tuple(state)
        action = int(action)
        if (state, action) not in env.learned_reward_cache:
            env.learned_reward_cache[(state, action)] = 0

    # print('Initial agent policy:\n', initial_agent_policy)

    for state_id in critical_states:
        state = all_states[state_id]
        action = initial_agent_policy[state]
        next_state, reward, _, done = env.step(state, action)
        state_xyz = env.continuous_position(state[:2])
        # to_state(state_xyz[0], state_xyz[1], state_xyz[2])
        print('State: ', state, 'Action: ', env.action_mapping[action], 'Next state: ', next_state)

        # Show the robot's action
        new_xyz = env.continuous_position(next_state[:2])
        new_joint_angles = compute_ik(kinova, new_xyz)
        time.sleep(1)
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(env, objectUids, kinova, new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)

        correction = input("Is the robot action correct? (yes/no): ").strip()
        if correction == "no":
            to_state(state_xyz[0], state_xyz[1], state_xyz[2])
            print('Demonstrate the correct action the robot has to take in this state.')
            safe_action = int(get_feedback_input_from_arm(start_compliance, end_compliance, env, kinova, state))
            # safe_action = oracle_policy[state]
            key_press = input("Press ENTER to continue...")
            # if key_press == "":

            for a in range(env.num_actions):
                if a != safe_action:
                    env.learned_reward_cache[(state, a)] = 2

def get_approval_feedback(kinova, objectUids, env, critical_states, initial_agent_policy, oracle_policy):
    all_sa_pairs = get_all_sa_pairs(env)
    for sa_pair in all_sa_pairs:
        state, action = tuple(sa_pair[0]), int(sa_pair[1])
        if (state, action) not in env.learned_reward_cache:
            env.learned_reward_cache[(state, action)] = 0

    random_sa_pairs = get_random_sa_pairs(env, critical_states)
    for (state, action) in random_sa_pairs:
        state_xyz = env.continuous_position(state[:2])
        next_state, reward, _, done = env.step(state, action)

        # Show the robot's action
        new_xyz = env.continuous_position(next_state[:2])
        new_joint_angles = compute_ik(kinova, new_xyz)
        time.sleep(1)
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(env, objectUids, kinova, new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)
        print('State: ', state, 'Action: ', env.action_mapping[action])
        feedback = input("Do you approve this action? (yes/no): ").strip().lower()
        if feedback == "no":
            env.learned_reward_cache[(state, action)] = 2
        else:
            env.learned_reward_cache[(state, action)] = 0

def get_dam_feedback(start_compliance, end_compliance, send_traj, to_state, kinova, objectUids, env, critical_states, initial_agent_policy, oracle_policy):
    all_states = env.get_states()
    all_sa_pairs = get_all_sa_pairs(env)
    for sa_pair in all_sa_pairs:
        state, action = tuple(sa_pair[0]), int(sa_pair[1])
        if (state, action) not in env.learned_reward_cache:
            env.learned_reward_cache[(state, action)] = 0

    for state_id in critical_states:
        state = tuple(all_states[state_id])
        # print('State: ', state)
        state_xyz = env.continuous_position(state[:2])
        to_state(state_xyz[0],state_xyz[1],state_xyz[2])


        # Set the robot arm to this state (in simulator)
        # state_xyz = env.continuous_position(state[:2])
        # joint_angles = compute_ik(kinova, state_xyz)
        # time.sleep(1)
        # kinova.cmd(joint_angles)
        # time.sleep(0.1)

        print('Demonstrate the action the robot has to take in this state.')
        key_press = input("Press ENTER to continue...")
        if key_press == "":
            agent_action = int(initial_agent_policy[state])
            oracle_action = get_feedback_input_from_arm(start_compliance, end_compliance, env, kinova, state)
            # print('Oracle policy: ', oracle_policy)
            # oracle_action = int(oracle_policy[state])
            # print('Oracle action: ', env.action_mapping[oracle_action])
            if agent_action!=oracle_action:
                env.learned_reward_cache[(state, agent_action)] = 2

def get_ranking_feedback(kinova, objectUids, env, critical_states, initial_agent_policy, oracle_policy):
    ranks = {}
    all_sa_pairs = get_all_sa_pairs(env)
    for sa_pair in all_sa_pairs:
        state, action = tuple(sa_pair[0]), int(sa_pair[1])
        if (state, action) not in env.learned_reward_cache:
            env.learned_reward_cache[(state, action)] = 0

    random_queries = get_random_ranking_queries(env, critical_states)
    for (state, a1, a2) in random_queries:
        state_xyz = env.continuous_position(state[:2])
        next_state_a1, reward, _, done = env.step(state, a1)
        # print('Next State: ', next_state_a1, 'Action 1: ', env.action_mapping[a1])
        a1_new_xyz = env.continuous_position(next_state_a1[:2])
        a1_new_joint_angles = compute_ik(kinova, a1_new_xyz)
        time.sleep(1)
        print('State: ', state, 'Action 1: ', env.action_mapping[a1])
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(env, objectUids, kinova, a1_new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)

        next_state_a2, reward, _, done = env.step(state, a2)
        a2_new_xyz = env.continuous_position(next_state_a2[:2])
        a2_new_joint_angles = compute_ik(kinova, a2_new_xyz)
        time.sleep(1)
        print('State: ', state, 'Action 2: ', env.action_mapping[a2])
        set_arm_to_end_effector_position(kinova, state_xyz)
        slow_move(env, objectUids, kinova, a2_new_joint_angles, duration=1.0, steps=100)
        time.sleep(0.1)

        feedback = input("Is action 1 or action 2 better? (1/2): ").strip()
        if feedback == "1":
            ranks[state] = [a1, a2]
        elif feedback == "2":
            ranks[state] = [a2, a1]
    map_to_labels(env, ranks)
