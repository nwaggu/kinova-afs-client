import numpy as np
import time
import os
import pybullet as p
import pybullet_data
from pybullet_api import PyBullet, Kinova
from vi import value_iteration
# from arm_mdp import ArmMDP
from kitchen_mdp import ArmMDP
from feedback.helper_functions import get_human_feedback

GRID_SIZE = 5  # 15x15 grid
TABLE_BOUNDS = [[0.2, 0.8], [-0.3, 0.3]]  # X, Y range



def compute_ik(kinova, target_pos):
    """
    Compute inverse kinematics to get joint positions for a given end-effector position.
    """
    end_effector_index = 8
    joint_angles = p.calculateInverseKinematics(
        kinova._id, end_effector_index, target_pos
    )
    return joint_angles[:7]

def main(gui=False):
    hz = 250
    dt = 1.0 / float(hz)
    pb = PyBullet(dt, gui=gui)
    kinova = Kinova(base_position=[0, 0, 0.6])
    urdfRootPath = pybullet_data.getDataPath()

    # Initialize MDP environment with factored state representation
    start_pos = (0, 2, 0)
    goal = (4, 2)
    obj_pos = (2, 2)  # Obstacle position in grid space
    mdp_env = ArmMDP(grid_size=GRID_SIZE, goal=goal, obstacles={obj_pos}, incomplete_reward=False)
    # Define the goal position in continuous space
    obj_cont_pos = mdp_env.continuous_position(obj_pos)

    # Load environment (table and obstacle)
    table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.4, 0, 0])
    objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=obj_cont_pos)

    q0 = np.deg2rad([0, 15, 180, -130, 0, 55, 90])
    kinova.reset(q0)


    # Compute policy using Value Iteration
    policy = value_iteration(mdp_env)

    # Visualize table bounds
    table_bounds_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[(TABLE_BOUNDS[0][1] - TABLE_BOUNDS[0][0]) / 2,
                     (TABLE_BOUNDS[1][1] - TABLE_BOUNDS[1][0]) / 2, 0.001],
        rgbaColor=[0.2, 0.2, 0.2, 1]  # Dark gray
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=table_bounds_visual,
        basePosition=[(TABLE_BOUNDS[0][0] + TABLE_BOUNDS[0][1]) / 2,
                      (TABLE_BOUNDS[1][0] + TABLE_BOUNDS[1][1]) / 2, 0.645]
    )

    # Visualize goal as a green square
    goal_marker = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.001], rgbaColor=[0, 1, 0, 1])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_marker, basePosition=mdp_env.continuous_position(goal))

    # Move end-effector to start position
    kinova.cmd(compute_ik(kinova, mdp_env.continuous_position(start_pos[:2])))

    pb.start()
    state = start_pos  # Start state is now (x, y, obstacle)

    while True:
        if (state[0], state[1]) == goal:
            print("Goal reached!")
            break

        action = policy[state]  # Follow policy
        next_state, reward, _, done = mdp_env.step(state, action)

        if next_state[:2] != state[:2]:  # Ensure valid move
            new_xyz = mdp_env.continuous_position(next_state[:2])
            new_joint_angles = compute_ik(kinova, new_xyz)

            # Move the robot
            kinova.cmd(new_joint_angles)
            time.sleep(0.1)  # Slow down simulation

            # Get human feedback with replay option
            feedback = get_human_feedback("approval", state, action, kinova, mdp_env, compute_ik)
            print(f"Human Feedback: {feedback}")

            print(f"Action: {action}, New State: {next_state}, Reward: {reward}")
            state = next_state
        else:
            print(f"Blocked move at {state}. Trying another action.")
            time.sleep(0.5)

    pb.stop()
    pb.close()
    return policy

if __name__ == "__main__":
    policy = main(True)
