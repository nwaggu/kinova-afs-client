import os
import time
import numpy as np
import pybullet as p
import pybullet_data
from src.pybullet_api import PyBullet, Kinova
from src.feedback.helper_functions import get_human_feedback

TABLE_BOUNDS=[[0.2, 0.8], [-0.3, 0.3]]
MAX_HEIGHT = 1.25  # Maximum height constraint in meters

def readMDPfile(filename, map_name, is_oracle=None):
    mdpFile = open(filename, 'r')
    readMDP = eval(mdpFile.read())
    grid = readMDP[map_name]
    return grid



def simulate_trajectory(grid, policy):
    trajectory = []
    trial_reward = 0
    terminal= False
    grid.reset()
    action = int(policy[grid.state])
    trajectory.append([tuple(grid.state), action])
    observation, reward, _, terminal = grid.step(grid.state, action)
    action = int(policy[observation])
    trajectory.append([tuple(observation), action])
    trial_reward += reward
    while not terminal:
        observation, reward, _, terminal = grid.step(observation, action)
        action = int(policy[observation])
        trajectory.append([tuple(observation), action])
        trial_reward += reward
    return trial_reward, trajectory

def compute_ik(kinova, target_pos):
    """
    Compute inverse kinematics to get joint positions for a given end-effector position.
    Respects the maximum height constraint for all joints.
    """
    # Ensure target position doesn't exceed maximum height
    target_pos = list(target_pos)  # Convert to list for modification
    target_pos[2] = min(target_pos[2], MAX_HEIGHT)

    end_effector_index = 8
    joint_angles = p.calculateInverseKinematics(
        kinova._id, end_effector_index, target_pos
    )
    
    # Get current joint positions
    current_joint_angles = kinova.get_joint_angles()
    
    # Check if any joint position exceeds MAX_HEIGHT
    for i in range(kinova.num_joints):
        joint_info = p.getJointInfo(kinova._id, i)
        if joint_info[2] in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
            # Get link state to check position
            link_state = p.getLinkState(kinova._id, i)
            if link_state[0][2] > MAX_HEIGHT:
                # If joint is too high, reduce target height
                target_pos[2] = max(0.1, target_pos[2] - 0.1)  # Reduce height but keep above ground
                # Recompute IK with adjusted target
                joint_angles = p.calculateInverseKinematics(
                    kinova._id, end_effector_index, target_pos
                )
                break
    
    return joint_angles[:7]

def setup_simulation(gui=False, reset=False):
    """Initialize PyBullet and Kinova arm."""
    if reset:
        urdfRootPath = pybullet_data.getDataPath()
        return urdfRootPath
    else:
        hz = 250
        dt = 1.0 / float(hz)
        pb = PyBullet(dt, gui=gui)
        kinova = Kinova(base_position=[0, 0, 0.6])
        urdfRootPath = pybullet_data.getDataPath()

        return pb, kinova, urdfRootPath

def setup_environment(urdfRootPath, mdp_env):
    """Load table and obstacle into the simulation."""
    table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.2, 0, 0])

    # Visualize goal
    goal = (np.where(mdp_env.grid == b'G')[0].item(), np.where(mdp_env.grid == b'G')[1].item())
    goal_pos = mdp_env.continuous_position(goal)
    # Place a tray at the goal position
    goal_tray = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"), basePosition=[goal_pos[0], goal_pos[1], 0.63], globalScaling=0.18)
    p.changeVisualShape(goal_tray, -1, rgbaColor=[0.0, 1.0, 0.0, 1])
    for i in range(p.getNumJoints(goal_tray)):
        p.changeVisualShape(goal_tray, i, rgbaColor=[0.0, 1.0, 0.0, 1])
    ## Draw a green square
    # goal_marker = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.055, 0.055, 0.001], rgbaColor=[0, 1, 0, 1])
    # p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_marker, basePosition=[goal_pos[0], goal_pos[1], 0.628])

    # Visualize start position
    start = (np.where(mdp_env.grid == b'A')[0].item(), np.where(mdp_env.grid == b'A')[1].item())
    start_pos = mdp_env.continuous_position(start)
    # Add yellow border around start position
    border_thickness = 0.005  # Thickness of the border lines
    border_size = 0.06  # Size of the border
    
    # Create four sides of the border
    # Top border
    top_border = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[border_size, border_thickness, 0.001],
        rgbaColor=[1.0, 1.0, 0.0, 1]  # Yellow color
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=top_border,
        basePosition=[start_pos[0], start_pos[1] + border_size - border_thickness, 0.627]
    )
    
    # Bottom border
    bottom_border = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[border_size, border_thickness, 0.001],
        rgbaColor=[1.0, 1.0, 0.0, 1]  # Yellow color
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=bottom_border,
        basePosition=[start_pos[0], start_pos[1] - border_size + border_thickness, 0.627]
    )
    
    # Left border
    left_border = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[border_thickness, border_size - 2*border_thickness, 0.001],
        rgbaColor=[1.0, 1.0, 0.0, 1]  # Yellow color
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=left_border,
        basePosition=[start_pos[0] - border_size + border_thickness, start_pos[1], 0.627]
    )
    
    # Right border
    right_border = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[border_thickness, border_size - 2*border_thickness, 0.001],
        rgbaColor=[1.0, 1.0, 0.0, 1]  # Yellow color
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=right_border,
        basePosition=[start_pos[0] + border_size - border_thickness, start_pos[1], 0.627]
    )

    # Load a block at start position
    block = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), 
                      basePosition=[start_pos[0], start_pos[1], 0.628])
    p.changeVisualShape(block, -1, rgbaColor=[0, 0, 1, 1])  # Blue color

    # Draw a yellow square
    # start_marker = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.055, 0.055, 0.001], rgbaColor=[1, 1, 0, 1])
    # p.createMultiBody(baseMass=0, baseVisualShapeIndex=start_marker, basePosition=[start_pos[0], start_pos[1], 0.628])

    object_uids = []  # Store all object UIDs
    for state in mdp_env.get_states():
        obj_cont_pos = None
        x, y, obj_1, obj_2 = state
        if (obj_1, obj_2) == (1, 1): # some obstacle but no NSE
            obj_cont_pos = mdp_env.continuous_position((x, y))
            surface = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.055, 0.055, 0.001],
                rgbaColor=[1.0, 1.0, 0.0, 1] # Yellow color
                # rgbaColor=[0.7, 0.9, 1.0, 1]  # Light blue color
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=surface,
                basePosition=[obj_cont_pos[0], obj_cont_pos[1], 0.628]
            )

        elif (obj_1, obj_2) == (2, 2):  # obstacle with mild NSE
            obj_cont_pos = mdp_env.continuous_position((x, y))
            surface = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.055, 0.055, 0.001],
                rgbaColor=[1.0, 0.5, 0.0, 1] # Orange color
                # rgbaColor=[0.5, 0.25, 0.0, 1]  # Brown color
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=surface,
                basePosition=[obj_cont_pos[0], obj_cont_pos[1], 0.628]
            )
        elif (obj_1, obj_2) == (2, 3): # obstacle with severe NSE
            obj_cont_pos = mdp_env.continuous_position((x, y))
            surface = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.055, 0.055, 0.001],
                rgbaColor=[1.0, 0.0, 0.0, 1] # Red color
                # rgbaColor=[0.5, 0.25, 0.0, 1]  # Brown color
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=surface,
                basePosition=[obj_cont_pos[0], obj_cont_pos[1], 0.628]
            )

    return object_uids  # Return list of all placed objects


def visualize_scene(mdp_env):
    """Visualize table bounds, goal, and other important landmarks."""
    table_bounds_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[(TABLE_BOUNDS[0][1] - TABLE_BOUNDS[0][0]) / 2,
                     (TABLE_BOUNDS[1][1] - TABLE_BOUNDS[1][0]) / 2, 0.000],
        rgbaColor=[0.2, 0.2, 0.2, 1]  # Dark gray
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=table_bounds_visual,
        basePosition=[(TABLE_BOUNDS[0][0] + TABLE_BOUNDS[0][1]) / 2,
                      (TABLE_BOUNDS[1][0] + TABLE_BOUNDS[1][1]) / 2, 0.62785]
    )

    # Add grid lines
    n_rows = mdp_env.n_rows
    n_cols = mdp_env.n_cols
    row_step = (TABLE_BOUNDS[0][1] - TABLE_BOUNDS[0][0]) / n_rows
    col_step = (TABLE_BOUNDS[1][1] - TABLE_BOUNDS[1][0]) / n_cols

    # Create vertical lines
    for i in range(n_cols + 1):
        x = TABLE_BOUNDS[0][0] + i * row_step
        line = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.001, (TABLE_BOUNDS[1][1] - TABLE_BOUNDS[1][0]) / 2, 0.001],
            rgbaColor=[0.3, 0.3, 0.3, 1]  # Slightly lighter gray
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=line,
            basePosition=[x, (TABLE_BOUNDS[1][0] + TABLE_BOUNDS[1][1]) / 2, 0.628]
        )

    # Create horizontal lines
    for i in range(n_rows + 1):
        y = TABLE_BOUNDS[1][0] + i * col_step
        line = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[(TABLE_BOUNDS[0][1] - TABLE_BOUNDS[0][0]) / 2, 0.001, 0.001],
            rgbaColor=[0.3, 0.3, 0.3, 1]  # Slightly lighter gray
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=line,
            basePosition=[(TABLE_BOUNDS[0][0] + TABLE_BOUNDS[0][1]) / 2, y, 0.628]
        )

    # Visualize height constraint with a transparent red plane
    height_constraint = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[(TABLE_BOUNDS[0][1] - TABLE_BOUNDS[0][0]) / 2,
                     (TABLE_BOUNDS[1][1] - TABLE_BOUNDS[1][0]) / 2, 0.001],
        rgbaColor=[1, 0, 0, 0.2]  # Semi-transparent red
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=height_constraint,
        basePosition=[(TABLE_BOUNDS[0][0] + TABLE_BOUNDS[0][1]) / 2,
                      (TABLE_BOUNDS[1][0] + TABLE_BOUNDS[1][1]) / 2, MAX_HEIGHT]
    )

def run_simulation(pb, kinova, mdp_env, policy, objectUid, feedback_file):
    """Main loop for running the simulation."""
    start_pos = (np.where(mdp_env.grid == b'A')[0].item(), np.where(mdp_env.grid == b'A')[1].item())
    goal = (np.where(mdp_env.grid == b'G')[0].item(), np.where(mdp_env.grid == b'G')[1].item())

    kinova.cmd(compute_ik(kinova, mdp_env.continuous_position(start_pos[:2])))
    pb.start()


    state = mdp_env.start_state
    while True:
        if (state[0], state[1]) == goal:
            print("Goal reached!")
            break

        action = policy[state]  # Follow policy
        next_state, reward, _, done = mdp_env.step(state, action)
        print(f"State: {state}, Action: {action}, New State: {next_state}, Reward: {reward}")

        new_xyz = mdp_env.continuous_position(next_state[:2])
        new_joint_angles = compute_ik(kinova, new_xyz)
        time.sleep(1)

        # Move the robot
        kinova.cmd(new_joint_angles)
        time.sleep(0.1)  # Slow down simulation

        # Get human feedback
        feedback_format = input("Enter a feedback format (approval/correction/dam): ").strip().lower()
        feedback = get_human_feedback(mdp_env, feedback_format, state, action, kinova, mdp_env, compute_ik)
        with open(feedback_file, "a") as f:
            if f.tell() == 0:
                f.write("feedback_format,state,action,feedback\n")
            f.write(f"{feedback_format},{state},{action},{feedback['feedback']}\n")

        state = next_state

    pb.stop()
    pb.close()
    return policy

# def run_simulation(pb, kinova, mdp_env, policy, start_pos, goal, objectUid, feedback_file):
#     """Main loop for running the simulation."""
#     kinova.cmd(compute_ik(kinova, mdp_env.continuous_position(start_pos[:2])))
#     pb.start()
#     trajectory = []

#     state = start_pos  # Start state is (x, y, obstacle)

#     while True:
#         if (state[0], state[1]) == goal:
#             print("Goal reached!")
#             break

#         action = policy[state[0], state[1]]  # Follow policy
#         trajectory.append(mdp_env.continuous_position(state[:2]))
#         next_state, reward, done = mdp_env.step(state, action)

#         new_xyz = mdp_env.continuous_position(next_state[:2])
#         new_joint_angles = compute_ik(kinova, new_xyz)

#         # Move the robot
#         kinova.cmd(new_joint_angles)
#         time.sleep(0.1)  # Slow down simulation

#         # Get human feedback with replay option
#         feedback = get_human_feedback("approval", state, action, kinova, mdp_env, compute_ik)
#         print(f"Human Feedback: {feedback}")
#         with open(feedback_file, "a") as f:
#             if f.tell() == 0:
#                 f.write("feedback_format,state,action,feedback\n")
#             f.write(f"approval,{state},{action},{feedback['approved']}\n")

#         print(f"Action: {action}, New State: {next_state}, Reward: {reward}")
#         state = next_state
#     print(f"Trajectory: {trajectory}")

#     pb.stop()
#     pb.close()
#     return policy
