from src.vi import value_iteration
from src.lrtdp import lrtdp
# from src.arm_mdp import ArmMDP
from src.kitchen_mdp import ArmMDP
from src.helper_functions import *
from src.main_approach.afs import afs
from src.baseline_method.ri import ri
from src.learn_preference import learn_human_preference
from src.metrics import get_oracle_performance
import rospy
from kortex_driver.srv import *
# from kortex_hardware.srv import *

user_id = 1
output_dir = 'stochastic_output'
feedback_file = os.path.join(output_dir, "feedback_from_user_"+str(user_id)+".csv")
os.makedirs(output_dir, exist_ok=True)
grid_file = 'src/grid_design_5x5.txt'

def main(gui=False):
    """Main function to run the modularized simulation."""
    baselines = 5
    option = 0

    """Main function to run the modularized simulation."""
    pb, kinova, urdfRootPath = setup_simulation(gui)

    if gui:
        # Set the camera
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,  # Distance from camera to target
            cameraYaw=0,       # Horizontal angle (0 is front, 90 is right)
            cameraPitch=-10,    # Vertical angle (negative for looking down)
            cameraTargetPosition=[0.5, 0, 0.9]  # Look at a higher point
        )

    # Initialize MDP environment
    grid = readMDPfile(grid_file, 0)
    mdp_env = ArmMDP(grid=grid, incomplete_reward=False)
    objectUids = setup_environment(urdfRootPath, mdp_env)
    print(mdp_env.get_states())
    oracle_policy = lrtdp(mdp_env, is_oracle=True)
    mdp_env.incomplete_reward = True
    naive_policy = lrtdp(mdp_env, is_oracle=False)
    all_states = mdp_env.get_states()
    print('Oracle policy:\n', oracle_policy)
    print('Naive policy:\n', naive_policy)

    # input()
    # all_states = []
    # for state in mdp_env.get_states():
    #     all_states.append(mdp_env.continuous_position(state[:2]))
    # print(all_states)
    visualize_scene(mdp_env)

    rospy.wait_for_service('robot/startCompliance')
    start_compliance = rospy.ServiceProxy('robot/startCompliance', Forward)
    rospy.wait_for_service('robot/endCompliance')
    end_compliance = rospy.ServiceProxy('robot/endCompliance', Forward)
    rospy.wait_for_service('robot/SendTraj')
    send_traj = rospy.ServiceProxy('robot/SendTraj', SendState)
    rospy.wait_for_service('robot/goToState')
    to_state = rospy.ServiceProxy('robot/goToState', Inverse)

    # start_compliance = None
    # end_compliance = None
    # send_traj = None
    # to_state = None

    if option==0: # default (run policy in the sim and test with one feedback format)
        return run_simulation(pb, kinova, mdp_env, oracle_policy, objectUids, feedback_file)
    elif option==1:
        import pandas as pd
        import ast
        traj_file = '/home/sharer/hw_test_ws/src/afs_robot_exp/stochastic_output/afs_learned_pref_cs_all_trajs/batch_2_15_25.csv'
        with open(traj_file, 'r') as file:
            lines = file.readlines()
        trajectories = []
        for line in lines[1:]:
            parts = line.strip().split(",", maxsplit=2)
            env, itera, traj = int(parts[0]), int(parts[1]), ast.literal_eval(parts[2])
            trajectories.append(traj)
        for t_id, t in enumerate(trajectories):
            print('Trajectory ', t_id)
            if t_id>=4:
                start_state = mdp_env.start_state
                start_state_xyz = mdp_env.continuous_position(start_state[:2])
                to_state(start_state_xyz[0], start_state_xyz[1], start_state_xyz[2])
                send_traj(t)

        # traj_id = [9, 4, 3, 2, 1, 0]
    elif option==2:
        learn_human_preference(start_compliance, end_compliance, to_state, objectUids, kinova, mdp_env, oracle_policy, naive_policy, num_feedback=5)
    elif option==3:
        afs_output_dir = output_dir+"/afs_learned_pref_random_cs"
        # get_oracle_performance(mdp_env, oracle_policy, naive_policy, afs_output_dir)
        afs(start_compliance, end_compliance, send_traj, to_state, pb, objectUids, kinova, mdp_env, oracle_policy, naive_policy, afs_output_dir, baseline=None, test_map_names=[1], filename=grid_file)
    elif option==4:
        ri_output_dir = output_dir+"/ri"
        ri(start_compliance, end_compliance, send_traj, to_state, pb, objectUids, kinova, mdp_env, ri_output_dir, baseline=None, test_map_names=range(1,4), filename=grid_file)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Run the robot simulation')
    parser.add_argument('--gui', action='store_true', help='Run with GUI')
    args = parser.parse_args()
    
    policy = main(args.gui)

'''
- set the original table extents
- python3.8.10, numpy1.17.4 doesn't work with pybullet>3.2.0
'''
