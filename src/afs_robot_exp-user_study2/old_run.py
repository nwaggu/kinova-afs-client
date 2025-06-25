from src.vi import value_iteration
from src.old_mdp import ArmMDP
from src.helper_functions import *

user_id = 1
output_dir = 'output'
feedback_file = os.path.join(output_dir, "feedback_from_user_"+str(user_id)+".csv")
os.makedirs(output_dir, exist_ok=True)

def main(gui=False):
    """Main function to run the modularized simulation."""
    pb, kinova, urdfRootPath = setup_simulation(gui)

    # Initialize MDP environment
    start_pos = (0, 2, 0)
    goal = (4, 2)
    obj_pos = (2, 2)  # Obstacle position in grid space
    mdp_env = ArmMDP(grid_size=GRID_SIZE, goal=goal, obstacles={obj_pos}, incomplete_reward=False)

    baselines = 5

    objectUid = setup_environment(urdfRootPath, mdp_env, obj_pos)
    policy = value_iteration(mdp_env)
    # Visualize table, obstacles, and goal
    visualize_scene(mdp_env, goal)

    # Run simulation loop
    for type in range(baselines):
        if type==0: # default (run policy in the sim and test with one feedback format)
            return run_simulation(pb, kinova, mdp_env, policy, start_pos, goal, objectUid, feedback_file)
        elif type==1: # AFS
            pass
            # return AFS()

if __name__ == "__main__":
    policy = main(True)
