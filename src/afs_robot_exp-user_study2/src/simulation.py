import numpy as np
from pybullet_api import *
#from optas.templates import Manager
import casadi as c

def main(gui=False):
    hz = 250
    dt = 1.0 / float(hz)
    pb = PyBullet(dt, gui=gui)
    kinova = Kinova(base_position=[0,0,0.6])
    urdfRootPath=pybullet_data.getDataPath()
    table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.4,0,0])
    objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[0.6,0,0.7])
    q0 = np.deg2rad([0, 15, 180, -130, 0, 55, 90])

    kinova.reset(q0)

    duration = 4.0  # seconds
    #planner = SimpleJointSpacePlanner(urdf_filename=kuka.urdf_filename, ee_link="tool_frame", duration=duration)

    qc = kinova.q()
    qF = np.deg2rad([15, 15, 90, -110, 0, 35, 0])

    #planner.reset(qc, pg, og, q0)

    #plan = planner.plan()
    #print(plan)

    pb.start()

    start = time.time()
    while True:
        t = time.time() - start
        if t < duration:
            #Currently sending static end position (change over time alternatively)
            kinova.cmd(qF)
        else:
            print("Completed motion")
            break

    if gui:
        while True:
            pass

    pb.stop()
    pb.close()

    return 0


if __name__ == "__main__":
    main(True)
