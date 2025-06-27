import pybullet as p
import pybullet_data as pd

p.connect(p.GUI) # Connect to the GUI
p.setAdditionalSearchPath(pd.getDataPath()) # Add data path

# Add a button
button_id = p.addUserDebugParameter("Start Simulation", 0, 1, 0)
prev_button_val = p.readUserDebugParameter(button_id) # Initial value

while True:
    p.stepSimulation()
    button_val = p.readUserDebugParameter(button_id)

    if button_val > prev_button_val:
        print("Start Simulation button pressed!") # Perform action
        prev_button_val = button_val

    # ... (rest of your simulation code)
