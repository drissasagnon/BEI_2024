# simulator
# How to Use the Simulator

In order to launch the software you need to install the dependencies. Python >= 3.12 is required :

```
pip install -r requirements.txt
```

Run main_IHM.py to launch the simulator (``python main_IHM.py``)

# Structure of the Simulator
# The simulator is organized into several components, each responsible for a specific functionality. Below is the structure of the project with an explanation of each part:

autopilot/
├── autopilot.py               # Contains the main logic for the autopilot system, including safety features.

ihm/
├── ihm.py                     # Handles the User Interface (UI) logic and interactions.

lateral_control/
├── proportional_control.py    # Implements the proportional lateral control algorithm.
├── pure_pursuit_control.py    # Implements the pure pursuit lateral control algorithm.

model/
├── vehicle_model.py           # Defines the mathematical model of the vehicle dynamics.

trajectory/
├── generate_trajectory.py     # Contains functions to generate and manage trajectories.

main_IHM.py                     # The entry point of the simulator. Launches the application.

requirements.txt                # Lists all the Python dependencies required for the project.
