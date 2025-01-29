# simulator

**Note:** This simulator is not the same as last year's version. It is a simplified example containing only the basic components. It was created to test the addition of trajectory generation, different types of lateral controllers, and the safety mechanism. The initial idea was to validate these features before reintegrating them into last year's full simulator. However, due to time constraints, only the trajectory addition was fully integrated, and the work on lateral controllers had started but was not completed.

# How to Use the Simulator

In order to launch the software you need to install the dependencies. Python >= 3.12 is required :

```
pip install -r requirements.txt
```

Run main_IHM.py to launch the simulator (``python main_IHM.py``)


### Running Tests

To run the unit tests for the Pure Pursuit lateral control algorithm, use the following command:

```
python3 -m pytest Lateral_control/test_pure_pursuit.py -v
```

## Structure of the Simulator

The simulator is organized into several components, each responsible for a specific functionality. Below is the structure of the project with an explanation of each part:

autopilot/
├── autopilot.py               # Contains the main logic for the autopilot system, including safety features.

ihm/
├── ihm.py                     # Handles the User Interface (UI) logic and interactions.

lateral_control/
├── proportional_control.py    # Implements the proportional lateral control algorithm.
├── pure_pursuit_control.py    # Implements the pure pursuit lateral control algorithm.
├── test_pure_pursuit.py       # Unit tests for the Pure Pursuit lateral control algorithm using pytest.
├──lateral_control_pure_pursuit_parameters.json # Configuration file for the Pure Pursuit lateral control algorithm.

Logs/
├── failure_logtxt             # Stores failure logs and error messages for debugging.

model/
├── vehicle_model.py           # Defines the mathematical model of the vehicle dynamics.

Safety_mecanism/
├──safety_mecanism.py          # Handles ECU failure by generating a safe parking trajectory, gradually reducing speed,
├                                and steering the vehicle to a controlled stop.
├
├──Alarme.wav                  # Audio alert used for safety warnings.

trajectory/
├── generate_trajectory.py     # Contains functions to generate and manage trajectories.

main_IHM.py                     # The entry point of the simulator. Launches the application.

requirements.txt                # Lists all the Python dependencies required for the project.
