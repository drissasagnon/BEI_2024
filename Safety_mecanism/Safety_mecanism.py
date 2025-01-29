####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
#safety_mecanism.py

import numpy as np
from Autopilot.autopilot import autopilot_step

def safety_mecanism(ecu_failure, path, pos_x_temp, pos_y_temp, theta_temp, velocity_temp, time, timeUpdate):
    """
    Handles ECU failure by shifting to a safe trajectory, gradually braking, and stopping the vehicle.

    :param ecu_failure: Boolean indicating whether the ECU failure mode is activated.
    :param path: The vehicle's original path.
    :param pos_x_temp: List of past x positions.
    :param pos_y_temp: List of past y positions.
    :param theta_temp: List of past orientations.
    :param velocity_temp: List of past velocities.
    :param time: Current simulation time.
    :param timeUpdate: Time step of the simulation.
    :return: (updated_speed, updated_steering_angle, stop_message)
    """
    if not ecu_failure:
        return velocity_temp[-1], 0, None  # No failure, return current speed and neutral steering angle

    # === Generate a new shifted trajectory for safe parking ===
    parking_trajectory = []
    shift_distance = 0.5  # Lateral shift of 0.5m to the right

    for i in range(len(path) - 1):
        current = np.array(path[i])
        next_point = np.array(path[i + 1])

        # Compute the directional vector
        direction_vector = next_point - current
        unit_vector = direction_vector / (np.linalg.norm(direction_vector) + 1e-6)  # Normalize
        normal_vector = np.array([-unit_vector[1], unit_vector[0]])  # Perpendicular (right shift)

        # Shift the point to the right
        new_point = current - shift_distance * normal_vector
        parking_trajectory.append(tuple(new_point))

    # === Gradually reduce speed (simulate braking) ===
    if velocity_temp[-1] > 0.5:
        updated_speed = max(0.5, velocity_temp[-1] - 0.1)
    else:
        updated_speed = velocity_temp[-1]  # Keep speed if already low

    # === Apply autopilot to follow the parking trajectory ===
    if time < 5:  
        updated_steering_angle = autopilot_step(
            pos_x_temp, pos_y_temp, parking_trajectory, theta_temp, timeUpdate, updated_speed
        )
        stop_message = None
    else:
        # Stop vehicle after 5s
        updated_speed = 0  
        updated_steering_angle = 0
        stop_message = f"✅ Vehicle stopped in a safe zone.\n📍 Final position: ({pos_x_temp[-1]:.2f}, {pos_y_temp[-1]:.2f})"

    return updated_speed, updated_steering_angle, stop_message
