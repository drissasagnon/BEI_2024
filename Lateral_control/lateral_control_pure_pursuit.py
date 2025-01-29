####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
#lateral_control_pure_pursuit.py

import numpy as np
import json
import os

# Load Pure Pursuit control parameters from JSON
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "lateral_control_pure_pursuit_parameters.json")

def load_pure_pursuit_config():
    """Load Pure Pursuit control parameters from a JSON file."""
    try:
        with open(CONFIG_PATH, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Error: Configuration file not found at {CONFIG_PATH}")
        return None
    except json.JSONDecodeError:
        print(f"Error: Failed to decode JSON from {CONFIG_PATH}")
        return None

# Load configuration
pure_pursuit_config = load_pure_pursuit_config()

# Ensure configuration is loaded properly
if pure_pursuit_config is None:
    raise RuntimeError("Failed to load Pure Pursuit configuration. Please check 'lateral_control_pure_pursuit_parameters.json'.")

# Extract parameters
Kdd = pure_pursuit_config["lookahead_gain"]  # Lookahead gain factor
min_lookahead_distance = pure_pursuit_config["min_lookahead_distance"]  # Minimum lookahead distance
default_speed = pure_pursuit_config["default_speed"]  # Default vehicle speed
max_steering_angle = np.radians(pure_pursuit_config["max_steering_angle_deg"])  # Max steering angle in radians
L = pure_pursuit_config["L"]  # Wheelbase of the vehicle (meters)

def lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path, speed=default_speed):
    """
    Pure Pursuit lateral control for calculating the steering angle.

    Parameters:
        pos_x_temp (list): List of the vehicle's x positions.
        pos_y_temp (list): List of the vehicle's y positions.
        path (list): List of (x, y) points representing the trajectory.
        speed (float): Vehicle speed (m/s), default from config.

    Returns:
        float: Steering angle (radians).
    """
    if not path or len(pos_x_temp) < 2 or len(pos_y_temp) < 2:
        return 0  # Not enough data to compute steering angle

    # Get current position
    current_pos = np.array([pos_x_temp[-1], pos_y_temp[-1]])

    # Compute distances from current position to all path points
    distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]

    # Find closest path point
    closest_idx = np.argmin(distances)

    # Compute lookahead distance based on speed
    lookahead_distance = max(Kdd * speed, min_lookahead_distance)

    # Find the target point on the path within the lookahead distance
    target_point = None
    for i in range(closest_idx, len(path)):
        if np.linalg.norm(np.array(path[i]) - current_pos) >= lookahead_distance:
            target_point = np.array(path[i])
            break

    if target_point is None:
        target_point = np.array(path[-1])

    # Compute vector to target
    path_vector = target_point - current_pos

    # Compute desired angle
    alpha = np.arctan2(path_vector[1], path_vector[0])

    # Compute vehicle's current orientation
    dx = pos_x_temp[-1] - pos_x_temp[-2]
    dy = pos_y_temp[-1] - pos_y_temp[-2]
    current_angle = np.arctan2(dy, dx)

    # Compute angular error
    angle_error = alpha - current_angle
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize to [-pi, pi]

    # Compute steering angle using Pure Pursuit formula
    steering_angle = np.arctan((2 * L * np.sin(angle_error)) / lookahead_distance)

    # Apply steering angle limit
    steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))

    return steering_angle
