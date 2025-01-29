import numpy as np

def lateral_control_proportional(pos_x_temp, pos_y_temp, path):
    """
    Proportional lateral controller to adjust the vehicle's steering angle based on the angular error.
    """
    # Verify that the trajectory and positions are valid
    if not path or len(pos_x_temp) < 2 or len(pos_y_temp) < 2:
        return 0  # Not enough data to calculate the steering angle

    # Extracting the vehicle's last position
    current_pos = np.array([pos_x_temp[-1], pos_y_temp[-1]])

    # Find the closest point on the trajectory
    distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]
    closest_idx = np.argmin(distances)

    # Select the next point on the trajectory
    next_idx = (closest_idx + 1) % len(path)
    target_point = np.array(path[next_idx])

    # Calculate the trajectory vector and the desired angle
    path_vector = target_point - current_pos
    desired_angle = np.arctan2(path_vector[1], path_vector[0])

    # Calculer l'orientation actuelle du véhicule avec les deux derniers points
    dx = pos_x_temp[-1] - pos_x_temp[-2]
    dy = pos_y_temp[-1] - pos_y_temp[-2]
    current_angle = np.arctan2(dy, dx)

    # Calculate the current orientation of the vehicle using the last two points
    angle_error = desired_angle - current_angle
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalization of the error

    # Proportional controller to adjust the steering angle
    k_p = 2.0 # Adjustable proportional gain
    steering_angle = k_p * angle_error  # Calculation of the steering angle

    return steering_angle  # Return the adjusted steering angle