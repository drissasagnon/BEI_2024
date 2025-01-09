# Control_lateral de type correcteur proportionnel
import numpy as np

def lateral_control(pos_x_temp, pos_y_temp, path):
    """
    Lateral control to calculate steering angle based on the trajectory and the new formulas for steering.
    """
    if not path or not pos_x_temp or not pos_y_temp:
        return 0

    # Ensure pos_x_temp and pos_y_temp are lists of numerical values, not lists of lists
    current_pos = np.array([pos_x_temp[-1], pos_y_temp[-1]])

    # Find the closest point on the trajectory
    distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]
    closest_idx = np.argmin(distances)

    # Select the next target point
    next_idx = (closest_idx + 1) % len(path)
    target_point = np.array(path[next_idx])

    # Calculate the path vector and its angle (alpha)
    path_vector = target_point - current_pos
    alpha = np.arctan2(path_vector[1], path_vector[0])

    # Vehicle parameters
    L = 0.5  # Distance between the front and rear axles of the vehicle (example value)
    ld = 1.0  # Distance from the rear axle to the point where the curvature is applied (example value)

    # Calculate steering angle delta using the formula provided
    angle_error = np.arctan((2 * L * np.sin(alpha)) / (ld))
    k_p = 2.0
    steering_angle = k_p * angle_error
    return steering_angle
