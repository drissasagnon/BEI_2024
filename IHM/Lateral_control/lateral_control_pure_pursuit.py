import numpy as np

def lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path,speed ,L=2.0, Kdd=0.5):
    """
    Pure Pursuit lateral control for calculating the steering angle.

    Parameters:
        pos_x_temp (list): List of the vehicle's x positions.
        pos_y_temp (list): List of the vehicle's y positions.
        path (list): List of (x, y) points representing the trajectory.
        L (float): Wheelbase of the vehicle (meters).
        speed_vehicle (float): Vehicle speed (m/s).
        Kdd (float): Lookahead distance scaling factor.

    Returns:
        float: Steering angle (radians).
    """
    # Check if path and position history are valid
    if not path or len(pos_x_temp) < 2 or len(pos_y_temp) < 2:
        return 0  # Not enough data to compute steering angle

    # Get the current position of the vehicle
    current_pos = np.array([pos_x_temp[-1], pos_y_temp[-1]])

    # Calculate distances from the current position to all points in the path
    distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]

    # Find the closest point on the path
    closest_idx = np.argmin(distances)

    # Determine the lookahead distance based on speed
    lookahead_distance = max(Kdd * speed, 1.0)  # Ensure a minimum lookahead distance of 1 meter

    # Find the target point within the lookahead distance
    target_point = None
    for i in range(closest_idx, len(path)):
        if np.linalg.norm(np.array(path[i]) - current_pos) >= lookahead_distance:
            target_point = np.array(path[i])
            break

    # If no valid target point is found, use the last point on the path
    if target_point is None:
        target_point = np.array(path[-1])

    # Calculate the vector from the current position to the target point
    path_vector = target_point - current_pos

    # Desired angle to the target point
    alpha = np.arctan2(path_vector[1], path_vector[0])

    # Calculate the current orientation of the vehicle using the last two positions
    dx = pos_x_temp[-1] - pos_x_temp[-2]
    dy = pos_y_temp[-1] - pos_y_temp[-2]
    current_angle = np.arctan2(dy, dx)

    # Calculate the angular error
    angle_error = alpha - current_angle
    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize the error to [-pi, pi]

    # Calculate the steering angle using the Pure Pursuit formula
    steering_angle = np.arctan((2 * L * np.sin(angle_error)) / lookahead_distance)

    # Apply steering angle saturation
    max_steering_angle = np.radians(30)  # Limit steering angle to ±30 degrees
    steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))

    return steering_angle
