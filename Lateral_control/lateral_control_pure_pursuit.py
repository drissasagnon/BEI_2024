import numpy as np

def lateral_control(pos_x_temp, pos_y_temp, path):
    """
    Contrôle latéral pour calculer l'angle de braquage basé sur la trajectoire et les nouvelles formules pour le braquage.
    """
    
    # Checks if the trajectory (path) and the vehicle's positions (pos_x_temp and pos_y_temp) are valid
    if not path or not pos_x_temp or not pos_y_temp:
        return 0  # If the data is invalid, return 0 to prevent any calculation error

    # Ensure that pos_x_temp and pos_y_temp are lists of numerical values, not nested lists
    current_pos = np.array([pos_x_temp[-1], pos_y_temp[-1]])  # Last position of the vehicle (x, y)

    # Find the closest point on the trajectory by calculating the distance between the vehicle and each point on the trajectory
    distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]
    closest_idx = np.argmin(distances)  # Index of the trajectory point closest to the vehicle

    # Select the next target point on the trajectory, after the closest point
    next_idx = (closest_idx + 1) % len(path)  # Ensure to return to the beginning of the trajectory if the end is reached
    target_point = np.array(path[next_idx])  # The next target point on the trajectory

    # Calculate the trajectory vector between the vehicle's current position and the target point
    path_vector = target_point - current_pos
    
    # Calculate the angle (alpha) between the vehicle's direction and the direction of the vector towards the target point
    alpha = np.arctan2(path_vector[1], path_vector[0])  # Using np.arctan2 to obtain the 2D angle

    # Vehicle parameters
    L = 2.0  # Distance between the front and rear axles of the vehicle (in meters)
    speed_vehicule = 1.0  # Assumed speed of the vehicle (1 m/s)
    Kdd = 0.5  # Scaling factor for the following distance (looks ahead)

    # Dynamic calculation of the following distance (ld) based on the vehicle's speed
    ld = Kdd * speed_vehicule  # The following distance is proportional to the vehicle's speed (in meters)

    # Calculate the steering angle (delta) using the Pure Pursuit formula
    steering_angle = np.arctan((2 * L * np.sin(alpha)) / ld)  

    return steering_angle  # Return the calculated steering angle, which will be used to control the vehicle's direction

