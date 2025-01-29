####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
# autopilot.py

from Lateral_control.lateral_control_pure_pursuit import lateral_control_pure_pursuit

def autopilot_step(pos_x_temp, pos_y_temp, path, theta_temp, time_update,speed):
    """
    Function to calculate the steering angle and update the vehicle's orientation in autopilot mode.
    """
    steering_angle = lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path,speed)  # Calculate the steering angle using lateral control
    theta_temp.append(theta_temp[-1] + steering_angle * time_update)   # Update the orientation
    return steering_angle
