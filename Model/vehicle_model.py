# modele-vehicule.py
import numpy as np

class VehicleModel:
    def __init__(self, initial_x=0, initial_y=0, initial_theta=0):
        self.pos_x = [initial_x]
        self.pos_y = [initial_y]
        self.theta = [initial_theta]

    def update_position(self, steering_angle, speed, time_update):
        direction = self.theta[-1]
        self.pos_x.append(self.pos_x[-1] + speed * np.cos(direction) * time_update)
        self.pos_y.append(self.pos_y[-1] + speed * np.sin(direction) * time_update)
        self.theta.append(self.theta[-1] + steering_angle * time_update)
    
    def get_position(self):
        return self.pos_x[-1], self.pos_y[-1], self.theta[-1]

    def get_rectangle(self):
        # Rectangle representing the vehicle
        rectangle_x = np.array([-1, 1, 1, -1, -1])
        rectangle_y = np.array([-0.5, -0.5, 0.5, 0.5, -0.5])
        
        # Rotation matrix for the vehicle's orientation
        rotation_matrix = np.array([
            [np.cos(self.theta[-1]), -np.sin(self.theta[-1])],
            [np.sin(self.theta[-1]), np.cos(self.theta[-1])]
        ])
        
        # Rotate the rectangle coordinates
        rotated_rectangle = np.dot(rotation_matrix, np.vstack((rectangle_x, rectangle_y)))
        
        # Translate the rectangle to the vehicle's current position
        rotated_rectangle[0, :] += self.pos_x[-1]
        rotated_rectangle[1, :] += self.pos_y[-1]
        
        return rotated_rectangle
