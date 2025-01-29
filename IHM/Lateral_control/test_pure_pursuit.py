import numpy as np

def test_pure_pursuit():
    """
    Test cases for the Pure Pursuit lateral controller.
    """

    # Trajectoire simple en ligne droite
    path = [(0, 0), (10, 0), (20, 0)]
    pos_x_temp = [0]
    pos_y_temp = [0]

    # Cas 1 : Le véhicule est aligné avec la trajectoire
    speed = 5.0  # m/s
    steering_angle = lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path, speed)
    print(f"Test 1 - Straight line, aligned: Steering angle = {steering_angle:.4f} radians")
    assert abs(steering_angle) < 1e-6  # L'angle doit être proche de 0

    # Cas 2 : Le véhicule est légèrement décalé de la trajectoire
    pos_x_temp = [0]
    pos_y_temp = [1]  # Décalé de 1 mètre
    steering_angle = lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path, speed)
    print(f"Test 2 - Offset from path: Steering angle = {steering_angle:.4f} radians")
    assert steering_angle > 0  # Le véhicule doit tourner vers la trajectoire

    # Cas 3 : Le véhicule est perpendiculaire à la trajectoire
    pos_x_temp = [0]
    pos_y_temp = [0]
    pos_x_temp.append(0)  # La position actuelle
    pos_y_temp.append(1)  # Perpendiculaire
    steering_angle = lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path, speed)
    print(f"Test 3 - Perpendicular to path: Steering angle = {steering_angle:.4f} radians")
    assert steering_angle > 0  # Le véhicule doit corriger sa direction

    # Cas 4 : Le véhicule est à la fin de la trajectoire
    pos_x_temp = [20]
    pos_y_temp = [0]
    steering_angle = lateral_control_pure_pursuit(pos_x_temp, pos_y_temp, path, speed)
    print(f"Test 4 - At the end of the path: Steering angle = {steering_angle:.4f} radians")
    assert abs(steering_angle) < 1e-6  # Aucun ajustement requis

    print("All Pure Pursuit tests passed successfully!")


# Exécutez les tests
#test_pure_pursuit()
