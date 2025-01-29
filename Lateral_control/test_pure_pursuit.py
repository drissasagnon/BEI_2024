####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
# test_pure_pursuit.py

import pytest
import numpy as np
from Lateral_control.lateral_control_pure_pursuit import lateral_control_pure_pursuit

def test_straight_path():
    """Test sur trajectoire droite"""
    pos_x = [0.0, 1.0, 2.0]
    pos_y = [0.0, 0.0, 0.0]
    path = [(x, 0.0) for x in range(0, 10)]
    steering = lateral_control_pure_pursuit(pos_x, pos_y, path)
    assert abs(steering) < 1e-6

def test_sharp_right_turn():
    """Test de virage serré à droite"""
    pos_x = [4.0, 4.1]
    pos_y = [0.0, 0.0]
    path = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    steering = lateral_control_pure_pursuit(pos_x, pos_y, path)
    assert np.isclose(steering, np.radians(30), atol=0.1)

def test_empty_path_handling():
    """Test de gestion de trajectoire vide"""
    steering = lateral_control_pure_pursuit([0.0, 1.0], [0.0, 0.0], [])
    assert steering == 0.0

def test_low_speed_behavior():
    """Test à basse vitesse"""
    pos_x = [0.0, 0.1]
    pos_y = [0.0, 0.0]
    path = [(x, 0.0) for x in range(0, 10)]
    steering = lateral_control_pure_pursuit(pos_x, pos_y, path, 0.5)
    assert abs(steering) < np.radians(5)

def test_steering_saturation():
    """Test de saturation de l'angle de braquage"""
    pos_x = [0.0, 0.1]
    pos_y = [0.0, 0.0]
    path = [(100.0, 100.0)]
    steering = lateral_control_pure_pursuit(pos_x, pos_y, path)
    assert abs(steering) == np.radians(30)

def test_off_path_correction():
    """Test de correction de déviation latérale initiale"""
    # Véhicule dévié à 1m au-dessus du chemin désiré
    pos_x = [0.0, 0.5]  # Avance en X
    pos_y = [1.0, 1.0]  # Reste bloqué à Y=1m
    path = [(x, 0.0) for x in range(0, 10)]  # Chemin idéal sur Y=0
    
    steering = lateral_control_pure_pursuit(pos_x, pos_y, path)
    
    # Vérifications
    assert steering < -np.radians(15)  # Braquage fort vers le bas (Y décroissant)
    assert steering == -np.radians(30)  # Respect saturation
   