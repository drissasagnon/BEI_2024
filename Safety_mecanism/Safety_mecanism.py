import time
import os
import numpy as np
from Lateral_control.lateral_control_pure_pursuit import lateral_control_pure_pursuit

def safety_mecanism(button_signal, path, vehicle_position, speed, callback=None):
    """
    Mode sécurité en cas de défaillance ECU : ralentissement, déviation et arrêt.

    :param vehicle_model: Objet modèle du véhicule.
    :param button_signal: Booléen indiquant si le bouton ECU Failure est activé.
    :param path: Trajectoire de base du véhicule.
    :param vehicle_position: Position actuelle (x, y).
    :param speed: Vitesse actuelle du véhicule.
    :param callback: Fonction pour afficher des messages (optionnel).
    :return: steering_angle calculé.
    """
    if not button_signal:
        return 0  # Pas d'erreur, retour angle neutre

    # === Génération de la nouvelle trajectoire décalée pour le stationnement ===
    parking_trajectory = []
    shift_distance = 0.5  # Décalage latéral de 0.25m vers la droite

    for i in range(len(path) - 1):
        current = np.array(path[i])
        next_point = np.array(path[i + 1])

        # Calcul du vecteur directionnel
        direction_vector = next_point - current
        unit_vector = direction_vector / (np.linalg.norm(direction_vector) + 1e-6)  # Normalisation
        normal_vector = np.array([-unit_vector[1], unit_vector[0]])  # Perpendiculaire (décalage à droite)

        # Déplacement du point vers la droite
        new_point = current - shift_distance * normal_vector
        parking_trajectory.append(tuple(new_point))


    return parking_trajectory
