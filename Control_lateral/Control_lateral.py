import numpy as np

def lateral_control(pos_x_temp, pos_y_temp, path):
    """
    Contrôle latéral pour calculer l'angle de braquage basé sur la trajectoire et les nouvelles formules pour le braquage.
    """
    
    # Vérifie si la trajectoire (path) et les positions du véhicule (pos_x_temp et pos_y_temp) sont valides
    if not path or not pos_x_temp or not pos_y_temp:
        return 0  # Si les données sont invalides, retourne 0 pour éviter toute erreur de calcul

    # Assure que pos_x_temp et pos_y_temp sont des listes de valeurs numériques, et non des listes imbriquées
    current_pos = np.array([pos_x_temp[-1], pos_y_temp[-1]])  # Dernière position du véhicule (x, y)

    # Trouve le point le plus proche sur la trajectoire en calculant la distance entre le véhicule et chaque point de la trajectoire
    distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]
    closest_idx = np.argmin(distances)  # L'indice du point de la trajectoire le plus proche du véhicule

    # Sélectionne le prochain point cible dans la trajectoire, après le point le plus proche
    next_idx = (closest_idx + 1) % len(path)  # S'assure de revenir au début de la trajectoire si on atteint la fin
    target_point = np.array(path[next_idx])  # Le point cible suivant sur la trajectoire

    # Calcul du vecteur de la trajectoire entre la position actuelle du véhicule et le point cible
    path_vector = target_point - current_pos
    
    # Calcul de l'angle (alpha) entre la direction du véhicule et la direction du vecteur vers le point cible
    alpha = np.arctan2(path_vector[1], path_vector[0])  # Utilisation de np.arctan2 pour obtenir l'angle en 2D

    # Paramètres du véhicule
    L = 2.0  # Distance entre les essieux avant et arrière du véhicule (en mètres)
    speed_vehicule = 1.0  # Vitesse supposée du véhicule (1 m/s)
    Kdd = 0.5  # Facteur de mise à l'échelle pour la distance de suivi (regarde l'avance)

    # Calcul dynamique de la distance de suivi (ld) basée sur la vitesse du véhicule
    ld = Kdd * speed_vehicule  # La distance de suivi est proportionnelle à la vitesse du véhicule (en mètres)

    # Calcul de l'angle de braquage (delta) en utilisant la formule de Pure Pursuit
    steering_angle = np.arctan((2 * L * np.sin(alpha)) / ld)  # Utilisation de la formule de Pure Pursuit pour déterminer l'angle de braquage

    return steering_angle  # Retourne l'angle de braquage calculé, qui sera utilisé pour contrôler la direction du véhicule
