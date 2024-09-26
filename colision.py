def detect_blocked_directions(robot_corners, obstacles):
    # Asumimos que el frente del robot está hacia el eje positivo y.
    threshold_forward = 0.5  # Ajustar según el tamaño y la tolerancia del robot
    threshold_side = 0.5  # Este umbral debe ser menor que la mitad de la anchura del robot para evitar detecciones diagonales.

    blocked_directions = {'adelante': False, 'atras': False, 'izquierda': False, 'derecha': False}

    # Calcular las coordenadas mínimas y máximas del rectángulo del robot
    min_x = min(-corner[0] for corner in robot_corners)
    max_x = max(-corner[0] for corner in robot_corners)
    min_y = min(-corner[1] for corner in robot_corners)
    max_y = max(-corner[1] for corner in robot_corners)

    for obstacle in obstacles:
        obstacle_x, obstacle_y = obstacle

        # Comprobar si el obstáculo está directamente alineado con el robot en el eje Y
        if min_x <= obstacle_x <= max_x:
            if obstacle_y > max_y and obstacle_y - max_y <= threshold_forward:
                blocked_directions['adelante'] = True
            elif obstacle_y < min_y and min_y - obstacle_y <= threshold_forward:
                blocked_directions['atras'] = True

        # Comprobar si el obstáculo está directamente alineado con el robot en el eje X
        if min_y <= obstacle_y <= max_y:
            if obstacle_x < min_x and min_x - obstacle_x <= threshold_side:
                blocked_directions['izquierda'] = True
            elif obstacle_x > max_x and obstacle_x - max_x <= threshold_side:
                blocked_directions['derecha'] = True

    return blocked_directions




# Probando la función con tus datos
robot_corners = [(0.5, 0.5), (0.5, 1.5), (1.5, 0.5), (1.5, 1.5)]  # Coordenadas de las esquinas del robot
obstacles = [(0 , 2), (0, 1), (0, 0), (2, 2) ]  # Lista de obstáculos

print(detect_blocked_directions(robot_corners, obstacles))

