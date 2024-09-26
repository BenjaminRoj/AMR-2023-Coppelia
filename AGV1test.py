import sim
import math
import sys
import time
import numpy as np
import Titulo as tl
import test_slam as slam
import matplotlib.pyplot as plt
import test1 as dbs

def detect_blocked_directions(robot_corners, obstacles):
    # Asumimos que el frente del robot está hacia el eje positivo y.
    threshold_forward = 1  # Ajustar según el tamaño y la tolerancia del robot
    threshold_side = 1  # Este umbral debe ser menor que la mitad de la anchura del robot para evitar detecciones diagonales.

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





sim.simxFinish(-1)  # Cierre cualquier conexión previa
clientID = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)  # Conéctate a CoppeliaSim

if clientID != -1:
    print('Conexión establecida con CoppeliaSim')
else:
    print('No se pudo establecer la conexión con CoppeliaSim')
    sys.exit('No se pudo conectar')
        
# Obtener identificadores de los motores
returnCode,h1 = sim.simxGetObjectHandle(clientID, ':/FLwheel_motor_2', sim.simx_opmode_blocking)
returnCode,h2 = sim.simxGetObjectHandle(clientID, './FRwheel_motor_2', sim.simx_opmode_blocking)
returnCode,h3 = sim.simxGetObjectHandle(clientID, './RRwheel_motor_2', sim.simx_opmode_blocking)
returnCode,h4 = sim.simxGetObjectHandle(clientID, './RLwheel_motor_2', sim.simx_opmode_blocking )
h = [h1, h2, h3, h4]

v = 100 * math.pi / 180
plt.figure()
#tl.MoveForward(v, clientID, h)

#mapa principal de bodega, redondeo para hacer grilla

grid = dbs.create_grid((-2.4, 2.4), (2.4, -2.3), 0.1)

while 1:
    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
    ##############################################################################################################################################################################
    points = [p1, p2, p3, p4]
    
    plt.clf()
    m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
    if m < -0.04 or m > 0.04:
        points = tl.enderezar(m, clientID, h)
        print(points)

    #tl.MoveForward(v, clientID, h)
    x_positions, y_positions = slam.Slam_Kalman(clientID, points)
    
    obstacles =  list(zip(x_positions, y_positions))
    bloqueos = slam.detect_blocked_directions(points, obstacles)
    print(bloqueos)
    center = tl.find_center_of_square(points)
    center = (round(center[0], 1), round(center[1], 1))
    #print(center)
    path = dbs.move_robot(grid, center, (0.0, 0.0), bloqueos)
    #print(path)
    goal = (0,0)
    while 1:
        path = dbs.move_robot(grid, center, (0.0, 0.0), bloqueos)
        if path:
            #print(path)
            _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
            _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
            _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
            _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
            ##############################################################################################################################################################################
            points = [p1, p2, p3, p4]
            center = tl.find_center_of_square(points)
            center = (round(center[0], 1), round(center[1], 1))
            
            distancia = math.sqrt((center[0] - goal[0])**2 + (center[0] - goal[1])**2)
            if distancia < 0.3:
                v = 50 * math.pi / 180
                print("v------")
            bloqueos = slam.detect_blocked_directions(points, obstacles)
            print(center)
            print(path[0])
            #    continue
            if center == (0.0, 0.0):
                tl.Stop(clientID, h)
                print("bajfjfdkjafk")
                continue
            
            elif path[0] == 'adelante' and path[0] in bloqueos:
                while 1:
                    print("aaaaaaaaaaaaaa")
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
                    points = [p1, p2, p3, p4]
                    m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
                    if m < -0.04 or m > 0.04:
                        points = tl.enderezar(m, clientID, h)

                    
                    mov = tl.find_center_of_square(points)
                    
                    tl.MoveForward(v, clientID, h)
                    if round(abs(mov[1] - center[1]), 1) == 0.1:
                        tl.MoveForward(v/2, clientID, h)
                        break
                    bloqueos = slam.detect_blocked_directions(points, obstacles)
                    if path[0] not in bloqueos:
                        tl.Stop(clientID, h)
                        path = dbs.move_robot(grid, center, (0.0, 0.0), bloqueos)
                        #print(path)
                        break
                    ##############################################################################################################################################################################
            elif path[0] == 'atras' and path[0] in bloqueos:
                print("ajajaja")
                while 1:
                    tl.MoveBackward(v, clientID, h)
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
                    m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
                    if m < -0.04 or m > 0.04:
                        points = tl.enderezar(m, clientID, h)
                    points = [p1, p2, p3, p4]
                    mov = tl.find_center_of_square(points)
                    if round(abs(mov[1] - center[1]), 1) == 0.1:
                        tl.MoveBackward(v/2, clientID, h)
                        print("hola\n\naaaa")
                        break
                    if path[0] in bloqueos:
                        print("hola")
                        tl.Stop(clientID, h)
                        path = dbs.move_robot(grid, center, (0.0, 0.0), bloqueos)
                        
                        break
                    #tl.MoveBackward(v, clientID, h)
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
                    ##############################################################################################################################################################################
            elif path[0] == 'izquierda'and path[0] in bloqueos:
                while 1:
                    tl.MoveLeft(v, clientID, h)
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
                    m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
                    if m < -0.04 or m > 0.04:
                        points = tl.enderezar(m, clientID, h)
                    points = [p1, p2, p3, p4]
                    mov = tl.find_center_of_square(points)
                    
                    if round(abs(mov[0] - center[0]), 1) == 0.1:
                        tl.MoveLeft(v/2, clientID, h)
                        break
                    if path[0] not in bloqueos:
                        tl.Stop(clientID, h)
                        path = dbs.move_robot(grid, center, (0.0, 0.0), bloqueos)
                        print(path)
                        break
                    #tl.MoveLeft(v, clientID, h)
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)               
            elif path[0] == 'derecha'and path[0] in bloqueos:
                while 1:
                    tl.MoveRight(v, clientID, h)
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
                    m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
                    if m < -0.04 or m > 0.04:
                        points = tl.enderezar(m, clientID, h)
                    points = [p1, p2, p3, p4]
                    mov = tl.find_center_of_square(points)
                    if round(abs(mov[0] - center[0]), 1) == 0.1:
                        print("ea")
                        tl.MoveRight(v/2, clientID, h)
                        break
                    if path[0] not in bloqueos:
                        tl.Stop(clientID, h)
                        path = dbs.move_robot(grid, center, (0.0, 0.0), bloqueos)
                        print(path)
                        break
                    #tl.MoveRight(v, clientID, h)
                    _, p1 = sim.simxGetObjectPosition(clientID, h1, -1, sim.simx_opmode_blocking)
                    _, p2 = sim.simxGetObjectPosition(clientID, h2, -1, sim.simx_opmode_blocking)
                    _, p3 = sim.simxGetObjectPosition(clientID, h3, -1, sim.simx_opmode_blocking)
                    _, p4 = sim.simxGetObjectPosition(clientID, h4, -1, sim.simx_opmode_blocking)
                    ##############################################################################################################################################################################
        else:
            print("noooooooo")
            break
        path.pop(0)

            
            
            
    '''          
    #break

    #break
    plt.plot([2.356140389882804, -2.385794158162124, -2.385794158162124, 2.356140389882804, 2.356140389882804],
             [2.41534566138077, 2.41534566138077, -2.3198942920368557, -2.3198942920368557, 2.41534566138077])
    plt.scatter(x_positions, y_positions, color='blue', label="a")
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.title('AGV 1')
    plt.grid()
    slam.simulate_robot_path(x_positions, y_positions, clientID, points)
    
    
    time.sleep(0.1)
    '''
plt.show()
#tl.MoveForward(v, clientID, h)


sim.simxGetPingTime(clientID)  # Asegurarse de que no se bloquea
sim.simxFinish(clientID)  # Cerrar la conexión al final

