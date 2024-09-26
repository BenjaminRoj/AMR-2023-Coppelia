import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
import sim
import math
import sys
import time

# Función para rotar un punto (x, y) alrededor del origen (0, 0)
def rotate_point(x, y, theta, p):
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    x_rotated = (x) * cos_theta - (y) * sin_theta
    y_rotated = (x) * sin_theta + (y) * cos_theta
    return x_rotated, y_rotated

# Función para rotar toda la lista de puntos
def rotate_points(data, theta, p):
    if p == 0: p=(0,0)
    return [rotate_point(x, y, theta, p) for x, y, z in data]
def rotate_points2(data, theta, p):
    return [rotate_point(x, y, theta, p) for x, y in data]

def leer_sensores(clientID, a, b):
    # Obtener la señal en forma de cadena
    returnCode, data1 = sim.simxGetStringSignal(clientID, "C", sim.simx_opmode_blocking)
    returnCode, data2 = sim.simxGetStringSignal(clientID, "D", sim.simx_opmode_blocking)
    lista_entrada1 = sim.simxUnpackFloats(data1)
    lista_entrada2 = sim.simxUnpackFloats(data2)
    # Usamos list comprehension para crear la lista de tuplas
    data1 = [(lista_entrada1[i], lista_entrada1[i+1], lista_entrada1[i+2]) for i in range(0, len(lista_entrada1), 3)]
    data2 = [(lista_entrada2[i], lista_entrada2[i+1], lista_entrada2[i+2]) for i in range(0, len(lista_entrada2), 3)]
    data2 = [(-x, -y, z) for x, y, z in data2]

    # Posición relativa entre los sensores (dx, dy)
    dx = 1.2  # Cambia esto según la separación real entre los sensores
    dy = 0.3  # Cambia esto según la separación real entre los sensores

    # Alinea las coordenadas del sensor trasero (data2) según la posición relativa
    data2 = [(x - dx/2, -(y + dy/2), z) for x, y, z in data2]
    data1 = [(x + dx/2, -(y - dy/2), z) for x, y, z in data1]

    # Supongamos que 'theta' es el ángulo en radianes que queremos rotar nuestros puntos
    # Por ejemplo, si quieres rotar 90 grados, sería np.pi/2
    theta = np.pi*13/ 4  # Cambia esto por el ángulo correcto de rotación

    # Aplicamos la rotación a los datos de los sensores
    data1 = rotate_points(data1, theta, 0)
    data2 = rotate_points(data2, theta, 0)
    
    return data1+data2


"""Calcula la poscion del robot y la muestra en grafico"""
def simulate_robot_path(x_positions, y_positions, clientID, points):
    x, y, z = zip(*points)
    #AGV
    plt.fill([-x[0], -x[1], -x[2], -x[3], -x[0]],
         [-y[0], -y[1], -y[2], -y[3], -y[0]], c='orange')
    
    
    plt.pause(0.1)  # Pausa para la animación
    # plt.clf() #borrar los datos del grafico para actualizarlo
    plt.legend()
    plt.show()


def Slam_Kalman(clientID, points, a, b):
    # Crear un filtro de Kalman extendido (EKF) para estimar la posición y la orientación
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.x = np.array([0, 0, 0, 0])  # Estado inicial [x, y, vx, vy]
    kf.F = np.array([[1, 0, 1, 0],
                    [0, 1, 0, 1],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])  # Matriz de transición de estado
    kf.H = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0]])  # Matriz de medición
    kf.P *= 10  # Covarianza inicial
    kf.R = np.diag([1, 1])  # Covarianza del ruido de la medición

    # Listas para almacenar las coordenadas
    x_positions = []
    y_positions = []

    # Calculate centroid
    centroid_x = sum(p[0] for p in points) / len(points)
    centroid_y = sum(p[1] for p in points) / len(points)

    pos = (centroid_x, centroid_y)

    x_positions = [x - pos[0] for x in x_positions]
    y_positions = [y - pos[1] for y in y_positions]
    m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])

    # Calcular el ángulo en radianes
    theta_radians = math.atan(m)
    
    data = leer_sensores(clientID, a, b)
    mapa = rotate_points2(data, theta_radians, pos)


    for coord in mapa:
        x, y = coord
        z = np.array([x, y])
        
        # Predicción del filtro de Kalman
        kf.predict()
        
        # Actualización del filtro de Kalman con la medición
        kf.update(z)
        
        # Obtener la estimación de la posición
        x_est, y_est, _, _ = kf.x
        
        # Guardar las estimaciones
        x_positions.append(x_est)
        y_positions.append(y_est)

    #mapa = list(zip(x_positions, y_positions))
    #mapa = rotate_points2(mapa, theta_radians, pos)
    x_positions, y_positions = zip(*mapa)
    #se dezplaza con la rotacion pero queda derecho
    x_positions = [x - pos[0] for x in x_positions]
    y_positions = [y - pos[1] for y in y_positions]

    # Graficar las estimaciones
    return x_positions, y_positions
    

#simulate_robot_path(x_positions, y_positions, clientID, points)




















