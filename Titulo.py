import sim
import math
import sys
import time

def find_center_of_square(square_corners):
    # Asumiendo que square_corners es una lista de cuatro tuplas (x, y),
    # donde cada tupla representa una esquina del cuadrado.
    # El centro de un cuadrado con coordenadas de esquinas conocidas puede ser encontrado
    # como el promedio de las coordenadas X y el promedio de las coordenadas Y.

    # Desempaquetar todas las coordenadas x y y
    x_coords, y_coords, _= zip(*square_corners)
    x_coords = [-x for x in x_coords]
    y_coords = [-y for y in y_coords]
    # Calcular el promedio de las coordenadas x y y
    center_x = sum(x_coords) / len(x_coords)
    center_y = sum(y_coords) / len(y_coords)

    # El centro es el punto (center_x, center_y)
    return (center_x, center_y)

def enderezar(m, clientID, h):
    while m < -0.04 or m > 0.04:
        _, p1 = sim.simxGetObjectPosition(clientID, h[0], -1, sim.simx_opmode_blocking)
        _, p2 = sim.simxGetObjectPosition(clientID, h[1], -1, sim.simx_opmode_blocking)
        _, p3 = sim.simxGetObjectPosition(clientID, h[2], -1, sim.simx_opmode_blocking)
        _, p4 = sim.simxGetObjectPosition(clientID, h[3], -1, sim.simx_opmode_blocking)
            ##############################################################################################################################################################################
        points = [p1, p2, p3, p4]
        m = (points[1][1] - points[0][1]) / (points[1][0] - points[0][0])
        # Calcular el ángulo en radianes
        theta_radians = math.atan(m)
        if m < -0.04:
            TurnLeft(20*math.pi / 180, clientID, h)
        elif m > 0.04 :
            TurnRight(20*math.pi / 180, clientID, h)
        else:
            Stop(clientID, h)
            return [p1, p2, p3, p4]
        
#-------------Mover diagonal atras izquierda-------------
def MoveAI(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], 0, sim.simx_opmode_blocking)
#-------------Mover diagonal adelante derecha------------
def MoveAD(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], 0, sim.simx_opmode_blocking)
#-------------Mover diagonal adelante izquierda
def MoveAI(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], v, sim.simx_opmode_blocking)
#-------------Mover diagonal adelante izquierda-------------
def MoveAIZ(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], v, sim.simx_opmode_blocking)
#-------------Mover diagonal atras derecha-------------
def MoveADD(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], -v, sim.simx_opmode_blocking)
#-------------Girar a la derecha-------------
def TurnRight(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], v, sim.simx_opmode_blocking)
#-------------Girar a la izquierda-------------
def TurnLeft(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], -v, sim.simx_opmode_blocking)
#-------------Mover hacia adelante-------------
def MoveForward(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], v, sim.simx_opmode_blocking)
#-------------Mover hacia atras-------------
def MoveBackward(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], -v, sim.simx_opmode_blocking)
#-------------Mover hacia izquierda-------------
def MoveLeft(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], v, sim.simx_opmode_blocking)
#-------------Mover hacia derecha-------------
def MoveRight(v, clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], -v, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], -v, sim.simx_opmode_blocking)
#-------------Detenerse-------------
def Stop(clientID, h):
    sim.simxSetJointTargetVelocity(clientID, h[0], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[1], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[2], 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, h[3], 0, sim.simx_opmode_blocking)
    

    #h1 rueda delantera izquierda + adelante - atras
    #h2 rueda delantera derecha - adelante + atras
    #h3 rueda trasera derecha - adelante + atras
    #h4 rueda trasera izquierda + adelante - atras


