import sim
import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

def init():
    sim.simxFinish(-1)  # Cierre cualquier conexión previa
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Conéctate a CoppeliaSim

    if clientID != -1:
        print('Conexión establecida con CoppeliaSim')
    else:
        print('No se pudo establecer la conexión con CoppeliaSim')
        sys.exit('No se pudo conectar')

    # Obtener la señal en forma de cadena
    returnCode, data = sim.simxGetStringSignal(clientID, "A", sim.simx_opmode_blocking)
        
    measuredData = sim.simxUnpackFloats(data)
    
    if returnCode == sim.simx_return_ok:
        # El valor de la señal se encuentra en data
        print("Valor de la señal:", measuredData[0:3])
    else:
        print("No se pudo obtener la señal.", measuredData)

    #convertir en una lista de cordenadas [[x, y, z], [x1, y1, z1]]
    measures = []
    print(len(measuredData))
    for i in range(len(measuredData)//3):
        measures.append([measuredData[i*3], measuredData[i*3+1], measuredData[i*3+2]])
            
    print(measures[0])
init()
