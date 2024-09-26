import multiprocessing
import logging

import AGV1

def AGVs(L):
    
    AGV1.AGV(L[0],L[1],L[3],L[2], L[4])
    print(L)
    


if __name__ == "__main__":
    h1 = [':/FLwheel_motor', './FRwheel_motor', './RRwheel_motor', './RLwheel_motor']
    h2 = [':/FLwheel_motor_2', './FRwheel_motor_2', './RRwheel_motor_2', './RLwheel_motor_2']
    h3 = [':/FLwheel_motor_3', './FRwheel_motor_3', './RRwheel_motor_3', './RLwheel_motor_3']
    Datos = [['127.0.0.1', 19999, h1, ["A", "B"], (0.5,1.5)],
             ['127.0.0.1', 19998, h2, ["C", "D"], (0,0)],
             ['127.0.0.1', 19996, h3, ["E", "F"], (-1.5,1.5)]]
    numeros = [1, 2]
    procesos = []

    for d in Datos:
        #print(d)
        proceso = multiprocessing.Process(target=AGVs, args=(d, ))
        procesos.append(proceso)
        proceso.start()







