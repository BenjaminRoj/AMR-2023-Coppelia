import numpy as np
import random
import heapq


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = {'up': None, 'down': None, 'right': None, 'left': None}

    def set_neighbors(self, grid, delta):
        self.neighbors['up'] = grid.get((self.x, self.y +0.1))
        self.neighbors['down'] = grid.get((self.x, self.y -0.1))
        self.neighbors['right'] = grid.get((self.x +0.1, self.y))
        self.neighbors['left'] = grid.get((self.x -0.1, self.y))

    def __repr__(self):
        return f"Node({self.x}, {self.y})"

def create_grid(top_left, bottom_right, delta):
    x_start, y_start = top_left
    x_end, y_end = bottom_right

    # Ajustar el rango de y si y_start es mayor que y_end
    y_range = np.arange(y_start, y_end - delta if y_start > y_end else y_end + delta, -delta if y_start > y_end else delta)
    
    grid = {(round(x, 1), round(y, 1)): Node(round(x, 1), round(y, 1))
            for x in np.arange(x_start, x_end + delta if x_start < x_end else x_end - delta, delta)
            for y in y_range}
    
    for node in grid.values():
        node.set_neighbors(grid, delta)
    
    return grid

def detect_collision():
    directions = ['adelante', 'atras', 'izquierda', 'derecha', None]
    return random.choice(directions)

def get_neighbors(node, grid, collisions, first_move):
    neighbors = []
    for direction, neighbor_node in node.neighbors.items():
        # Si es el primer movimiento, verificar colisiones
        if first_move and collisions.get(direction, False):
            continue
        if neighbor_node:  # Asegurar que el vecino exista en la grilla
            neighbors.append((neighbor_node, direction))
    return neighbors


from collections import deque

def bfs_move_robot(grid, start_node, goal_node, collisions):
    queue = deque([(start_node, [], True)])  # (nodo actual, camino hasta ahora, primer movimiento)
    visited = set([start_node])

    while queue:
        current_node, path, first_move = queue.popleft()
        #print(f"Visiting: {current_node} with path: {path}")  # Depuración

        if current_node == goal_node:
            return path

        for direction, neighbor_coords in current_node.neighbors.items():
            if first_move and collisions.get(direction, False):
                print(f"Collision detected on first move in direction: {direction}")  # Depuración
                continue
            neighbor_node = grid.get(neighbor_coords)
            if neighbor_node and neighbor_node not in visited:
                visited.add(neighbor_node)
                queue.append((neighbor_node, path + [direction], False))  # False indica que ya no es el primer movimiento

    return None  # No se encontró camino

def move_robot(grid, start_coords, goal_coords, collisions):
    start_node = grid.get(start_coords)
    goal_node = grid.get(goal_coords)

    if not start_node or not goal_node:
        print("El nodo de inicio o el nodo objetivo no están en la grilla")
        return

    # Generar vecinos para los nodos
    for node in grid.values():
        
        x, y = node.x, node.y
        node.neighbors['adelante'] = (x, round(y+0.1, 1))
        #print(node.neighbors['up'])
        node.neighbors['atras'] = (x, round(y-0.1, 1))
        node.neighbors['izquierda'] = (round(x-0.1, 1), y)
        node.neighbors['derecha'] = (round(x+0.1, 1), y)

    path = bfs_move_robot(grid, start_node, goal_node, collisions)
    if path is None:
        print("No path found to the goal")
    else:
        #print("Path:", ' -> '.join(path))
        return path

# Ejemplo de uso:
'''grid = create_grid((-2.4, 2.4), (2.4, -2.3), 0.1)
start = (1.6, -0.1)
goal = (1.6, 1.2)
collisions = {'up': True, 'down': False, 'left': False, 'right': False}
print(move_robot(grid, start, goal, collisions))
'''
