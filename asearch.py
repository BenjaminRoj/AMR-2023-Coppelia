import numpy as np
import random
import heapq


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = {'up': None, 'down': None, 'right': None, 'left': None}

    def set_neighbors(self, grid, delta):
        self.neighbors['up'] = grid.get((self.x, round(self.y + delta, 1)))
        self.neighbors['down'] = grid.get((self.x, round(self.y - delta, 1)))
        self.neighbors['right'] = grid.get((round(self.x + delta, 1), self.y))
        self.neighbors['left'] = grid.get((round(self.x - delta, 1), self.y))

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
    directions = ['up', 'down', 'left', 'right', None]
    return random.choice(directions)

def heuristic(node, goal_node):
    return np.sqrt((node.x - goal_node.x) ** 2 + (node.y - goal_node.y) ** 2)



def dfs_move_robot(grid, current_node, goal_node, visited, path, attempt):
    if current_node == goal_node:
        print("Goal reached")
        return True

    if attempt > 10:  # Limitar el número de intentos para evitar un bucle infinito
        return False

    visited.add((current_node, attempt))
    #print(f"Attempt {attempt}: Current Node: {current_node.x}, {current_node.y}")

    for direction, neighbor in current_node.neighbors.items():
        if neighbor and (neighbor, attempt + 1) not in visited:
            collision = detect_collision()
            
            # Verificar colisiones
            if direction == collision:
                print(f"Attempt {attempt}: Collision detected in direction: {direction}, cannot move here.")
                continue

            print(f"Attempt {attempt}: Moving {direction} to Node: {neighbor.x}, {neighbor.y}")
            path.append(direction)
            if dfs_move_robot(grid, neighbor, goal_node, visited, path, attempt + 1):
                return True
            #path.pop()  # Retroceder si no se encuentra el camino

    return False  # No se encontró camino al objetivo

def move_robot(grid, start_coords, goal_coords):
    start_node = grid[start_coords]
    goal_node = grid[goal_coords]

    visited = set()
    path = []
    if not dfs_move_robot(grid, start_node, goal_node, visited, path, 1):
        print("No path found to the goal")
    else:
        print("Path:", ' -> '.join(path))

# Crear la cuadrícula y ejecutar el algoritmo
grid = create_grid((-2, -2), (3, 3), 0.1)
print(grid)
start_coords = (-0.2, -0.2)
goal_coords = (0.2, 0.2)
#move_robot(grid, start_coords, goal_coords)
