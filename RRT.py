import math
import random
import matplotlib.pyplot as plt


def get_random_node():
    """Genera un nodo aleatorio dentro del espacio de trabajo"""
    return {"x": random.uniform(rand_area[0], rand_area[1]),
            "y": random.uniform(rand_area[0], rand_area[1])}

def get_nearest_node_index(rnd_node):
    """Encuentra el nodo más cercano en node_list al nodo rnd_node"""
    distances = [
        (node["x"] - rnd_node["x"])**2 + (node["y"] - rnd_node["y"])**2
        for node in node_list
    ]
    return distances.index(min(distances))

def steer(from_node, to_node):
    """Expande el árbol hacia to_node desde from_node"""
    dx, dy = to_node["x"] - from_node["x"], to_node["y"] - from_node["y"]
    distance = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)
    distance = min(distance, expand_dis)  # Limita la distancia al máximo permitido
    new_node = {
        "x": from_node["x"] + distance * math.cos(angle),
        "y": from_node["y"] + distance * math.sin(angle),
        "parent": from_node  # Guarda la referencia al nodo padre
    }
    return new_node

def calc_distance(node1, node2):
    """Calcula la distancia euclidiana entre dos nodos"""
    return math.hypot(node1["x"] - node2["x"], node1["y"] - node2["y"])

def draw_graph(path=None):
    """Dibuja el árbol y el camino final (si existe)"""
    plt.clf()
    for node in node_list:
        if node["parent"] is not None:
            plt.plot([node["x"], node["parent"]["x"]],
                     [node["y"], node["parent"]["y"]], "-g")
    plt.plot(start[0], start[1], "xr")  # Nodo inicial
    plt.plot(goal[0], goal[1], "xr")  # Nodo objetivo
    if path is not None:  # Si el camino final está disponible, dibújalo
        for i in range(len(path) - 1):
            plt.plot([path[i][0], path[i + 1][0]],
                     [path[i][1], path[i + 1][1]], "-r")
    plt.grid(True)
    plt.pause(0.01)

def generate_final_path(node):
    """Genera el camino final desde el nodo objetivo hasta el inicial"""
    path = []
    while node is not None:
        path.append((node["x"], node["y"]))
        node = node["parent"]
    return path[::-1]  # Invierte el camino para que vaya de inicio a objetivo

# Variables
start = (0, 0)
goal = (50, 50)
rand_area = (0, 60)
expand_dis = 5.0
max_iter = 200
node_list = [{"x": start[0], "y": start[1], "parent": None}]


# Algoritmo principal
path = None
for _ in range(max_iter):
    rnd_node = get_random_node()  
    nearest_index = get_nearest_node_index(rnd_node) 
    nearest_node = node_list[nearest_index]
    new_node = steer(nearest_node, rnd_node)  

    node_list.append(new_node)

    # Verifica si el nuevo nodo está cerca del objetivo
    if calc_distance(new_node, {"x": goal[0], "y": goal[1]}) <= expand_dis:
        final_node = steer(new_node, {"x": goal[0], "y": goal[1]})  
        node_list.append(final_node)
        path = generate_final_path(final_node)  
        break

    draw_graph(path=None)  # Dibuja el árbol en crecimiento

# Visualización final
draw_graph(path)
if path is not None:
    print("¡Camino encontrado!")
    print(path)
else:
    print("No se encontró un camino.")
plt.show()
