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
    if check_collision(from_node, new_node):  # Verifica si el segmento no tiene colisión
        return new_node
    return None  # Nodo inválido por colisión

def calc_distance(node1, node2):
    """Calcula la distancia euclidiana entre dos nodos"""
    return math.hypot(node1["x"] - node2["x"], node1["y"] - node2["y"])

def check_collision(node1, node2):
    """Verifica si el segmento entre node1 y node2 intersecta algún obstáculo"""
    # Verificar colisiones con los límites de la cancha
    if not is_inside_pitch(node2):
        return False  # Nodo fuera de la cancha

    # Verificar colisiones con las áreas restringidas
    for rect in restricted_areas:
        if crosses_rectangle(node1, node2, rect):
            return False  # Segmento cruza un área restringida

    # Verificar colisiones con los obstáculos circulares (jugadores)
    for (ox, oy, r) in obstacles:
        steps = max(int(calc_distance(node1, node2) / 0.5), 1)  # Más puntos para mayor precisión
        for i in range(steps + 1):
            t = i / steps
            x = node1["x"] + t * (node2["x"] - node1["x"])
            y = node1["y"] + t * (node2["y"] - node1["y"])
            if math.hypot(x - ox, y - oy) <= r:  # Puntos dentro del círculo
                return False  # Colisión detectada

    return True  # Sin colisiones

def is_inside_pitch(node):
    """Verifica si un nodo está dentro de los límites de la cancha"""
    return (pitch_limits[0] <= node["x"] <= pitch_limits[2] and
            pitch_limits[1] <= node["y"] <= pitch_limits[3])

def crosses_rectangle(node1, node2, rect):
    """Verifica si un segmento cruza un rectángulo"""
    x_min, y_min, x_max, y_max = rect
    steps = max(int(calc_distance(node1, node2) / 0.5), 1)
    for i in range(steps + 1):
        t = i / steps
        x = node1["x"] + t * (node2["x"] - node1["x"])
        y = node1["y"] + t * (node2["y"] - node1["y"])
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return True
    return False

def draw_graph(path=None):
    """Dibuja el árbol, límites de la cancha, obstáculos y el camino final (si existe)"""
    plt.clf()
    # Dibuja los límites de la cancha
    x_min, y_min, x_max, y_max = pitch_limits
    plt.plot([x_min, x_max, x_max, x_min, x_min],
             [y_min, y_min, y_max, y_max, y_min], "-b")

    # Dibuja las áreas restringidas
    for rect in restricted_areas:
        x_min, y_min, x_max, y_max = rect
        plt.plot([x_min, x_max, x_max, x_min, x_min],
                 [y_min, y_min, y_max, y_max, y_min], "-r")

    # Dibuja los obstáculos circulares
    for (ox, oy, r) in obstacles:
        circle = plt.Circle((ox, oy), r, color='orange', alpha=0.5)
        plt.gca().add_patch(circle)

    # Dibuja el árbol
    for node in node_list:
        if node["parent"] is not None:
            plt.plot([node["x"], node["parent"]["x"]],
                     [node["y"], node["parent"]["y"]], "-g")
    # Dibuja el camino final
    if path is not None:
        for i in range(len(path) - 1):
            plt.plot([path[i][0], path[i + 1][0]],
                     [path[i][1], path[i + 1][1]], "-y")
    plt.plot(start[0], start[1], "xr")  # Nodo inicial
    plt.plot(goal[0], goal[1], "xr")  # Nodo objetivo
    plt.grid(True)
    plt.pause(0.01)

def generate_final_path(node):
    """Genera el camino final desde el nodo objetivo hasta el inicial"""
    path = []
    while node is not None:
        path.append((node["x"], node["y"]))
        node = node["parent"]
    return path[::-1]  # Invierte el camino para que vaya de inicio a objetivo

#------------------ Variables ------------------------
start = (11, 25)
goal = (80, 25)
rand_area = (0, 100)
expand_dis = 5.0
max_iter = 300
node_list = [{"x": start[0], "y": start[1], "parent": None}]

# Definir los límites de la cancha (x_min, y_min, x_max, y_max)
pitch_limits = (0, 0, 100, 50)

# Definir áreas restringidas (rectángulos: x_min, y_min, x_max, y_max)
restricted_areas = [
    (0, 15, 10, 35),  # Área del arquero izquierdo 
    (90, 15, 100, 35)  # Área del arquero derecho
]

# Definir obstáculos circulares (x, y, radio)
obstacles = [
    (30, 20, 3),  # Robot 1
    (50, 30, 3),  # Robot 2
    (70, 25, 3)   # Robot 3
]
#camino final
path = None
#-------------------------------------------------------
# Algoritmo principal
for _ in range(max_iter):
    rnd_node = get_random_node()  
    nearest_index = get_nearest_node_index(rnd_node) 
    nearest_node = node_list[nearest_index]
    new_node = steer(nearest_node, rnd_node)  

    if new_node:  # Solo añade nodos válidos
        node_list.append(new_node)

        # Verifica si el nuevo nodo está cerca del objetivo
        if calc_distance(new_node, {"x": goal[0], "y": goal[1]}) <= expand_dis:
            final_node = steer(new_node, {"x": goal[0], "y": goal[1]})  
            if final_node:  # Verifica que la conexión al objetivo sea válida
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
