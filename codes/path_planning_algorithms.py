import pygame
import math
from queue import PriorityQueue

# --- Configuración Inicial ---
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Simulación: A* vs Campos Potenciales")

# Colores (RGB)
RED = (255, 0, 0)       # Fin
GREEN = (0, 255, 0)     # Inicio
BLUE = (0, 0, 255)      # Robot
YELLOW = (255, 255, 0)  # Camino óptimo
WHITE = (255, 255, 255)  # Vacío
BLACK = (0, 0, 0)       # Pared/Obstáculo
PURPLE = (128, 0, 128)  # Visitado (A*)
ORANGE = (255, 165, 0)  # Open Set (A*)
PINK = (255, 105, 180)  # Rastro Campos Potenciales
GREY = (128, 128, 128)


class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    # Métodos de estado
    def is_barrier(self): return self.color == BLACK
    def is_start(self): return self.color == GREEN
    def is_end(self): return self.color == RED
    def reset(self): self.color = WHITE

    # Métodos visuales
    def make_start(self): self.color = GREEN
    def make_closed(self): self.color = PURPLE
    def make_open(self): self.color = ORANGE
    def make_barrier(self): self.color = BLACK
    def make_end(self): self.color = RED
    def make_path(self): self.color = YELLOW
    def make_robot(self): self.color = BLUE
    # Color para el rastro de Potenciales
    def make_potential_path(self): self.color = PINK

    def draw(self, win):
        pygame.draw.rect(
            win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        # Permitimos 4 direcciones (Arriba, Abajo, Izq, Der)
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col - 1])

# --- ALGORITMO 1: A* (A-Star) ---


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw, start):
    path = []
    while current in came_from:
        current = came_from[current]
        if current != start:
            path.append(current)
            current.make_path()
        draw()
    return path[::-1]  # Retornar camino invertido


def algorithm_astar(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}

    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            path = reconstruct_path(came_from, end, draw, start)
            end.make_end()
            return path

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + \
                    h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return None

# --- ALGORITMO 2: CAMPOS POTENCIALES ---


def calculate_potential(node, end, obstacles):
    # 1. Potencial Atractivo (Distancia al objetivo)
    # Usamos distancia Euclidiana para un gradiente suave
    d_goal = math.sqrt((node.row - end.row)**2 + (node.col - end.col)**2)
    k_att = 1.0
    u_att = 0.5 * k_att * (d_goal**2)  # Fórmula cuadrática clásica

    # 2. Potencial Repulsivo (Obstáculos)
    u_rep = 0
    k_rep = 10000.0  # Ganancia de repulsión alta para evitar paredes
    influence_radius = 1  # Solo afecta si está cerca (en celdas)

    # Optimización: Solo revisar obstáculos cercanos, no todos
    # (En una implementación real usaríamos un Quadtree o KD-Tree, aquí filtramos simple)
    for obs in obstacles:
        d_obs = math.sqrt((node.row - obs.row)**2 + (node.col - obs.col)**2)

        if d_obs <= influence_radius and d_obs > 0:
            # Fórmula de repulsión clásica: 0.5 * k * (1/d - 1/d0)^2
            u_rep += 0.5 * k_rep * \
                ((1.0 / d_obs) - (1.0 / influence_radius))**2

    return u_att + u_rep


def algorithm_potential_fields(draw, grid, start, end):
    # Recopilar lista de obstáculos para no escanear toda la grilla cada vez
    obstacles = []
    for row in grid:
        for node in row:
            if node.is_barrier():
                obstacles.append(node)

    current = start
    path = []

    # Límite de pasos para evitar bucles infinitos (Mínimos Locales)
    max_steps = 2000
    steps = 0

    while current != end and steps < max_steps:
        steps += 1

        # Manejo de eventos para no congelar la ventana
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        best_neighbor = None
        min_potential = float('inf')

        # Buscar el vecino con MENOR potencial (Descenso del Gradiente)
        # Nota: Aquí no usamos current.neighbors precalculados porque queremos
        # evaluar todos los adyacentes geométricos incluso si no están "conectados"
        # en el grafo, aunque nos moveremos solo a los válidos.

        # Re-evaluamos vecinos válidos al momento
        valid_neighbors = []
        r, c = current.row, current.col
        candidates = [
            (r+1, c), (r-1, c), (r, c+1), (r, c-1),  # Ortogonales
            # Diagonales (Opcional, mejora fluidez)
            (r+1, c+1), (r-1, c-1), (r+1, c-1), (r-1, c+1)
        ]

        for nr, nc in candidates:
            if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]):
                neighbor = grid[nr][nc]
                # No volver atrás inmediatamente no es restricción aquí
                if not neighbor.is_barrier() and neighbor != current:
                    valid_neighbors.append(neighbor)

        # Evaluar potenciales
        for neighbor in valid_neighbors:
            p = calculate_potential(neighbor, end, obstacles)
            if p < min_potential:
                min_potential = p
                best_neighbor = neighbor

        # Moverse
        if best_neighbor:
            # Detección de Mínimo Local simple: si el mejor vecino tiene mayor potencial que yo
            current_pot = calculate_potential(current, end, obstacles)
            if min_potential >= current_pot:
                print("¡Atrapado en Mínimo Local!")
                break

            current = best_neighbor
            if current != end:
                path.append(current)
                current.make_potential_path()  # Color ROSA para diferenciar
            draw()
        else:
            break  # Sin salida

    return path

# --- Funciones Generales ---


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid


def draw_grid_lines(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    win.fill(WHITE)
    for row in grid:
        for node in row:
            node.draw(win)
    draw_grid_lines(win, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col


def move_robot(path, draw, start, end):
    prev = start
    for node in path:
        pygame.time.delay(50)
        node.make_robot()
        if prev != start:
            if prev.color == BLUE:  # Restaurar color de camino si el robot se va
                prev.make_path()
        prev = node
        draw()
    end.make_end()


def main(win, width):
    ROWS = 40  # Reducido un poco para mejorar rendimiento en cálculo de potenciales
    grid = make_grid(ROWS, width)

    start = None
    end = None

    print("Controles:")
    print("Click Izq: Colocar Inicio, Fin y Muros")
    print("Click Der: Borrar")
    print("ESPACIO: Ejecutar A* (Garantiza óptimo)")
    print("TECLA 'P': Ejecutar Campos Potenciales (Físico, puede caer en mínimos locales)")
    print("TECLA 'C': Limpiar")

    run = True
    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # Click Izq
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                if not start and node != end:
                    start = node
                    start.make_start()
                elif not end and node != start:
                    end = node
                    end.make_end()
                elif node != end and node != start:
                    node.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # Click Der
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if start and end:
                    # Actualizar vecinos para grafo
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)

                    if event.key == pygame.K_SPACE:
                        print("Ejecutando A*...")
                        path = algorithm_astar(lambda: draw(
                            win, grid, ROWS, width), grid, start, end)
                        if path:
                            move_robot(path, lambda: draw(
                                win, grid, ROWS, width), start, end)

                    if event.key == pygame.K_p:
                        print("Ejecutando Campos Potenciales...")
                        path = algorithm_potential_fields(lambda: draw(
                            win, grid, ROWS, width), grid, start, end)
                        # Nota: En campos potenciales, el "path" se dibuja en tiempo real,
                        # pero podemos volver a animar el robot encima si queremos.
                        if path:
                            move_robot(path, lambda: draw(
                                win, grid, ROWS, width), start, end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    print("Lienzo limpio")

    pygame.quit()


if __name__ == "__main__":
    main(WIN, WIDTH)
