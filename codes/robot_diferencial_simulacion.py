import pygame
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
from scipy.linalg import solve_continuous_are
import math

# --- CONFIGURACIÓN DE PANTALLA ---
ANCHO_PANTALLA = 1000
ALTO_PANTALLA = 800
PPM = 100.0  # Pixels Per Meter (1 metro = 100px)
FPS = 50

COLOR_FONDO = (20, 20, 30)
COLOR_ROBOT = (0, 200, 255)
COLOR_OBJETIVO = (255, 50, 50)
COLOR_TRAYECTORIA = (100, 100, 100)

# ==========================================
# 1. CÓDIGO DE RED NEURONAL Y FÍSICA
# ==========================================

# Red Neuronal para identificar la dinámica del robot diferencial


class NeuralIdentifier(nn.Module):
    def __init__(self):
        super(NeuralIdentifier, self).__init__()
        self.fc1 = nn.Linear(5, 64)
        self.tanh = nn.Tanh()
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, 3)  # [dx, dy, dtheta]

    def forward(self, x, u):
        if x.dim() > u.dim():
            u = u.expand(x.size(0), -1, -1)
        elif u.dim() > x.dim():
            x = x.expand(u.size(0), -1, -1)
        xu = torch.cat((x, u), dim=-1)
        out = self.tanh(self.fc1(xu))
        out = self.tanh(self.fc2(out))
        return self.fc3(out)

# Normaliza un ángulo a [-pi, pi]


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Obtiene las matrices linealizadas A, B alrededor de (x_val, u_val) usando jacobianos


def get_linearized_matrices(model, x_val, u_val):
    x_t = torch.tensor(x_val, dtype=torch.float32,
                       requires_grad=True).unsqueeze(0)
    u_t = torch.tensor(u_val, dtype=torch.float32,
                       requires_grad=True).unsqueeze(0)

    # Jacobians: A = df/dx, B = df/du
    jac_x = torch.autograd.functional.jacobian(lambda x: model(x, u_t), x_t)
    jac_u = torch.autograd.functional.jacobian(lambda u: model(x_t, u), u_t)

    A = jac_x.detach().numpy().reshape(3, 3)
    B = jac_u.detach().numpy().reshape(3, 2)
    return A, B

# Calcula la ganancia LQR K dado A, B, Q, R


def compute_lqr_gain(A, B, Q, R):
    try:
        # A veces A/B causan singularidad en el sistema, especialmente parados
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ (B.T @ P)
        return K
    except Exception as e:
        return None

# ==========================================
# 2. CLASE DE CONTROL (EL CEREBRO)
# ==========================================

# Cerebro que usa LQR con modelo aprendido por red neuronal


class CerebroLQR:
    def __init__(self):
        # Cargar modelo entrenado
        self.model = torch.load(
            'modelo_NN_robot_diferencial.pt', weights_only=False)

        print("Inicializando pesos (o cargando modelo)...")
        self.model.eval()  # Modo evaluación

        # Matrices de Costo LQR
        self.Q = np.diag([8.0, 8.0, 0.0005])  # Penalizar mucho error X, Y
        self.R = np.diag([0.5, 0.5])      # Costo de energía (v, w)

        self.K_curr = np.zeros((2, 3))    # Ganancia inicial
        self.u_prev = np.zeros(2)         # Control anterior

    def obtener_control(self, x_actual, x_objetivo):
        # Calcular error de estado
        error = x_actual - x_objetivo
        error[2] = normalize_angle(error[2])

        distancia = np.linalg.norm(error[:2])

        # Zona muerta para evitar oscilaciones finales
        if distancia < 0.05:
            return np.zeros(2)

        # Si el robot está parado, el Jacobiano B
        # suele ser nulo o singular. Fingimos que se mueve un poco.
        u_query = self.u_prev.copy()
        if np.abs(u_query[0]) < 0.01:
            u_query[0] = 0.1

        # Linealización usando la Red Neuronal
        A_est, B_est = get_linearized_matrices(self.model, x_actual, u_query)

        # Calcular K (LQR)
        K_new = compute_lqr_gain(A_est, B_est, self.Q, self.R)

        if K_new is not None:
            self.K_curr = K_new

        # Ley de Control: u = -K * error
        u_control = -self.K_curr @ error

        # Saturación (Límites del robot real)
        # Velocidad lineal max 2 m/s, angular max pi rad/s
        u_control[0] = np.clip(u_control[0], -2.0, 2.0)
        u_control[1] = np.clip(u_control[1], -np.pi, np.pi)

        self.u_prev = u_control
        return u_control

# ==========================================
# 3. CLASE ROBOT (VISUAL + FÍSICA)
# ==========================================

# Robot diferencial con representación en Pygame


class RobotPygame:
    def __init__(self, x_m, y_m):
        # Estado físico en METROS (Sistema cartesiano estándar: Y arriba)
        self.state = np.array([x_m, y_m, 0.0], dtype=np.float32)
        self.rastro = []

        # Dimensiones gráficas
        self.radio_px = 15

    def actualizar_fisica(self, u_control, dt):
        v, w = u_control
        theta = self.state[2]

        # Ecuaciones cinemáticas diferenciales (Modelo Real)
        # dx = v * cos(theta)
        # dy = v * sin(theta)
        # dth = w
        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dtheta = w

        # Integración de Euler
        self.state[0] += dx * dt
        self.state[1] += dy * dt
        self.state[2] += dtheta * dt
        self.state[2] = normalize_angle(self.state[2])

        # Guardar rastro para dibujar
        if len(self.rastro) == 0 or np.linalg.norm(self.state[:2] - self.rastro[-1]) > 0.05:
            self.rastro.append(self.state[:2].copy())
            if len(self.rastro) > 200:
                self.rastro.pop(0)

    def dibujar(self, pantalla):
        # Conversión de Coordenadas: Mundo Matemático -> Pantalla Pygame
        # X pantalla = X mundo * PPM
        # Y pantalla = ALTO_PANTALLA - (Y mundo * PPM)  (Invertir eje Y)

        px_x = int(self.state[0] * PPM)
        px_y = int(ALTO_PANTALLA - (self.state[1] * PPM))

        # Dibujar rastro
        if len(self.rastro) > 1:
            puntos_px = []
            for p in self.rastro:
                px = int(p[0] * PPM)
                py = int(ALTO_PANTALLA - (p[1] * PPM))
                puntos_px.append((px, py))
            pygame.draw.lines(pantalla, COLOR_TRAYECTORIA, False, puntos_px, 2)

        # Dibujar cuerpo robot
        pygame.draw.circle(pantalla, COLOR_ROBOT, (px_x, px_y), self.radio_px)

        # Dibujar indicador de dirección
        theta = self.state[2]
        # Nota: En pygame para dibujar una linea con ángulo matemático,
        # recordamos que Y en pantalla es invertido, así que usamos -sin
        end_x = px_x + int(math.cos(theta) * self.radio_px * 1.5)
        end_y = px_y - int(math.sin(theta) * self.radio_px * 1.5)
        pygame.draw.line(pantalla, (255, 255, 255),
                         (px_x, px_y), (end_x, end_y), 3)

# ==========================================
# 4. LOOP PRINCIPAL
# ==========================================


def main():
    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO_PANTALLA, ALTO_PANTALLA))
    pygame.display.set_caption("LQR Neural - Robot Diferencial")
    reloj = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)

    # Posición inicial (Metros): Centro de la ventana
    start_x = (ANCHO_PANTALLA / PPM) / 2
    start_y = (ALTO_PANTALLA / PPM) / 2

    robot = RobotPygame(start_x, start_y)
    cerebro = CerebroLQR()

    # Objetivo inicial (el mismo lugar)
    target = np.array([start_x, start_y, 0.0])

    corriendo = True
    while corriendo:
        dt = reloj.tick(FPS) / 1000.0  # dt en segundos

        # --- EVENTOS ---
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                corriendo = False

            if evento.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                # Convertir click (Pantalla) a objetivo (Mundo Matemático)
                t_x = mx / PPM
                t_y = (ALTO_PANTALLA - my) / PPM  # Invertir Y

                # Asignamos el objetivo.
                # Nota: Dejamos el ángulo objetivo en 0 o igual al actual
                target = np.array([t_x, t_y, 0.0])
                print(f"Nuevo objetivo: ({t_x:.2f}, {t_y:.2f})")

        # --- CONTROL Y FÍSICA ---
        # 1. El cerebro decide 'u' basado en el estado actual y el objetivo
        u = cerebro.obtener_control(robot.state, target)

        # 2. El robot aplica la física
        robot.actualizar_fisica(u, dt)

        # --- DIBUJO ---
        pantalla.fill(COLOR_FONDO)

        # Dibujar cuadrícula (Grid de 1 metro)
        for i in range(0, int(ANCHO_PANTALLA/PPM) + 1):
            pygame.draw.line(pantalla, (40, 40, 50),
                             (i*PPM, 0), (i*PPM, ALTO_PANTALLA))
        for i in range(0, int(ALTO_PANTALLA/PPM) + 1):
            y_screen = ALTO_PANTALLA - (i*PPM)
            pygame.draw.line(pantalla, (40, 40, 50),
                             (0, y_screen), (ANCHO_PANTALLA, y_screen))

        # Dibujar objetivo
        tgt_px_x = int(target[0] * PPM)
        tgt_px_y = int(ALTO_PANTALLA - (target[1] * PPM))
        pygame.draw.circle(pantalla, COLOR_OBJETIVO, (tgt_px_x, tgt_px_y), 8)
        pygame.draw.line(pantalla, COLOR_OBJETIVO,
                         (tgt_px_x-10, tgt_px_y), (tgt_px_x+10, tgt_px_y), 1)
        pygame.draw.line(pantalla, COLOR_OBJETIVO, (tgt_px_x,
                         tgt_px_y-10), (tgt_px_x, tgt_px_y+10), 1)

        # Dibujar Robot
        robot.dibujar(pantalla)

        # Info Texto
        txt_v = font.render(f"V (lin): {u[0]:.2f} m/s", True, (200, 200, 200))
        txt_w = font.render(
            f"W (ang): {u[1]:.2f} rad/s", True, (200, 200, 200))
        txt_pos = font.render(f"Pos: {robot.state[:2]}", True, (200, 200, 200))

        pantalla.blit(txt_v, (10, 10))
        pantalla.blit(txt_w, (10, 30))
        pantalla.blit(txt_pos, (10, 50))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
