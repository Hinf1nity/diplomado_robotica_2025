import pygame
import math
import torch
import torch.nn as nn
import numpy as np
from scipy.linalg import solve_continuous_are

# --- Constantes ---
PPM = 100.0
ANCHO_PANTALLA = 800
ALTO_PANTALLA = 600
COLOR_FONDO = (30, 30, 30)
COLOR_ROBOT = (0, 255, 100)
COLOR_OBJETIVO = (255, 50, 50)
COLOR_GRID = (50, 50, 50)
FPS = 60
ACTION_DIM = 2  # vl, w
STATE_DIM = 3   # x, y, theta


class NeuralIdentifier(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(NeuralIdentifier, self).__init__()
        # Entrada: 3 estados + 2 controles = 5
        self.fc1 = nn.Linear(input_dim, 64)
        self.tanh = nn.Tanh()  # Tanh suele ir mejor con senos/cosenos
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, output_dim)  # Salida: [dx, dy, dtheta]

    def forward(self, x, u):
        # Corrección de dimensiones (Broadcasting)
        if x.dim() > u.dim():
            u = u.expand(x.size(0), -1, -1)
        elif u.dim() > x.dim():
            x = x.expand(u.size(0), -1, -1)

        xu = torch.cat((x, u), dim=-1)
        out = self.tanh(self.fc1(xu))
        out = self.tanh(self.fc2(out))
        return self.fc3(out)


class RobotDiferencial:
    def __init__(self, x, y, ancho_robot):
        self.x = x
        self.y = y
        self.angulo = 0  # Radianes (0 apunta a la derecha)
        self.ancho = ancho_robot  # L

        # Objetivo (inicia donde está el robot para que no se mueva)
        self.target_x = x
        self.target_y = y

        # Dimensiones físicas para dibujar (0.4m largo x 0.3m ancho)
        self.largo_cuerpo_m = 0.4
        self.ancho_cuerpo_m = 0.3

        self.vl = 0
        self.vr = 0
        self.velocidad_lineal = 0
        self.velocidad_angular = 0
        self.historial = []

        self.Q = np.diag([0.1, 75, 10])
        self.R = np.diag([0.001, 50])
        self.K_current = np.zeros((ACTION_DIM, STATE_DIM))
        self.u_prev = [0.0, 0.0]  # vl, w
        self.modelo = torch.load(
            'modelo_NN_robot_diferencial.pt', weights_only=False)
        self.modelo.eval()

    def set_objetivo(self, x, y):
        self.target_x = x
        self.target_y = y

    def normalizar_angulo(self, angulo):
        # Mantiene el ángulo entre -pi y pi para giros eficientes
        while angulo > math.pi:
            angulo -= 2 * math.pi
        while angulo < -math.pi:
            angulo += 2 * math.pi
        return angulo

    def get_linearized_matrices(self, model, x_current, u_current):
        # Preparamos tensores (Batch size = 1)
        x_tensor = torch.tensor(
            x_current, dtype=torch.float32, requires_grad=True).unsqueeze(0)
        u_tensor = torch.tensor(
            u_current, dtype=torch.float32, requires_grad=True).unsqueeze(0)

        # Jacobianos
        # jac_x será [Batch, Output_dim, Input_dim_x] -> [1, 3, 3]
        jac_x = torch.autograd.functional.jacobian(
            lambda x: model(x, u_tensor), x_tensor)
        # jac_u será [Batch, Output_dim, Input_dim_u] -> [1, 3, 2]
        jac_u = torch.autograd.functional.jacobian(
            lambda u: model(x_tensor, u), u_tensor)

        # Limpieza de dimensiones para numpy
        A = jac_x.detach().numpy().reshape(3, 3)
        B = jac_u.detach().numpy().reshape(3, 2)
        return A, B

    def compute_lqr_gain(self, A, B):
        try:
            P = solve_continuous_are(A, B, self.Q, self.R)
            # K = inv(R) * B.T * P
            K = np.linalg.inv(self.R) @ (B.T @ P)
            return K
        except:
            return None

    def actualizar_control(self):
        x_measured = np.array([self.x, -self.y, self.angulo])
        x_measured[2] = self.normalizar_angulo(x_measured[2])
        x_target = np.array([self.target_x, -self.target_y, 0.0])
        x_target[2] = self.normalizar_angulo(x_target[2])
        print("Target:", x_target)
        print("Actual:", x_measured)
        if np.abs(self.u_prev[0]) < 0.01:
            self.u_prev[0] = 0.1
        A_est, B_est = self.get_linearized_matrices(
            self.modelo, x_measured, self.u_prev)

        # --- C. CÁLCULO DE GANANCIA LQR ---
        K_new = self.compute_lqr_gain(A_est, B_est)
        if K_new is not None:
            self.K_current = K_new

        # --- D. LEY DE CONTROL PARA SEGUIMIENTO ---
        # Para seguir una referencia, el LQR actúa sobre el ERROR.
        # u = -K * (x_actual - x_deseado)
        error = x_measured - x_target
        u_control = -self.K_current @ error

        # Saturación
        u_control[0] = np.clip(u_control[0], -2, 2)
        u_control[1] = np.clip(u_control[1], -math.pi/4, math.pi/4)
        print("Control:", u_control)
        self.u_prev = u_control
        # --- E. APLICAR LEY DE CONTROL ---
        # Controladores (Lógica de comportamiento)
        self.velocidad_lineal = u_control[0]
        self.velocidad_angular = u_control[1]

        # self.vr = self.velocidad_lineal + \
        #     (self.velocidad_angular * self.ancho / 2)
        # self.vl = self.velocidad_lineal - \
        #     (self.velocidad_angular * self.ancho / 2)

        # dist_error = np.linalg.norm(x_measured[:2] - x_target[:2])
        # if dist_error < 0.05:
        #     print(f"Objetivo alcanzado")
        #     self.vl = 0
        #     self.vr = 0

    def mover(self, dt):
        # dt está en segundos
        # v = (self.vr + self.vl) / 2      # m/s
        # omega = (self.vr - self.vl) / self.ancho  # rad/s

        # self.angulo += omega * dt
        # self.angulo = self.normalizar_angulo(self.angulo)

        # Actualizar posición (metros)
        # Nota: Pygame Y es positivo hacia abajo, por eso restamos seno
        # self.x += v * math.cos(self.angulo) * dt
        # self.y -= v * math.sin(self.angulo) * dt
        self.angulo += self.velocidad_angular * dt
        self.angulo = self.normalizar_angulo(self.angulo)
        self.x += self.velocidad_lineal * math.cos(self.angulo) * dt
        self.y -= self.velocidad_lineal * math.sin(self.angulo) * dt

        # Guardar rastro (cada 5cm)
        if not self.historial or math.hypot(self.x - self.historial[-1][0], self.y - self.historial[-1][1]) > 0.05:
            self.historial.append((self.x, self.y))
        if len(self.historial) > 500:
            self.historial.pop(0)

    def dibujar(self, pantalla):
        # --- CONVERSIÓN DE COORDENADAS (Mundo -> Pantalla) ---
        px_x = int(self.x * PPM)
        px_y = int(self.y * PPM)

        target_px_x = int(self.target_x * PPM)
        target_px_y = int(self.target_y * PPM)

        # Dibujar línea al objetivo
        pygame.draw.line(pantalla, (80, 80, 80), (px_x, px_y),
                         (target_px_x, target_px_y), 1)
        pygame.draw.circle(pantalla, COLOR_OBJETIVO,
                           (target_px_x, target_px_y), 5)

        # Dibujar rastro
        if len(self.historial) > 1:
            puntos_pantalla = [(int(p[0]*PPM), int(p[1]*PPM))
                               for p in self.historial]
            pygame.draw.lines(pantalla, (100, 100, 100),
                              False, puntos_pantalla, 2)

        # Dibujar Robot
        # Creamos una superficie con el tamaño escalado en píxeles
        ancho_px = int(self.largo_cuerpo_m * PPM)
        alto_px = int(self.ancho_cuerpo_m * PPM)

        surf_robot = pygame.Surface((ancho_px, alto_px), pygame.SRCALPHA)

        # Cuerpo y ruedas
        pygame.draw.rect(surf_robot, COLOR_ROBOT,
                         (0, 0, ancho_px, alto_px), border_radius=4)
        # Frente (indicador rojo)
        pygame.draw.rect(surf_robot, (255, 50, 50),
                         (ancho_px - 10, 5, 10, alto_px - 10))

        # Rotación y Blit
        angulo_grados = math.degrees(self.angulo)
        img_rotada = pygame.transform.rotate(surf_robot, angulo_grados)
        rect_rotado = img_rotada.get_rect(center=(px_x, px_y))
        pantalla.blit(img_rotada, rect_rotado)

# --- Main ---


def dibujar_grid(pantalla):
    # Dibuja líneas cada 1 metro
    ancho_m = int(ANCHO_PANTALLA / PPM)
    alto_m = int(ALTO_PANTALLA / PPM)

    for x in range(ancho_m + 1):
        pygame.draw.line(pantalla, COLOR_GRID, (x * PPM, 0),
                         (x * PPM, ALTO_PANTALLA))
        # Etiquetas de texto
        font = pygame.font.SysFont("Arial", 10)
        lbl = font.render(f"{x}m", True, COLOR_GRID)
        pantalla.blit(lbl, (x * PPM + 2, 2))

    for y in range(alto_m + 1):
        pygame.draw.line(pantalla, COLOR_GRID, (0, y * PPM),
                         (ANCHO_PANTALLA, y * PPM))
        lbl = font.render(f"{y}m", True, COLOR_GRID)
        pantalla.blit(lbl, (2, y * PPM + 2))


def main():
    pygame.init()
    pantalla = pygame.display.set_mode((ANCHO_PANTALLA, ALTO_PANTALLA))
    pygame.display.set_caption(f"Simulación Física Real - 1m = {int(PPM)}px")
    reloj = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)

    # Robot empieza en (2m, 3m) con ancho entre ruedas de 0.27m (27cm)
    robot = RobotDiferencial(2.0, 3.0, 0.27)

    corriendo = True
    while corriendo:
        dt = reloj.tick(FPS) / 1000.0

        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                corriendo = False
            if evento.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                # --- CONVERSIÓN INPUT (Pantalla -> Mundo) ---
                target_m_x = mx / PPM
                target_m_y = my / PPM
                robot.set_objetivo(target_m_x, target_m_y)

        robot.actualizar_control()
        robot.mover(dt)

        pantalla.fill(COLOR_FONDO)
        dibujar_grid(pantalla)  # Fondo de cuadrícula
        robot.dibujar(pantalla)

        # UI Datos Reales
        v_real = (robot.vl + robot.vr) / 2
        info_pos = font.render(
            f"Pos: ({robot.x:.2f}m, {robot.y:.2f}m)", True, (255, 255, 255))
        info_vel = font.render(f"Vel: {v_real:.2f} m/s", True, (255, 255, 255))
        pantalla.blit(info_pos, (10, ALTO_PANTALLA - 50))
        pantalla.blit(info_vel, (10, ALTO_PANTALLA - 30))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
