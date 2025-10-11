#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo ROS2: EvasionPlanner (robot diferencial con 2 ruedas, usando Twist)

Este nodo:
- Se suscribe a un LaserScan (LIDAR) para leer distancias alrededor del robot.
- Calcula si hay un obstáculo en el sector frontal.
- Publica un mensaje Twist:
    * linear.x -> velocidad lineal de la rueda izquierda (float32)
    * linear.y -> velocidad lineal de la rueda derecha (float32)
    * angular.z se deja en 0.0 (no usamos velocidad angular global en este caso).
- Lógica de comportamiento:
    * Si no hay obstáculo peligroso al frente -> ambas ruedas avanzan igual (recto).
    * Si hay obstáculo al frente -> gira hacia el lado con más espacio.
      Para girar se usa un método simple: una rueda se queda parada (0.0 m/s)
      y la otra rueda avanza a velocidad de giro (m/s). Esto hace un giro pivotando.
"""

# ----------------------------
# IMPORTACIONES
# ----------------------------
import math                                   # Para conversiones angulares y constantes (inf)
import rclpy                                  # Cliente ROS2 en Python
from rclpy.node import Node                   # Clase base para nodos
from sensor_msgs.msg import LaserScan         # Mensaje del LIDAR
from geometry_msgs.msg import Twist           # Usaremos Twist en lugar de Float32MultiArray


# ----------------------------
# CLASE PRINCIPAL: EVASIONPLANNER
# ----------------------------
class EvasionPlanner(Node):
    """
    Nodo que implementa la evasión reactiva para un robot diferencial.
    """

    def __init__(self):
        # Inicializa la clase base Node con el nombre del nodo.
        # Este nombre será el que aparece en `ros2 node list` (ej: /evasion_planner).
        super().__init__('cmd_evasion')

        # ------------------------
        # DECLARACIÓN DE PARÁMETROS
        # ------------------------
        self.declare_parameter('scan_topic', '/robot/front_laser/scan')   # Tópico del LIDAR
        self.declare_parameter('twist_topic', '/cmd_vel')                 # Tópico donde publicaremos Twist
        self.declare_parameter('forward_speed', 0.25)                     # Velocidad recta (m/s)
        self.declare_parameter('rotate_speed', 0.25)                      # Velocidad al girar (m/s)
        self.declare_parameter('min_distance', 0.6)                       # Umbral de seguridad frontal (m)
        self.declare_parameter('front_angle_deg', 30.0)                   # Medio-ángulo frontal (grados)
        self.declare_parameter('decision_rate', 10.0)                     # Frecuencia de decisión (Hz)

        # ------------------------
        # LECTURA DE PARÁMETROS
        # ------------------------
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        self.forward_speed = float(self.get_parameter('forward_speed').get_parameter_value().double_value)
        self.rotate_speed = float(self.get_parameter('rotate_speed').get_parameter_value().double_value)
        self.min_distance = float(self.get_parameter('min_distance').get_parameter_value().double_value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').get_parameter_value().double_value)
        self.decision_rate = float(self.get_parameter('decision_rate').get_parameter_value().double_value)

        # ------------------------
        # SUSCRIPCIÓN: LIDAR
        # ------------------------
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # ------------------------
        # PUBLICADOR: velocidades de las ruedas (Twist)
        # ------------------------
        # Twist tiene los campos lineales y angulares.
        # Usaremos linear.x e y para las dos ruedas.
        self.pub_twist = self.create_publisher(
            Twist,
            self.twist_topic,
            10
        )

        # ------------------------
        # VARIABLES INTERNAS
        # ------------------------
        self.scan = None

        # ------------------------
        # TEMPORIZADOR
        # ------------------------
        timer_period = 1.0 / self.decision_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"EvasionPlanner iniciado. Nodo: '/evasion_planner'. "
            f"Subscribing: '{self.scan_topic}'  Publishing: '{self.twist_topic}'"
        )

    # ----------------------------
    # CALLBACK DEL LIDAR
    # ----------------------------
    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    # ----------------------------
    # LÓGICA DE DECISIÓN
    # ----------------------------
    def timer_callback(self):
        if self.scan is None:
            return

        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        ranges = list(self.scan.ranges)

        # Limpiamos valores inválidos
        ranges = [
            r if (r is not None and r > 0.0 and not math.isinf(r))
            else float('inf')
            for r in ranges
        ]
        if len(ranges) == 0:
            return

        half_width_rad = math.radians(self.front_angle_deg)

        try:
            front_idx = int(round((0.0 - angle_min) / angle_inc))
        except Exception:
            front_idx = len(ranges) // 2
        front_idx = max(0, min(front_idx, len(ranges) - 1))

        if angle_inc == 0:
            half_span_idx = len(ranges) // 4
        else:
            half_span_idx = int(round(half_width_rad / abs(angle_inc)))

        front_start = max(0, front_idx - half_span_idx)
        front_end = min(len(ranges) - 1, front_idx + half_span_idx)
        front_ranges = ranges[front_start:front_end + 1] if front_end >= front_start else []
        min_front = min(front_ranges) if front_ranges else float('inf')

        left_start = front_end + 1
        left_end = min(len(ranges), left_start + half_span_idx)
        left_ranges = ranges[left_start:left_end] if left_start < len(ranges) else []
        min_left = min(left_ranges) if left_ranges else float('inf')

        right_end = front_start
        right_start = max(0, right_end - half_span_idx)
        right_ranges = ranges[right_start:right_end] if right_start < right_end else []
        min_right = min(right_ranges) if right_ranges else float('inf')

        # Mensaje Twist
        twist_msg = Twist()

        if min_front < self.min_distance:
            if min_left > min_right:
                # Giro a la izquierda: rueda izquierda parada, derecha avanza
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = float(self.rotate_speed)
                self.get_logger().info(
                    f"Obstáculo frontal ({min_front:.2f} m). Giro IZQ. "
                    f"Vel izq: {twist_msg.linear.x:.2f}, vel der: {twist_msg.linear.y:.2f}"
                )
            else:
                # Giro a la derecha: rueda izquierda avanza, derecha parada
                twist_msg.linear.x = float(self.rotate_speed)
                twist_msg.linear.y = 0.0
                self.get_logger().info(
                    f"Obstáculo frontal ({min_front:.2f} m). Giro DER. "
                    f"Vel izq: {twist_msg.linear.x:.2f}, vel der: {twist_msg.linear.y:.2f}"
                )
        else:
            # Avanzar recto
            twist_msg.linear.x = float(self.forward_speed)
            twist_msg.linear.y = float(self.forward_speed)
            self.get_logger().info(
                f"Camino libre ({min_front:.2f} m). Recto. "
                f"Vel izq: {twist_msg.linear.x:.2f}, vel der: {twist_msg.linear.y:.2f}"
            )

        twist_msg.angular.z = 0.0  # No usamos rotación global
        self.pub_twist.publish(twist_msg)


# ----------------------------
# MAIN
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = EvasionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
