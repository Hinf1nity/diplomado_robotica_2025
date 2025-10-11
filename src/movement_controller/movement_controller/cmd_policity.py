import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class cmd_policity(Node):
    def __init__(self):
        super().__init__('dual_motor_limiter')

        # Límites de velocidad para cada motor
        self.max_motor_left = 1.0   # límite motor izquierdo
        self.max_motor_right = 1.0  # límite motor derecho

        # Suscriptor al tópico de entrada
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_topic',   # tópico de entrada
            self.listener_callback,
            10
        )

        # Publicador al tópico de salida
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_smooth',  # tópico de salida
            10
        )

        self.get_logger().info("Nodo cmd_policity iniciado. Escuchando en /motors_in")

    def listener_callback(self, msg: Twist):
        # Crear mensaje limitado
        limited_msg = Twist()

        # Motor izquierdo = linear.l
        limited_msg.linear.l = max(min(msg.linear.l, self.max_motor_left), -self.max_motor_left)

        # Motor derecho = linear.r
        limited_msg.linear.r = max(min(msg.linear.r, self.max_motor_right), -self.max_motor_right)


        # Publicar el mensaje limitado
        self.publisher.publish(limited_msg)

        # Logs para ver la entrada y salida
        self.get_logger().info(
            f"Entrada -> Motor Izq: {msg.linear.l:.2f}, Motor Der: {msg.linear.r:.2f}"
        )
        self.get_logger().info(
            f"Salida  -> Motor Izq: {limited_msg.linear.l:.2f}, Motor Der: {limited_msg.linear.r:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = cmd_policiy ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
