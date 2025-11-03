import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from sympy import Matrix, symbols, cos, sin, pi


class JointSubscriber(Node):

    def __init__(self):
        super().__init__('joint_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.t = symbols('t')
        self.Rx = Matrix([[1, 0, 0],
                          [0, cos(self.t), -sin(self.t)],
                          [0, sin(self.t), cos(self.t)]])
        self.Ry = Matrix([[cos(self.t), 0, sin(self.t)],
                          [0, 1, 0],
                          [-sin(self.t), 0, cos(self.t)]])
        self.Rz = Matrix([[cos(self.t), -sin(self.t), 0],
                          [sin(self.t), cos(self.t), 0],
                          [0, 0, 1]])
        self.p0 = Matrix([[0.4],
                          [0],
                          [0]])

    def listener_callback(self, msg):
        self.get_logger().info('Received joint states:')
        for name, position in zip(msg.name, msg.position):
            self.get_logger().info(f'{name}: {position}')
        t1 = msg.position[0]
        t2 = msg.position[1]
        p1 = self.Rz.subs(self.t, t1) * self.Ry.subs(self.t, -t2) * self.p0
        self.get_logger().info(
            f'End Effector Position: x={p1[0]}, y={p1[1]}, z={p1[2]}')


def main(args=None):
    rclpy.init(args=args)

    joint_subscriber = JointSubscriber()

    rclpy.spin(joint_subscriber)

    joint_subscriber.destroy_node()
    rclpy.shutdown()
