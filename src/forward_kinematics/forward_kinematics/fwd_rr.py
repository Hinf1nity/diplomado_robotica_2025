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
        self.d = symbols('d')
        self.a = symbols('a')
        self.alpha = symbols('alpha')
        self.L1 = 0.4
        self.L2 = 0.3
        self.t1 = symbols('t1')
        self.t2 = symbols('t2')
        self.Tz = Matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, self.d],
                          [0, 0, 0, 1]])
        self.Tx = Matrix([[1, 0, 0, self.a],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        self.Rx = Matrix([[1, 0, 0, 0],
                          [0, cos(self.alpha), -sin(self.alpha), 0],
                          [0, sin(self.alpha), cos(self.alpha), 0],
                          [0, 0, 0, 1]])
        self.Rz = Matrix([[cos(self.t), -sin(self.t), 0, 0],
                          [sin(self.t), cos(self.t), 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        self.A = self.Rz * self.Tz * self.Tx * self.Rx
        self.A0 = self.A.subs({self.t: 0, self.d: 0,
                               self.a: 0, self.alpha: pi/2})
        self.A01 = self.A.subs({self.t: self.t1, self.d: 0,
                                self.a: self.L1, self.alpha: 0})
        self.A12 = self.A.subs({self.t: self.t2, self.d: 0,
                                self.a: self.L2, self.alpha: 0})

    def listener_callback(self, msg):
        self.get_logger().info('Received joint states:')
        for name, position in zip(msg.name, msg.position):
            self.get_logger().info(f'{name}: {position}')
        j1 = msg.position[1]
        j2 = msg.position[2]
        T = self.A0 * self.A01 * self.A12
        res = T.subs({self.t1: j1, self.t2: j2})
        self.get_logger().info('End Effector Transformation Matrix:')
        self.get_logger().info(f'{res}')
        self.get_logger().info(
            f'End Effector Position: x={res[0, 3]}, y={res[1, 3]}, z={res[2, 3]}')


def main(args=None):
    rclpy.init(args=args)

    joint_subscriber = JointSubscriber()

    rclpy.spin(joint_subscriber)

    joint_subscriber.destroy_node()
    rclpy.shutdown()
