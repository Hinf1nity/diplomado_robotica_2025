import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from sympy import symbols, Matrix, eye, sin, cos, expand, pprint, pi
from scipy.spatial.transform import Rotation as R_scipy
import numpy as np


class JointSubscriber(Node):

    def __init__(self):
        super().__init__('forward_kinematics')
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.sub_callback, 10)
        self.subscription
        self.t1, self.t2, self.t3, self.t4, self.t5 = symbols(
            'theta_1 theta_2 theta_3 theta_4 theta_5', real=True)  # joint angles
        self._final_matrix = self.calculate_forward_kinematics_matrix()

    def sub_callback(self, msg):
        j1 = msg.position[0]
        j2 = msg.position[1]
        j3 = msg.position[2]
        j4 = msg.position[4]
        j5 = msg.position[5]
        res = self._final_matrix.subs(
            {self.t1: j1, self.t2: j2, self.t3: j3, self.t4: j4, self.t5: j5})
        rotation = R_scipy.from_matrix(np.array(res)[0:3, 0:3])
        quaternion = rotation.as_quat()
        print("Position:")
        print("x: ", res[0, 3], " y: ", res[1, 3], " z: ", res[2, 3])
        print("Orientation:")
        print("x: ", quaternion[0], " y: ", quaternion[1],
              " z: ", quaternion[2], " w: ", quaternion[3])

    def calculate_forward_kinematics_matrix(self):
        theta = symbols('theta', real=True)
        w0, w1, w2 = symbols('omega_0 omega_1 omega_2', real=True)
        w = Matrix([w0, w1, w2])
        skew_w = Matrix([
            [0, -w[2], w[1]],
            [w[2], 0, -w[0]],
            [-w[1], w[0], 0]
        ])

        # Rodrigues rotation matrix
        R = eye(3) + sin(theta)*skew_w + (1 - cos(theta))*(skew_w**2)
        qx, qy, qz = symbols('q_x q_y q_z', real=True)
        q = Matrix([[qx], [qy], [qz]])

        v = -skew_w*q
        Rv = (eye(3)*theta + (1 - cos(theta)) *
              (skew_w) + (theta-sin(theta))*(skew_w**2))*v

        T = Matrix([[R[0, 0], R[0, 1], R[0, 2], Rv[0]], [R[1, 0], R[1, 1], R[1, 2], Rv[1]], [
                   R[2, 0], R[2, 1], R[2, 2], Rv[2]], [0, 0, 0, 1]])

        M = Matrix([[0, 0, 1, 0.895107], [0, 1, 0, 0],
                   [-1, 0, 0, 1.26315], [0, 0, 0, 1]])
        T1 = T.subs({theta: self.t1, w0: 0, w1: 0,
                    w2: 1, qx: 0, qy: 0, qz: 0.45})
        T2 = T.subs({theta: self.t2, w0: 0, w1: 1,
                    w2: 0, qx: 0.155, qy: 0, qz: 0.45})
        T3 = T.subs({theta: self.t3, w0: 0, w1: -1, w2: 0,
                    qx: 0.154877, qy: 0, qz: 1.064})
        T4 = T.subs({theta: self.t4, w0: 0, w1: 1, w2: 0,
                    qx: 0.795107, qy: 0, qz: 1.26326})
        T5 = T.subs({theta: self.t5, w0: -1, w1: 0, w2: 0,
                    qx: 0.795107, qy: 0, qz: 1.26326})
        return T1*T2*T3*T4*T5*M


def main(args=None):
    rclpy.init(args=args)

    joints_sub = JointSubscriber()

    rclpy.spin(joints_sub)

    joints_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
