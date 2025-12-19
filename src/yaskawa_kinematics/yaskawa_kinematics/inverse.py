import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from sympy import Matrix, symbols, cos, sin, pi, diff
from random import random

t1 = symbols('t1')
t2 = symbols('t2')
t3 = symbols('t3')
t4 = symbols('t4')
t5 = symbols('t5')

header = Header()
joint_msg = JointState()
joint_msg.name = ['joint_1_s', 'joint_2_l',
                  'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
alpha = 0.3  # learning rate
iterations = 100


px = (1.26326 - 1.26326*cos(t5))*((sin(t2)*sin(t3)*cos(t1) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + (sin(t2)*cos(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t4)) + 1.26315*((sin(t2)*sin(t3)*cos(t1) + cos(t1)*cos(t2)*cos(t3))*sin(t4) + (sin(t2)*cos(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*cos(t4))*cos(t5) + (sin(t2)*sin(t3)*cos(t1) + cos(t1)*cos(t2)*cos(t3))*(-1.26326*sin(t4) - 0.795107*cos(t4) + 0.795107) + 0.895107*(sin(t2)*sin(t3)
                                                                                                                                                                                                                                                                                                                                                                                                                              * cos(t1) + cos(t1)*cos(t2)*cos(t3))*cos(t4) + (sin(t2)*cos(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*(0.795107*sin(t4) - 1.26326*cos(t4) + 1.26326) - 0.895107*(sin(t2)*cos(t1)*cos(t3) - sin(t3)*cos(t1)*cos(t2))*sin(t4) + (-0.45*sin(t2) - 0.155*cos(t2) + 0.155)*cos(t1) + (-0.154877*sin(t3) - 1.064*cos(t3) + 1.064)*sin(t2)*cos(t1) + (1.064*sin(t3) - 0.154877*cos(t3) + 0.154877)*cos(t1)*cos(t2) + 0.000110000000000054*sin(t1)*sin(t5)


py = (1.26326 - 1.26326*cos(t5))*((sin(t1)*sin(t2)*sin(t3) + sin(t1)*cos(t2)*cos(t3))*sin(t4) + (sin(t1)*sin(t2)*cos(t3) - sin(t1)*sin(t3)*cos(t2))*cos(t4)) + 1.26315*((sin(t1)*sin(t2)*sin(t3) + sin(t1)*cos(t2)*cos(t3))*sin(t4) + (sin(t1)*sin(t2)*cos(t3) - sin(t1)*sin(t3)*cos(t2))*cos(t4))*cos(t5) + (sin(t1)*sin(t2)*sin(t3) + sin(t1)*cos(t2)*cos(t3))*(-1.26326*sin(t4) - 0.795107*cos(t4) + 0.795107) + 0.895107*(sin(t1)*sin(t2)
                                                                                                                                                                                                                                                                                                                                                                                                                              * sin(t3) + sin(t1)*cos(t2)*cos(t3))*cos(t4) + (sin(t1)*sin(t2)*cos(t3) - sin(t1)*sin(t3)*cos(t2))*(0.795107*sin(t4) - 1.26326*cos(t4) + 1.26326) - 0.895107*(sin(t1)*sin(t2)*cos(t3) - sin(t1)*sin(t3)*cos(t2))*sin(t4) + (-0.45*sin(t2) - 0.155*cos(t2) + 0.155)*sin(t1) + (-0.154877*sin(t3) - 1.064*cos(t3) + 1.064)*sin(t1)*sin(t2) + (1.064*sin(t3) - 0.154877*cos(t3) + 0.154877)*sin(t1)*cos(t2) - 0.000110000000000054*sin(t5)*cos(t1)

pz = (1.26326 - 1.26326*cos(t5))*((sin(t2)*sin(t3) + cos(t2)*cos(t3))*cos(t4) + (-sin(t2)*cos(t3) + sin(t3)*cos(t2))*sin(t4)) + 1.26315*((sin(t2)*sin(t3) + cos(t2)*cos(t3))*cos(t4) + (-sin(t2)*cos(t3) + sin(t3)*cos(t2))*sin(t4))*cos(t5) + (sin(t2)*sin(t3) + cos(t2)*cos(t3))*(0.795107*sin(t4) - 1.26326*cos(t4) + 1.26326) - 0.895107 * \
    (sin(t2)*sin(t3) + cos(t2)*cos(t3))*sin(t4) + (-sin(t2)*cos(t3) + sin(t3)*cos(t2))*(-1.26326*sin(t4) - 0.795107*cos(t4) + 0.795107) + 0.895107*(-sin(t2)*cos(t3) + sin(t3)
                                                                                                                                                    * cos(t2))*cos(t4) + (-0.154877*sin(t3) - 1.064*cos(t3) + 1.064)*cos(t2) - (1.064*sin(t3) - 0.154877*cos(t3) + 0.154877)*sin(t2) + 0.155*sin(t2) - 0.45*cos(t2) + 0.45

J = Matrix([[diff(px, t1), diff(px, t2), diff(px, t3), diff(px, t4), diff(px, t5)], [diff(py, t1), diff(py, t2), diff(
    py, t3), diff(py, t4), diff(py, t5)], [diff(pz, t1), diff(pz, t2), diff(pz, t3), diff(pz, t4), diff(pz, t5)]])


class targetSubscriber(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.subscription = self.create_subscription(
            Point, 'target', self.sub_callback, 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription

    def sub_callback(self, msg):
        target = Matrix([msg.x, msg.y, msg.z])
        ti = Matrix([random(), random(), random(), random(), random()])
        for i in range(iterations):
            cp = Matrix([px.subs([(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])]), py.subs(
                [(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])]), pz.subs([(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])])])
            e = target-cp
            Jsubs = J.subs([(t1, ti[0]), (t2, ti[1]),
                           (t3, ti[2]), (t4, ti[3]), (t5, ti[4])])
            Jinv = Jsubs.H*(Jsubs*Jsubs.H)**-1
            dt = Jinv*e
            ti = ti+alpha*dt

            header.stamp = self.get_clock().now().to_msg()
            joint_angles = [ti[0], ti[1], ti[2], -pi, ti[3], ti[4]]
            joint_msg.header = header
            joint_msg.position = joint_angles
            self.publisher_.publish(joint_msg)
        print("Move completed")


def main(args=None):
    rclpy.init(args=args)

    target_sub = targetSubscriber()

    rclpy.spin(target_sub)
    target_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
