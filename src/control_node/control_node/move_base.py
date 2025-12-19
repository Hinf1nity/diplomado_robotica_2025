import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVel(Node):
    def __init__(self):
        super().__init__('cmd_pub')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = 0.
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVel()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()