import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odometry',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        self.get_logger().info(
            f'Received Odometry - Position: x={x}, y={y} | '
            f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w} | '
            f'Linear Velocity: x={vx}, y={vy} | '
            f'Angular Velocity: z={angular_velocity}'
        )

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()