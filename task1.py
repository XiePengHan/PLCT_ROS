import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer_ = self.create_timer(0.1, self.publish_velocity)
        self.current_x = 0.0
        self.current_y = 0.0
        self.state = 0
        self.start_x = 0.0
        self.start_y = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def publish_velocity(self):
        msg = Twist()
        if self.state == 0:  # Move forward along x-axis
            if self.current_x - self.start_x < 2.0:
                msg.linear.x = 1.0
            else:
                msg.linear.x = 0.0
                self.state = 1  # Change direction
        elif self.state == 1:  # Turn 90 degrees
            msg.angular.z = 1.57
            self.state = 2  # Ready to move forward along y-axis
        elif self.state == 2:  # Move forward along y-axis
            if self.current_y - self.start_y < 1.0:
                msg.linear.y = 1.0
            else:
                msg.linear.y = 0.0
                self.state = 3  # Change direction
        elif self.state == 3:  # Turn 90 degrees
            msg.angular.z = 1.57
            self.state = 0  # Ready to move forward along x-axis again
            self.start_x = self.current_x
            self.start_y = self.current_y

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

