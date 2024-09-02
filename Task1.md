打开一个终端并运行以下命令来启动turtlesim仿真界面：
```
ros2 run turtlesim turtlesim_node
```
在ROS2的工作空间中，创建一个新的功能包，然后运行以下命令
```
ros2 pkg create --build-type ament_python turtle_control
```
1.长方形轨迹
```
cd ~/ros2_ws/src/turtle_control/turtle_control/
touch rectangle_motion.py
chmod +x rectangle_motion.py
```
编辑python文件
```
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
```
2.实现服务调用功能
```
touch spawn_turtle.py
chmod +x spawn_turtle.py
```
编辑服务调用python文件
```
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.cli = self.create_client(Spawn, '/spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Spawn.Request()

    def send_request(self, x, y, theta, name):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    response = turtle_spawner.send_request(2.0, 2.0, 0.2, '')
    if response is not None:
        turtle_spawner.get_logger().info(f'Successfully spawned turtle: {response.name}')
    else:
        turtle_spawner.get_logger().error('Service call failed')
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
最后编译
```
cd ~/ros2_ws/
colcon build
```
对于任务1:ros2 run turtle_control rectangle_motion
对于任务2:ros2 run turtle_control spawn_turtle
