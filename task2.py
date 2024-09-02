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

