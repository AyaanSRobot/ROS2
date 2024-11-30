import sys

import rclpy
from rclpy.node import Node
from santa_sim_interface import control_sleighn


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.client=self.create_client(control_sleighn,'Control_Sleighn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = control_sleighn.Request()

    def send_request(self, msg):
        
        
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
