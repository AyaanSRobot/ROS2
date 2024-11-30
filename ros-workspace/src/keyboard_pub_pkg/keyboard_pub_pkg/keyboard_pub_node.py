from curtsies import Input

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(
            msg_type=String, 
            topic='key_press', 
            qos_profile=10,
        )

    def publish(self, key: str):
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main():
    rclpy.init()
    publisher = KeyboardPublisher()
    with Input() as input_generator: #Input() provides user keypress events and other control events
        for e in input_generator:
            publisher.publish(e)


if __name__ == '__main__':
    main()
