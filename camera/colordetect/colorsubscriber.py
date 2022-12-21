import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

class ColorSubscriber(Node):

    def __init__(self):
        super().__init__("ColorSubscriber")
        self.subscription = self.create_subscription(
            ColorRGBA,
            'rgb_value',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(str(msg))
        
def main(args=None):
    rclpy.init(args=args)

    sub = ColorSubscriber()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        