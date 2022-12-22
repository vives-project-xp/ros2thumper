import rclpy
from rclpy.node import Node
from led_command_interface.msg import LedCommand
import time
import board
import neopixel

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)


def rainbow_cycle(wait):
    for j in range(255):
        for i in range(24):
            pixel_index = (i * 256 // 24) + j
            pixels_C[12 + i] = wheel(pixel_index & 255)
        pixels_C.show()
        time.sleep(wait)

def get_rgb_values(value):
    r = ((value >> 16) & 255)
    g = ((value >> 8) & 255)
    b = ((value >> 0) & 255)
    return (g, r, b ,0)


pixel_pin_A = board.D18
pixel_pin_B = board.D12
pixel_pin_C = board.D21
# pixel_pin_D = board.D21

num_pixels_A = 24
num_pixels_B = 24
num_pixels_C = 24
# num_pixels_D = 12

ORDER = neopixel.RGBW

pixels_A = neopixel.NeoPixel(
    pixel_pin_A, num_pixels_A, brightness=0.2, auto_write=False, pixel_order=ORDER
)

pixels_B = neopixel.NeoPixel(
    pixel_pin_B, num_pixels_B, brightness=0.2, auto_write=False, pixel_order=ORDER
)

pixels_C = neopixel.NeoPixel(
    pixel_pin_C, num_pixels_C, brightness=0.2, auto_write=False, pixel_order=ORDER
)

# pixels_D = neopixel.NeoPixel(
#     pixel_pin_D, num_pixels_D, brightness=0.2, auto_write=False, pixel_order=ORDER
# )

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.subscription = self.create_subscription(
            LedCommand,
            'ledValue',
            self.led_command_callback,
            10)
        # self.led_command_sub = rclpy.Subscriber('/ledValue', LedCommand, self.led_command_callback)
        self.subscription
        self.get_logger().info("This node just says 'Hello'")

    def led_command_callback(self, msg):
        self.get_logger().info("This test just if 'callback works'")
        self.get_logger().info(str(msg.hazerd_light[0]))
        for i in range(12):
            pixels_A[i] = get_rgb_values(msg.front_left[i])
        for i in range(12):
            pixels_A[12 + i] = get_rgb_values(msg.front_right[i])
        pixels_A.show()

        for i in range(12):
            pixels_B[i] = get_rgb_values(msg.back_left[i])
        for i in range(12):
            pixels_B[12 + i] = get_rgb_values(msg.back_right[i])
        pixels_B.show()

        for i in range(12):
            pixels_C[i] = get_rgb_values(msg.hazerd_light[i])

        rainbow_cycle(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()











