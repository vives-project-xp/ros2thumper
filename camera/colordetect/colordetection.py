import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
import cv2



class ColorDetection(Node):
    def __init__(self):
    	super().__init__("RGB_publisher")
    	self.writer = self.create_publisher(ColorRGBA, 'rgb_value', 10)
    	while(True):
        	color = VideoCamera().get_frame()
        	msg = ColorRGBA()
        	msg.r = float(color[2])
        	msg.g = float(color[1])
        	msg.b = float(color[0])
        	self.writer.publish(msg)
        	self.get_logger().info(str(msg))
        
class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        
    def __del__(self):
        self.video.release()
    
    def get_frame(self):
        success, image = self.video.read()
        
        color = 0
        if success:
            height, width = image.shape[:2]
            cx = int(width/2)
            cy = int(height/2)
            color = image[cy, cx]
            b, g, r = color
        return color

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()