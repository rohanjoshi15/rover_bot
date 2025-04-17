import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class ProximityWarning(Node):
    def __init__(self):
        super().__init__('proximity_warning')
        self.sub = self.create_subscription(
            Float32,
            '/min_distance',
            self.distance_callback,
            10
        )
        self.pub = self.create_publisher(String, '/proximity_warning', 10)

    def distance_callback(self, msg):
        if msg.data <= 1:
            self.get_logger().warn('wall is close')
            self.pub.publish(String(data='STOP'))

def main():
    rclpy.init()
    node = ProximityWarning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

