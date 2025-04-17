import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(Float32, '/min_distance', 10)

    def scan_callback(self, msg):
        ranges = list(msg.ranges)      
        front_dist = min(ranges)

        self.get_logger().info(f'Front Distance: {front_dist:.2f} m')
        self.pub.publish(Float32(data=front_dist))

def main():
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
