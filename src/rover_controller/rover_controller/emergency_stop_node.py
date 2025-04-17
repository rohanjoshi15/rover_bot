import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.sub = self.create_subscription(
            String,
            '/proximity_warning',
            self.warning_callback,
            10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def warning_callback(self, msg):
        if msg.data == 'STOP':
             self.get_logger().info('Emergency stop triggered')
             stop = Twist()
             stop.linear.x = 0.0
             stop.angular.z =  0.0
             self.pub.publish(stop)
        
            


def main():
    rclpy.init()
    node = EmergencyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

