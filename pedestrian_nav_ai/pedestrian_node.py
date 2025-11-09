import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PedestrianNode(Node):
    def __init__(self):
        super().__init__('pedestrian_node')
        self.publisher_ = self.create_publisher(String, 'pedestrian_info', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Pedestrian Node Initialized')

    def timer_callback(self):
        msg = String()
        msg.data = 'Pedestrian Node Active...'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
