import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.start_followme_loop_client = self.create_client(Trigger, 'start_followme_loop')

    def wait_for_service_and_execute(self, timeout_sec):
        self.get_logger().info('Waiting for the start_followme_loop service...')
        
        if not self.start_followme_loop_client.wait_for_service(timeout_sec):
            self.get_logger().error('Failed to connect to the start_followme_loop service')
            return

        self.get_logger().info('start_followme_loop service is available. Proceeding with the code.')

        # Your code to execute when the service is available goes here

def main():
    rclpy.init()
    node = MyNode()
    node.wait_for_service_and_execute(timeout_sec=100.0)  # Wait for up to 300 seconds
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
