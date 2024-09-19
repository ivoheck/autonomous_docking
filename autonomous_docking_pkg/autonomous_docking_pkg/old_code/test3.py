import rclpy
from rclpy.node import Node

class MyNode3(Node):
    def __init__(self):
        super().__init__('my_node_3')
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer callback Test 3')
        # Stop the node after one callback
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode3()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()