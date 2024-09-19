#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.nav2.org/commander_api/index.html
#https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class GlobalPlaner(Node):
    def __init__(self):
        super().__init__('global_planer')
        self.subscription_dock_goal = self.create_subscription(
            String,
            '/dock_goal',
            self.callback_dock_goal,
            1)
        self.subscription_dock_goal
        
        self.subscription_current_state = self.create_subscription(
            String,
            '/current_state',
            self.callback_current_state,
            1)
        self.subscription_current_state
        
        self.subscription_current_ws = self.create_subscription(
            String,
            '/current_ws',
            self.callback_current_ws,
            1)
        self.subscription_current_ws
        
        self.subscription_current_pos = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_current_pos,
            1)
        self.subscription_current_pos

        self.publisher_start_docking = self.create_publisher(Bool, '/start_dock', 1)
        self.publisher_start_nav2 = self.create_publisher(Bool, '/start_nav2', 1)

        self.current_state = None
        self.current_ws = None

    #TODO: do this
    def stop_all(self):
        pass

    def start_docking(self,ws):
        #TODO: add workplace check
        msg = Bool()
        msg.data = True
        self.publisher_start_docking.publish(msg)

    def callback_current_pos(self,msg):
        self.current_pose = msg
        
    def callback_current_ws(self,msg):
        self.current_state = msg.data
        
    def callback_current_state(self,msg):
        self.current_ws = msg.data

    def get_path(self,start,goal):
        pass
        
    def callback_dock_goal(self,msg):
        self.stop_all()
        self.get_path(self.current_ws,msg)

def main(args=None):
    rclpy.init(args=args)

    global_planer = GlobalPlaner()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(global_planer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            global_planer.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()