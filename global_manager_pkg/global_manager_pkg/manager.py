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

class Manager(Node):
    def __init__(self):
        super().__init__('manager')
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

        self.publisher_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.publisher_start_docking = self.create_publisher(Bool, '/start_dock', 1)
        self.publisher_start_nav2 = self.create_publisher(Bool, '/start_nav2', 1)

        init_pose = PoseStamped()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.x = 0.0
        init_pose.pose.orientation.y = 0.0
        init_pose.pose.orientation.z = 0.0
        init_pose.pose.orientation.w = 1.0
        self.current_pose = init_pose
        
        #msg = Bool()
        #msg.data = True
        #self.publisher_start_nav2.publish(msg)

        self.nav = BasicNavigator()
        self.nav.setInitialPose(init_pose)
        #self.nav.waitUntilNav2Active()
        self.nav.lifecycleStartup()

        self.current_state = None
        self.current_ws = None

    #TODO: do this
    def stop_all(self):
        pass

    def get_nav_goal_pos(self,goal):
        #TODO: get info from config
        return_pose = PoseStamped()
        if goal == 'ws_1':
            #TODO: add orientation
            return_pose.header.frame_id = 'map'
            return_pose.header.stamp = self.nav.get_clock().now().to_msg()
            return_pose.pose.position.x = 0.52
            return_pose.pose.position.y = -3.96
            return return_pose
        
        else:
            return None
        
    def set_navigation_goal(self,nav_goal_pose,ws):
        
        #msg = PoseStamped()
        #msg.pose = nav_goal_pose
        #msg.header.frame_id = 'map'
        #self.publisher_goal_pose.publish(nav_goal_pose)

        self.get_logger().info(f'Navigat to {ws}')

        
        start = PoseStamped()
        #TODO: must be dynamic
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        #path = self.nav.getPath(start, nav_goal_pose)
        #smoothed_path = self.nav.smoothPath(path)
        self.nav.goToPose(nav_goal_pose)

        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #    self.nav.cancelTask()
            #    return False
            
        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            #msg = Bool()
            #msg.data = False
            #self.publisher_start_nav2.publish(msg)
            return True
        else:
            return False

        

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
        
    def callback_dock_goal(self,msg):
        self.stop_all()
        nav_goal_pose = self.get_nav_goal_pos(msg.data)

        if nav_goal_pose is None:
            self.get_logger().info('No Ws pose info')
            return
        
        #TODO: nach ein paar malen aufhören
        res = self.set_navigation_goal(nav_goal_pose,msg.data)
        while res == False:
            res = self.set_navigation_goal(nav_goal_pose,msg.data)
        
        self.get_logger().info('Start Docking')
        self.start_docking(msg.data)
        

        #self.set_navigation_goal(nav_goal_pose,msg.data)

        #while self.nav_goal_not_reached():
        #    pass

def main(args=None):
    rclpy.init(args=args)

    manager = Manager()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure that the shutdown happens only if the context was initialized
        if rclpy.ok():
            executor.shutdown()
            manager.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()