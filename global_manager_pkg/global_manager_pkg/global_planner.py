#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://docs.nav2.org/commander_api/index.html
#https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py
#https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
#https://roboticsbackend.com/ros2-yaml-params/

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from custom_interfaces.msg import DockTrigger
import time

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#current_state: navigation, docking, undocking, waiting, docking_fail

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        #Subcriber
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

        #Publisher
        self.publisher_start_docking = self.create_publisher(DockTrigger, '/start_dock', 1)
        self.publisher_start_undocking = self.create_publisher(Bool, '/start_undock', 1)

        #Parameter
        self.declare_parameter('start_place', 'ws_0') 

        init_pose = PoseStamped()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.x = 0.0
        init_pose.pose.orientation.y = 0.0
        init_pose.pose.orientation.z = 0.0
        init_pose.pose.orientation.w = 1.0
        
        self.nav = BasicNavigator()
        self.nav.setInitialPose(init_pose)
        self.nav.lifecycleStartup()

        self.current_state = 'waiting'
        self.current_ws = self.get_parameter('start_place').value #Startpunkt
        self.current_nav_goal = None
        self.current_dock_goal = None

        self.start_nav_time = None

        self.get_logger().info('Starting planer loop')
        self.timer = self.create_timer(1, self.global_planer) 

    def global_planer(self):
        self.get_logger().info(str(self.current_ws))

        #Warten bis Docking beendet ist
        if self.current_state == 'docking':
            self.get_logger().info('Docking to WS')
            return

        #Warten bis Abdocken beendet ist
        if self.current_state == 'undocking':
            self.get_logger().info('Undocking from WS')
            return
        
        if self.current_state == 'docking_fail':
            self.get_logger().info('Docking to workspace faild')
            self.current_state = 'waiting'
            self.current_ws = 'no_ws'
            self.current_nav_goal = None
        
        #Check ob es aktives Navigationsziel gibt
        if self.current_nav_goal is  None:
            self.get_logger().info('Waiting for Dock Goal')
            return
            
        if self.current_nav_goal == self.current_ws:
            self.get_logger().info('Goal reached')
            return

        #Es gibt ein Ziel aber die Navigation wurde noch nicht initialisiert.
        if self.current_state == 'waiting':
            self.get_logger().info('Start new Navigation')
            nav_goal_pose = self.get_nav_goal_pos(self.current_nav_goal)

            if nav_goal_pose is None:
                self.get_logger().info('No Ws pose info')
                return

            res = self.set_navigation_goal(nav_goal_pose,self.current_nav_goal)
            self.current_state = 'navigating'
            self.current_ws = 'no_ws'

        #Es gibt ein Ziel welches noch nicht initialisiert ist und der Roboter ist angedockt.
        if self.current_state == 'docked':
            self.get_logger().info('Start undocking')
            self.start_undocking() #self.current_state wird durch undocking_node geändert zu undocking

        #Navigation läuft
        if self.current_state == 'navigating':
            #TODO: add timeout
            if not self.nav.isTaskComplete():
                self.get_logger().info('Navigation to goal')
                return

            result = self.nav.getResult()

            #TODO: add else
            if result == TaskResult.SUCCEEDED:
                self.declare_parameter(f'{self.current_nav_goal}.docking', False)

                if self.get_parameter(f'{self.current_nav_goal}.docking').value:
                    self.start_docking(self.current_nav_goal)
                    self.current_state = 'docking'
                    self.get_logger().info('Start docking')
                
                else:
                    self.current_state = 'waiting'
                    self.current_ws = self.current_nav_goal
                    self.get_logger().info('waiting')

                self.undeclare_parameter(f'{self.current_nav_goal}.docking')

                self.current_nav_goal = None

    def get_nav_goal_pos(self,goal):
        return_pose = PoseStamped()
        return_pose.header.frame_id = 'map'
        return_pose.header.stamp = self.nav.get_clock().now().to_msg() 
        
        try:
            self.declare_parameter(f'{goal}.pose.x', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{goal}.pose.y', rclpy.Parameter.Type.DOUBLE)
            return_pose.pose.position.x = self.get_parameter(f'{goal}.pose.x').value
            return_pose.pose.position.y = self.get_parameter(f'{goal}.pose.y').value
            self.undeclare_parameter(f'{goal}.pose.x')
            self.undeclare_parameter(f'{goal}.pose.y')
        except:
            return None

        try:
            self.declare_parameter(f'{goal}.orientation.x', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{goal}.orientation.y', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{goal}.orientation.z', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f'{goal}.orientation.w', rclpy.Parameter.Type.DOUBLE)
            return_pose.pose.orientation.x = self.get_parameter(f'{goal}.orientation.x').value
            return_pose.pose.orientation.y = self.get_parameter(f'{goal}.orientation.y').value
            return_pose.pose.orientation.z = self.get_parameter(f'{goal}.orientation.z').value
            return_pose.pose.orientation.w = self.get_parameter(f'{goal}.orientation.w').value
            self.undeclare_parameter(f'{goal}.orientation.x')
            self.undeclare_parameter(f'{goal}.orientation.y')
            self.undeclare_parameter(f'{goal}.orientation.z')
            self.undeclare_parameter(f'{goal}.orientation.w')
        except:
            #Default Orientierung verwenden
            return_pose.pose.orientation.x = 0.0
            return_pose.pose.orientation.y = 0.0
            return_pose.pose.orientation.z = 0.0
            return_pose.pose.orientation.w = 1.0

        return return_pose
        
    def set_navigation_goal(self,nav_goal_pose,ws):

        self.get_logger().info(f'Navigat to {ws}')
        self.start_nav_time = time.time()
        self.nav.goToPose(nav_goal_pose)

    def start_docking(self,ws):
        if self.start_nav_time is not None:
            self.get_logger().info('Navigation time' + ' ' + str(time.time() - self.start_nav_time) + 's')
            self.start_nav_time = None
        else:
            self.get_logger().info('Unable to measure navigation time')

        #Default ws ist 1
        try:
            ws_nr = int(ws.split('_')[1])
            self.get_logger().info(f'Dock to ws: {ws_nr}')
        except:
            self.get_logger().info('Dock to Default ws: 1')
            ws_nr = 1

        msg = DockTrigger()
        msg.trigger = True
        msg.wsnumber = ws_nr
        self.publisher_start_docking.publish(msg)

    def start_undocking(self):
        msg = Bool()
        msg.data = True
        self.publisher_start_undocking.publish(msg)
        
    def callback_dock_goal(self,msg):
        self.current_nav_goal = msg.data     
        self.current_dock_goal = msg.data     
        
    def callback_current_state(self,msg):
        self.current_state = msg.data
        self.get_logger().info(str(self.current_state))
        
    def callback_current_ws(self,msg):
        self.current_ws = msg.data

def main(args=None):
    rclpy.init(args=args)

    global_planner = GlobalPlanner()

    rclpy.spin(global_planner)
    
    global_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()