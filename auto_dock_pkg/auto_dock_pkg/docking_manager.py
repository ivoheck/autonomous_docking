import rclpy
from rclpy.node import Node
from custom_interfaces.msg import DockTrigger
from custom_interfaces.msg import DockFeedback
from std_msgs.msg import String
import time

class DockingManager(Node):
    def __init__(self):
        super().__init__('docking_manager')
        self.subscription = self.create_subscription(
            DockTrigger,
            '/start_dock',
            self.listener_callback_start_dock,
            1)
        self.subscription

        self.subscription_dock_feedback = self.create_subscription(
            DockFeedback,
            '/dock_feedback',
            self.listener_callback_dock_feedback,
            1)
        self.subscription_dock_feedback

        self.publisher_find_qr_node_state = self.create_publisher(DockTrigger,'dock_trigger/find_qr_trigger', 1)
        self.publisher_drive_to_qr_node_state = self.create_publisher(DockTrigger,'dock_trigger/drive_to_qr_trigger', 1)
        self.publisher_face_wall_node_state = self.create_publisher(DockTrigger,'dock_trigger/face_wall_trigger', 1)
        self.publisher_drive_to_wall_node_state = self.create_publisher(DockTrigger,'dock_trigger/drive_to_wall_trigger', 1)
        self.publisher_qr_center_node_state = self.create_publisher(DockTrigger,'dock_trigger/qr_center_trigger', 1)
        self.publisher_final_docking_node_state = self.create_publisher(DockTrigger,'dock_trigger/final_docking_trigger', 1)
        self.publisher_qr_scan_node_state = self.create_publisher(DockTrigger,'dock_trigger/qr_scan_trigger', 1)
        self.publisher_current_state = self.create_publisher(String, '/current_state', 1)
        self.publisher_current_ws = self.create_publisher(String, '/current_ws', 1)

        self.current_dock_stamp = None
        self.current_dock_state = None
        self.current_ws = None

        #if __name__ == '__main__':
        #    msg_find_qr = DockTrigger()
        #    msg_find_qr.trigger = True
        #    msg_find_qr.time = int(time.time())
        #    self.current_dock_stamp = int(time.time())
        #    self.current_dock_state = 'find_qr'
        #    
        #    msg_qr_scan = DockTrigger()
        #    msg_qr_scan.trigger = True
        #    msg_qr_scan.wsnumber = 1
        #
        #    self.publisher_qr_scan_node_state.publish(msg=msg_qr_scan)
        #    self.publisher_find_qr_node_state.publish(msg=msg_find_qr)

    def stop_all_processes(self):
        msg = DockTrigger()
        msg.trigger = False

        self.publisher_qr_scan_node_state.publish(msg=msg)
        self.publisher_find_qr_node_state.publish(msg=msg)
        self.publisher_drive_to_qr_node_state.publish(msg=msg)
        self.publisher_face_wall_node_state.publish(msg=msg)
        self.publisher_drive_to_wall_node_state.publish(msg=msg)
        self.publisher_qr_center_node_state.publish(msg=msg) 
        self.publisher_final_docking_node_state.publish(msg=msg)

        self.get_logger().error('Shudown all Docking Nodes')

        self.current_dock_stamp = None
        self.current_dock_state = None
        self.current_ws = None
        

    def listener_callback_dock_feedback(self,msg):
        if msg.process == 'find_qr':
            if msg.success == True and msg.time == self.current_dock_stamp:
                self.get_logger().info('find_qr success')
                self.get_logger().info('starting drive_to_qr')

                msg_drive_to_qr = DockTrigger()
                msg_drive_to_qr.trigger = True
                msg_drive_to_qr.time = self.current_dock_stamp
                self.current_dock_state = 'drive_to_qr'

                self.publisher_drive_to_qr_node_state.publish(msg=msg_drive_to_qr)

            elif msg.success == False and msg.time == self.current_dock_stamp:
                self.get_logger().error('find_qr failed')
                self.get_logger().info('docking time fail' + ' ' + str(time.time() - self.current_dock_stamp))

                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)
                self.stop_all_processes()

            return

        if msg.process == 'drive_to_qr':
            if msg.success == True and msg.time == self.current_dock_stamp:
                self.get_logger().info('drive_to_qr success')
                self.get_logger().info('starting face_wall')

                msg_face_wall = DockTrigger()
                msg_face_wall.trigger = True
                msg_face_wall.time = self.current_dock_stamp
                self.current_dock_state = 'face_wall'

                self.publisher_face_wall_node_state.publish(msg=msg_face_wall)

            elif msg.success == False and msg.time == self.current_dock_stamp:
                self.get_logger().error('drive_to_qr failed')
                self.get_logger().info('docking time fail' + ' ' + str(time.time() - self.current_dock_stamp))

                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)
                self.stop_all_processes()

            return

        if msg.process == 'face_wall':
            if msg.success == True and msg.time == self.current_dock_stamp:
                self.get_logger().info('face_wall success')
                self.get_logger().info('starting drive_to_wall')

                msg_drive_to_wall = DockTrigger()
                msg_drive_to_wall.trigger = True
                msg_drive_to_wall.time = self.current_dock_stamp
                self.current_dock_state = 'drive_to_wall'

                self.publisher_drive_to_wall_node_state.publish(msg=msg_drive_to_wall)

            elif msg.success == False and msg.time == self.current_dock_stamp:
                self.get_logger().error('face_wall failed')
                self.get_logger().info('docking time fail' + ' ' + str(time.time() - self.current_dock_stamp))

                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)
                self.stop_all_processes()

            return

        if msg.process == 'drive_to_wall':
            if msg.success == True and msg.time == self.current_dock_stamp:
                self.get_logger().info('drive_to_wall success')
                self.get_logger().info('starting qr_center')

                msg_qr_center = DockTrigger()
                msg_qr_center.trigger = True
                msg_qr_center.time = self.current_dock_stamp
                self.current_dock_state = 'qr_center'

                self.publisher_qr_center_node_state.publish(msg=msg_qr_center)

            elif msg.success == False and msg.time == self.current_dock_stamp:
                self.get_logger().error('drive_to_wall failed')
                self.get_logger().info('docking time fail' + ' ' + str(time.time() - self.current_dock_stamp))

                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)
                self.stop_all_processes()
            
            return
        
        if msg.process == 'qr_center':
            if msg.success == True and msg.time == self.current_dock_stamp:
                self.get_logger().info('qr_center success')
                self.get_logger().info('starting docking')

                msg_final_docking = DockTrigger()
                msg_final_docking.trigger = True
                msg_final_docking.time = self.current_dock_stamp
                self.current_dock_state = 'final_docking'

                self.publisher_final_docking_node_state.publish(msg=msg_final_docking)

            elif msg.success == False and msg.time == self.current_dock_stamp:
                self.get_logger().error('qr_center failed')
                self.get_logger().info('docking time fail' + ' ' + str(time.time() - self.current_dock_stamp))

                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)
                self.stop_all_processes()

            return
        
        if msg.process == 'final_docking':
            if msg.success == True and msg.time == self.current_dock_stamp:
                self.get_logger().info('final_docking success')
                self.get_logger().info('docking time succses' + ' ' + str(time.time() - self.current_dock_stamp))

                self.current_dock_stamp = None

                msg_current_state = String()
                msg_current_state.data = 'docked'
                self.publisher_current_state.publish(msg=msg_current_state)

                msg_qr_scan = DockTrigger()
                msg_qr_scan.trigger = False
                self.publisher_qr_scan_node_state.publish(msg=msg_qr_scan)

                msg_ws = String()
                msg_ws.data = f'{self.current_ws}'
                self.publisher_current_ws.publish(msg_ws)

                return

            elif msg.success == False and msg.time == self.current_dock_stamp:
                self.get_logger().error('final_docking failed')
                self.get_logger().info('docking time fail' + ' ' + str(time.time() - self.current_dock_stamp))

                msg = String()
                msg.data = 'docking_fail'
                self.publisher_current_state.publish(msg)
                self.stop_all_processes()

    def listener_callback_start_dock(self,msg):
        if msg.trigger:
            self.current_ws = msg.wsnumber

            self.get_logger().info('starting qr_scan')

            msg_qr_scan = DockTrigger()
            msg_qr_scan.trigger = True
            msg_qr_scan.wsnumber = msg.wsnumber
            self.publisher_qr_scan_node_state.publish(msg=msg_qr_scan)

            self.get_logger().info('starting find_qr')
            msg_find_qr = DockTrigger()
            msg_find_qr.trigger = True

            self.current_dock_stamp = time.time()
            msg_find_qr.time = self.current_dock_stamp
            self.current_dock_state = 'find_qr'

            self.publisher_find_qr_node_state.publish(msg=msg_find_qr)

            msg_current_state = String()
            msg_current_state.data = 'docking'
            self.publisher_current_state.publish(msg=msg_current_state)

        

def main(args=None):
    rclpy.init(args=args)

    docking_manager = DockingManager()

    rclpy.spin(docking_manager)
    
    docking_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()