#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32
from priority_interfaces.action import PriorityAction
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

class PriorityActionClient(Node):
    def __init__(self):
        super().__init__('priority_action_client')
        
        # Action clients
        self.high_priority_client = ActionClient(
            self, PriorityAction, 'high_priority_action')
        self.low_priority_client = ActionClient(
            self, PriorityAction, 'low_priority_action')
        
        # Command subscriber
        self.subscription = self.create_subscription(
            Int32,
            'task_commands',
            self.command_callback,
            10)
        
        self.current_goal_handle = None
        self.current_priority = -1
        self.get_logger().info('Priority action client ready')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

        if msg.data == self.current_priority:
            self.get_logger().info('Ignoring same priority command')
            return
        
        if msg.data not in [0, 1]:
            self.get_logger().error('Invalid priority')
            return
            
        if self.current_goal_handle and msg.data < self.current_priority:
            self.get_logger().info('Ignoring lower priority command')
            return
            
        self.send_goal(msg.data)

    def send_goal(self, priority):
        # Cancel current goal if exists
        if self.current_goal_handle:
            self.get_logger().info('Cancelling current goal')
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(lambda _: self._send_new_goal(priority))
        else:
            self._send_new_goal(priority)

    def _send_new_goal(self, priority):
        goal_msg = PriorityAction.Goal()
        goal_msg.priority = priority
        goal_msg.order = 15
        
        client = self.high_priority_client if priority == 1 else self.low_priority_client
        self.get_logger().info(f'Sending priority {priority} goal')
        
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda future: self._goal_response_callback(future, priority))

    def _goal_response_callback(self, future, priority):
        goal_handle = future.result()
        # if not goal_handle.accepted:
        #     self.get_logger().error('Goal rejected')
        #     return
            
        self.current_goal_handle = goal_handle
        self.current_priority = priority
        self.get_logger().info(f'Started priority {priority} goal')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result_response = future.result()
        status = result_response.status
        
        self.current_goal_handle = None
        self.current_priority = -1
        
        try:
            result = result_response.result
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Current goal succeeded with result: {result.sequence}')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f'Previous goal was canceled with partial result: {result.sequence}')
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info(f'Previous goal was aborted with partial result: {result.sequence}')
            else:
                self.get_logger().info(f'Current goal finished with status: {status} and result: {result.sequence}')
                
        except Exception as e:
            self.get_logger().error(f'Error handling goal result: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    client = PriorityActionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()