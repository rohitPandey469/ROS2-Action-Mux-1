#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from priority_interfaces.action import PriorityAction

class LowPriorityActionServer(Node):
    def __init__(self):
        super().__init__('low_priority_action_server')
        self._action_server = ActionServer(
            self,
            PriorityAction,
            'low_priority_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Low priority action server is ready')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received LOW priority goal request with order {goal_request.order}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for LOW priority goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing LOW priority goal: wait for {goal_handle.request.order} seconds')
        
        feedback_msg = PriorityAction.Feedback()
        feedback_msg.partial_sequence = []
        
        # Count from 1 to order (seconds to wait)
        for i in range(1, goal_handle.request.order + 1):
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warning('LOW priority goal was canceled!')
                result = PriorityAction.Result()
                result.sequence = feedback_msg.partial_sequence
                return result
                
            # Wait for 1 second
            time.sleep(1)
            
            # Add the current second to the sequence
            feedback_msg.partial_sequence.append(i)
            
            # Publish feedback
            self.get_logger().info(f'LOW priority task: {i} seconds completed of {goal_handle.request.order}')
            goal_handle.publish_feedback(feedback_msg)

        # Goal completed successfully
        goal_handle.succeed()
        
        # Create and return the final result
        result = PriorityAction.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('LOW priority goal completed successfully!')
        return result

def main(args=None):
    rclpy.init(args=args)
    low_priority_server = LowPriorityActionServer()
    
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        # Add nide
        executor.add_node(low_priority_server)
        
        # Spin
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        low_priority_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()