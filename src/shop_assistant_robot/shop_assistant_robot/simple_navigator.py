#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.response_pub = self.create_publisher(String, '/assistant/response', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Simple Navigator Node Started!')
        if self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('✅ Nav2 is ready!')
        else:
            self.get_logger().warn('⚠️  Nav2 not available yet')
    
    def goal_callback(self, goal_pose):
        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y
        self.get_logger().info(f'📍 Received goal: ({x:.2f}, {y:.2f})')
        
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal_pose
        
        self.send_response("🚀 Navigating to destination...")
        send_goal_future = self.nav_client.send_goal_async(
            action_goal, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            self.send_response("Sorry, can't navigate there.")
            return
        
        self.get_logger().info('✅ Goal accepted! Robot moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info('🤖 Navigating...', throttle_duration_sec=2.0)
    
    def result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('🎉 Navigation succeeded!')
            self.send_response("✅ We've arrived! The product should be right here.")
        else:
            self.get_logger().error(f'❌ Navigation failed: {status}')
            self.send_response("Sorry, I couldn't reach the destination.")
    
    def send_response(self, text):
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()