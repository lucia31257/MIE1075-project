#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select

class TextInput(Node):
    def __init__(self):
        super().__init__('simple_input')
        
        self.command_pub = self.create_publisher(String, '/user_command', 10)
        self.response_sub = self.create_subscription(
            String, '/assistant/response', self.response_callback, 10)
        
        self.timer = self.create_timer(0.5, self.check_input)
        
        self.get_logger().info('Simple Input Node Started!')
        print("\n" + "="*50)
        print("SHOP ASSISTANT - Enter your command:")
        print("Try: 'find milk' or 'go to kitchen'")
        print("="*50)
        print("\n>> ", end='', flush=True)
    
    def check_input(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            user_text = sys.stdin.readline().strip()
            if user_text:
                msg = String()
                msg.data = user_text
                self.command_pub.publish(msg)
                self.get_logger().info(f'Command: {user_text}')
    
    def response_callback(self, msg):
        print(f"\n[ROBOT]: {msg.data}")
        print(">> ", end='', flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = TextInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nGoodbye!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()