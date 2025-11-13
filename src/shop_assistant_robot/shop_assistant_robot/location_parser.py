#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class LocationParser(Node):
    def __init__(self):
        super().__init__('location_parser')
        
        self.command_sub = self.create_subscription(
            String, '/user_command', self.command_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.response_pub = self.create_publisher(String, '/assistant/response', 10)
        
        self.locations = {
            'kitchen': {'x': 1.5, 'y': 1.0},
            'bedroom': {'x': -1.0, 'y': 1.5},
            'living room': {'x': 0.5, 'y': -1.0},
            'entrance': {'x': -2.0, 'y': -0.5},
        }
        
        self.products = {
            'milk': {'location': 'kitchen', 'price': 3.99},
            'cheese': {'location': 'entrance', 'price': 4.99},
            'bread': {'location': 'living room', 'price': 2.49},
            'apple': {'location': 'bedroom', 'price': 0.99},
        }
        
        self.get_logger().info('Location Parser Node Started!')
    
    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Processing: {command}')
        
        if 'find' in command or 'where' in command:
            self.handle_find_command(command)
        elif 'go to' in command or 'take me to' in command:
            self.handle_goto_command(command)
        else:
            self.send_response("Try 'find milk' or 'go to kitchen'")
    
    def handle_find_command(self, command):
        for product_name, product_info in self.products.items():
            if product_name in command:
                location_name = product_info['location']
                price = product_info['price']
                self.send_response(
                    f"Found {product_name}! Price: ${price}. "
                    f"It's in the {location_name}. Follow me!")
                self.navigate_to_location(location_name)
                return
        self.send_response("Sorry, I don't know where that product is.")
    
    def handle_goto_command(self, command):
        for location_name in self.locations.keys():
            if location_name in command:
                self.send_response(f"Heading to {location_name}. Follow me!")
                self.navigate_to_location(location_name)
                return
        self.send_response("Sorry, I don't know that location.")
    
    def navigate_to_location(self, location_name):
        if location_name not in self.locations:
            return
        
        loc = self.locations[location_name]
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = loc['x']
        goal_msg.pose.position.y = loc['y']
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Goal: {location_name} at ({loc["x"]}, {loc["y"]})')
    
    def send_response(self, text):
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocationParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()