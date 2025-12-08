import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import sys

# Global variable to store the received target
received_target = None

# Create a node specifically for communication (Subscriber & Publisher)
class CommsNode(Node):
    def __init__(self):
        super().__init__('robot_comms_node')
        
        # 1. Subscribe to voice commands
        # Topic: /voice_command/target_coords
        # Message Type: Point (contains x, y)
        self.subscription = self.create_subscription(
            Point,
            '/voice_command/target_coords',
            self.listener_callback,
            10)
        
        # 2. Publisher to notify image recognition system
        # Topic: /start_detection
        # Message Type: Bool (True/False)
        self.detection_publisher = self.create_publisher(Bool, '/start_detection', 10)
        
        print(">>> Comms Node Started: Waiting for voice commands...")

    def listener_callback(self, msg):
        global received_target
        print(f"\n[Command Received] New target: x={msg.x}, y={msg.y}")
        received_target = msg

    def send_arrival_signal(self):
        msg = Bool()
        msg.data = True
        self.detection_publisher.publish(msg)
        print(">>> [Signal Sent] Notified image recognition to start (start_detection = True)")

def main():
    # 1. Initialize ROS 2
    rclpy.init(args=sys.argv)
    
    # Instantiate the communication node
    comms_node = CommsNode()
    
    # Instantiate the navigator
    navigator = BasicNavigator()

    # ==========================================================
    # Automatically set initial pose
    # ==========================================================
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0 
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0 

    print("Setting initial pose automatically...")
    navigator.setInitialPose(initial_pose)
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ready! Waiting for tasks.")
    
    # ==========================================================
    # Main Loop: Listen for commands and execute navigation
    # ==========================================================
    global received_target
    
    try:
        while rclpy.ok():
            # 1. Check for new messages on the topic
            # spin_once allows comms_node to check if a message arrived
            rclpy.spin_once(comms_node, timeout_sec=0.1)
            
            # 2. If a target was received
            if received_target is not None:
                goal_x = received_target.x
                goal_y = received_target.y
                
                # Reset global variable to avoid repeating the same task
                received_target = None
                
                # Construct the goal pose
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.orientation.w = 1.0

                print(f"Navigating to: x={goal_x}, y={goal_y} ...")
                navigator.goToPose(goal_pose)

                # Loop to print feedback while moving
                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback:
                        print(f"Distance remaining: {feedback.distance_remaining:.2f} m", end='\r')
                    
                    # Important: Keep spinning to maintain ROS connection during movement
                    rclpy.spin_once(comms_node, timeout_sec=0.1)

                # Handle the result
                result = navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    print("\n✅ [Arrived] Mission Complete!")
                    
                    # === KEY STEP: Notify Jessica's program ===
                    comms_node.send_arrival_signal()
                    # ==========================================
                    
                elif result == TaskResult.CANCELED:
                    print("\n❌ Mission Canceled.")
                elif result == TaskResult.FAILED:
                    print("\n❌ Mission Failed.")
                
                print("\nWaiting for next voice command...")

    except KeyboardInterrupt:
        print("Program shutting down")

    # Cleanup
    navigator.lifecycleShutdown()
    comms_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()