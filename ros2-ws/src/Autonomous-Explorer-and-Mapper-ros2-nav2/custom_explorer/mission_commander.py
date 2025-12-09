import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import sys

# Global variable to store the received target
received_target = None

class CommsNode(Node):
    def __init__(self):
        super().__init__('robot_comms_node')
        
        # 1. Subscribe to voice commands
        self.subscription = self.create_subscription(
            PoseStamped,
            '/voice_command/target_coords',
            self.listener_callback,
            10)
        
        # 2. Publisher to notify image recognition system
        self.detection_publisher = self.create_publisher(Bool, '/start_detection', 10)
        
        # 3. Publisher for status updates
        self.status_publisher = self.create_publisher(String, '/search/status', 10)
        
        print(">>> Comms Node Started: Waiting for voice commands (PoseStamped)...")

    def listener_callback(self, msg):
        global received_target
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        status_msg = f"New command received: x={target_x:.2f}, y={target_y:.2f}"
        self.publish_status(status_msg)
        print(f"\n[Command Received] {status_msg}")
        
        received_target = msg

    def send_arrival_signal(self):
        msg = Bool()
        msg.data = True
        self.detection_publisher.publish(msg)
        print(">>> [Signal Sent] Notified image recognition to start")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)

def main():
    rclpy.init(args=sys.argv)
    
    comms_node = CommsNode()
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

    # --- Modified Section Start ---
    
    msg_init = "Setting initial pose automatically..."
    print(msg_init)
    comms_node.publish_status(msg_init)
    
    navigator.setInitialPose(initial_pose)

    msg_wait = "Waiting for Nav2 to activate..."
    print(msg_wait)
    comms_node.publish_status(msg_wait)
    
    navigator.waitUntilNav2Active()
    
    msg_ready = "Nav2 is ready! Waiting for tasks."
    print(msg_ready)
    comms_node.publish_status(msg_ready)
    
    # --- Modified Section End ---
    
    # ==========================================================
    # Main Loop
    # ==========================================================
    global received_target
    
    try:
        while rclpy.ok():
            rclpy.spin_once(comms_node, timeout_sec=0.1)
            
            if received_target is not None:
                goal_pose = received_target
                received_target = None

                # Refresh timestamp
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                if not goal_pose.header.frame_id:
                    goal_pose.header.frame_id = 'map'

                target_x = goal_pose.pose.position.x
                target_y = goal_pose.pose.position.y

                start_msg = f"Navigating to: x={target_x:.2f}, y={target_y:.2f} ..."
                print(start_msg)
                comms_node.publish_status(start_msg)
                
                navigator.goToPose(goal_pose)

                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback:
                        dist_msg = f"Distance remaining: {feedback.distance_remaining:.2f} m"
                        print(dist_msg, end='\r')
                        comms_node.publish_status(dist_msg)
                    
                    rclpy.spin_once(comms_node, timeout_sec=0.1)

                result = navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    success_msg = "Mission Complete! Arrived at destination."
                    print(f"\n✅ {success_msg}")
                    comms_node.publish_status(success_msg)
                    comms_node.send_arrival_signal()
                    
                elif result == TaskResult.CANCELED:
                    cancel_msg = "Mission Canceled."
                    print(f"\n❌ {cancel_msg}")
                    comms_node.publish_status(cancel_msg)

                elif result == TaskResult.FAILED:
                    fail_msg = "Mission Failed."
                    print(f"\n❌ {fail_msg}")
                    comms_node.publish_status(fail_msg)
                
                wait_msg = "Waiting for next voice command..."
                print(wait_msg)
                comms_node.publish_status(wait_msg)

    except KeyboardInterrupt:
        print("Program shutting down")

    navigator.lifecycleShutdown()
    comms_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()