import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import sys

def main():
    # 1. Initialize ROS 2
    rclpy.init(args=sys.argv)
    
    # Create the navigator object
    navigator = BasicNavigator()

    # ADDED STEP: Automatically set the initial pose
    # This eliminates the need to manually click "2D Pose Estimate" in RViz.
    # ==========================================================
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # IMPORTANT: These coordinates must match the spawn location in your Launch file!
    # If your robot spawns at x=0.0, y=0.0, keep these as 0.0.
    initial_pose.pose.position.x = 0.0 
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0 # 1.0 means facing forward (0 degrees)

    # Send the initial pose to AMCL
    print("Setting initial pose automatically...")
    navigator.setInitialPose(initial_pose)
    # ==========================================================

    # Wait for Nav2 to fully activate
    # It should activate automatically now that the initial pose is set.
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ready!")
    
    # ==========================================================
    # 2. Set Destination (Interactive Input)
    # ==========================================================
    try:
        input_x = input("Please enter target X coordinate (e.g., 2.0): ")
        input_y = input("Please enter target Y coordinate (e.g., -1.0): ")
        
        goal_x = float(input_x)
        goal_y = float(input_y)
        goal_w = 1.0
    except ValueError:
        print("Invalid input! Using default coordinates (2.0, -1.0)")
        goal_x = 2.0
        goal_y = -1.0
        goal_w = 1.0

    # 3. Construct the Goal Message
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = goal_w

    # 4. Send Goal
    print(f"[Received Input] Target coordinates: x={goal_x}, y={goal_y}")
    print("Planning path and starting navigation...")
    
    # Send the goal to Nav2
    navigator.goToPose(goal_pose)

    # ==========================================================
    # 5. Feedback Loop (Report position periodically)
    # ==========================================================
    while not navigator.isTaskComplete():
        # Get feedback from the navigation task
        feedback = navigator.getFeedback()
        
        if feedback:
            # current_pose contains the robot's real-time coordinates
            current_x = feedback.current_pose.pose.position.x
            current_y = feedback.current_pose.pose.position.y
            remaining_dist = feedback.distance_remaining
            
            # Print current status
            print(f"[Navigating] Current Pose: x={current_x:.2f}, y={current_y:.2f}")
            print(f"             Distance Remaining: {remaining_dist:.2f} m")

        # Report every 1.5 seconds
        time.sleep(1.5)

    # ==========================================================
    # 6. Handle Final Result
    # ==========================================================
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print("\n✅ [Arrived] Mission Complete! Robot reached the goal.")
    elif result == TaskResult.CANCELED:
        print("\n❌ Mission Canceled.")
    elif result == TaskResult.FAILED:
        print("\n❌ Mission Failed (Path blocked or invalid goal).")

    # Shutdown
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()