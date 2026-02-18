import rclpy
import threading
import time
import math
import numpy as np
import sys

from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import Undock

class Turtlebot(Node):
    def __init__(self):
        super().__init__('turtlebot_move')
        self.get_logger().info("Press Ctrl + C to terminate")
        
        # Publisher for velocity
        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.undock_action_client = ActionClient(self, Undock, '/undock')

        # 1. Linear: 0.94 
        self.linear_calib = 0.94   
        # 2. Angular: 1.05 
        self.angular_calib = 1.05

    def undock(self):
        """Perform the undock action."""
        self.get_logger().info('Undocking...')
        self.undock_action_client.wait_for_server()
        
        goal_msg = Undock.Goal()
        send_goal_future = self.undock_action_client.send_goal_async(goal_msg)
        
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected!')
            return

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)

        self.get_logger().info('Undock complete.')

    def move_robot(self, linear_x, angular_z, duration):
        """Moves the robot for a specific duration (Sim Time)."""
        # Apply calibration
        if linear_x != 0:
            duration = duration * self.linear_calib
        if angular_z != 0:
            duration = duration * self.angular_calib

        start_sim_time = self.get_clock().now()
        target_duration = rclpy.duration.Duration(seconds=duration)
        
        self.get_logger().info(f"Command: Lin={linear_x} m/s, Ang={angular_z} rad/s, Time={duration:.2f}s")

        while (self.get_clock().now() - start_sim_time) < target_duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)
            
            self.vel_pub.publish(msg)
            time.sleep(0.05) 

        # Stop
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.vel_pub.publish(stop_msg)
        time.sleep(1.0)

    def run(self):
        self.undock()
        time.sleep(1.0)
        self.get_logger().info("Starting Diamond Path (3-4-5 Triangle)...")

        speed = 0.2          # 0.2 m/s
        turn_speed = 0.2     # 0.2 rad/s
        dist_hypotenuse = 5.0 

        
        rad_53 = 53.13 * (math.pi / 180.0) 
        rad_36 = 36.87 * (math.pi / 180.0)

        # SEGMENT 1: Start (0,0) -> (3, -4)
        self.get_logger().info("--- Segment 1 : (0,0) -> (3, -4) Turning Right 53.13 deg ---")
        self.move_robot(0.0, -turn_speed, rad_53 / turn_speed)
        
        self.get_logger().info("--- Segment 1 : Moving 5m ---")
        self.move_robot(speed, 0.0, dist_hypotenuse / speed)

        # SEGMENT 2: (3, -4) -> (6, 0)
        self.get_logger().info("--- Segment 2 : (3, -4) -> (6, 0) Turning Left 106.26 deg ---")
        turn_angle = rad_53 * 2.0
        self.move_robot(0.0, turn_speed, turn_angle / turn_speed)
        
        self.get_logger().info("--- Segment 2 : Moving 5m ---")
        self.move_robot(speed, 0.0, dist_hypotenuse / speed)

        # SEGMENT 3: (6, 0) -> (3, 4)
        self.get_logger().info("--- Segment 3 : (6, 0) -> (3, 4) Turning Left 73.74 deg ---")
        turn_angle = rad_36 * 2.0
        self.move_robot(0.0, turn_speed, turn_angle / turn_speed)
        
        self.get_logger().info("--- Segment 3 : Moving 5m ---")
        self.move_robot(speed, 0.0, dist_hypotenuse / speed)

        # SEGMENT 4: (3, 4) -> (0, 0)
        self.get_logger().info("--- Segment 4 : (3, 4) -> (0, 0) Turning Left 106.26 deg ---")
        turn_angle = rad_53 * 2.0
        self.move_robot(0.0, turn_speed, turn_angle / turn_speed)
        
        self.get_logger().info("--- Segment 4 : Moving 5m ---")
        self.move_robot(speed, 0.0, dist_hypotenuse / speed)

        self.get_logger().info("Diamond complete.")

def open_loop_entry_point_function(args=None):
    rclpy.init(args=args)
    turtlebot_node = Turtlebot()
    
    sim_time_param = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
    turtlebot_node.set_parameters([sim_time_param])

    # Run in background thread
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_node,), daemon=True)
    thread.start()

    try:
        turtlebot_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown sequence
        turtlebot_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    open_loop_entry_point_function()
