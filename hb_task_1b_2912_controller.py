#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
import time

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')

       
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.vel = Twist()

        
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0

       
        self.rate = self.create_rate(100)  # Control loop rate

       
        self.x_goals = [1, -1, -1, 1, 0]  
        self.y_goals = [1, 1, -1, -1, 0] 
        self.theta_goals = [0, math.pi/2, -math.pi, -math.pi/2, 0]  
        self.index = 0  
        self.flag = 0   
        self.at_goal_time = None  

        # Initialize P controller gains (adjust as needed)
        self.kp_linear = 0.5
        self.kp_angular = 0.5

    def odometry_callback(self, msg):
        # Extract and update the robot's pose from the odometry message
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y

        # Convert the quaternion to Euler angles to get the orientation in radians
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.hb_theta = yaw

    def control_loop(self):
        while rclpy.ok():
            # Calculate errors in global frame
            error_x = self.x_goals[self.index] - self.hb_x
            error_y = self.y_goals[self.index] - self.hb_y
            error_theta = self.theta_goals[self.index] - self.hb_theta

            # Implement P controllers for linear and angular velocities
            self.vel.linear.x = self.kp_linear * error_x
            self.vel.linear.y = self.kp_linear * error_y
            self.vel.angular.z = self.kp_angular * error_theta

            # Ensure control velocities are within safe limits
            self.limit_velocities()

            # Publish the control velocities as a Twist message
            self.publisher.publish(self.vel)

            # Safety check: limit control loop rate
            self.rate.sleep()

            # Check if the goal has been reached
            if self.goal_reached(error_x, error_y, error_theta):
                if self.at_goal_time is None:
                    self.at_goal_time = time.time()
                elif time.time() - self.at_goal_time >= 1.0:
                    # Stay at the goal pose for at least 1 second
                    self.at_goal_time = None  # Reset the timer
                    self.index += 1  # Move to the next goal

                    if self.index >= len(self.x_goals):
                        self.index = 0

    def limit_velocities(self):
        # Define maximum linear and angular velocities here (adjust as needed)
        max_linear_velocity = 1.0  # Maximum linear velocity in m/s
        max_angular_velocity = 1.0  # Maximum angular velocity in rad/s

        # Limit linear velocity
        if abs(self.vel.linear.x) > max_linear_velocity:
            self.vel.linear.x = max_linear_velocity
        if abs(self.vel.linear.y) > max_linear_velocity:
            self.vel.linear.y = max_linear_velocity

        # Limit angular velocity
        if abs(self.vel.angular.z) > max_angular_velocity:
            self.vel.angular.z = max_angular_velocity

    def goal_reached(self, error_x, error_y, error_theta):
        # Define error thresholds for position and orientation (adjust as needed)
        position_threshold = 0.1  # Maximum allowed position error in meters
        orientation_threshold = 0.1  # Maximum allowed orientation error in radians

        # Check if the robot's position and orientation errors are within the thresholds
        position_error = math.sqrt(error_x**2 + error_y**2)
        
        return position_error < position_threshold and abs(error_theta) < orientation_threshold
    
    def stay_at_goal(self, duration):
        start_time = time.time()

        while (time.time() - start_time) < duration:
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.angular.z = 0.0

            self.publisher.publish(self.vel)

            time.sleep(1.0 / self.rate.get_rate())


def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()
    ebot_controller.control_loop()
    rclpy.spin(ebot_controller)
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
