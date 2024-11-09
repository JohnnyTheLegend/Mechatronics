#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import transforms3d
import math
import numpy as np

class Trajectory(Node):
    def __init__(self):

        super().__init__('Trajectory_node')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Point, '/robot_path', 10)
        self.goal_publisher = self.create_publisher(Point, '/robot_goal', 10)
        self.pose_publisher = self.create_publisher(Point, '/robot_pose', 10)

        self.create_subscription(Odometry, '/odom',self.update_pose,  10)
        self.side = 2.0
        self.x_coords = [-self.side,self.side,self.side,-self.side]
        self.y_coords = [self.side,self.side,-self.side,-self.side]
        self.index = 0
        self.steps = 0
        self.gain = 2.0
        self.steps_duration = 20
        self.pose = Pose()
        self.init_pose = Pose()
        self.rotation_tolerance = 0.1
        self.distance_tolerance = 0.1
        self.pose_updated = False
        self.odom = Odometry()
        
        ## Timers
        self.rate = 2  # 10 Hz
        self.create_timer(1.0 / self.rate, self.move2goal)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        
        quaternion = data.pose.pose.orientation
        quaternion2 = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion2, axes='sxyz')
        self.pose.theta = roll
        

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)
    
    ## Normalize angle
    def normalize_angle(self,angle):
        """Normalize an angle to be within [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def steering_angle(self, goal_pose):
        angle = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        print(f"Goal pose: ({goal_pose.x},{goal_pose.y}, current: ({self.pose.x},{self.pose.y}))")
        return angle

    def angular_vel(self, goal_pose, constant=1):
        """Calculate angular velocity with angle normalization and optional clamping."""
        # Calculate the desired steering angle
        steering_angle = self.steering_angle(goal_pose)
        
        # Calculate the angle difference and normalize it
        angle_diff = self.normalize_angle(steering_angle - self.pose.theta)
        
        # Calculate the angular velocity
        angular_velocity = constant * angle_diff
        
        return angular_velocity
    
    def generate_trajectory(self,initial_pos,goal_pos, duration, elapsed):
        if elapsed > duration:
            elapsed = duration

        fraction = elapsed / duration

        # Linearly interpolate each coordinate
        x_current = initial_pos.x + (goal_pos.x - initial_pos.x) * fraction
        y_current = initial_pos.y + (goal_pos.y - initial_pos.y) * fraction
        
        res = Pose()
        res.x = x_current
        res.y = y_current

        return (res)
    
            
    def move2goal(self):  

        vel_msg = Twist()
        current_goal = Pose()
        current_goal.x = self.x_coords[self.index]
        current_goal.y = self.y_coords[self.index]
        
        intermediate_pos = self.generate_trajectory(self.init_pose, current_goal,self.steps_duration,self.steps)
        #print(f"Current position: ({self.pose.x},{self.pose.y})")  
        #print(f"Intermediate position: ({intermediate_pos.x}, {intermediate_pos.y})")

        print(f"Robot rotation: {self.pose.theta}, desired rotation: {self.steering_angle(intermediate_pos)}")
        
        angle_diff = self.angular_vel(intermediate_pos, constant=1.0)
        
        if abs(angle_diff) < self.rotation_tolerance:
            vel_msg.linear.x = self.linear_vel(intermediate_pos) * self.gain
            vel_msg.angular.z = 0.0
        else: 
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = angle_diff
        
        if self.euclidean_distance(intermediate_pos) <= self.distance_tolerance:
            self.steps +=1
            vel_msg.angular.z = 0.0 ## Because after the goal pos is overshooted, steering angle changes alot
            print(f"Current error: {self.euclidean_distance(intermediate_pos)}")            
        
        if self.steps > self.steps_duration:
            print(f"Main goal reached! ({current_goal.x},{current_goal.y})")
            self.init_pose = self.pose
            self.index +=1
            self.steps = 0
            
        if self.index == 4:
            self.index = 0
            
        ## Publishing   
        path_msg = Point()
        path_msg.x = intermediate_pos.x
        path_msg.y = intermediate_pos.y
        
        goal_msg = Point()
        goal_msg.x = current_goal.x
        goal_msg.y = current_goal.y
        
        pose_msg = Point()
        pose_msg.x = self.pose.x
        pose_msg.y = self.pose.y
        
        self.velocity_publisher.publish(vel_msg)
        self.path_publisher.publish(path_msg)
        self.goal_publisher.publish(goal_msg)
        self.pose_publisher.publish(pose_msg)


if __name__ == '__main__':
    try:
        rclpy.init()  # Initialize rclpy

        node = Trajectory()
        #node.set_goal()
        rclpy.spin(node)
        
    except ROSInterruptException:
        pass
    finally:
        node.destroy_node()  # Destroy the node to clean up resources
        rclpy.shutdown()  # Shut down rclpy