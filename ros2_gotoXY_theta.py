#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import transforms3d
import math
from std_msgs.msg import Float32MultiArray
import threading

class GoToXY(Node):
    def __init__(self):

        super().__init__('GoToXY_node')
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.err_publisher = self.create_publisher(Float32MultiArray, '/robot_err', 10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        #self.odom_subscriber = rclpy.Subscriber('/odom', Odometry, self.update_pose)
        self.create_subscription(Odometry, '/odom',self.update_pose,  10)

        self.gain = 0.2
        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_set = False
        self.init_goal_set = False
        self.pose_updated = False
        self.odom = Odometry()
        self.rate = 2  # 10 Hz
        self.rotation_tolerance = 0.01
        
        self.create_timer(1.0 / self.rate, self.move2goal2)
        
        ## Thread for publishing errors
        self.error_thread = threading.Thread(target=self.publish_errors_continuously)
        self.error_thread.daemon = True
        self.error_thread.start()

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
        #print(f"Roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        self.pose.theta = roll
        
    def set_goal(self):
        """Set the goal position from user input."""
        self.goal_pose.x = float(input("Set your x goal: "))
        self.goal_pose.y = float(input("Set your y goal: "))
        self.goal_pose.theta = math.radians(float(input("Enter theta (in degrees [-180;180]): ")))
        self.distance_tolerance = 0.1
        self.goal_set = True
        self.init_goal_set = True
        self.get_logger().info("New goal was entered!")

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
    
    def publish_errors_continuously(self):
        """Continuously publish positional and rotational errors."""
        while rclpy.ok():
            if self.init_goal_set:
                position_diff = self.euclidean_distance(self.goal_pose)
                rotation_diff = self.goal_pose.theta - self.pose.theta

                msg_err = Float32MultiArray()
                msg_err.data = [position_diff, rotation_diff]

                self.err_publisher.publish(msg_err)
            
            time.sleep(1.0 / self.rate)  # Set the error publishing rate
            
    def move2goal2(self):
        if not self.goal_set:
            return  # Goal has not been set yet     
        
        vel_msg = Twist()
        rotation_diff = self.goal_pose.theta - self.pose.theta
        position_diff = self.euclidean_distance(self.goal_pose)
        
        if  position_diff >= self.distance_tolerance:
            angle_diff = self.angular_vel(self.goal_pose, constant=1)
            vel_msg.linear.x = self.linear_vel(self.goal_pose) * self.gain
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = angle_diff
              
            print(f"Current position: ({self.pose.x},{self.pose.y})")    

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
        else:
            if abs(rotation_diff) < self.rotation_tolerance:
            # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.goal_set = False
                self.velocity_publisher.publish(vel_msg)
                self.get_logger().info("Goal reached!!")
                print(f"Difference in rotation: {rotation_diff}")
                self.set_goal()
            else:
                print(f"Rotating to desired angle..")
                print(f"Desired rotation: {self.goal_pose.theta}, current rotation: {self.pose.theta}")
                vel_msg.angular.z = rotation_diff
                self.velocity_publisher.publish(vel_msg)
            


if __name__ == '__main__':
    try:
        rclpy.init()  # Initialize rclpy

        node = GoToXY()
        node.set_goal()
        rclpy.spin(node)
        
    except ROSInterruptException:
        pass
    finally:
        node.destroy_node()  # Destroy the node to clean up resources
        rclpy.shutdown()  # Shut down rclpy