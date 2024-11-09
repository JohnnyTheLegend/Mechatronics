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


class GoToXY(Node):
    def __init__(self):

        super().__init__('GoToXY_node')
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        #self.odom_subscriber = rclpy.Subscriber('/odom', Odometry, self.update_pose)
        self.create_subscription(Odometry, '/odom',self.update_pose,  10)

        self.gain = 0.2
        self.pose = Pose()
        self.goal_pose = Pose()
        self.pose_updated = False
        self.odom = Odometry()
        self.rate = 2  # 10 Hz
        
        self.create_timer(1.0 / self.rate, self.move2goal2)

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
        self.distance_tolerance = 0.1
        self.goal_set = True
        print(f"Steering angle to goal: {self.steering_angle(self.goal_pose)}")
        print(f"Difference in rotation: {self.angular_vel(self.goal_pose,constant=1)}")

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

    def move2goal(self):
        print("TImer method called!!")
        """Moves the turtle to the goal."""   
        if not self.goal_set:
            return  # Goal has not been set yet     
        
        vel_msg = Twist()
        
        if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:
            print("Moving towards goal!!")
            print(f"Current position: ({self.pose.x},{self.pose.y})")
            print(f"Difference in rotation: {self.angular_vel(self.goal_pose,constant=1)}")
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(self.goal_pose) * self.gain
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("Published!")
        else:
            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            self.goal_set = False
            print("Goal reached!!")
            self.set_goal()
            
    def move2goal2(self):
        if not self.goal_set:
            return  # Goal has not been set yet     
        
        vel_msg = Twist()
        
        if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:
            angle_diff = self.angular_vel(self.goal_pose, constant=1)
            if abs(angle_diff) < 0.1:
                print("Moving towards goal!!")
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(self.goal_pose) * self.gain
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

            else:
                # Angular velocity in the z-axis.
                print("Rotating towards goal")
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = angle_diff
              
            print(f"Current position: ({self.pose.x},{self.pose.y})")    
            print(f"Difference in rotation: Robot: {self.pose.theta}, goal: {self.steering_angle(self.goal_pose)}, diff: {angle_diff}")

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("Published!")
        else:
            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            self.goal_set = False
            print("Goal reached!!")
            self.set_goal()


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