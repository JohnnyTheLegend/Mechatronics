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


class GoToXY(Node):
    def __init__(self):

        super().__init__('GoToXY_node')
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        #self.odom_subscriber = rclpy.Subscriber('/odom', Odometry, self.update_pose)
        self.create_subscription(Odometry, '/odom',self.update_pose,  10)

        self.pose = Pose()
        self.pose_updated = False
        self.odom = Odometry()
        self.rate = 10  # 10 Hz
        self.sleep_duration = 1.0 / self.rate 

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        self.pose_updated = True
        print(f"Current position: ({self.pose.x},{self.pose.y})")
        # self.pose.x = round(self.pose.x, 4)
        # self.pose.y = round(self.pose.y, 4)

        # position = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation
        # linear_velocity = msg.twist.twist.linear
        # angular_velocity = msg.twist.twist.angular

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

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""        
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            print("Moving towards goal!!")
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            time.sleep(self.sleep_duration)

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        rclpy.init()  # Initialize rclpy

        node = GoToXY()
        rclpy.spin(node)
        
    except ROSInterruptException:
        pass
    finally:
        node.destroy_node()  # Destroy the node to clean up resources
        rclpy.shutdown()  # Shut down rclpy