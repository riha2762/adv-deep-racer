import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class BugAlgorithmNode(Node):
    def __init__(self):
        super().__init__('bug_algorithm_node')
        
        # Publisher for controlling the velocity of the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'rplidar_ros/scan',
            self.lidar_callback,
            10
        )
        
        # Initializing variables
        self.moving = True
        # self.linear_speed = 1.0
        # self.angular_speed = 0.5
        self.min_distance_to_obstacle = 1.0  # Meters

    def lidar_callback(self, msg: LaserScan):
        # Extract the LIDAR scan ranges
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Determine the indices corresponding to -45 degrees and +45 degrees
        start_angle = -math.radians(45)
        end_angle = math.radians(45)
        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)
        
        # Get the distances in the range of -45 to +45 degrees
        front_ranges = ranges[start_index:end_index]
        
        # Check for obstacles within the goal direction
        obstacle_in_front = any(distance < self.min_distance_to_obstacle for distance in front_ranges if not math.isinf(distance))
        
        if obstacle_in_front:
            self.follow_obstacle()
        else:
            self.move_straight()

    def move_straight(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Moving straight towards goal...')

    def follow_obstacle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Obstacle detected! Turning to avoid...')

def main(args=None):
    rclpy.init(args=args)
    
    bug_algorithm_node = BugAlgorithmNode()
    
    rclpy.spin(bug_algorithm_node)
    
    bug_algorithm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
