import rclpy
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from array import array
import sys, termios, tty # takes in keyboard input

class FollowWallNode(Node):
    throttle = 0.0
    angle = 0.0

    def __init__(self):
        super().__init__('follow_wall__node')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.get_logger().info("Follow wall  node has started.")
        self.reverse_run = True
       #self.settings = termios.tcgetattr(sys.stdin)

        # Subscriber for LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'rplidar_ros/scan',
            self.lidar_callback,
            10
        )
        
        # Initializing variables
        self.moving = True

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)  # Read 1 character first
            if key == '\x1b':  # If the first character is an escape character
                key += sys.stdin.read(2)  # Read the next 2 characters (for arrow keys)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def lidar_callback(self, msg: LaserScan):
        # Extract the LIDAR scan ranges

        distances = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
       
        for i in range(len(distances)):
            if math.isinf(distances[i]):
                distances[i] = 0.0
        dist_means = []
        len_dist = len(distances)
        for i in range(8):
            dist_mean_ = distances[int(len_dist*i/8):int(len_dist*(i+1)/8)]
            dist_means.append(math.fsum(dist_mean_)/len(dist_mean_))
        for i in range(8):
            print(i," ",dist_means[i])

        # Check for obstacles within the goal direction
        #Follow wall on the right
        dist_right = (dist_means[5]+dist_means[6])/2
       
        self.run(dist_right)

    def run(self,dist_right):
        MIN_DISTANCE_RIGHT = 0.3
        MAX_DISTANCE_RIGHT = 0.7
        MEAN_DISTANCE_RIGHT = 0.5
        wheel_msg = ServoCtrlMsg()
        throttle_increment = 0.5  # Increment step for throttle
        wheel_angle_increment = 1.0     # Increment step for angle
        max_angle_value = 1.0            # Maximum value for throttle and angle
        min_angle_value = -1.0           # Minimum value for throttle and angle

       # throttle_input = 0.61
        throttle_input = 0.0
        if self.reverse_run:
            throttle_input *= -1
            max_angle_value *= -1
            min_angle_value *= -1
        #key = self.get_key()

        if dist_right < MIN_DISTANCE_RIGHT:
            #turn left
            self.get_logger().info("Turning left")
            self.throttle = throttle_input
            self.angle = max_angle_value
        elif dist_right > MAX_DISTANCE_RIGHT:
            #turn right
            self.get_logger().info('Turning right')
            self.throttle = throttle_input
            self.angle = min_angle_value
        else:
            #straight
            self.get_logger().info('Moving straight')
            self.throttle = throttle_input
            self.angle = 0.0
        # Populate the control message
        wheel_msg.throttle = self.throttle
        wheel_msg.angle = self.angle
        # Publish the control message
        self.get_logger().info("Publishing message: throttle={}, angle={}".format(wheel_msg.throttle, wheel_msg.angle))
        self.wheel_publisher.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    follow_wall_node = FollowWallNode()
    
    rclpy.spin(follow_wall_node)
    
    follow_wall_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
