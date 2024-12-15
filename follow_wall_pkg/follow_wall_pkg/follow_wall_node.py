import rclpy
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from simple_pid import PID
import sys, termios, tty # takes in keyboard input
import threading
class FollowWallNode(Node):

    def __init__(self):
        super().__init__('follow_wall__node')
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.get_logger().info("Follow wall node with PID control has started.")
        
       #self.settings = termios.tcgetattr(sys.stdin)

        # Subscriber for LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'rplidar_ros/scan',
            self.lidar_callback,
            10
        )
        # always keep the mobile 1.0 meters away from right wall
        self.desired_distance_right = 1.0 

        #self.angle_pid = PID(-0.7,0.0,0.0,setpoint = 0.0)
        #self.angle_pid.output_limits = (-1.0,1.0)

        self.reverse_run = False


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
        #dist_right = dist_means[6]
        dist_left = dist_means[1]
         
        error_left = self.desired_distance_right - dist_left
        self.run(error_left)

    def run(self,error_left):
        base_throttle = 0
        Kp = -0.5
        angle_corrected = Kp*error_left
        if angle_corrected > 1:
            angle_corrected = 1
        if angle_corrected < -1:
            angle_corrected = -1

        if self.reverse_run:
            throttle_input *= -1
            angle_corrected *= -1
        # Populate the control message
        wheel_msg = ServoCtrlMsg()
        wheel_msg.throttle = base_throttle
        wheel_msg.angle = angle_corrected
        # Publish the control message
        self.get_logger().info(f"Error: {error_right:.2f}, Throttle: {wheel_msg.throttle:.2f}, Angle: {wheel_msg.angle:.2f}")
        self.wheel_publisher.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    
    follow_wall_node = FollowWallNode()
    
    rclpy.spin(follow_wall_node)
    
    follow_wall_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
