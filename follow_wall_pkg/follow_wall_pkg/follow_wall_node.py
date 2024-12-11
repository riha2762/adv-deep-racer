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
        # always keep the mobile 2 meters away from right wall
        self.desired_distance_right = 2

        self.angle_pid = PID(0.7,0.0,0.0,setpoint = 0.0)
        self.angle_pid.output_limits = (-1.0,1.0)

        self.stop = False
        self.moving = True
        self.reverse_run = False

       # threading.Thread(target = self.listen_for_keyboard_input,daemon = True).start()
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
    def listen_for_keyboard_input(self):
        while True:
            key = self.get_key()
            if key == 's':
                self.stop = True
                self.get_logger().info("robot stop")
            elif key == 'r':
                self.stop = False
                self.get_logger().info("resumed")
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
        
        error_right = dist_right - self.desired_distance_right 
        self.run(error_right)

    def run(self,error_right):
        self.stop = False
        if self.stop:
            self.get_logger.info("stop")
            wheel_msg = ServoCtrlMsg()
            wheel_msg.throttle = 0.0
            wheel_msg.angle = 0.0
            self.wheel_publisher.publish(wheel_msg)
            return
        else:

            base_throttle = 0.6
            angle_corrected = self.angle_pid(error_right)
            # throttle_corrected = base_throttle + self.throttle_pid(abs(error_right))

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
