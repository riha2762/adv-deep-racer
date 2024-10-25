""" 

Steps:
1. Add file name to setup.py:
    entry_points={
        'console_scripts': [
            'teleop_deepracer_node = teleop_deepracer.teleop_deepracer:main', #This is an example name (it should be filename = package.package:main)
        ],
    },

2. Make python file an executable - chmod +x ~/file_path/teleop_deepracer_node.py
3. source foxy, source setup.py
4. colcon build --select-package <package>
5. ros2 run <package> <executable>

*****************************************************************************************************************
Error handling : 
1. If build issue arise, enter the below into package.xml file 
  <depend>rclpy</depend>
2. Check if ros node is running called "teleop_deepracer_node"
    - ros2 node list
3. Check if topics are running 
    - ros2 topic list/echo

"""


from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import rclpy
from rclpy.node import Node
import sys, termios, tty # takes in keyboard input


class TeleopDeepracer(Node):

    def __init__(self):
        super().__init__('teleop_deepracer_node')
        
        # Publisher for the servo control messages
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.get_logger().info("Teleop node has started. Use arrow keys to control the Deepracer.")

        
        self.settings = termios.tcgetattr(sys.stdin)
        
    # This did not work for turtlesim, so I have commented it out    
    # def get_key(self): # Don't get it, it was GPT written - may have to edit if it doesnt work right
    #     tty.setraw(sys.stdin.fileno()) 
    #     key = sys.stdin.read(1)
    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    #     return key
    
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

    def run(self):
        wheel_msg = ServoCtrlMsg()
        try:
            while True:
                key = self.get_key()
                
                # Control throttle and steering based on key input
                if key == '\x1b[A':  # Up arrow
                    wheel_msg.throttle = 0.30
                    wheel_msg.angle = 0.0
                elif key == '\x1b[B':  # Down arrow
                    wheel_msg.throttle = -0.30
                    wheel_msg.angle = 0.0
                elif key == '\x1b[C':  # Right arrow
                    wheel_msg.throttle = 0.0
                    wheel_msg.angle = 0.5
                elif key == '\x1b[D':  # Left arrow
                    wheel_msg.throttle = 0.0
                    wheel_msg.angle = -0.5
                elif key == '\x03':  # Ctrl+C to exit
                    break
                else:
                    # Stop if no valid key is pressed
                    wheel_msg.throttle = 0.0
                    wheel_msg.angle = 0.0

                # Publish the control message
                self.get_logger().info("Publishing message: throttle={}, angle={}".format(wheel_msg.throttle, wheel_msg.angle))
                self.wheel_publisher.publish(wheel_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            wheel_msg.throttle = 0.0
            wheel_msg.angle = 0.0
            self.wheel_publisher.publish(wheel_msg)
            self.get_logger().info(f"Published Message: throttle={wheel_msg.throttle}, steering={wheel_msg.angle}")



def main(args=None):
    rclpy.init(args=args)
    node = TeleopDeepracer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


