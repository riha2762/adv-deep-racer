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

    throttle = 0.0
    angle = 0.0

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
        throttle_increment = 0.05  # Increment step for throttle
        angle_increment = 0.1     # Increment step for angle
        max_value = 1.0            # Maximum value for throttle and angle
        min_value = -1.0           # Minimum value for throttle and angle

        try:
            while True:
                key = self.get_key()
                
                # Control throttle and steering based on key input
                if key == '\x1b[A':  # Up arrow
                    self.throttle = min(self.throttle + throttle_increment, max_value)
                elif key == '\x1b[B':  # Down arrow
                    self.throttle = max(self.throttle - throttle_increment, min_value)
                elif key == '\x1b[C':  # Right arrow
                    self.angle = max(self.angle - angle_increment, min_value)
                elif key == '\x1b[D':  # Left arrow
                    self.angle = min(self.angle + angle_increment, max_value)
                elif key == '\x03':  # Ctrl+C to exit
                    break
                #if space, stop robot
                elif key == ' ':
                    self.throttle = 0.0
                    self.angle = 0.0
                else:
                    # Gradually reduce throttle and angle to zero if no key is pressed
                    if self.throttle > 0:
                        self.throttle = max(self.throttle - throttle_increment, 0)
                    elif self.throttle < 0:
                        self.throttle = min(self.throttle + throttle_increment, 0)

                    if self.angle > 0:
                        self.angle = max(self.angle - angle_increment, 0)
                    elif self.angle < 0:
                        self.angle = min(self.angle + angle_increment, 0)

                # Populate the control message
                wheel_msg.throttle = self.throttle
                wheel_msg.angle = self.angle

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

    try:
        # Call the run method to start the teleoperation loop
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()


