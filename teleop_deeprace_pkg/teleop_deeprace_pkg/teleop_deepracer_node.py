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


import pygame
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import rclpy
from rclpy.node import Node

class TeleopDeepracer(Node):
    throttle = 0.0
    angle = 0.0

    def __init__(self):
        super().__init__('teleop_deepracer_node')
        
        # Publisher for the servo control messages
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self.get_logger().info("Teleop node has started. Use the PS4 controller to control the Deepracer.")

        # Initialize pygame and the joystick
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def run(self):
        wheel_msg = ServoCtrlMsg()
        throttle_increment = 0.05
        angle_increment = 0.05

        max_value = 1.0
        min_value = -1.0

        try:
            while True:
                # Process pygame events
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        # Axis 1 is typically the left stick's vertical axis (throttle)
                        # Axis 3 is typically the right stick's horizontal axis (steering)
                        self.throttle = -self.controller.get_axis(1)  # Inverted for forward/reverse control
                        self.angle = self.controller.get_axis(3)

                        # Clamp values to the range [-1, 1]
                        self.throttle = max(min(self.throttle, max_value), min_value)
                        self.angle = max(min(self.angle, max_value), min_value)

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
