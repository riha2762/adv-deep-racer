""" 

If build issue arise, enter the below into package.xml file 
  <depend>rclpy</depend>
  <depend>sys</depend>
  <depend>termios</depend>
  <depend>tty</depend>
  
"""




from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import rclpy
from rclpy.node import Node
import sys
import termios # takes in keyboard input
import tty

class TeleopTurtle(Node):

    def __init__(self):
        super().__init__('teleop_turtle_node')
        
        # Publisher for the servo control messages
        self.wheel_publisher = self.create_publisher(ServoCtrlMsg, "/ctrl_pkg/servo_msg", 10)
        self._logger('teleop_turtle_node has started')
        
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self): # Don't get it, it was GPT written - may have to edit if it doesnt work right
        tty.setraw(sys.stdin.fileno()) 
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        wheel_msg = ServoCtrlMsg()
        try:
            while True:
                key = self.get_key()
                
                # Control throttle and steering based on key input
                if key == 'w':  # Forward
                    wheel_msg.throttle = 0.30
                    wheel_msg.angle = 0.0
                elif key == 's':  # Reverse
                    wheel_msg.throttle = -0.30
                    wheel_msg.angle = 0.0
                elif key == 'a':  # Left
                    wheel_msg.throttle = 0.0
                    wheel_msg.angle = 0.5
                elif key == 'd':  # Right
                    wheel_msg.throttle = 0.0
                    wheel_msg.angle = -0.5
                elif key == '\x03':  # Ctrl+C to exit
                    break
                else:
                    # Stop if no valid key is pressed
                    wheel_msg.throttle = 0.0
                    wheel_msg.angle = 0.0

                # Publish the control message
                self.wheel_publisher.publish(wheel_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            wheel_msg.throttle = 0.0
            wheel_msg.angle = 0.0
            self.wheel_publisher.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_turtle = TeleopTurtle()

    try:
        rclpy.spin(teleop_turtle)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_turtle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


