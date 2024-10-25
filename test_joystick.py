import pygame

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Check if any joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a controller.")
    exit()

# Initialize the first joystick
controller = pygame.joystick.Joystick(0)
controller.init()

print(f"Controller detected: {controller.get_name()}")

# Main loop to read the joystick values
try:
    while True:
        # Process events
        pygame.event.pump()

        # Read axis values
        left_stick_y = controller.get_axis(1)  # Left stick vertical axis
        right_stick_x = controller.get_axis(3)  # Right stick horizontal axis

        # Read button values (example for button 0)
        button_x = controller.get_button(0)  # X button on PS4 controller

        # Print the values
        print(f"Left Stick Y: {left_stick_y:.2f}, Right Stick X: {right_stick_x:.2f}, X Button: {button_x}")

        # Delay to avoid flooding the output
        pygame.time.wait(100)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    pygame.quit()
