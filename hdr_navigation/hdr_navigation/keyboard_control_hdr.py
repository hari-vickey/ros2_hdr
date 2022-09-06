"""
Holonomic Robot
Keyboard Controller
"""
# Importing Dependencies
import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float64MultiArray

# Class KeyBoardController
class KeyboardController(Node):
    """
    class KeyboardController
    """
    # Constructor
    def __init__(self):
        """
        constructor
        """
        # Initializing Node
        super().__init__('hdr_keyboard_controller')
        self.pub_velocity = self.create_publisher(Float64MultiArray,
                                               '/omni_wheel_controller/commands',
                                               1)
        self.speed = 10.0
        print("Holonomic Drive Robot using Omni Wheel")
        print("Keyboard Controller\n ^ - Move Forward \n v - Move Backward \
               > - Move Right \n < - Move Left \n rctrl - rotate \n del - stop")
        print("Speed Controller\n Increase Speed - rShift \n Decrease Speed - rAlt")
        with Listener(on_press = self.show) as listener:   
            listener.join()

    # Show Function
    def show(self, key):
        value = Float64MultiArray()
        if key == Key.shift_r:
            self.speed += 5
        if key == Key.alt_r:
            self.speed -= 5
        if key == Key.up:
            value.data = [-self.speed, self.speed, 0.0]
        elif key == Key.right:
            value.data = [0.0, self.speed, -self.speed]
        elif key == Key.left:
            value.data = [0.0, -self.speed, self.speed]
        elif key == Key.down:
            value.data = [self.speed, -self.speed, 0.0]
        elif key == Key.ctrl_r:
            value.data = [self.speed, self.speed, self.speed]
        elif key == Key.delete:
            value.data = [0.0, 0.0, 0.0]
        self.pub_velocity.publish(value)

def main():
    rclpy.init()
    keyboard_control = KeyboardController()
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        keyboard_control.destroy_node()

if __name__ == '__main__':
    main()