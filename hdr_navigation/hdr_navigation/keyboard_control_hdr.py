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
        self.pubwheel1 = self.create_publisher(Float64MultiArray, 
                                               '/wheel1_drive_controller/commands',
                                               1)
        self.pubwheel2 = self.create_publisher(Float64MultiArray, 
                                               '/wheel2_drive_controller/commands',
                                               1)
        self.pubwheel3 = self.create_publisher(Float64MultiArray, 
                                               '/wheel3_drive_controller/commands',
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
        wheel1_value = Float64MultiArray()
        wheel2_value = Float64MultiArray()
        wheel3_value = Float64MultiArray()
        if key == Key.shift_r:
            self.speed += 5
        if key == Key.alt_r:
            self.speed -= 5
        if key == Key.up:
            wheel1_value.data = [-self.speed]
            wheel2_value.data = [self.speed]
            wheel3_value.data = [0.0]
        elif key == Key.right:
            wheel1_value.data = [0.0]
            wheel2_value.data = [self.speed]
            wheel3_value.data = [-self.speed]
        elif key == Key.left:
            wheel1_value.data = [0.0]
            wheel2_value.data = [-self.speed]
            wheel3_value.data = [self.speed]
        elif key == Key.down:
            wheel1_value.data = [self.speed]
            wheel2_value.data = [-self.speed]
            wheel3_value.data = [0.0]
        elif key == Key.ctrl_r:
            wheel1_value.data = [self.speed]
            wheel2_value.data = [self.speed]
            wheel3_value.data = [self.speed]
        elif key == Key.delete:
            wheel1_value.data = [0.0]
            wheel2_value.data = [0.0]
            wheel3_value.data = [0.0]
        self.pubwheel1.publish(wheel1_value)
        self.pubwheel2.publish(wheel2_value)
        self.pubwheel3.publish(wheel3_value)

def main():
    rclpy.init()
    keyboard_control = KeyboardController()
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        keyboard_control.destroy_node()

if __name__ == '__main__':
    main()