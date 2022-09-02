"""
Holonomic Robot
Keyboard Controller
"""
# Importing Dependencies
import rclpy
from rclpy.node import Node
from pynput.keyboard import Key, Listener
from geometry_msgs.msg import Twist

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
        self.speed = 2.0
        self.pubwheel1 = self.create_publisher(Twist, '/cmd_vel', 1)
        print("Differential Drive Robot (Buggy)")
        print("Keyboard Controller\n ^ - Move Forward \n v - Move Backward \
               > - Move Right \n < - Move Left \n rctrl - stop")
        print("Speed Controller\n Increase Speed - rShift \n Decrease Speed - rAlt")
        with Listener(on_press = self.show) as listener:   
            listener.join()

    # Show Function
    def show(self, key):
        value = Twist()
        if key == Key.shift_r:
            self.speed += 1.0
        if key == Key.alt_r:
            self.speed -= 1.0
        if key == Key.up:
            value.linear.x = self.speed
            value.angular.z = 0.0
        elif key == Key.right:
            value.linear.x = 0.0
            value.angular.z = -self.speed
        elif key == Key.left:
            value.linear.x = 0.0
            value.angular.z = self.speed
        elif key == Key.down:
            value.linear.x = -self.speed
            value.angular.z = 0.0
        elif key == Key.ctrl_r:
            value.linear.x = 0.0
            value.angular.z = 0.0

        self.pubwheel1.publish(value)

    # Timer Callback
    def timer_callback(self):
        """
        Timer Callback for publishing values to the controller periodically.
        """
        wheel1_value = Float64MultiArray()
        wheel2_value = Float64MultiArray()
        wheel3_value = Float64MultiArray()
        wheel1_value.data = [5.0]
        wheel2_value.data = [-5.0]
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