#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from arm_ros2.msg import ArmInputs


# Based on arm_keyboard_controller.py / Adapted for GUI control

class GuiControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_gui_controller")

        self.speedMultiplier = 1

        self.inputPublisher = self.create_publisher(ArmInputs, "arm_inputs", 10)
        self.statePublisher = self.create_publisher(String, "arm_state", 10)
        
    def on_press(self, command):
        GuiToController = ArmInputs()

        GuiToController.l_horizontal = 0
        GuiToController.l_vertical   = 0
        GuiToController.r_horizontal = 0
        GuiToController.r_vertical   = 0
        GuiToController.l1           = 0
        GuiToController.r1           = 0
        GuiToController.l2           = 0
        GuiToController.r2           = 0
        GuiToController.x            = 0
        GuiToController.o            = 0
        GuiToController.share        = 0
        GuiToController.options      = 0
        GuiToController.r3           = 0

        #self.speedMultiplier = 6900
        try:
            # left vertical joystick emulation
            if command == "Forward" or command == "joint1plus":
                GuiToController.l_vertical = 1 * self.speedMultiplier
            elif command == "Backward" or command == "joint1minus":
                GuiToController.l_vertical = -1 * self.speedMultiplier

            # left horizontal joystick emulation
            if command == "Left" or command == "joint0plus":
                GuiToController.l_horizontal = 1 * self.speedMultiplier
            elif command == "Right" or command == "joint0minus":
                GuiToController.l_horizontal = -1 * self.speedMultiplier

            # left and right triggers
            if command == "Up" or command == "joint5plus":
                GuiToController.r2 = 1 * self.speedMultiplier
            if command == "Down" or command == "joint5minus":
                GuiToController.l2 = 1 * self.speedMultiplier

            # Rx
            if command == "Rx" or command == "joint4plus":
                for i in range(100):
                    GuiToController.l1 = 1 #* 127#int(self.speedMultiplier)#/100)
            elif command == "-Rx" or command == "joint4minus":
                GuiToController.r1 = 1 #* 127#int(self.speedMultiplier)#/100)
            
            # right vertical joystick emulation
            if command == "Ry" or command == "joint3plus":
                GuiToController.r_vertical = 1 * self.speedMultiplier
            elif command == "-Ry" or command == "joint3minus":
                GuiToController.r_vertical = -1 * self.speedMultiplier

            # right horizontal joystick emulation
            if command == "Rz" or command == "joint2plus":
                GuiToController.r_horizontal = 1 * self.speedMultiplier
            elif command == "-Rz" or command == "joint2minus":
                GuiToController.r_horizontal = -1 * self.speedMultiplier

            # shape button emulation
            if command == "Open Grip" or command == "joint6plus":
                GuiToController.x = 1 
                self.get_logger().info("grip opened!!!")
            if command == "Close Grip" or command == "joint6minus":
                GuiToController.o = 1 
                self.get_logger().info("grip not opened!!!")
            if command == "useless":
                GuiToController.triangle = 1
            if command == "useless":
                GuiToController.square = 1

            # other buttons
            if command == "useless":
                GuiToController.share = 1
            if command == "useless":
                GuiToController.options = 1
            if command == "useless":
                GuiToController.r3 = 1
                
            # emulate d-pad as arrow keys
            if command == "Manual":
                msg = String()
                msg.data = "Manual"
                self.statePublisher.publish(msg)
                self.speedMultiplier = 1#6900
            if command == "Inverse Kin":
                msg = String()
                msg.data = "IK"
                self.statePublisher.publish(msg)
                self.speedMultiplier = 1#69
            if command == "Setup":
                msg = String()
                msg.data = "Setup"
                self.statePublisher.publish(msg)
            if command == "Idle":
                msg = String()
                msg.data = "Idle"
                self.statePublisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in on_press: {e}")

        # Only joint 5 (the slow moving one) is published 100 times
        # (TEMPORARY FIX) 
        #if (command == "joint4plus" or command == "joint4minus"):    
            #for i in range(100):    
                #self.inputPublisher.publish(GuiToController)
        #else:
        self.inputPublisher.publish(GuiToController)

    def on_release(self):
        keyboardToController = ArmInputs()

        keyboardToController.l_horizontal = 0
        keyboardToController.l_vertical   = 0
        keyboardToController.r_horizontal = 0
        keyboardToController.r_vertical   = 0
        keyboardToController.l1           = 0
        keyboardToController.r1           = 0
        keyboardToController.l2           = 0
        keyboardToController.r2           = 0
        keyboardToController.x            = 0
        keyboardToController.o            = 0
        keyboardToController.share        = 0
        keyboardToController.options      = 0
        keyboardToController.r3           = 0

        self.inputPublisher.publish(keyboardToController)

def main(args=None):
    rclpy.init(args=args)
    try:
        gui_controller_node = GuiControllerNode()
        rclpy.spin(gui_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gui_controller_node' in locals():
            gui_controller_node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()