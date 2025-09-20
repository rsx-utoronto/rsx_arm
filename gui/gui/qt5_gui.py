#! /usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit, QComboBox, QSlider, QGridLayout, QGroupBox, QSpinBox
from PyQt5.QtGui import QPixmap, QIcon, QImage
from PyQt5.QtCore import Qt, QTimer
import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import pi
import yaml

# Imports video display functionality
from .gui_camera import ROSVideoSubscriber
# Class for sending commands to manipulator
from .arm_gui_controller import GuiControllerNode

# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'IK')))

from ik import ik_library


class RobotControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Robot Control Interface")

        screen = QApplication.primaryScreen()  # Get primary screen (default monitor)

        screen_size = screen.size()

        # Set the window size and maximum size to 90% of the screen width and 2/3 of the screen height
        self.setGeometry(100, 100, int(screen_size.width() *
                         9/10), int(screen_size.height()*2/3))
        self.setMaximumSize(int(screen_size.width()*9/10),
                            int(screen_size.height()*2/3))

        # The icon for the GUI!
        self.setWindowIcon(QIcon(os.environ['GUI_DIR'] + 'RsxLogo.png'))

        # Load the YAML file

        # Ideally find a way to change this so that the gui doesn't have to be run from the exact same location every time
        with open(os.environ['GUI_DIR'] + 'config.yaml', 'r') as file:
            config = yaml.safe_load(file)

        # Load the camera topics from the YAML file
        self.camera_topics = config['camera_topics']
        self.camera_topic_name = [camera['topic']
                                  for camera in self.camera_topics]
        self.camera_compressed = [camera['compressed']
                                  for camera in self.camera_topics]
        self.camera_aspect_ratio = eval(config['camera_aspect_ratio'])

        # Initialize all of the buttons, labels, and other widgets
        self.initUI()

        # If science mode is on, bottom 3 joints and gripper
        self.scienceOn = False

        self.controller = GuiControllerNode()  # Initialize controller

        # Get the arm effector positions
        self.end_effector_coords = self.node.create_subscription(
            Float32MultiArray,
            "arm_end_effector_pos",
            self.update_end_effector_coords,
            10
        )

        # Update the joint display values
        self.joint_display_values = self.node.create_subscription(
            Float32MultiArray,
            "arm_curr_pos",
            self.set_joint_display,
            10
        )

        # The default mode when you start the GUI is Idle
        self.mode_buttons["Idle"].click()

    def resizeEvent(self, event):
        camera_width = self.power_on.width() + self.power_off.width() + \
            self.move_origin.width()
        self.coord_view_label.setFixedWidth(int(camera_width))
        self.coord_view_label.setFixedHeight(
            int(camera_width*self.camera_aspect_ratio))
        super().resizeEvent(event)  # Call parent class method

    # Send commands to the controller

    def button_is_clicked(self, command):
        self.controller.on_press(command)  # Send command to controller
        self.controller.on_release()       # Reset
        print(command)

        # Grey Out Buttons based on Mode
        if command == "Manual":  # Grey Out the IK buttons
            for _, button in self.arrow_buttons.items():
                button.setEnabled(False)
            if self.scienceOn:
                for i in range(6):
                    self.joint_buttons[i].setEnabled(True)
            else:
                for button in self.joint_buttons:
                    button.setEnabled(True)

        if command == "Inverse Kin":  # Grey Out the joint control buttons
            for _, button in self.arrow_buttons.items():
                button.setEnabled(True)
            for button in self.joint_buttons:
                button.setEnabled(False)

        if command == "Idle":
            for _, button in self.arrow_buttons.items():
                button.setEnabled(False)
            for button in self.joint_buttons:
                button.setEnabled(False)

        # Currently toggles between manual and science mode whenever you press any of the module buttons
        # Intended use is to eventually move ot a certain position with each module.
        # Since no additional information was given, this is the current implementation.
        if command in ["Module 1", "Module 2", "Module 3", "Module 4"]:
            # gray out only 4 - 6 in forward kinematics
            self.scienceOn = not self.scienceOn
            if not self.scienceOn:
                # need to set the joint buttons to be enabled
                if not self.arrow_buttons["Up"].isEnabled() or (self.arrow_buttons["Up"].isEnabled() and self.joint_buttons[0].isEnabled()):
                    # if IK mode is not ON or both IK mode and manual mode is ON, then enable joint buttons again
                    # we do not want to enable buttons back if IK mode is ON
                    for button in self.joint_buttons:
                        button.setEnabled(True)

            else:
                if self.joint_buttons[0].isEnabled():
                    # if the first three joints are enabled, then manual mode is ON
                    if self.joint_buttons[6].isEnabled():
                        # toggle ON and OFF
                        for k in range(6, 14):
                            self.joint_buttons[k].setEnabled(False)
                    else:
                        for k in range(6, 14):
                            self.joint_buttons[k].setEnabled(True)
                else:
                    # not in manual mode, keep joints buttons OFF
                    for k in range(6, 14):
                        self.joint_buttons[k].setEnabled(False)

    # Forward Kinematics (Individual Joint Angles) (MUST BE IN MANUAL MODE)

    def update_joint_value(self, joint_index, increment):
        """
        Args:
            joint_index (int): Index of the joint to update.
            increment (int): Whether increment or decrement the value.

        Joint 0: l/r L joystick
        Joint 1: up/down L joystick
        Joint 2: up/down R joystick
        Joint 3: l/r R joystick
        Joint 4: L1 - R1
        Joint 5: L2 - R2
        """
        # Set in Manual Mode
        # self.button_is_clicked("Manual")

        # Map +/- buttons to playstation controller buttons (+/- may be flipped, NEED TO CHECK)
        command_translator = {
            (0, 1): "joint0plus",
            (0, -1): "joint0minus",
            (1, 1): "joint1plus",
            (1, -1): "joint1minus",
            (2, 1): "joint2plus",
            (2, -1): "joint2minus",
            (3, 1): "joint3plus",
            (3, -1): "joint3minus",
            (4, 1): "joint4plus",
            (4, -1): "joint4minus",
            (5, 1): "joint5plus",
            (5, -1): "joint5minus",
            (6, 1): "joint6plus",
            (6, -1): "joint6minus"
        }
        self.button_is_clicked(command_translator[(joint_index, increment)])

    def set_joint_display(self, data):
        """
        Args:
            joint_index (int): Index of the joint to update.
            value (int): New value to set.
        """
        # Updates each of the joint values in the GUI
        for joint in range(6):
            self.joint_displays[joint].setText(str(round(data.data[joint], 1)))

    # Receive camera feed
    def update_image(self, qt_image):
        # Clear old pixmap
        self.coord_view_label.clear()
        # Update the QLabel with the latest frame
        pixmap = QPixmap.fromImage(qt_image)
        self.coord_view_label.setPixmap(pixmap)
        # scales the image to fit the label size
        self.coord_view_label.setScaledContents(True)

    # Switch camera feeds
    def on_view_selected(self, index):
        # Destroy the old subscription
        if hasattr(self.video_subscriber, 'sub'):
            self.node.destroy_subscription(self.video_subscriber.sub)
        self.video_subscriber = ROSVideoSubscriber(
            self.node, self.camera_topic_name[index], self.camera_compressed[index])
        self.video_subscriber.frame_received.connect(self.update_image)

    # Update the end effector coordinate section
    def update_end_effector_coords(self, data):
        # Get the data from the right rostopic
        x = data.data[3]
        y = data.data[4]
        z = data.data[5]
        rx = data.data[0]
        ry = data.data[1]
        rz = data.data[2]

        # Actually change the text within the GUI
        self.x_coord.setText(f"X: {x:.2f} mm")
        self.y_coord.setText(f"Y: {y:.2f} mm")
        self.z_coord.setText(f"Z: {z:.2f} mm")
        self.rx_coord.setText(f"Rx: {rx:.2f}°")
        self.ry_coord.setText(f"Ry: {ry:.2f}°")
        self.rz_coord.setText(f"Rz: {rz:.2f}°")

    # Update the publishing rate multiplier
    def update_publish_multiplier(self, increment):
        # Set the frequency multiplier and make sure it doesn't go to 0
        self.frequencyMultiplier += increment
        self.frequencyMultiplier = max(self.frequencyMultiplier, 0.1)
        # TODO: add maximum value
        # Change the text box within the GUI
        self.freq_display.setText(str(round(self.frequencyMultiplier, 1)))

        # Actually change the frequencies for the required buttons (joints, then IK)
        for button in self.joint_buttons:
            button.setAutoRepeatInterval(int(100/self.frequencyMultiplier))

        for direction, button in self.arrow_buttons.items():
            button.setAutoRepeatInterval(int(100/self.frequencyMultiplier))

    # Main GUI code

    def initUI(self):
        # Main Layout
        main_layout = QHBoxLayout()

        # Coordinates Control Section:w
        coordinates_group = QGroupBox(
            "Inverse Kinematics: End Effector Coordinates and Rotation")
        coord_layout = QGridLayout()

        # Arrow Buttons for movement
        directions = ["Up", "Down", "Left", "Right", "Forward", "Backward",
                      "Rx", "Ry", "Rz", "-Rx", "-Ry", "-Rz", "Open Grip", "Close Grip"]
        self.arrow_buttons = {d: QPushButton(d) for d in directions}

        for direction, button in self.arrow_buttons.items():
            button.clicked.connect(
                lambda _, d=direction: self.button_is_clicked(d))
            # Allow each button to be held down and continuously input
            button.setAutoRepeat(True)

        coord_layout.addWidget(self.arrow_buttons["Up"], 1, 1)
        coord_layout.addWidget(self.arrow_buttons["Down"], 3, 1)
        coord_layout.addWidget(self.arrow_buttons["Left"], 2, 0)
        coord_layout.addWidget(self.arrow_buttons["Right"], 2, 2)
        coord_layout.addWidget(self.arrow_buttons["Forward"], 0, 0)
        coord_layout.addWidget(self.arrow_buttons["Backward"], 0, 2)

        # Rotation Controls
        coord_layout.addWidget(self.arrow_buttons["Rx"], 4, 0)
        coord_layout.addWidget(self.arrow_buttons["-Rx"], 4, 1)
        coord_layout.addWidget(self.arrow_buttons["Ry"], 5, 0)
        coord_layout.addWidget(self.arrow_buttons["-Ry"], 5, 1)
        coord_layout.addWidget(self.arrow_buttons["Rz"], 6, 0)
        coord_layout.addWidget(self.arrow_buttons["-Rz"], 6, 1)

        # Grip Control
        coord_layout.addWidget(self.arrow_buttons["Open Grip"], 5, 2)
        coord_layout.addWidget(self.arrow_buttons["Close Grip"], 6, 2)

        coordinates_group.setLayout(coord_layout)

        # 3D View Section
        view_group = QGroupBox("Camera View")
        view_layout = QVBoxLayout()

        self.camera_view_box = QComboBox()
        camera_names = [camera['name'] for camera in self.camera_topics]
        self.camera_view_box.addItems(camera_names)
        view_layout.addWidget(self.camera_view_box)

        view_layout.addStretch(1)

        self.coord_view_label = QLabel("Cameras Goes Here")
        # self.coord_view_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        view_layout.addWidget(self.coord_view_label)

        # Initialize the ROS video subscriber
        self.video_subscriber = ROSVideoSubscriber(
            self.node, self.camera_topic_name[0], self.camera_compressed[0])
        self.video_subscriber.frame_received.connect(self.update_image)

        self.camera_view_box.currentIndexChanged.connect(self.on_view_selected)
        # -------------------------------- _

        # Arm Power Options
        power_group = QGroupBox("Arm Power")
        power_layout = QHBoxLayout()

        power_layout.setContentsMargins(0, 0, 0, 0)

        self.power_on = QPushButton("ON")
        self.power_on.clicked.connect(lambda _: self.button_is_clicked("ON"))
        self.power_off = QPushButton("OFF")
        self.power_off.clicked.connect(lambda _: self.button_is_clicked("OFF"))
        self.move_origin = QPushButton("Move to Origin")
        self.move_origin.clicked.connect(
            lambda _: self.button_is_clicked("Move to Origin"))

        power_group.setStyleSheet('QPushButton {font-size: 20px}')
        self.power_on.setStyleSheet(
            'QPushButton {background-color: #00FF00; color: #000000}')
        self.power_off.setStyleSheet('QPushButton {background-color: #FF0000}')

        power_layout.addWidget(self.power_on)
        power_layout.addWidget(self.power_off)
        power_layout.addWidget(self.move_origin)

        power_group.setLayout(power_layout)
        view_layout.addWidget(power_group)
        view_group.setLayout(view_layout)

        # Modules for Science Team
        module_types = ["Module 1", "Module 2", "Module 3", "Module 4"]
        module_buttons = {m: QPushButton(m) for m in module_types}

        science_modules = QGroupBox("Science Modules")
        science_layout = QVBoxLayout()

        science_layout.addWidget(module_buttons["Module 1"])
        science_layout.addWidget(module_buttons["Module 2"])
        science_layout.addWidget(module_buttons["Module 3"])
        science_layout.addWidget(module_buttons["Module 4"])

        for modules, button in module_buttons.items():
            button.clicked.connect(
                lambda _, m=modules: self.button_is_clicked(m))

        science_modules.setLayout(science_layout)

        self.joint_displays = []
        self.joint_buttons = []  # Store references to the buttons

        # Joints Control Section
        joints_group = QGroupBox("Joints Control")
        joints_layout = QVBoxLayout()

        self.joint_controls = []
        for i in range(7):
            joint_control = QHBoxLayout()
            if i < 6:
                joint_label = QLabel(f"Joint {i + 1}")
            else:
                joint_label = QLabel("Gripper")

            # Label to display the joint angle
            joint_display = QLabel("0")
            joint_display.setStyleSheet(
                "border: 1px solid black; padding: 5px;")
            joint_display.setFixedWidth(50)
            joint_display.setAlignment(Qt.AlignmentFlag.AlignCenter)

            # Increment and decrement buttons
            inc_button = QPushButton("+")
            dec_button = QPushButton("-")

            # Store buttons in the list for bulk control
            self.joint_buttons.append(inc_button)
            self.joint_buttons.append(dec_button)

            # Connect buttons to functions
            inc_button.clicked.connect(
                lambda _, idx=i: self.update_joint_value(idx, 1))
            dec_button.clicked.connect(
                lambda _, idx=i: self.update_joint_value(idx, -1))

            # Allow each button to be held and continuously move joint
            inc_button.setAutoRepeat(True)
            dec_button.setAutoRepeat(True)

            # Store the joint displays
            self.joint_displays.append(joint_display)

            # Add widgets to the layout
            joint_control.addWidget(joint_label)
            joint_control.addWidget(dec_button)
            joint_control.addWidget(joint_display)
            joint_control.addWidget(inc_button)

            joints_layout.addLayout(joint_control)

        joints_group.setLayout(joints_layout)

        # Coordinates Section (with Rotation Information)
        coords_group = QGroupBox("End Effector Coordinates")
        coords_layout = QVBoxLayout()

        # Position Information
        self.x_coord = QLineEdit("X: 0.0 mm")
        self.x_coord.setReadOnly(True)
        self.y_coord = QLineEdit("Y: 0.0 mm")
        self.y_coord.setReadOnly(True)
        self.z_coord = QLineEdit("Z: 0.0 mm")
        self.z_coord.setReadOnly(True)

        coords_layout.addWidget(self.x_coord)
        coords_layout.addWidget(self.y_coord)
        coords_layout.addWidget(self.z_coord)

        # Rotation Information
        self.rx_coord = QLineEdit("Rx: 0.0°")
        self.rx_coord.setReadOnly(True)
        self.ry_coord = QLineEdit("Ry: 0.0°")
        self.ry_coord.setReadOnly(True)
        self.rz_coord = QLineEdit("Rz: 0.0°")
        self.rz_coord.setReadOnly(True)

        coords_layout.addWidget(self.rx_coord)
        coords_layout.addWidget(self.ry_coord)
        coords_layout.addWidget(self.rz_coord)

        coords_group.setLayout(coords_layout)

        # Create the box in the GUI for each of the arm's modes
        modes_group = QGroupBox("Modes")
        modes_layout = QGridLayout()

        mode_types = ["Idle", "Setup", "Manual", "Inverse Kin",
                      "Dig", "Pick up", "Custom 1", "Custom 2"]
        self.mode_buttons = {m: QPushButton(m) for m in mode_types}

        modes_layout.addWidget(self.mode_buttons["Idle"], 0, 0)
        modes_layout.addWidget(self.mode_buttons["Setup"], 0, 1)
        modes_layout.addWidget(self.mode_buttons["Manual"], 1, 0)
        modes_layout.addWidget(self.mode_buttons["Inverse Kin"], 1, 1)
        modes_layout.addWidget(self.mode_buttons["Dig"], 2, 0)
        modes_layout.addWidget(self.mode_buttons["Pick up"], 2, 1)
        modes_layout.addWidget(self.mode_buttons["Custom 1"], 3, 0)
        modes_layout.addWidget(self.mode_buttons["Custom 2"], 3, 1)

        # Add callback to each of the modes
        for modes, button in self.mode_buttons.items():
            button.clicked.connect(
                lambda _, m=modes: self.button_is_clicked(m))

        modes_group.setLayout(modes_layout)

        # Movement rate box
        frequency_group = QGroupBox("Movement rate")

        joint_pub_freq = QHBoxLayout()

        self.frequencyButtons = {}

        # Add one of the buttons to change the frequency, as well as callback function
        self.frequencyButtons["Manualminus"] = QPushButton("-")
        self.frequencyButtons["Manualminus"].clicked.connect(
            lambda: self.update_publish_multiplier(-0.1))

        # Label to display the rate
        self.frequencyMultiplier = 1
        self.freq_display = QLabel("1")
        self.freq_display.setStyleSheet(
            "border: 1px solid black; padding: 5px;")
        self.freq_display.setFixedWidth(50)
        self.freq_display.setFixedHeight(30)
        self.freq_display.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Add one of the buttons to change the frequency, as well as callback function
        self.frequencyButtons["Manualplus"] = QPushButton("+")
        self.frequencyButtons["Manualplus"].clicked.connect(
            lambda: self.update_publish_multiplier(0.1))

        # Add widgets to layout
        joint_pub_freq.addWidget(self.frequencyButtons["Manualminus"])
        joint_pub_freq.addWidget(self.freq_display)
        joint_pub_freq.addWidget(self.frequencyButtons["Manualplus"])

        frequency_group.setLayout(joint_pub_freq)

        # Try to make the buttons take up the minimum size possible
        frequency_group.adjustSize()

        # Add all the stuff to the right side layout
        coords_layout.addWidget(modes_group)
        coords_layout.addWidget(science_modules)
        coords_layout.addWidget(frequency_group)
        coords_group.setLayout(coords_layout)

        # Adding sections to the main layout
        main_layout.addWidget(coordinates_group)
        main_layout.addWidget(view_group, stretch=3)
        main_layout.addWidget(joints_group)
        main_layout.addWidget(coords_group)

        # Main Vertical Layout
        main_vertical_layout = QVBoxLayout(self)
        main_vertical_layout.addLayout(main_layout)
        # main_vertical_layout.addLayout(bottom_layout)
        self.setLayout(main_vertical_layout)


class GuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui_node')
        self.get_logger().info('Robot GUI Node initialized')


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create Qt application
    app = QApplication(sys.argv)

    try:
        # Create ROS2 node
        gui_node = GuiNode()

        # Create GUI with the node
        window = RobotControlGUI(gui_node)
        window.show()

        # Create a QTimer to periodically spin the ROS2 node
        timer = QTimer()
        timer.timeout.connect(
            lambda: rclpy.spin_once(gui_node, timeout_sec=0.01))
        timer.start(10)  # 10ms = 100Hz

        # Run the Qt event loop
        exit_code = app.exec()

    except KeyboardInterrupt:
        pass
    finally:
        if 'gui_node' in locals():
            gui_node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
