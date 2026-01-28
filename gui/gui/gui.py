from PyQt6.QtWidgets import QMainWindow
import PyQt6.QtWidgets as QtWidgets
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import sys

class MainWindow(QMainWindow, Node):
    def __init__(self):
        # This will initialize the node under the name MainWindow along with the window itself
        QMainWindow.__init__(self, node_name="MainWindow")

        self.test_subscriber = self.create_subscription(
            String, "test_topic", self.test_callback, 10)
        self.test_publisher = self.create_publisher(
            String, "test_topic", 10)

        self.setWindowTitle("RSX ARM GUI")
        self.setGeometry(100, 100, 800, 600)

    def test_callback(self, msg):
        pass


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    ui = MainWindow()
    app.processEvents()
    ui.show()

    exec = MultiThreadedExecutor()
    exec.add_node(ui)
    while rclpy.ok():
        try:
            exec.wait_for_ready_callbacks(0)
            exec.spin_once()
        except:
            pass
        app.processEvents()
    app.quit()
    exec.remove_node(ui)