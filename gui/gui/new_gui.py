#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget

class SimpleWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simple PyQt Window")
        self.setGeometry(200, 200, 800, 500)  # x, y, width, height

def main():
    app = QApplication(sys.argv)
    window = SimpleWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()