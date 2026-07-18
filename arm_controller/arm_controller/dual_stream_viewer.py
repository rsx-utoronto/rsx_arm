#!/usr/bin/env python3
"""
ROS2 Humble - Dual Video Stream PyQt5 Viewer
Subscribes to two image topics (CompressedImage or raw Image) and
displays them side-by-side in a dark, industrial-style GUI.

Dependencies:
    pip install PyQt5 opencv-python
    sudo apt install ros-humble-cv-bridge

Usage:
    python3 dual_stream_viewer.py
    # or as a ROS2 node:
    ros2 run <your_package> dual_stream_viewer \
        --ros-args \
        -p topic1:=/camera1/image/compressed \
        -p topic2:=/camera2/image/compressed \
        -p topic1_type:=compressed \
        -p topic2_type:=compressed

Parameters:
    topic1       (str, default: '/camera/image/compressed')
    topic2       (str, default: '/camera2/image/compressed')
    topic1_type  (str, 'compressed' | 'raw',  default: 'compressed')
    topic2_type  (str, 'compressed' | 'raw',  default: 'compressed')
    window_title (str, default: 'ROS2 Dual Stream Viewer')
"""

import sys
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QFrame, QSizePolicy, QStatusBar, QPushButton, QGroupBox,
    QGridLayout, QLineEdit, QComboBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QSize
from PyQt5.QtGui import QImage, QPixmap, QFont, QPalette, QColor, QFontDatabase


# ─────────────────────────────────────────────────────────────────────────────
#  Colour palette  (industrial dark theme)
# ─────────────────────────────────────────────────────────────────────────────
PALETTE = {
    'bg':        '#0d0f14',
    'panel':     '#13161d',
    'border':    '#1f2430',
    'accent':    '#00d4ff',
    'accent2':   '#ff6b35',
    'text':      '#c8d0e0',
    'text_dim':  '#4a5568',
    'ok':        '#00e676',
    'warn':      '#ffab40',
    'err':       '#ff1744',
    'header_bg': '#0a0c11',
}

STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: {PALETTE['bg']};
    color: {PALETTE['text']};
    font-family: 'Courier New', monospace;
}}
QLabel {{
    color: {PALETTE['text']};
}}
QFrame#stream_frame {{
    background-color: {PALETTE['panel']};
    border: 1px solid {PALETTE['border']};
    border-radius: 4px;
}}
QLabel#stream_label {{
    background-color: #000000;
    border: none;
}}
QLabel#title_label {{
    color: {PALETTE['accent']};
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 2px;
    padding: 4px 8px;
    background-color: {PALETTE['header_bg']};
    border-bottom: 1px solid {PALETTE['border']};
}}
QLabel#title_label2 {{
    color: {PALETTE['accent2']};
    font-size: 11px;
    font-weight: bold;
    letter-spacing: 2px;
    padding: 4px 8px;
    background-color: {PALETTE['header_bg']};
    border-bottom: 1px solid {PALETTE['border']};
}}
QLabel#status_dot {{
    font-size: 10px;
    padding: 2px 6px;
}}
QLabel#stats_label {{
    color: {PALETTE['text_dim']};
    font-size: 9px;
    padding: 2px 8px;
    font-family: 'Courier New', monospace;
}}
QLabel#header_title {{
    color: {PALETTE['accent']};
    font-size: 13px;
    font-weight: bold;
    letter-spacing: 3px;
    padding: 6px 12px;
}}
QPushButton {{
    background-color: {PALETTE['border']};
    color: {PALETTE['text']};
    border: 1px solid {PALETTE['text_dim']};
    border-radius: 3px;
    padding: 4px 12px;
    font-size: 10px;
    font-family: 'Courier New', monospace;
}}
QPushButton:hover {{
    background-color: {PALETTE['accent']};
    color: #000000;
    border-color: {PALETTE['accent']};
}}
QLineEdit {{
    background-color: {PALETTE['border']};
    color: {PALETTE['text']};
    border: 1px solid {PALETTE['text_dim']};
    border-radius: 3px;
    padding: 3px 6px;
    font-family: 'Courier New', monospace;
    font-size: 10px;
}}
QComboBox {{
    background-color: {PALETTE['border']};
    color: {PALETTE['text']};
    border: 1px solid {PALETTE['text_dim']};
    border-radius: 3px;
    padding: 3px 6px;
    font-family: 'Courier New', monospace;
    font-size: 10px;
}}
QComboBox::drop-down {{
    border: none;
}}
QComboBox QAbstractItemView {{
    background-color: {PALETTE['panel']};
    color: {PALETTE['text']};
    selection-background-color: {PALETTE['accent']};
    selection-color: #000000;
}}
QStatusBar {{
    background-color: {PALETTE['header_bg']};
    color: {PALETTE['text_dim']};
    font-size: 9px;
    font-family: 'Courier New', monospace;
    border-top: 1px solid {PALETTE['border']};
}}
QGroupBox {{
    color: {PALETTE['text_dim']};
    border: 1px solid {PALETTE['border']};
    border-radius: 4px;
    margin-top: 8px;
    font-size: 9px;
    font-family: 'Courier New', monospace;
    padding-top: 4px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
    color: {PALETTE['text_dim']};
}}
"""


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 subscriber thread
# ─────────────────────────────────────────────────────────────────────────────

class RosThread(QThread):
    """Runs rclpy.spin() in a background thread."""

    def __init__(self, node: Node):
        super().__init__()
        self._node = node

    def run(self):
        rclpy.spin(self._node)

    def stop(self):
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
#  Single stream panel widget
# ─────────────────────────────────────────────────────────────────────────────

class StreamPanel(QFrame):
    """Displays one video stream with stats overlay."""

    def __init__(self, channel_id: int, topic: str, topic_type: str, parent=None):
        super().__init__(parent)
        self.setObjectName('stream_frame')

        self.channel_id  = channel_id
        self._topic      = topic
        self._topic_type = topic_type
        self._frame_count = 0
        self._fps        = 0.0
        self._last_time  = time.time()
        self._connected  = False

        accent = PALETTE['accent'] if channel_id == 1 else PALETTE['accent2']
        title_obj = 'title_label' if channel_id == 1 else 'title_label2'

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # ── Header bar ────────────────────────────────────────────────────────
        header = QWidget()
        header.setStyleSheet(f'background-color: {PALETTE["header_bg"]};'
                             f'border-bottom: 1px solid {PALETTE["border"]};')
        hbox = QHBoxLayout(header)
        hbox.setContentsMargins(0, 0, 0, 0)

        self.title_lbl = QLabel(f'CH{channel_id} ▸ {topic}')
        self.title_lbl.setObjectName(title_obj)
        hbox.addWidget(self.title_lbl)

        hbox.addStretch()

        self.dot_lbl = QLabel('● WAITING')
        self.dot_lbl.setObjectName('status_dot')
        self.dot_lbl.setStyleSheet(f'color: {PALETTE["warn"]};')
        hbox.addWidget(self.dot_lbl)

        layout.addWidget(header)

        # ── Video label ───────────────────────────────────────────────────────
        self.video_lbl = QLabel()
        self.video_lbl.setObjectName('stream_label')
        self.video_lbl.setAlignment(Qt.AlignCenter)
        self.video_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_lbl.setMinimumSize(320, 240)
        # Placeholder
        self._show_placeholder()
        layout.addWidget(self.video_lbl, stretch=1)

        # ── Stats bar ─────────────────────────────────────────────────────────
        self.stats_lbl = QLabel(f'TOPIC: {topic}  |  TYPE: {topic_type}  |  FPS: --  |  RES: --')
        self.stats_lbl.setObjectName('stats_label')
        layout.addWidget(self.stats_lbl)

    # ── Placeholder ───────────────────────────────────────────────────────────

    def _show_placeholder(self):
        w = max(self.video_lbl.width(),  320)
        h = max(self.video_lbl.height(), 240)
        img = np.zeros((h, w, 3), dtype=np.uint8)
        # draw crosshatch
        for i in range(0, w, 40):
            cv2.line(img, (i, 0), (i, h), (25, 30, 40), 1)
        for j in range(0, h, 40):
            cv2.line(img, (0, j), (w, j), (25, 30, 40), 1)
        text = f'CH{self.channel_id}  AWAITING SIGNAL'
        tw, th = cv2.getTextSize(text, cv2.FONT_HERSHEY_PLAIN, 1.0, 1)[0]
        cv2.putText(img, text,
                    ((w - tw) // 2, (h + th) // 2),
                    cv2.FONT_HERSHEY_PLAIN, 1.0, (50, 80, 100), 1,
                    cv2.LINE_AA)
        self._display_frame(img)

    # ── Public API ────────────────────────────────────────────────────────────

    def update_frame(self, frame: np.ndarray):
        """Called from the Qt main thread via signal."""
        self._frame_count += 1
        now = time.time()
        dt  = now - self._last_time
        if dt >= 1.0:
            self._fps       = self._frame_count / dt
            self._frame_count = 0
            self._last_time = now

        h, w = frame.shape[:2]
        self.stats_lbl.setText(
            f'TOPIC: {self._topic}  |  TYPE: {self._topic_type}  |  '
            f'FPS: {self._fps:.1f}  |  RES: {w}x{h}')

        if not self._connected:
            self._connected = True
            accent = PALETTE['accent'] if self.channel_id == 1 else PALETTE['accent2']
            self.dot_lbl.setText('● LIVE')
            self.dot_lbl.setStyleSheet(f'color: {PALETTE["ok"]};')

        self._display_frame(frame)

    def set_disconnected(self):
        self._connected = False
        self.dot_lbl.setText('● NO SIGNAL')
        self.dot_lbl.setStyleSheet(f'color: {PALETTE["err"]};')

    def set_topic(self, topic: str, topic_type: str):
        self._topic      = topic
        self._topic_type = topic_type
        self._connected  = False
        self.title_lbl.setText(f'CH{self.channel_id} ▸ {topic}')
        self.dot_lbl.setText('● WAITING')
        self.dot_lbl.setStyleSheet(f'color: {PALETTE["warn"]};')
        self._show_placeholder()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _display_frame(self, frame: np.ndarray):
        """Convert BGR ndarray → QPixmap and scale into label."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        lw = self.video_lbl.width()
        lh = self.video_lbl.height()
        scaled = pixmap.scaled(lw, lh, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_lbl.setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._connected:
            self._show_placeholder()


# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 dual viewer node
# ─────────────────────────────────────────────────────────────────────────────

class DualViewerNode(Node):
    """ROS2 node that subscribes to two image topics and emits Qt signals."""

    def __init__(self,
                 frame1_cb,
                 frame2_cb,
                 topic1:      str = '/camera/image/compressed',
                 topic2:      str = '/camera2/image/compressed',
                 topic1_type: str = 'compressed',
                 topic2_type: str = 'compressed'):
        super().__init__('dual_stream_viewer')

        # Declare parameters
        self.declare_parameter('topic1',      topic1)
        self.declare_parameter('topic2',      topic2)
        self.declare_parameter('topic1_type', topic1_type)
        self.declare_parameter('topic2_type', topic2_type)
        self.declare_parameter('window_title', 'ROS2 Dual Stream Viewer')

        t1  = self.get_parameter('topic1').value
        t2  = self.get_parameter('topic2').value
        tt1 = self.get_parameter('topic1_type').value
        tt2 = self.get_parameter('topic2_type').value

        self._frame1_cb = frame1_cb
        self._frame2_cb = frame2_cb

        self._sub1 = self._make_sub(t1, tt1, self._frame1_cb)
        self._sub2 = self._make_sub(t2, tt2, self._frame2_cb)

        self.get_logger().info(f'Sub1: {t1} ({tt1})')
        self.get_logger().info(f'Sub2: {t2} ({tt2})')

    # ── Subscription factory ──────────────────────────────────────────────────

    def _make_sub(self, topic: str, topic_type: str, callback):
        if topic_type.lower() == 'compressed':
            return self.create_subscription(
                CompressedImage, topic,
                lambda msg, cb=callback: cb(self._decode_compressed(msg)),
                10)
        else:
            return self.create_subscription(
                Image, topic,
                lambda msg, cb=callback: cb(self._decode_raw(msg)),
                10)

    # ── Decoders ──────────────────────────────────────────────────────────────

    @staticmethod
    def _decode_compressed(msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return frame

    @staticmethod
    def _decode_raw(msg: Image):
        encoding = msg.encoding.lower()
        dtype = np.uint8

        if encoding in ('rgb8',):
            frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, 3)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding in ('bgr8',):
            frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, 3)
        elif encoding in ('mono8', '8uc1'):
            frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif encoding in ('16uc1', 'mono16'):
            frame = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            frame = (frame / 256).astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            # Fallback: try to reshape as BGR8
            frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, -1)
            if frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame

    # ── Live re-subscribe ─────────────────────────────────────────────────────

    def resubscribe(self, stream_id: int, topic: str, topic_type: str):
        """Tear down old sub and create a new one at runtime."""
        cb = self._frame1_cb if stream_id == 1 else self._frame2_cb
        new_sub = self._make_sub(topic, topic_type, cb)
        if stream_id == 1:
            self.destroy_subscription(self._sub1)
            self._sub1 = new_sub
        else:
            self.destroy_subscription(self._sub2)
            self._sub2 = new_sub
        self.get_logger().info(f'Resubscribed CH{stream_id} → {topic} ({topic_type})')


# ─────────────────────────────────────────────────────────────────────────────
#  Main window
# ─────────────────────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):

    frame1_signal = pyqtSignal(object)
    frame2_signal = pyqtSignal(object)

    def __init__(self, node: DualViewerNode,
                 topic1: str, topic2: str,
                 topic1_type: str, topic2_type: str):
        super().__init__()
        self._node = node

        self.setWindowTitle('ROS2 DUAL STREAM VIEWER')
        self.setMinimumSize(900, 560)
        self.setStyleSheet(STYLESHEET)

        # Connect signals (thread-safe frame delivery)
        self.frame1_signal.connect(self._on_frame1)
        self.frame2_signal.connect(self._on_frame2)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── Top header ────────────────────────────────────────────────────────
        header = QWidget()
        header.setStyleSheet(
            f'background-color: {PALETTE["header_bg"]};'
            f'border: 1px solid {PALETTE["border"]};'
            f'border-radius: 3px;')
        hbox = QHBoxLayout(header)
        hbox.setContentsMargins(8, 4, 8, 4)

        logo = QLabel('◈  ROS2 DUAL STREAM VIEWER')
        logo.setObjectName('header_title')
        hbox.addWidget(logo)
        hbox.addStretch()

        self.node_status = QLabel('NODE: INITIALIZING')
        self.node_status.setStyleSheet(
            f'color: {PALETTE["warn"]}; font-size: 9px; padding: 2px 8px;')
        hbox.addWidget(self.node_status)

        root.addWidget(header)

        # ── Stream panels ─────────────────────────────────────────────────────
        streams_widget = QWidget()
        streams_layout = QHBoxLayout(streams_widget)
        streams_layout.setContentsMargins(0, 0, 0, 0)
        streams_layout.setSpacing(6)

        self.panel1 = StreamPanel(1, topic1, topic1_type)
        self.panel2 = StreamPanel(2, topic2, topic2_type)

        streams_layout.addWidget(self.panel1)
        streams_layout.addWidget(self.panel2)
        root.addWidget(streams_widget, stretch=1)

        # ── Controls ──────────────────────────────────────────────────────────
        ctrl_group = QGroupBox('TOPIC CONFIGURATION')
        ctrl_grid  = QGridLayout(ctrl_group)
        ctrl_grid.setContentsMargins(8, 12, 8, 8)
        ctrl_grid.setSpacing(4)

        # Channel 1
        ctrl_grid.addWidget(QLabel('CH1 Topic:'), 0, 0)
        self.t1_edit = QLineEdit(topic1)
        ctrl_grid.addWidget(self.t1_edit, 0, 1)
        ctrl_grid.addWidget(QLabel('Type:'), 0, 2)
        self.t1_type = QComboBox()
        self.t1_type.addItems(['compressed', 'raw'])
        self.t1_type.setCurrentText(topic1_type)
        ctrl_grid.addWidget(self.t1_type, 0, 3)
        btn1 = QPushButton('APPLY CH1')
        btn1.clicked.connect(lambda: self._apply_topic(1))
        ctrl_grid.addWidget(btn1, 0, 4)

        # Channel 2
        ctrl_grid.addWidget(QLabel('CH2 Topic:'), 1, 0)
        self.t2_edit = QLineEdit(topic2)
        ctrl_grid.addWidget(self.t2_edit, 1, 1)
        ctrl_grid.addWidget(QLabel('Type:'), 1, 2)
        self.t2_type = QComboBox()
        self.t2_type.addItems(['compressed', 'raw'])
        self.t2_type.setCurrentText(topic2_type)
        ctrl_grid.addWidget(self.t2_type, 1, 3)
        btn2 = QPushButton('APPLY CH2')
        btn2.clicked.connect(lambda: self._apply_topic(2))
        ctrl_grid.addWidget(btn2, 1, 4)

        root.addWidget(ctrl_group)

        # ── Status bar ────────────────────────────────────────────────────────
        sb = QStatusBar()
        self.setStatusBar(sb)
        sb.showMessage('Ready — Waiting for image topics...')

        # ── Node status update timer ──────────────────────────────────────────
        self._ok_timer = QTimer(self)
        self._ok_timer.timeout.connect(self._set_node_ok)
        self._ok_timer.setSingleShot(True)
        self._ok_timer.start(2000)

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _set_node_ok(self):
        self.node_status.setText('NODE: RUNNING')
        self.node_status.setStyleSheet(
            f'color: {PALETTE["ok"]}; font-size: 9px; padding: 2px 8px;')

    def _on_frame1(self, frame):
        if frame is not None:
            self.panel1.update_frame(frame)

    def _on_frame2(self, frame):
        if frame is not None:
            self.panel2.update_frame(frame)

    def _apply_topic(self, ch: int):
        if ch == 1:
            topic = self.t1_edit.text().strip()
            ttype = self.t1_type.currentText()
            self.panel1.set_topic(topic, ttype)
        else:
            topic = self.t2_edit.text().strip()
            ttype = self.t2_type.currentText()
            self.panel2.set_topic(topic, ttype)

        self._node.resubscribe(ch, topic, ttype)
        self.statusBar().showMessage(f'CH{ch} resubscribed → {topic} ({ttype})', 3000)

    # ── ROS callbacks (called from ROS thread → emit signal) ──────────────────

    def ros_frame1(self, frame):
        self.frame1_signal.emit(frame)

    def ros_frame2(self, frame):
        self.frame2_signal.emit(frame)


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    # Default topics — override via ROS2 parameters or the GUI controls
    TOPIC1      = '/webcam/compressed'
    TOPIC2      = '/webcam2/compressed'
    TOPIC1_TYPE = 'compressed'   # 'compressed' | 'raw'
    TOPIC2_TYPE = 'compressed'

    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setApplicationName('ROS2 Dual Stream Viewer')

    # Temporary callbacks that get overridden once the window exists
    _pending1 = []
    _pending2 = []

    def cb1(f): _pending1.append(f)
    def cb2(f): _pending2.append(f)

    node = DualViewerNode(cb1, cb2,
                          TOPIC1, TOPIC2,
                          TOPIC1_TYPE, TOPIC2_TYPE)

    window = MainWindow(node, TOPIC1, TOPIC2, TOPIC1_TYPE, TOPIC2_TYPE)

    # Wire callbacks properly now that the window exists
    node._frame1_cb = window.ros_frame1
    node._frame2_cb = window.ros_frame2

    # Flush anything that arrived during init
    for f in _pending1: window.ros_frame1(f)
    for f in _pending2: window.ros_frame2(f)

    window.show()

    # Spin ROS in a background thread
    ros_thread = RosThread(node)
    ros_thread.start()

    exit_code = app.exec_()

    ros_thread.stop()
    ros_thread.wait(2000)
    node.destroy_node()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
