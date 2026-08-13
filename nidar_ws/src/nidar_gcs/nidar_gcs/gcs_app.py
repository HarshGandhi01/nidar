import sys
import math
import time
import rclpy
from rclpy.node import Node
from threading import Thread

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTabWidget, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QProgressBar, QTextEdit
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QImage, QPixmap

from sensor_msgs.msg import Image, LaserScan, NavSatFix
from nav_msgs.msg import Odometry
from nidar_interfaces.msg import Survivor, DroneStatus, SwarmTask
from std_msgs.msg import String

class ROSBridgeSignals(QObject):
    airmouse_pose = pyqtSignal(float, float, float)
    airmouse_survivor = pyqtSignal(int, float, float, str)
    airmouse_status = pyqtSignal(str, float, str)
    rescueswarm_survivor = pyqtSignal(int, float, float, str)
    delivery_status = pyqtSignal(str)
    cam_image = pyqtSignal(QImage)

class GCSROSNode(Node):
    def __init__(self, signals):
        super().__init__('nidar_gcs_node')
        self.signals = signals

        self.create_subscription(Odometry, '/airmouse/odom', self.odom_cb, 10)
        self.create_subscription(Survivor, '/airmouse/survivor_detected', self.airmouse_surv_cb, 10)
        self.create_subscription(DroneStatus, '/airmouse/status', self.airmouse_status_cb, 10)
        
        self.create_subscription(Survivor, '/rescueswarm/survivor_detected', self.rescueswarm_surv_cb, 10)
        self.create_subscription(String, '/rescueswarm/delivery_status', self.delivery_cb, 10)

        self.pub_abort = self.create_publisher(String, '/airmouse/abort', 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.signals.airmouse_pose.emit(p.x, p.y, yaw)

    def airmouse_surv_cb(self, msg):
        self.signals.airmouse_survivor.emit(msg.id, msg.local_pos.x, msg.local_pos.y, msg.grid_coordinate)

    def airmouse_status_cb(self, msg):
        self.signals.airmouse_status.emit(msg.drone_id, msg.battery_percentage, msg.state)

    def rescueswarm_surv_cb(self, msg):
        self.signals.rescueswarm_survivor.emit(msg.id, msg.latitude, msg.longitude, msg.status)

    def delivery_cb(self, msg):
        self.signals.delivery_status.emit(msg.data)

    def send_abort(self):
        msg = String()
        msg.data = "OPERATOR EMERGENCY ABORT"
        self.pub_abort.publish(msg)


class MapCanvas2D(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(450, 450)
        self.drone_pos = (0.5, 0.5, 0.0)
        self.drone_path = [(0.5, 0.5)]
        self.survivors = [] # list of (id, x, y, grid)

    def update_drone_pose(self, x, y, yaw):
        self.drone_pos = (x, y, yaw)
        self.drone_path.append((x, y))
        self.update()

    def add_survivor(self, surv_id, x, y, grid):
        self.survivors.append((surv_id, x, y, grid))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background (Dark grid)
        painter.fillRect(self.rect(), QColor(25, 30, 40))

        w = self.width()
        h = self.height()
        margin = 30
        grid_size = min(w - 2 * margin, h - 2 * margin)

        # Draw 15x15m Arena Grid (1.5m grid lines -> 10x10 cells)
        cell_px = grid_size / 10.0

        pen_grid = QPen(QColor(60, 70, 90), 1, Qt.DashLine)
        painter.setPen(pen_grid)
        
        for i in range(11):
            x = margin + i * cell_px
            y = margin + i * cell_px
            painter.drawLine(int(x), margin, int(x), margin + int(grid_size))
            painter.drawLine(margin, int(y), margin + int(grid_size), int(y))

        # Grid Axis Labels (Rows A-J, Cols 1-10)
        painter.setPen(QColor(180, 190, 210))
        font = QFont("SansSerif", 9, QFont.Bold)
        painter.setFont(font)

        for col in range(10):
            painter.drawText(int(margin + col * cell_px + cell_px/2 - 4), margin - 8, str(col + 1))
        for row in range(10):
            row_char = chr(ord('A') + row)
            painter.drawText(margin - 20, int(margin + (9 - row) * cell_px + cell_px/2 + 4), row_char)

        # Helper to map arena coords (0..15m) to screen coords
        def arena2screen(ax, ay):
            sx = margin + (ax / 15.0) * grid_size
            sy = margin + (1.0 - ay / 15.0) * grid_size
            return int(sx), int(sy)

        # Draw Drone Path
        if len(self.drone_path) > 1:
            pen_path = QPen(QColor(0, 220, 255, 180), 2)
            painter.setPen(pen_path)
            for i in range(len(self.drone_path) - 1):
                x1, y1 = arena2screen(*self.drone_path[i])
                x2, y2 = arena2screen(*self.drone_path[i+1])
                painter.drawLine(x1, y1, x2, y2)

        # Draw Survivors
        for surv_id, sx, sy, sgrid in self.survivors:
            px, py = arena2screen(sx, sy)
            painter.setBrush(QBrush(QColor(255, 50, 50)))
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawEllipse(px - 8, py - 8, 16, 16)
            painter.setPen(QColor(255, 255, 255))
            painter.drawText(px + 10, py + 4, f"S{surv_id} ({sgrid})")

        # Draw Drone Marker
        dx, dy, dyaw = self.drone_pos
        dpx, dpy = arena2screen(dx, dy)
        painter.setBrush(QBrush(QColor(0, 255, 120)))
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.drawEllipse(dpx - 10, dpy - 10, 20, 20)

        # Draw heading direction vector
        hx = dpx + int(18 * math.cos(dyaw))
        hy = dpy - int(18 * math.sin(dyaw))
        painter.setPen(QPen(QColor(0, 255, 120), 3))
        painter.drawLine(dpx, dpy, hx, hy)


class NIDARGroundControlStation(QMainWindow):
    def __init__(self, ros_node, signals):
        super().__init__()
        self.ros_node = ros_node
        self.signals = signals

        self.setWindowTitle("NIDAR Ground Control Station (GCS) - Mission Control")
        self.setGeometry(100, 100, 1280, 800)

        self.init_ui()
        self.connect_signals()

        # Mission Timer (30 minutes countdown)
        self.start_timestamp = time.time()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_timer)
        self.timer.start(1000)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Header Bar
        header = QHBoxLayout()
        title_label = QLabel("NIDAR MISSION CONTROL GCS")
        title_label.setFont(QFont("SansSerif", 16, QFont.Bold))
        title_label.setStyleSheet("color: #00E5FF; padding: 5px;")
        
        self.timer_label = QLabel("MISSION TIMER: 30:00")
        self.timer_label.setFont(QFont("Monospace", 14, QFont.Bold))
        self.timer_label.setStyleSheet("color: #FFD700; background: #1E222D; padding: 8px; border-radius: 4px;")

        self.btn_abort = QPushButton("EMERGENCY ABORT / RECALL")
        self.btn_abort.setFont(QFont("SansSerif", 12, QFont.Bold))
        self.btn_abort.setStyleSheet("background: #FF2E93; color: white; padding: 10px; border-radius: 6px;")
        self.btn_abort.clicked.connect(self.on_abort_clicked)

        header.addWidget(title_label)
        header.addStretch()
        header.addWidget(self.timer_label)
        header.addWidget(self.btn_abort)
        main_layout.addLayout(header)

        # Tabs for Track 1 & Track 2
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { font-size: 13px; font-weight: bold; padding: 8px 16px; }")

        # Track 1 AirMouse Tab
        tab_airmouse = QWidget()
        layout_am = QHBoxLayout(tab_airmouse)

        # Left: 2D SLAM Map
        gb_map = QGroupBox("Track 1: AirMouse - Continuous 2D Indoor SLAM & Grid Map")
        gb_map_layout = QVBoxLayout(gb_map)
        self.map_canvas = MapCanvas2D()
        gb_map_layout.addWidget(self.map_canvas)
        layout_am.addWidget(gb_map, stretch=3)

        # Right: Telemetry & Survivor Table
        gb_info = QGroupBox("AirMouse Telemetry & Detected Survivors")
        gb_info_layout = QVBoxLayout(gb_info)

        self.lbl_am_status = QLabel("Drone Status: EXPLORING | Battery: 100%")
        self.lbl_am_status.setFont(QFont("SansSerif", 11, QFont.Bold))
        self.lbl_am_status.setStyleSheet("color: #00FF88;")
        gb_info_layout.addWidget(self.lbl_am_status)

        self.table_am_survivors = QTableWidget(0, 4)
        self.table_am_survivors.setHorizontalHeaderLabels(["ID", "Grid Coord", "Position (X, Y)", "Status"])
        self.table_am_survivors.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        gb_info_layout.addWidget(self.table_am_survivors)

        layout_am.addWidget(gb_info, stretch=2)
        self.tabs.addTab(tab_airmouse, "Track 1 - AirMouse (Indoor GPS-Denied)")

        # Track 2 RescueSwarm Tab
        tab_rescueswarm = QWidget()
        layout_rs = QVBoxLayout(tab_rescueswarm)

        gb_rs_info = QGroupBox("Track 2: RescueSwarm - Multi-Drone Swarm & Geotagged Aid Delivery")
        gb_rs_layout = QVBoxLayout(gb_rs_info)

        self.table_rs_survivors = QTableWidget(0, 4)
        self.table_rs_survivors.setHorizontalHeaderLabels(["Survivor ID", "GPS Latitude", "GPS Longitude", "Delivery Status"])
        self.table_rs_survivors.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        gb_rs_layout.addWidget(self.table_rs_survivors)

        self.txt_delivery_log = QTextEdit()
        self.txt_delivery_log.setReadOnly(True)
        self.txt_delivery_log.setStyleSheet("background: #11151F; color: #00FFCC; font-family: Monospace;")
        gb_rs_layout.addWidget(self.txt_delivery_log)

        layout_rs.addWidget(gb_rs_info)
        self.tabs.addTab(tab_rescueswarm, "Track 2 - RescueSwarm (Outdoor Multi-Drone)")

        main_layout.addWidget(self.tabs)

        # Apply Global Dark Styling
        self.setStyleSheet("""
            QMainWindow { background-color: #121620; color: #FFFFFF; }
            QGroupBox { font-weight: bold; border: 1px solid #2C3545; margin-top: 6px; color: #00E5FF; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
            QTableWidget { background-color: #1A202C; color: #FFFFFF; gridline-color: #2D3748; }
            QHeaderView::section { background-color: #2D3748; color: #00E5FF; font-weight: bold; }
        """)

    def connect_signals(self):
        self.signals.airmouse_pose.connect(self.map_canvas.update_drone_pose)
        self.signals.airmouse_survivor.connect(self.on_airmouse_survivor)
        self.signals.airmouse_status.connect(self.on_airmouse_status)
        self.signals.rescueswarm_survivor.connect(self.on_rescueswarm_survivor)
        self.signals.delivery_status.connect(self.on_delivery_status)

    def on_airmouse_survivor(self, surv_id, x, y, grid):
        self.map_canvas.add_survivor(surv_id, x, y, grid)
        row = self.table_am_survivors.rowCount()
        self.table_am_survivors.insertRow(row)
        self.table_am_survivors.setItem(row, 0, QTableWidgetItem(f"Survivor #{surv_id}"))
        self.table_am_survivors.setItem(row, 1, QTableWidgetItem(grid))
        self.table_am_survivors.setItem(row, 2, QTableWidgetItem(f"({x:.2f}, {y:.2f})"))
        self.table_am_survivors.setItem(row, 3, QTableWidgetItem("DETECTED & TAGGED"))

    def on_airmouse_status(self, drone_id, battery, state):
        self.lbl_am_status.setText(f"Drone Status: {state} | Battery: {battery:.1f}%")

    def on_rescueswarm_survivor(self, surv_id, lat, lon, status):
        row = self.table_rs_survivors.rowCount()
        self.table_rs_survivors.insertRow(row)
        self.table_rs_survivors.setItem(row, 0, QTableWidgetItem(f"Survivor #{surv_id}"))
        self.table_rs_survivors.setItem(row, 1, QTableWidgetItem(f"{lat:.6f}"))
        self.table_rs_survivors.setItem(row, 2, QTableWidgetItem(f"{lon:.6f}"))
        self.table_rs_survivors.setItem(row, 3, QTableWidgetItem(status))

    def on_delivery_status(self, text):
        self.txt_delivery_log.append(f"[{time.strftime('%H:%M:%S')}] {text}")

    def on_abort_clicked(self):
        self.ros_node.send_abort()
        self.timer_label.setText("ABORTED")
        self.timer_label.setStyleSheet("color: #FF0055; background: #330011; padding: 8px; border-radius: 4px;")

    def update_timer(self):
        elapsed = time.time() - self.start_timestamp
        rem = max(0, 1800 - int(elapsed))
        mins = rem // 60
        secs = rem % 60
        self.timer_label.setText(f"MISSION TIMER: {mins:02d}:{secs:02d}")


def main(args=None):
    rclpy.init(args=args)
    signals = ROSBridgeSignals()
    ros_node = GCSROSNode(signals)

    spin_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    gcs = NIDARGroundControlStation(ros_node, signals)
    gcs.show()
    
    ret = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
