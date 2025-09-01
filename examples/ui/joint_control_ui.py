from PyQt5 import QtWidgets, QtCore
from rs_protocol import PacketID, Mode
from typing import List

class JointControlUI(QtWidgets.QWidget):
    def __init__(self, manipulator):
        super().__init__()
        self.manipulator = manipulator
        self.setWindowTitle("Joint Control UI")
        self.setStyleSheet("background-color: rgb(21, 57, 86); color: #CCCCCC;")
        self.device_id_map = {}
        self.position_labels = []
        self.mode_labels = []
        self.velocity_labels = []
        self.torque_labels = []
        self.velocity_value = 1.0

        layout = QtWidgets.QVBoxLayout()
        
        # Velocity scale
        velocity_layout = QtWidgets.QHBoxLayout()
        velocity_label = QtWidgets.QLabel("Velocity Scale:")
        self.velocity_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.velocity_slider.setMinimum(0)
        self.velocity_slider.setMaximum(100)
        self.velocity_slider.setValue(100)
        self.velocity_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #E24D30;
                height: 8px; /* the groove expands to the size of the slider */
                background: #C0C0C0; /* Background color of the groove */
                margin: 2px 0;
                border-radius: 4px;
            }

            QSlider::handle:horizontal {
                background: #E24D30; /* Color of the slider handle */
                border: 1px solid #999999;
                width: 18px;
                margin: -5px 0; /* handle is placed on top of the groove */
                border-radius: 9px;
            }

            QSlider::add-page:horizontal {
                background: #CCCCCC; /* Color of the part of the groove to the right of the handle */
            }

            QSlider::sub-page:horizontal {
                background: #E24D30; /* Color of the part of the groove to the left of the handle */
                border-radius: 4px;
            }
        """)

        self.velocity_slider.valueChanged.connect(self.on_velocity_slider)
        velocity_layout.addWidget(velocity_label)
        velocity_layout.addWidget(self.velocity_slider)
        layout.addLayout(velocity_layout)

        for i, actuator in enumerate(self.manipulator.actuators):
            self.register_callbacks(actuator)
            self.device_id_map[actuator.device_id] = i

            frame = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(f"ID: {actuator.device_id}")
            label.setFixedWidth(150)
            frame.addWidget(label)

            mode_label = QtWidgets.QLabel("Mode: --")
            mode_label.setFixedWidth(100)
            frame.addWidget(mode_label)
            self.mode_labels.append(mode_label)

            position_label = QtWidgets.QLabel("q: 0.00")
            position_label.setFixedWidth(100)
            frame.addWidget(position_label)
            self.position_labels.append(position_label)

            torque_label = QtWidgets.QLabel("𝜏: 0.0")
            torque_label.setFixedWidth(100)
            frame.addWidget(torque_label)
            self.torque_labels.append(torque_label)

            velocity_label = QtWidgets.QLabel("q̇: 0.0")
            velocity_label.setFixedWidth(100)
            frame.addWidget(velocity_label)
            self.velocity_labels.append(velocity_label)

            minus_button = QtWidgets.QPushButton("-")
            plus_button = QtWidgets.QPushButton("+")
            minus_button.setFixedWidth(30)
            plus_button.setFixedWidth(30)
            frame.addWidget(minus_button)
            frame.addWidget(plus_button)

            # Use a QTimer to continuously send velocity commands while button is held
            minus_timer = QtCore.QTimer(self)
            plus_timer = QtCore.QTimer(self)
            minus_timer.setInterval(50)
            plus_timer.setInterval(50)

            def start_minus_timer(idx=i):
                self.hold_velocity(idx, -1)
                minus_timer.timeout.connect(lambda: self.hold_velocity(idx, -1))
                minus_timer.start()

            def stop_minus_timer(idx=i):
                minus_timer.stop()
                self.set_velocity(idx, 0)
                minus_timer.timeout.disconnect()

            def start_plus_timer(idx=i):
                self.hold_velocity(idx, 1)
                plus_timer.timeout.connect(lambda: self.hold_velocity(idx, 1))
                plus_timer.start()

            def stop_plus_timer(idx=i):
                plus_timer.stop()
                self.set_velocity(idx, 0)
                plus_timer.timeout.disconnect()

            minus_button.pressed.connect(start_minus_timer)
            minus_button.released.connect(stop_minus_timer)
            plus_button.pressed.connect(start_plus_timer)
            plus_button.released.connect(stop_plus_timer)

            layout.addLayout(frame)

        self.setLayout(layout)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(20)

    def register_callbacks(self, actuator):
        actuator.protocol.register_callback(actuator.device_id, PacketID.POSITION, self.position_callback)
        actuator.protocol.register_callback(actuator.device_id, PacketID.VELOCITY, self.velocity_callback)
        actuator.protocol.register_callback(actuator.device_id, PacketID.TORQUE, self.torque_callback)
        actuator.protocol.register_callback(actuator.device_id, PacketID.MODE, self.mode_callback)

    def on_velocity_slider(self, value):
        self.velocity_value = value / 100.0

    def hold_velocity(self, joint_index: int, direction: int):
        self.set_velocity(joint_index, self.velocity_value * direction)

    def set_velocity(self, joint_index: int, velocity: float):
        self.manipulator.protocol.write(self.manipulator.actuators[joint_index].device_id, PacketID.VELOCITY, [velocity])

    def position_callback(self, device_id, packet_id, data):
        self.position_labels[self.device_id_map[device_id]].setText(f"q: {data[0]:+6.2f}")

    def velocity_callback(self, device_id, packet_id, data):
        self.velocity_labels[self.device_id_map[device_id]].setText(f"q̇: {data[0]:+6.1f}")

    def torque_callback(self, device_id, packet_id, data):
        self.torque_labels[self.device_id_map[device_id]].setText(f"𝜏: {data[0]:+6.1f}")

    def mode_callback(self, device_id, packet_id, data):
        self.mode_labels[self.device_id_map[device_id]].setText(f"{Mode(data[0]).name}")

    def update(self):
        self.manipulator.update()

    def run(self):
        self.show()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.exec_()
