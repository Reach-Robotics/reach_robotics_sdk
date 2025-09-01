
import numpy as np
from typing import List

import pyqtgraph.opengl as gl
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer, Qt

from device.manipulator import Manipulator


TRANSLATE_VELOCITY_SCALE = 400.0  # Scale factor for translational velocity (mm/s)
ROTATE_VELOCITY_SCALE = 1.0 # Scale factor for rotational velocity (rad/s)


def create_triad(T: np.ndarray, size: float = 0.1) -> List[gl.GLLinePlotItem]:
    origin = T[:3, 3]
    axes = T[:3, :3]
    colors = [(1,0,0,1), (0,1,0,1), (0,0,1,1)]
    arrows = []
    for i in range(3):
        arrow = gl.GLLinePlotItem(
            pos=np.array([origin, origin + axes[:,i]*size]),
            color=colors[i],
            width=3,
            antialias=True
        )
        arrows.append(arrow)
    return arrows


def update_triad(arrows: List[gl.GLLinePlotItem], T: np.ndarray, size: float = 0.1) -> None:
    origin = T[:3, 3]
    axes = T[:3, :3]
    for i in range(3):
        pos = np.array([origin, origin + axes[:,i]*size])
        arrows[i].setData(pos=pos)


class KinematicControlUI(QtWidgets.QWidget):
    def __init__(self, manipulator: Manipulator) -> None:
        super(KinematicControlUI, self).__init__()
        
        self.manipulator = manipulator
        
        self.setWindowTitle('Kinematic Control UI')
        self.resize(1000, 600)
        self._init_ui()
        self._init_triads()
        self._init_update_loop()

        self.btn_state = [0] * 6

    def _init_ui(self) -> None:
        layout = QtWidgets.QHBoxLayout(self)
        
        # 3D view
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor((9, 31, 57))
        
        self.view.setCameraPosition(distance=1.5)
        layout.addWidget(self.view, stretch=1)

        # Control panel
        self.control_panel = QtWidgets.QWidget()
        self.control_layout = QtWidgets.QVBoxLayout(self.control_panel)
        layout.addWidget(self.control_panel)

        self.setStyleSheet("background-color: rgb(21, 57, 86); color: #CCCCCC;")
        
        self._add_sliders()
        self._add_velocity_controls()
        self._add_grid()

    def _add_sliders(self) -> None:
        self.translational_gain_slider = self._add_slider('Translate Gain:', 50)
        self.rotational_gain_slider = self._add_slider('Rotate Gain:', 50)

    def _add_slider(self, label_text: str, default: int) -> QtWidgets.QSlider:
        row = QtWidgets.QHBoxLayout()
        label = QtWidgets.QLabel(label_text)
        label.setFixedWidth(150)
        slider = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        slider.setValue(default)
        slider.setStyleSheet("""
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

        row.addWidget(label)
        row.addWidget(slider)
        self.control_layout.addLayout(row)
        return slider
    
    def _add_velocity_controls(self) -> None:
        for i, label in enumerate(['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']):
            self._add_velocity_control(label, i)

    def _add_velocity_control(self, label: str, axis_index: int) -> None:
        row = QtWidgets.QHBoxLayout()
        _label = QtWidgets.QLabel(label)
        
        btn_minus = QtWidgets.QPushButton('-')
        btn_plus = QtWidgets.QPushButton('+')
        
        row.addWidget(_label)
        row.addWidget(btn_minus)
        row.addWidget(btn_plus)
        
        self.control_layout.addLayout(row)

        btn_minus.pressed.connect(lambda idx=axis_index: self._set_velocity_state(idx, -1, True))
        btn_minus.released.connect(lambda idx=axis_index: self._set_velocity_state(idx, -1, False))
        btn_plus.pressed.connect(lambda idx=axis_index: self._set_velocity_state(idx, 1, True))
        btn_plus.released.connect(lambda idx=axis_index: self._set_velocity_state(idx, 1, False))

    def set_state(self, axis_index: int, direction: int, pressed: bool) -> None:
        self.btn_state[axis_index] = direction if pressed else 0

    def get_ee_velocity_demand(self, t_gain: float, r_gain: float) -> List[float]:
        v_ee = [0.0] * 6
        for i in range(3):
            v_ee[i] = self.btn_state[i] * t_gain
        for i in range(3, 6):
            v_ee[i] = self.btn_state[i] * r_gain
        return v_ee
    
    def _set_velocity_state(self, axis_index: int, direction: int, state: bool) -> None:
        self.set_state(axis_index, direction, state)

    def _add_grid(self) -> None:
        grid = gl.GLGridItem()
        grid.setSize(1,1)
        grid.setSpacing(0.1,0.1)
        self.view.addItem(grid)

    def _init_triads(self) -> None:
        self.triad_transforms = {
            'Global': np.eye(4),
            'Base': np.eye(4),
            'EndEffector': np.eye(4)
        }

        self.triad_arrows = {}
        for name, T in self.triad_transforms.items():
            self.triad_arrows[name] = create_triad(T, size=0.1)
            for arrow in self.triad_arrows[name]:
                self.view.addItem(arrow)

    def _init_update_loop(self) -> None:
        self.loop_timer = QTimer()
        self.loop_timer.timeout.connect(self._update_loop)
        self.loop_timer.start(10)  # ms

    def _update_loop(self) -> None:
        self.manipulator.update()
        self.update_ee_velocity()
        self.update_visualization()

    def update_visualization(self) -> None:
        T_eg = self.manipulator.get_end_effector_transform()
        T_bg = self.manipulator.get_base_transform()

        update_triad(self.triad_arrows['EndEffector'], T_eg)
        update_triad(self.triad_arrows['Base'], T_bg)

    def update_ee_velocity(self) -> None:
        t_gain = self.translational_gain_slider.value() * TRANSLATE_VELOCITY_SCALE * 0.01
        r_gain = self.rotational_gain_slider.value() * ROTATE_VELOCITY_SCALE * 0.01
        v_ee = self.get_ee_velocity_demand(t_gain, r_gain)
        self.manipulator.ee_velocity_global(v_ee)