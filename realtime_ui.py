"""
Realtime PMSM dq‑FOC simulation with Qt visualization.

Controls
- Mode: Speed or Torque
- Targets: rpm or torque, d‑axis current
- Start, Pause, Reset; CSV export

Visualization
- Live Matplotlib plot of speed [rpm] and i_q [A]

Requires: PySide6 or PyQt5; matplotlib (optional but recommended)
"""

from __future__ import annotations

import sys
from typing import Optional
import time

try:
    from PySide6 import QtWidgets
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QFormLayout, QDoubleSpinBox,
        QSpinBox, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QFileDialog,
        QMessageBox, QGroupBox
    )
    from PySide6 import QtCore
    PYSIDE = True
except Exception:
    from PyQt5 import QtWidgets  # type: ignore
    from PyQt5.QtWidgets import (  # type: ignore
        QApplication, QMainWindow, QWidget, QFormLayout, QDoubleSpinBox,
        QSpinBox, QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QFileDialog,
        QMessageBox, QGroupBox
    )
    from PyQt5 import QtCore  # type: ignore
    PYSIDE = False

# Matplotlib (optional)
HAS_MPL = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    HAS_MPL = True
except Exception:
    HAS_MPL = False

from control import MotorParams, Gains, PMSMSim


class LivePlot(QWidget):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        if HAS_MPL:
            try:
                matplotlib.use("Qt5Agg")
            except Exception:
                pass
            self.fig, self.ax = plt.subplots(2, 1, figsize=(6, 5), constrained_layout=True)
            self.canvas = FigureCanvas(self.fig)
            layout.addWidget(self.canvas)
            # Lines
            self.tdata = []
            self.rpm = []
            self.iq = []
            (self.l_rpm,) = self.ax[0].plot([], [], label="speed [rpm]")
            self.ax[0].set_xlabel("t [s]"); self.ax[0].set_ylabel("rpm"); self.ax[0].grid(True); self.ax[0].legend()
            (self.l_iq,) = self.ax[1].plot([], [], color="tab:red", label="i_q [A]")
            self.ax[1].set_xlabel("t [s]"); self.ax[1].set_ylabel("A"); self.ax[1].grid(True); self.ax[1].legend()
        else:
            self.msg = QLabel("matplotlib not installed — plotting disabled")
            layout.addWidget(self.msg)

    def append(self, t: float, rpm: float, iq: float, max_window: float = 10.0):
        if not HAS_MPL:
            return
        self.tdata.append(t)
        self.rpm.append(rpm)
        self.iq.append(iq)
        # Keep a sliding window
        while self.tdata and (self.tdata[-1] - self.tdata[0]) > max_window:
            self.tdata.pop(0); self.rpm.pop(0); self.iq.pop(0)
        self.l_rpm.set_data(self.tdata, self.rpm)
        self.l_iq.set_data(self.tdata, self.iq)
        # Rescale axes
        for ax in self.ax:
            ax.relim(); ax.autoscale_view()
        self.canvas.draw_idle()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Realtime PMSM dq‑FOC Sim")
        cw = QWidget(self); self.setCentralWidget(cw)

        # Controls
        form = QFormLayout()

        self.spin_rpm = QDoubleSpinBox(); self.spin_rpm.setRange(0, 200000); self.spin_rpm.setValue(2000)
        self.spin_torque = QDoubleSpinBox(); self.spin_torque.setRange(-5.0, 5.0); self.spin_torque.setDecimals(4); self.spin_torque.setSingleStep(0.01); self.spin_torque.setValue(0.05)
        self.spin_idref = QDoubleSpinBox(); self.spin_idref.setRange(-50.0, 50.0); self.spin_idref.setDecimals(3); self.spin_idref.setSingleStep(0.1); self.spin_idref.setValue(0.0)
        self.spin_dt = QDoubleSpinBox(); self.spin_dt.setRange(1e-5, 0.01); self.spin_dt.setDecimals(6); self.spin_dt.setSingleStep(0.0001); self.spin_dt.setValue(0.0005)
        self.spin_vis = QDoubleSpinBox(); self.spin_vis.setRange(0.01, 0.5); self.spin_vis.setDecimals(3); self.spin_vis.setSingleStep(0.01); self.spin_vis.setValue(0.02)
        self.spin_tload = QDoubleSpinBox(); self.spin_tload.setRange(0.0, 5.0); self.spin_tload.setDecimals(4); self.spin_tload.setSingleStep(0.005); self.spin_tload.setValue(0.0)

        motor_box = QGroupBox("Motor Params")
        mform = QFormLayout(motor_box)
        self.spin_R = QDoubleSpinBox(); self.spin_R.setRange(0.01, 50.0); self.spin_R.setDecimals(4); self.spin_R.setValue(0.5)
        self.spin_Ld = QDoubleSpinBox(); self.spin_Ld.setRange(1e-6, 1.0); self.spin_Ld.setDecimals(6); self.spin_Ld.setValue(0.0002)
        self.spin_Lq = QDoubleSpinBox(); self.spin_Lq.setRange(1e-6, 1.0); self.spin_Lq.setDecimals(6); self.spin_Lq.setValue(0.0002)
        self.spin_Kt = QDoubleSpinBox(); self.spin_Kt.setRange(1e-4, 1.0); self.spin_Kt.setDecimals(5); self.spin_Kt.setValue(0.06)
        self.spin_p = QSpinBox(); self.spin_p.setRange(1, 32); self.spin_p.setValue(4)
        self.spin_J = QDoubleSpinBox(); self.spin_J.setRange(1e-6, 1.0); self.spin_J.setDecimals(7); self.spin_J.setValue(2.0e-4)
        self.spin_B = QDoubleSpinBox(); self.spin_B.setRange(0.0, 0.1); self.spin_B.setDecimals(7); self.spin_B.setValue(1.0e-4)
        self.spin_V = QDoubleSpinBox(); self.spin_V.setRange(1.0, 1000.0); self.spin_V.setDecimals(1); self.spin_V.setValue(24.0)
        self.spin_Imax = QDoubleSpinBox(); self.spin_Imax.setRange(0.1, 200.0); self.spin_Imax.setDecimals(2); self.spin_Imax.setValue(10.0)

        for label, w in [
            ("R [Ω]", self.spin_R), ("Ld [H]", self.spin_Ld), ("Lq [H]", self.spin_Lq),
            ("Kt [N·m/A]", self.spin_Kt), ("Pole Pairs [p]", self.spin_p), ("J [kg·m²]", self.spin_J),
            ("B [N·m·s/rad]", self.spin_B), ("Vbus [V]", self.spin_V), ("Imax [A]", self.spin_Imax)
        ]:
            mform.addRow(label, w)

        # Mode selector via buttons
        self.btn_speed = QtWidgets.QRadioButton("Speed Mode"); self.btn_speed.setChecked(True)
        self.btn_torque = QtWidgets.QRadioButton("Torque Mode")

        form.addRow(self.btn_speed)
        form.addRow(self.btn_torque)
        form.addRow("Target Speed [rpm]", self.spin_rpm)
        form.addRow("Target Torque [N·m]", self.spin_torque)
        form.addRow("d-axis Current [A]", self.spin_idref)
        form.addRow("Plant dt [s]", self.spin_dt)
        form.addRow("UI period [s]", self.spin_vis)
        form.addRow("Load Torque [N·m]", self.spin_tload)

        self.btn_start = QPushButton("Start"); self.btn_pause = QPushButton("Pause"); self.btn_reset = QPushButton("Reset")
        self.btn_export = QPushButton("Export CSV…")
        row = QHBoxLayout(); row.addWidget(self.btn_start); row.addWidget(self.btn_pause); row.addWidget(self.btn_reset); row.addWidget(self.btn_export)
        self.lbl = QLabel("Idle")

        self.plot = LivePlot()

        left = QVBoxLayout(); left.addLayout(form); left.addWidget(motor_box); left.addLayout(row); left.addWidget(self.lbl)
        root = QHBoxLayout(cw); root.addLayout(left, 0); root.addWidget(self.plot, 1)

        # Simulation objects
        self.sim: Optional[PMSMSim] = None
        self.running = False
        self.t = 0.0
        self._csv_log = []

        # Timer
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.on_tick)

        # Signals
        self.btn_start.clicked.connect(self.on_start)
        self.btn_pause.clicked.connect(self.on_pause)
        self.btn_reset.clicked.connect(self.on_reset)
        self.btn_export.clicked.connect(self.on_export)

    def build_params(self) -> tuple[MotorParams, Gains]:
        p = MotorParams(
            R=self.spin_R.value(), Ld=self.spin_Ld.value(), Lq=self.spin_Lq.value(),
            Kt=self.spin_Kt.value(), p=int(self.spin_p.value()), J=self.spin_J.value(),
            B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value(),
        )
        g = Gains()
        return p, g

    def on_start(self):
        if self.sim is None:
            p, g = self.build_params()
            self.sim = PMSMSim(p, g)
        # Apply refs
        self.sim.id_ref = self.spin_idref.value()
        self.running = True
        period = self.spin_vis.value()
        self.timer.start(int(period * 1000))
        self.lbl.setText("Running…")

    def on_pause(self):
        self.running = False
        self.timer.stop()
        self.lbl.setText("Paused")

    def on_reset(self):
        if self.sim is not None:
            self.sim.reset()
        self.t = 0.0
        self.plot.tdata.clear(); self.plot.rpm.clear(); self.plot.iq.clear()
        self._csv_log.clear()
        self.plot.append(0.0, 0.0, 0.0)
        self.lbl.setText("Reset")

    def on_export(self):
        if not self._csv_log:
            QMessageBox.information(self, "Export CSV", "Run the simulation first.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "realtime.csv", "CSV Files (*.csv)")
        if not path:
            return
        try:
            import csv
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s","omega_rad_s","speed_rpm","iq_a","id_a","vd_v","vq_v","Te_nm","v_mag_v"])
                w.writerows(self._csv_log)
        except Exception as e:
            QMessageBox.critical(self, "Save error", str(e))
            return
        QMessageBox.information(self, "Export CSV", f"Saved: {path}")

    def on_tick(self):
        if not self.running or self.sim is None:
            return
        vis_period = self.spin_vis.value()
        dt = self.spin_dt.value()
        steps = max(1, int(vis_period / dt))
        mode = "speed" if self.btn_speed.isChecked() else "torque"
        rpm_target = self.spin_rpm.value()
        torque_ref = self.spin_torque.value()
        tload = self.spin_tload.value()

        last = None
        for _ in range(steps):
            omega, id_, iq_, vd, vq, Te, vmag = self.sim.step(dt, mode, rpm_target, torque_ref, tload)
            self.t += dt
            last = (self.t, omega, id_, iq_, vd, vq, Te, vmag)
            # Log every step; for large runs, consider thinning
            self._csv_log.append((self.t, omega, omega * 60.0 / (2.0 * 3.141592653589793), iq_, id_, vd, vq, Te, vmag))

        if last is not None:
            t, omega, id_, iq_, vd, vq, Te, vmag = last
            rpm = omega * 60.0 / (2.0 * 3.141592653589793)
            self.plot.append(t, rpm, iq_)
            self.lbl.setText(f"t={t:.2f}s rpm={rpm:.0f} iq={iq_:.2f}A Te={Te:.3f}N·m |v|={vmag:.1f}V")


def main() -> int:
    app = QApplication(sys.argv)
    w = MainWindow(); w.resize(1100, 650); w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

