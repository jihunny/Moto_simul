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
import os
import argparse

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

# Plotting: prefer pyqtgraph for smooth realtime; fallback to matplotlib
HAS_PG = False
HAS_MPL = False
try:
    import pyqtgraph as pg  # type: ignore
    HAS_PG = True
except Exception:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
        HAS_MPL = True
    except Exception:
        HAS_MPL = False

from control import MotorParams, Gains, PMSMSim
from control.bldc import MotorParamsBLDC, GainsBLDC, BLDCQSim


class LivePlot(QWidget):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.tdata = []
        self.rpm = []
        self.iq = []
        if HAS_PG:
            pg.setConfigOptions(antialias=True)
            self.plot_rpm = pg.PlotWidget(title="speed [rpm]")
            self.plot_iq = pg.PlotWidget(title="i_q [A]")
            self.cur_rpm = self.plot_rpm.plot([], [], pen=pg.mkPen('y', width=2))
            self.cur_iq = self.plot_iq.plot([], [], pen=pg.mkPen('c', width=2))
            layout.addWidget(self.plot_rpm)
            layout.addWidget(self.plot_iq)
        elif HAS_MPL:
            try:
                matplotlib.use("Qt5Agg")
            except Exception:
                pass
            self.fig, self.ax = plt.subplots(2, 1, figsize=(6, 5), constrained_layout=True)
            self.canvas = FigureCanvas(self.fig)
            layout.addWidget(self.canvas)
            (self.l_rpm,) = self.ax[0].plot([], [], label="speed [rpm]")
            self.ax[0].set_xlabel("t [s]"); self.ax[0].set_ylabel("rpm"); self.ax[0].grid(True); self.ax[0].legend()
            (self.l_iq,) = self.ax[1].plot([], [], color="tab:red", label="i_q [A]")
            self.ax[1].set_xlabel("t [s]"); self.ax[1].set_ylabel("A"); self.ax[1].grid(True); self.ax[1].legend()
        else:
            self.msg = QLabel("Install pyqtgraph or matplotlib for plots")
            layout.addWidget(self.msg)

    def append(self, t: float, rpm: float, iq: float, max_window: float = 10.0):
        if not (HAS_PG or HAS_MPL):
            return
        self.tdata.append(t)
        self.rpm.append(rpm)
        self.iq.append(iq)
        # Keep a sliding window
        while self.tdata and (self.tdata[-1] - self.tdata[0]) > max_window:
            self.tdata.pop(0); self.rpm.pop(0); self.iq.pop(0)
        if HAS_PG:
            self.cur_rpm.setData(self.tdata, self.rpm)
            self.cur_iq.setData(self.tdata, self.iq)
            self.plot_rpm.enableAutoRange()
            self.plot_iq.enableAutoRange()
        elif HAS_MPL:
            self.l_rpm.set_data(self.tdata, self.rpm)
            self.l_iq.set_data(self.tdata, self.iq)
            for ax in self.ax:
                ax.relim(); ax.autoscale_view()
            self.canvas.draw_idle()


class MainWindow(QMainWindow):
    def __init__(self, preset_path: Optional[str] = None):
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

        # Motor/Control selection
        sel_box = QGroupBox("Selection")
        sel_row = QHBoxLayout(sel_box)
        self.rb_pmsm = QtWidgets.QRadioButton("PMSM")
        self.rb_bldc = QtWidgets.QRadioButton("BLDC")
        self.rb_pmsm.setChecked(True)
        self.rb_vec = QtWidgets.QRadioButton("Vector (FOC)")
        self.rb_trap = QtWidgets.QRadioButton("Trapezoidal (six-step)")
        self.rb_vec.setChecked(True)
        self.rb_trap.setEnabled(False)  # Placeholder for future
        for w in (self.rb_pmsm, self.rb_bldc, self.rb_vec, self.rb_trap):
            sel_row.addWidget(w)

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

        # Gains box
        gains_box = QGroupBox("FOC Gains (PI)")
        gform = QFormLayout(gains_box)
        self.spin_spd_kp = QDoubleSpinBox(); self.spin_spd_kp.setRange(0.0, 1000.0); self.spin_spd_kp.setDecimals(4); self.spin_spd_kp.setValue(0.01)
        self.spin_spd_ki = QDoubleSpinBox(); self.spin_spd_ki.setRange(0.0, 5000.0); self.spin_spd_ki.setDecimals(4); self.spin_spd_ki.setValue(2.0)
        self.spin_id_kp = QDoubleSpinBox(); self.spin_id_kp.setRange(0.0, 1000.0); self.spin_id_kp.setDecimals(4); self.spin_id_kp.setValue(0.8)
        self.spin_id_ki = QDoubleSpinBox(); self.spin_id_ki.setRange(0.0, 5000.0); self.spin_id_ki.setDecimals(4); self.spin_id_ki.setValue(120.0)
        self.spin_iq_kp = QDoubleSpinBox(); self.spin_iq_kp.setRange(0.0, 1000.0); self.spin_iq_kp.setDecimals(4); self.spin_iq_kp.setValue(0.8)
        self.spin_iq_ki = QDoubleSpinBox(); self.spin_iq_ki.setRange(0.0, 5000.0); self.spin_iq_ki.setDecimals(4); self.spin_iq_ki.setValue(120.0)
        for label, w in [("Speed Kp", self.spin_spd_kp), ("Speed Ki", self.spin_spd_ki), ("Id Kp", self.spin_id_kp), ("Id Ki", self.spin_id_ki), ("Iq Kp", self.spin_iq_kp), ("Iq Ki", self.spin_iq_ki)]:
            gform.addRow(label, w)

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

        # Preset buttons
        self.btn_save_preset = QPushButton("Save Preset…"); self.btn_load_preset = QPushButton("Load Preset…")
        preset_row = QHBoxLayout(); preset_row.addWidget(self.btn_save_preset); preset_row.addWidget(self.btn_load_preset)

        left = QVBoxLayout(); left.addLayout(form); left.addWidget(sel_box); left.addWidget(motor_box); left.addWidget(gains_box); left.addLayout(row); left.addLayout(preset_row); left.addWidget(self.lbl)
        root = QHBoxLayout(cw); root.addLayout(left, 0); root.addWidget(self.plot, 1)

        # Menu bar (presets, exit)
        menubar = self.menuBar()
        m_file = menubar.addMenu("File")
        act_load = QtWidgets.QAction("Load Preset…", self)
        act_save = QtWidgets.QAction("Save Preset…", self)
        act_load_default = QtWidgets.QAction("Load Default Preset", self)
        act_exit = QtWidgets.QAction("Exit", self)
        m_file.addAction(act_load)
        m_file.addAction(act_save)
        m_file.addSeparator()
        m_file.addAction(act_load_default)
        m_file.addSeparator()
        m_file.addAction(act_exit)

        # Simulation objects
        self.sim: Optional[object] = None
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
        self.btn_save_preset.clicked.connect(self.on_save_preset)
        self.btn_load_preset.clicked.connect(self.on_load_preset)
        act_save.triggered.connect(self.on_save_preset)
        act_load.triggered.connect(self.on_load_preset)
        act_load_default.triggered.connect(self.on_load_default_preset)
        act_exit.triggered.connect(self.close)

        # Optional preset autoload
        if preset_path:
            self._load_preset_from_path(preset_path)
        else:
            # Auto-load default preset if present
            default_path = os.path.join(os.path.dirname(__file__), "presets", "default_spmsm.json")
            if os.path.exists(default_path):
                self._load_preset_from_path(default_path)
                self.lbl.setText(f"Loaded default preset: {default_path}")

    def build_params(self):
        if self.rb_bldc.isChecked():
            p = MotorParamsBLDC(
                R=self.spin_R.value(), L=self.spin_Ld.value(), Kt=self.spin_Kt.value(), Ke=self.spin_Kt.value(),
                J=self.spin_J.value(), B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value(),
            )
            g = GainsBLDC(
                spd_kp=self.spin_spd_kp.value(), spd_ki=self.spin_spd_ki.value(),
                cur_kp=self.spin_id_kp.value(), cur_ki=self.spin_id_ki.value(),
            )
            return p, g
        else:
            p = MotorParams(
                R=self.spin_R.value(), Ld=self.spin_Ld.value(), Lq=self.spin_Lq.value(),
                Kt=self.spin_Kt.value(), p=int(self.spin_p.value()), J=self.spin_J.value(),
                B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value(),
            )
            g = Gains(
                spd_kp=self.spin_spd_kp.value(), spd_ki=self.spin_spd_ki.value(),
                id_kp=self.spin_id_kp.value(), id_ki=self.spin_id_ki.value(),
                iq_kp=self.spin_iq_kp.value(), iq_ki=self.spin_iq_ki.value(),
            )
            return p, g

    def on_start(self):
        if self.sim is None:
            p, g = self.build_params()
            if self.rb_bldc.isChecked():
                self.sim = BLDCQSim(p, g)
            else:
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

    def _load_preset_from_path(self, path: str):
        import json
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        m = d.get("motor", {})
        self.spin_R.setValue(float(m.get("R", self.spin_R.value())))
        self.spin_Ld.setValue(float(m.get("Ld", self.spin_Ld.value())))
        self.spin_Lq.setValue(float(m.get("Lq", self.spin_Lq.value())))
        self.spin_Kt.setValue(float(m.get("Kt", self.spin_Kt.value())))
        self.spin_p.setValue(int(m.get("p", int(self.spin_p.value()))))
        self.spin_J.setValue(float(m.get("J", self.spin_J.value())))
        self.spin_B.setValue(float(m.get("B", self.spin_B.value())))
        self.spin_V.setValue(float(m.get("Vbus", self.spin_V.value())))
        self.spin_Imax.setValue(float(m.get("Imax", self.spin_Imax.value())))

        g = d.get("gains", {})
        self.spin_spd_kp.setValue(float(g.get("spd_kp", self.spin_spd_kp.value())))
        self.spin_spd_ki.setValue(float(g.get("spd_ki", self.spin_spd_ki.value())))
        self.spin_id_kp.setValue(float(g.get("id_kp", self.spin_id_kp.value())))
        self.spin_id_ki.setValue(float(g.get("id_ki", self.spin_id_ki.value())))
        self.spin_iq_kp.setValue(float(g.get("iq_kp", self.spin_iq_kp.value())))
        self.spin_iq_ki.setValue(float(g.get("iq_ki", self.spin_iq_ki.value())))

        t = d.get("targets", {})
        self.spin_rpm.setValue(float(t.get("rpm", self.spin_rpm.value())))
        self.spin_torque.setValue(float(t.get("torque", self.spin_torque.value())))
        self.spin_idref.setValue(float(t.get("id_ref", self.spin_idref.value())))

    def _preset_dict(self):
        return {
            "motor": {
                "R": self.spin_R.value(), "Ld": self.spin_Ld.value(), "Lq": self.spin_Lq.value(),
                "Kt": self.spin_Kt.value(), "p": int(self.spin_p.value()), "J": self.spin_J.value(),
                "B": self.spin_B.value(), "Vbus": self.spin_V.value(), "Imax": self.spin_Imax.value(),
            },
            "gains": {
                "spd_kp": self.spin_spd_kp.value(), "spd_ki": self.spin_spd_ki.value(),
                "id_kp": self.spin_id_kp.value(), "id_ki": self.spin_id_ki.value(),
                "iq_kp": self.spin_iq_kp.value(), "iq_ki": self.spin_iq_ki.value(),
            },
            "targets": {
                "rpm": self.spin_rpm.value(), "torque": self.spin_torque.value(),
                "id_ref": self.spin_idref.value(),
            },
        }

    def on_save_preset(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Preset", "motor_preset.json", "JSON Files (*.json)")
        if not path:
            return
        try:
            import json
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._preset_dict(), f, indent=2)
        except Exception as e:
            QMessageBox.critical(self, "Save preset error", str(e))

    def on_load_preset(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Preset", "", "JSON Files (*.json)")
        if not path:
            return
        try:
            self._load_preset_from_path(path)
        except Exception as e:
            QMessageBox.critical(self, "Load preset error", str(e))

    def on_load_default_preset(self):
        default_path = os.path.join(os.path.dirname(__file__), "presets", "default_spmsm.json")
        if not os.path.exists(default_path):
            QMessageBox.information(self, "Load Default Preset", "Default preset not found under presets/default_spmsm.json")
            return
        try:
            self._load_preset_from_path(default_path)
            self.lbl.setText(f"Loaded default preset: {default_path}")
        except Exception as e:
            QMessageBox.critical(self, "Load preset error", str(e))

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
            if isinstance(self.sim, PMSMSim):
                omega, id_, iq_, vd, vq, Te, vmag = self.sim.step(dt, mode, rpm_target, torque_ref, tload)
                self.t += dt
                last = (self.t, omega, id_, iq_, vd, vq, Te, vmag)
                self._csv_log.append((self.t, omega, omega * 60.0 / (2.0 * 3.141592653589793), iq_, id_, vd, vq, Te, vmag))
            else:
                omega, i, v, Te = self.sim.step(dt, mode, rpm_target, torque_ref, tload)
                self.t += dt
                rpm = omega * 60.0 / (2.0 * 3.141592653589793)
                last = (self.t, omega, 0.0, i, 0.0, v, Te, abs(v))
                self._csv_log.append((self.t, omega, rpm, i, 0.0, 0.0, v, Te, abs(v)))

        if last is not None:
            t, omega, id_or0, iq_or_i, vd_or0, vq_or_v, Te, vmag = last
            rpm = omega * 60.0 / (2.0 * 3.141592653589793)
            self.plot.append(t, rpm, iq_or_i)
            self.lbl.setText(f"t={t:.2f}s rpm={rpm:.0f} I/iq={iq_or_i:.2f}A Te={Te:.3f}N·m |v|={vmag:.1f}V")


def main() -> int:
    parser = argparse.ArgumentParser(description="Realtime PMSM dq‑FOC simulation UI")
    parser.add_argument("--preset", type=str, default=None, help="Path to preset JSON to load on startup")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    w = MainWindow(preset_path=args.preset); w.resize(1100, 650); w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
