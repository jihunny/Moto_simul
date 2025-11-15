"""
Qt UI (batch) for running a single simulation and plotting the results.
Supports PMSM dq-FOC (via motor_sim.simulate) and BLDC (vector/trapezoidal).
Run: python qt_ui.py
"""

from __future__ import annotations

import sys
from typing import Optional

try:
    from PySide6 import QtWidgets, QtCore
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QFormLayout, QDoubleSpinBox, QSpinBox,
        QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QFileDialog, QMessageBox, QGroupBox, QCheckBox
    )
except Exception:
    from PyQt5 import QtWidgets, QtCore  # type: ignore
    from PyQt5.QtWidgets import (  # type: ignore
        QApplication, QMainWindow, QWidget, QFormLayout, QDoubleSpinBox, QSpinBox,
        QPushButton, QLabel, QHBoxLayout, QVBoxLayout, QFileDialog, QMessageBox, QGroupBox, QCheckBox
    )

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

from motor_sim import MotorParams, SimConfig, simulate, write_csv, rad_s_to_rpm
from control.bldc import MotorParamsBLDC, GainsBLDC, BLDCQSim, BLDCTrapSim


class PlotWidget(QWidget):
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
        else:
            layout.addWidget(QLabel("matplotlib not installed — plotting disabled"))

    def update_plot(self, log):
        if not HAS_MPL:
            return
        t = [row[0] for row in log]
        rpm = [row[2] for row in log]
        ia = [row[3] for row in log]
        ax0, ax1 = self.ax
        ax0.clear(); ax1.clear()
        ax0.plot(t, rpm, label="speed [rpm]"); ax0.set_xlabel("t [s]"); ax0.set_ylabel("rpm"); ax0.grid(True); ax0.legend()
        ax1.plot(t, ia, color="tab:red", label="current [A]"); ax1.set_xlabel("t [s]"); ax1.set_ylabel("A"); ax1.grid(True); ax1.legend()
        self.canvas.draw_idle()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Sim — Qt UI")
        cw = QWidget(self); self.setCentralWidget(cw)

        form = QFormLayout()
        self.spin_rpm = QDoubleSpinBox(); self.spin_rpm.setRange(0, 100000); self.spin_rpm.setValue(2000)
        self.spin_dur = QDoubleSpinBox(); self.spin_dur.setRange(0.01, 60); self.spin_dur.setDecimals(3); self.spin_dur.setSingleStep(0.1); self.spin_dur.setValue(1.5)
        self.spin_dt = QDoubleSpinBox(); self.spin_dt.setRange(1e-5, 0.01); self.spin_dt.setDecimals(6); self.spin_dt.setSingleStep(0.0001); self.spin_dt.setValue(0.0005)
        self.spin_tload = QDoubleSpinBox(); self.spin_tload.setRange(0.0, 5.0); self.spin_tload.setDecimals(4); self.spin_tload.setSingleStep(0.005); self.spin_tload.setValue(0.05)
        self.spin_tstep = QDoubleSpinBox(); self.spin_tstep.setRange(0.0, 60.0); self.spin_tstep.setDecimals(3); self.spin_tstep.setSingleStep(0.05); self.spin_tstep.setValue(0.3)
        self.spin_tref = QDoubleSpinBox(); self.spin_tref.setRange(-5.0, 5.0); self.spin_tref.setDecimals(4); self.spin_tref.setSingleStep(0.01); self.spin_tref.setValue(0.05)
        self.spin_idref = QDoubleSpinBox(); self.spin_idref.setRange(-50.0, 50.0); self.spin_idref.setDecimals(3); self.spin_idref.setSingleStep(0.05); self.spin_idref.setValue(0.0)

        # Selection
        sel_box = QGroupBox("Selection"); sel_row = QHBoxLayout(sel_box)
        self.rb_pmsm = QtWidgets.QRadioButton("PMSM"); self.rb_bldc = QtWidgets.QRadioButton("BLDC"); self.rb_pmsm.setChecked(True)
        self.rb_vec = QtWidgets.QRadioButton("Vector (FOC)"); self.rb_trap = QtWidgets.QRadioButton("Trapezoidal"); self.rb_vec.setChecked(True)
        for w in (self.rb_pmsm, self.rb_bldc, self.rb_vec, self.rb_trap): sel_row.addWidget(w)

        motor_box = QGroupBox("Motor Params"); motor_form = QFormLayout(motor_box)
        self.spin_R = QDoubleSpinBox(); self.spin_R.setRange(0.01, 50.0); self.spin_R.setDecimals(4); self.spin_R.setValue(0.5)
        self.spin_L = QDoubleSpinBox(); self.spin_L.setRange(1e-6, 1.0); self.spin_L.setDecimals(6); self.spin_L.setValue(0.0002)
        self.spin_Kt = QDoubleSpinBox(); self.spin_Kt.setRange(1e-4, 1.0); self.spin_Kt.setDecimals(5); self.spin_Kt.setValue(0.06)
        self.spin_Ke = QDoubleSpinBox(); self.spin_Ke.setRange(1e-4, 1.0); self.spin_Ke.setDecimals(5); self.spin_Ke.setValue(0.06)
        self.spin_J = QDoubleSpinBox(); self.spin_J.setRange(1e-6, 1.0); self.spin_J.setDecimals(7); self.spin_J.setValue(2.0e-4)
        self.spin_B = QDoubleSpinBox(); self.spin_B.setRange(0.0, 0.1); self.spin_B.setDecimals(7); self.spin_B.setValue(1.0e-4)
        self.spin_V = QDoubleSpinBox(); self.spin_V.setRange(1.0, 1000.0); self.spin_V.setDecimals(1); self.spin_V.setValue(24.0)
        self.spin_Imax = QDoubleSpinBox(); self.spin_Imax.setRange(0.1, 200.0); self.spin_Imax.setDecimals(2); self.spin_Imax.setValue(10.0)
        self.spin_p = QSpinBox(); self.spin_p.setRange(1, 32); self.spin_p.setValue(4)
        for label, w in [("R [Ω]", self.spin_R), ("L [H]", self.spin_L), ("Kt [N·m/A]", self.spin_Kt), ("Ke [V·s/rad]", self.spin_Ke), ("J [kg·m²]", self.spin_J), ("B [N·m·s/rad]", self.spin_B), ("Vbus [V]", self.spin_V), ("Imax [A]", self.spin_Imax), ("Pole Pairs [p]", self.spin_p)]: motor_form.addRow(label, w)

        mode_box = QGroupBox("Control Mode"); mode_row = QHBoxLayout(mode_box)
        self.btn_mode_speed = QtWidgets.QRadioButton("Speed"); self.btn_mode_torque = QtWidgets.QRadioButton("Torque"); self.btn_mode_speed.setChecked(True)
        mode_row.addWidget(self.btn_mode_speed); mode_row.addWidget(self.btn_mode_torque)

        form.addRow(sel_box); form.addRow(mode_box)
        form.addRow("Target Speed [rpm]", self.spin_rpm)
        form.addRow("Target Torque [N·m]", self.spin_tref)
        form.addRow("d-axis Current [A]", self.spin_idref)
        form.addRow("Duration [s]", self.spin_dur)
        form.addRow("Δt [s]", self.spin_dt)
        form.addRow("Load Torque [N·m]", self.spin_tload)
        form.addRow("Load Step [s]", self.spin_tstep)

        self.btn_run = QPushButton("Run Simulation"); self.btn_save = QPushButton("Export CSV"); self.chk_plot = QCheckBox("Auto-plot after run"); self.chk_plot.setChecked(True)
        btn_row = QHBoxLayout(); btn_row.addWidget(self.btn_run); btn_row.addWidget(self.btn_save); btn_row.addWidget(self.chk_plot)
        self.lbl_status = QLabel("Ready")
        self.plot = PlotWidget()

        left_col = QVBoxLayout(); left_col.addLayout(form); left_col.addWidget(motor_box); left_col.addLayout(btn_row); left_col.addWidget(self.lbl_status)
        root = QHBoxLayout(cw); root.addLayout(left_col, 0); root.addWidget(self.plot, 1)

        self._log = []
        self.btn_run.clicked.connect(self.on_run); self.btn_save.clicked.connect(self.on_save); self.btn_mode_speed.toggled.connect(self.on_mode_change)
        self.on_mode_change()

    def on_mode_change(self):
        is_speed = self.btn_mode_speed.isChecked(); self.spin_rpm.setEnabled(is_speed); self.spin_tref.setEnabled(not is_speed)

    def read_config(self):
        if self.rb_bldc.isChecked():
            params = MotorParamsBLDC(R=self.spin_R.value(), L=self.spin_L.value(), Kt=self.spin_Kt.value(), Ke=self.spin_Ke.value(), p=int(self.spin_p.value()), J=self.spin_J.value(), B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value())
            return params, None
        else:
            params = MotorParams(R=self.spin_R.value(), L=self.spin_L.value(), Kt=self.spin_Kt.value(), J=self.spin_J.value(), B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value(), p=int(self.spin_p.value()))
            cfg = SimConfig(duration=self.spin_dur.value(), dt=self.spin_dt.value(), mode="speed" if self.btn_mode_speed.isChecked() else "torque", rpm_target=self.spin_rpm.value(), torque_ref=self.spin_tref.value(), t_load_step=self.spin_tstep.value(), t_load=self.spin_tload.value(), id_ref=self.spin_idref.value())
            return params, cfg

    def on_run(self):
        params, cfg = self.read_config(); self.lbl_status.setText("Running..."); QApplication.processEvents()
        if isinstance(params, MotorParamsBLDC):
            duration = self.spin_dur.value(); dt = self.spin_dt.value(); steps = int(duration / dt)
            mode = "speed" if self.btn_mode_speed.isChecked() else "torque"; rpm_target = self.spin_rpm.value(); torque_ref = self.spin_tref.value(); t_load_step = self.spin_tstep.value(); t_load = self.spin_tload.value()
            sim = BLDCQSim(params, GainsBLDC()) if self.rb_vec.isChecked() else BLDCTrapSim(params, GainsBLDC())
            t = 0.0; log = []
            for _ in range(steps + 1):
                load = t_load if t >= t_load_step else 0.0
                res = sim.step(dt, mode, rpm_target, torque_ref, load)
                if isinstance(sim, BLDCQSim):
                    omega, i, v, Te = res
                else:
                    omega, i, v, Te, duty = res
                rpm = rad_s_to_rpm(omega); vmag = abs(v)
                log.append((t, omega, rpm, i, 0.0, vmag, load, 0.0, 0.0, 0.0, 0.0, 0.0, v, Te))
                t += dt
            self._log = log
        else:
            try:
                self._log = simulate(params, cfg)
            except Exception as e:
                QMessageBox.critical(self, "Simulation error", str(e)); self.lbl_status.setText("Error"); return

        last = self._log[-1]; speed_rpm = last[2]; iq = last[3]; vmag = last[5]; t_load = last[6]
        self.lbl_status.setText(f"Final: {speed_rpm:.1f} rpm, iq={iq:.2f} A, |v|={vmag:.2f} V, load {t_load:.3f} N·m")
        if HAS_MPL and self.chk_plot.isChecked(): self.plot.update_plot(self._log)

    def on_save(self):
        if not self._log:
            QMessageBox.information(self, "Export CSV", "No data to export"); return
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "results.csv", "CSV Files (*.csv)")
        if not path: return
        try:
            write_csv(path, self._log)
        except Exception as e:
            QMessageBox.critical(self, "Save error", str(e)); return
        QMessageBox.information(self, "Export CSV", "Saved successfully")


def main() -> int:
    app = QApplication(sys.argv); w = MainWindow(); w.resize(1000, 600); w.show(); return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

