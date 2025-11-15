"""
Qt UI for motor_sim.py simulation.

Features
- Editable simulation parameters (speed, load, time, dt)
- Run simulation and view summary
- Plot results if matplotlib is available
- Export results to CSV

Requirements
- PySide6 (preferred) or PyQt5
- Optional: matplotlib for inline plotting

Run
  python qt_ui.py
"""

from __future__ import annotations

import sys
from typing import Optional

HAS_QT = False
PYSIDE = False
try:  # Prefer PySide6
    from PySide6 import QtWidgets, QtCore
    from PySide6.QtWidgets import (
        QApplication,
        QMainWindow,
        QWidget,
        QFormLayout,
        QDoubleSpinBox,
        QSpinBox,
        QPushButton,
        QLabel,
        QHBoxLayout,
        QVBoxLayout,
        QFileDialog,
        QMessageBox,
        QGroupBox,
        QCheckBox,
    )
    PYSIDE = True
    HAS_QT = True
except Exception:
    try:  # Fallback to PyQt5
        from PyQt5 import QtWidgets, QtCore  # type: ignore
        from PyQt5.QtWidgets import (  # type: ignore
            QApplication,
            QMainWindow,
            QWidget,
            QFormLayout,
            QDoubleSpinBox,
            QSpinBox,
            QPushButton,
            QLabel,
            QHBoxLayout,
            QVBoxLayout,
            QFileDialog,
            QMessageBox,
            QGroupBox,
            QCheckBox,
        )
        PYSIDE = False
        HAS_QT = True
    except Exception:
        HAS_QT = False


# Optional matplotlib embedding
HAS_MPL = False
try:
    import matplotlib
    matplotlib.use("Agg")  # Safe default; will switch to Qt backend after app init
    import matplotlib.pyplot as plt

    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

    HAS_MPL = True
except Exception:
    HAS_MPL = False

from motor_sim import MotorParams, SimConfig, simulate, write_csv, rad_s_to_rpm


if HAS_QT:
    class PlotWidget(QWidget):
        def __init__(self, parent: Optional[QWidget] = None):
            super().__init__(parent)
            layout = QVBoxLayout(self)
            if HAS_MPL:
                # Switch backend to Qt now that app exists
                try:
                    matplotlib.use("Qt5Agg")
                except Exception:
                    pass

                self.fig, self.ax = plt.subplots(2, 1, figsize=(6, 5), constrained_layout=True)
                self.canvas = FigureCanvas(self.fig)
                layout.addWidget(self.canvas)
            else:
                self.msg = QLabel("matplotlib not installed — plotting disabled")
                layout.addWidget(self.msg)

        def update_plot(self, log):
            if not HAS_MPL:
                return
            t = [row[0] for row in log]
            rpm = [row[2] for row in log]
            ia = [row[3] for row in log]

            ax0, ax1 = self.ax
            ax0.clear(); ax1.clear()
            ax0.plot(t, rpm, label="speed [rpm]")
            ax0.set_xlabel("t [s]")
            ax0.set_ylabel("rpm")
            ax0.grid(True)
            ax0.legend()

            ax1.plot(t, ia, color="tab:red", label="current [A]")
            ax1.set_xlabel("t [s]")
            ax1.set_ylabel("A")
            ax1.grid(True)
            ax1.legend()

            self.canvas.draw_idle()


    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("BLDC FOC Sim — Qt UI")
            cw = QWidget(self)
            self.setCentralWidget(cw)

            # Controls
            form = QFormLayout()

            self.spin_rpm = QDoubleSpinBox(); self.spin_rpm.setRange(0, 100000); self.spin_rpm.setValue(2000)
            self.spin_dur = QDoubleSpinBox(); self.spin_dur.setRange(0.01, 60); self.spin_dur.setDecimals(3); self.spin_dur.setSingleStep(0.1); self.spin_dur.setValue(1.5)
            self.spin_dt = QDoubleSpinBox(); self.spin_dt.setRange(1e-5, 0.01); self.spin_dt.setDecimals(6); self.spin_dt.setSingleStep(0.0001); self.spin_dt.setValue(0.0005)
            self.spin_tload = QDoubleSpinBox(); self.spin_tload.setRange(0.0, 5.0); self.spin_tload.setDecimals(4); self.spin_tload.setSingleStep(0.005); self.spin_tload.setValue(0.05)
            self.spin_tstep = QDoubleSpinBox(); self.spin_tstep.setRange(0.0, 60.0); self.spin_tstep.setDecimals(3); self.spin_tstep.setSingleStep(0.05); self.spin_tstep.setValue(0.3)
            # Torque reference (used in torque mode)
            self.spin_tref = QDoubleSpinBox(); self.spin_tref.setRange(-5.0, 5.0); self.spin_tref.setDecimals(4); self.spin_tref.setSingleStep(0.01); self.spin_tref.setValue(0.05)
            # d-axis current reference
            self.spin_idref = QDoubleSpinBox(); self.spin_idref.setRange(-50.0, 50.0); self.spin_idref.setDecimals(3); self.spin_idref.setSingleStep(0.05); self.spin_idref.setValue(0.0)

            # Motor params group
            motor_box = QGroupBox("Motor Params")
            motor_form = QFormLayout(motor_box)
            self.spin_R = QDoubleSpinBox(); self.spin_R.setRange(0.01, 50.0); self.spin_R.setDecimals(4); self.spin_R.setValue(0.5)
            self.spin_L = QDoubleSpinBox(); self.spin_L.setRange(1e-6, 1.0); self.spin_L.setDecimals(6); self.spin_L.setValue(0.0002)
            self.spin_Kt = QDoubleSpinBox(); self.spin_Kt.setRange(1e-4, 1.0); self.spin_Kt.setDecimals(5); self.spin_Kt.setValue(0.06)
            self.spin_Ke = QDoubleSpinBox(); self.spin_Ke.setRange(1e-4, 1.0); self.spin_Ke.setDecimals(5); self.spin_Ke.setValue(0.06)
            self.spin_J = QDoubleSpinBox(); self.spin_J.setRange(1e-6, 1.0); self.spin_J.setDecimals(7); self.spin_J.setValue(2.0e-4)
            self.spin_B = QDoubleSpinBox(); self.spin_B.setRange(0.0, 0.1); self.spin_B.setDecimals(7); self.spin_B.setValue(1.0e-4)
            self.spin_V = QDoubleSpinBox(); self.spin_V.setRange(1.0, 1000.0); self.spin_V.setDecimals(1); self.spin_V.setValue(24.0)
            self.spin_Imax = QDoubleSpinBox(); self.spin_Imax.setRange(0.1, 200.0); self.spin_Imax.setDecimals(2); self.spin_Imax.setValue(10.0)
            motor_form.addRow("R [Ω]", self.spin_R)
            motor_form.addRow("L [H]", self.spin_L)
            motor_form.addRow("Kt [N·m/A]", self.spin_Kt)
            motor_form.addRow("Ke [V·s/rad]", self.spin_Ke)
            motor_form.addRow("J [kg·m²]", self.spin_J)
            motor_form.addRow("B [N·m·s/rad]", self.spin_B)
            motor_form.addRow("Vbus [V]", self.spin_V)
            motor_form.addRow("Imax [A]", self.spin_Imax)
            # Additional PMSM param: pole pairs
            self.spin_p = QSpinBox(); self.spin_p.setRange(1, 32); self.spin_p.setValue(4)
            motor_form.addRow("Pole Pairs [p]", self.spin_p)

            # Mode selection
            mode_box = QGroupBox("Control Mode")
            mode_row = QHBoxLayout(mode_box)
            self.btn_mode_speed = QtWidgets.QRadioButton("Speed")
            self.btn_mode_torque = QtWidgets.QRadioButton("Torque")
            self.btn_mode_speed.setChecked(True)
            mode_row.addWidget(self.btn_mode_speed)
            mode_row.addWidget(self.btn_mode_torque)

            form.addRow(mode_box)
            form.addRow("Target Speed [rpm]", self.spin_rpm)
            form.addRow("Target Torque [N·m]", self.spin_tref)
            form.addRow("d-axis Current [A]", self.spin_idref)
            form.addRow("Duration [s]", self.spin_dur)
            form.addRow("dt [s]", self.spin_dt)
            form.addRow("Load Torque [N·m]", self.spin_tload)
            form.addRow("Load Step [s]", self.spin_tstep)

            self.btn_run = QPushButton("Run Simulation")
            self.btn_save = QPushButton("Export CSV")
            self.chk_plot = QCheckBox("Auto-plot after run")
            self.chk_plot.setChecked(True)
            btn_row = QHBoxLayout()
            btn_row.addWidget(self.btn_run)
            btn_row.addWidget(self.btn_save)
            btn_row.addWidget(self.chk_plot)

            self.lbl_status = QLabel("Ready")

            # Plot area
            self.plot = PlotWidget()

            left_col = QVBoxLayout()
            left_col.addLayout(form)
            left_col.addWidget(motor_box)
            left_col.addLayout(btn_row)
            left_col.addWidget(self.lbl_status)

            root = QHBoxLayout(cw)
            root.addLayout(left_col, 0)
            root.addWidget(self.plot, 1)

            # State
            self._log = None

            # Signals
            self.btn_run.clicked.connect(self.on_run)
            self.btn_save.clicked.connect(self.on_save)
            self.btn_mode_speed.toggled.connect(self.on_mode_change)
            self.on_mode_change()  # initialize visibility

        def read_config(self) -> tuple[MotorParams, SimConfig]:
            params = MotorParams(
                R=self.spin_R.value(),
                L=self.spin_L.value(),
                Kt=self.spin_Kt.value(),
                J=self.spin_J.value(),
                B=self.spin_B.value(),
                Vbus=self.spin_V.value(),
                Imax=self.spin_Imax.value(),
                p=int(self.spin_p.value()),
            )
            cfg = SimConfig(
                duration=self.spin_dur.value(),
                dt=self.spin_dt.value(),
                mode="speed" if self.btn_mode_speed.isChecked() else "torque",
                rpm_target=self.spin_rpm.value(),
                torque_ref=self.spin_tref.value(),
                t_load_step=self.spin_tstep.value(),
                t_load=self.spin_tload.value(),
                id_ref=self.spin_idref.value(),
            )
            return params, cfg

        def on_mode_change(self):
            # Toggle visibility of target controls based on mode
            is_speed = self.btn_mode_speed.isChecked()
            self.spin_rpm.setEnabled(is_speed)
            self.spin_tref.setEnabled(not is_speed)

        def on_run(self):
            params, cfg = self.read_config()
            self.lbl_status.setText("Running...")
            QApplication.processEvents()
            try:
                self._log = simulate(params, cfg)
            except Exception as e:
                QMessageBox.critical(self, "Simulation error", str(e))
                self.lbl_status.setText("Error")
                return

            # Summary (align with extended log schema)
            last = self._log[-1]
            speed_rpm = last[2]; iq = last[3]; vmag = last[5]; t_load = last[6]
            self.lbl_status.setText(
                f"Final: {speed_rpm:.1f} rpm, iq={iq:.2f} A, |v|={vmag:.2f} V, load {t_load:.3f} N·m"
            )

            if HAS_MPL and self.chk_plot.isChecked():
                self.plot.update_plot(self._log)

        def on_save(self):
            if not self._log:
                QMessageBox.information(self, "Export CSV", "No data to export")
                return
            path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "results.csv", "CSV Files (*.csv)")
            if not path:
                return
            try:
                write_csv(path, self._log)
            except Exception as e:
                QMessageBox.critical(self, "Save error", str(e))
                return
            QMessageBox.information(self, "Export CSV", "Saved successfully")


def main() -> int:
    # If Qt bindings are not available, run a headless simulation fallback.
    if not HAS_QT:
        print("No Qt bindings (PySide6/PyQt5) found. Running headless simulation...")
        params = MotorParams()
        cfg = SimConfig()
        log = simulate(params, cfg)
        last = log[-1]
        speed_rpm = last[2]; iq = last[3]; vmag = last[5]; t_load = last[6]
        print(f"Final: {speed_rpm:.1f} rpm, iq={iq:.2f} A, |v|={vmag:.2f} V, load {t_load:.3f} N·m")
        return 0

    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(1000, 600)
    w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())










