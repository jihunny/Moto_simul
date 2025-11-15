"""
Realtime motor simulation UI (Qt) with PMSM dq-FOC and BLDC modes.

Features
- Selection: PMSM or BLDC; control: Vector (FOC), Trapezoidal, Six-step (3ph)
- Live plots using pyqtgraph (fallback to matplotlib is not implemented here)
- Space vector (alpha-beta) diagram
- Rolling plots: speed, I/iq, torque, |V|, PWM, Id/Iq, Id/Iq error, phase currents, gate duty
- Preset save/load including plot toggles
"""

from __future__ import annotations

import sys, os, json, argparse, math
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

import pyqtgraph as pg  # type: ignore

from control import MotorParams as PMSMParams, Gains as PMSMGains, PMSMSim
from control.bldc import MotorParamsBLDC, GainsBLDC, BLDCQSim, BLDCTrapSim, BLDCSixStepSim


class LivePlot(QWidget):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.t = []
        self.speed = []
        self.iq = []
        self.torque = []
        self.vmag = []
        self.pwm = []
        self.id_hist, self.iq_hist = [], []
        self.id_err_hist, self.iq_err_hist = [], []
        self.ia_hist, self.ib_hist, self.ic_hist = [], [], []
        self.da, self.db, self.dc = [], [], []

        pg.setConfigOptions(antialias=True)
        layout = QVBoxLayout(self)

        self.space = pg.PlotWidget(title="Space Vector (alpha-beta)")
        self.space.setAspectLocked(True, ratio=1)
        self.space.showGrid(x=True, y=True, alpha=0.3)
        self.sv_v = self.space.plot([], [], pen=pg.mkPen('y', width=3))
        self.sv_e = self.space.plot([], [], pen=pg.mkPen('r', width=2))
        self.space.setVisible(False)
        layout.addWidget(self.space)

        self.p_speed = pg.PlotWidget(title="speed [rpm]")
        self.c_speed = self.p_speed.plot([], [], pen=pg.mkPen('y', width=2))
        layout.addWidget(self.p_speed)

        self.p_iq = pg.PlotWidget(title="I/iq [A]")
        self.c_iq = self.p_iq.plot([], [], pen=pg.mkPen('c', width=2))
        layout.addWidget(self.p_iq)

        self.p_torque = pg.PlotWidget(title="torque [N*m]")
        self.c_torque = self.p_torque.plot([], [], pen=pg.mkPen('m', width=2))
        self.p_torque.setVisible(False)
        layout.addWidget(self.p_torque)

        self.p_vmag = pg.PlotWidget(title="|v| [V]")
        self.c_vmag = self.p_vmag.plot([], [], pen=pg.mkPen('w', width=2))
        self.p_vmag.setVisible(False)
        layout.addWidget(self.p_vmag)

        self.p_pwm = pg.PlotWidget(title="PWM ratio")
        self.c_pwm = self.p_pwm.plot([], [], pen=pg.mkPen('g', width=2))
        self.p_pwm.setVisible(False)
        layout.addWidget(self.p_pwm)

        self.p_idiq = pg.PlotWidget(title="Id/Iq [A]")
        self.c_id = self.p_idiq.plot([], [], pen=pg.mkPen('b', width=2))
        self.c_iq2 = self.p_idiq.plot([], [], pen=pg.mkPen('c', width=2))
        self.p_idiq.setVisible(False)
        layout.addWidget(self.p_idiq)

        self.p_err = pg.PlotWidget(title="Id/Iq error [A]")
        self.c_id_err = self.p_err.plot([], [], pen=pg.mkPen('r', width=2))
        self.c_iq_err = self.p_err.plot([], [], pen=pg.mkPen('y', width=2))
        self.p_err.setVisible(False)
        layout.addWidget(self.p_err)

        self.p_coils = pg.PlotWidget(title="Phase currents i_a/i_b/i_c [A]")
        self.c_ia = self.p_coils.plot([], [], pen=pg.mkPen('r', width=2))
        self.c_ib = self.p_coils.plot([], [], pen=pg.mkPen('g', width=2))
        self.c_ic = self.p_coils.plot([], [], pen=pg.mkPen('b', width=2))
        self.p_coils.setVisible(False)
        layout.addWidget(self.p_coils)

        self.p_duty = pg.PlotWidget(title="Gate duty a/b/c [0..1]")
        self.c_da = self.p_duty.plot([], [], pen=pg.mkPen('r', width=2))
        self.c_db = self.p_duty.plot([], [], pen=pg.mkPen('g', width=2))
        self.c_dc = self.p_duty.plot([], [], pen=pg.mkPen('b', width=2))
        self.p_duty.setVisible(False)
        layout.addWidget(self.p_duty)

    def toggle(self, name: str, on: bool):
        getattr(self, name).setVisible(on)

    def append(self, t: float, rpm: float, iq: float, torque: float, vmag: float, extra: dict | None):
        self.t.append(t); self.speed.append(rpm); self.iq.append(iq); self.torque.append(torque); self.vmag.append(vmag)
        if extra:
            self.pwm.append(extra.get('pwm', 0.0))
            self.id_hist.append(extra.get('id', 0.0)); self.iq_hist.append(extra.get('iq', iq))
            self.id_err_hist.append(extra.get('id_err', 0.0)); self.iq_err_hist.append(extra.get('iq_err', 0.0))
            self.ia_hist.append(extra.get('i_a', 0.0)); self.ib_hist.append(extra.get('i_b', 0.0)); self.ic_hist.append(extra.get('i_c', 0.0))
            self.da.append(extra.get('duty_a', 0.0)); self.db.append(extra.get('duty_b', 0.0)); self.dc.append(extra.get('duty_c', 0.0))
            if self.space.isVisible():
                va = extra.get('v_alpha', 0.0); vb = extra.get('v_beta', 0.0)
                ea = extra.get('e_alpha', 0.0); eb = extra.get('e_beta', 0.0)
                self.sv_v.setData([0, va], [0, vb]); self.sv_e.setData([0, ea], [0, eb])
                r = max(1.0, abs(va), abs(vb), abs(ea), abs(eb))
                self.space.setXRange(-r, r); self.space.setYRange(-r, r)

        # Trim to 10s window
        while self.t and (self.t[-1] - self.t[0]) > 10.0:
            for arr in (self.t, self.speed, self.iq, self.torque, self.vmag, self.pwm,
                        self.id_hist, self.iq_hist, self.id_err_hist, self.iq_err_hist,
                        self.ia_hist, self.ib_hist, self.ic_hist, self.da, self.db, self.dc):
                if arr: arr.pop(0)

        # Update curves
        self.c_speed.setData(self.t, self.speed)
        self.c_iq.setData(self.t, self.iq)
        if self.p_torque.isVisible(): self.c_torque.setData(self.t, self.torque)
        if self.p_vmag.isVisible(): self.c_vmag.setData(self.t, self.vmag)
        if self.p_pwm.isVisible(): self.c_pwm.setData(self.t, self.pwm)
        if self.p_idiq.isVisible(): self.c_id.setData(self.t, self.id_hist); self.c_iq2.setData(self.t, self.iq_hist)
        if self.p_err.isVisible(): self.c_id_err.setData(self.t, self.id_err_hist); self.c_iq_err.setData(self.t, self.iq_err_hist)
        if self.p_coils.isVisible(): self.c_ia.setData(self.t, self.ia_hist); self.c_ib.setData(self.t, self.ib_hist); self.c_ic.setData(self.t, self.ic_hist)
        if self.p_duty.isVisible(): self.c_da.setData(self.t, self.da); self.c_db.setData(self.t, self.db); self.c_dc.setData(self.t, self.dc)


class MainWindow(QMainWindow):
    def __init__(self, preset_path: Optional[str] = None):
        super().__init__()
        self.setWindowTitle("Realtime Motor Sim")
        cw = QWidget(self); self.setCentralWidget(cw)

        form = QFormLayout()
        self.spin_rpm = QDoubleSpinBox(); self.spin_rpm.setRange(0, 200000); self.spin_rpm.setValue(2000)
        self.spin_torque = QDoubleSpinBox(); self.spin_torque.setRange(-5.0, 5.0); self.spin_torque.setDecimals(4); self.spin_torque.setSingleStep(0.01); self.spin_torque.setValue(0.05)
        self.spin_idref = QDoubleSpinBox(); self.spin_idref.setRange(-50.0, 50.0); self.spin_idref.setDecimals(3); self.spin_idref.setSingleStep(0.1); self.spin_idref.setValue(0.0)
        self.spin_dt = QDoubleSpinBox(); self.spin_dt.setRange(1e-5, 0.01); self.spin_dt.setDecimals(6); self.spin_dt.setSingleStep(0.0001); self.spin_dt.setValue(0.0005)
        self.spin_vis = QDoubleSpinBox(); self.spin_vis.setRange(0.01, 0.5); self.spin_vis.setDecimals(3); self.spin_vis.setSingleStep(0.01); self.spin_vis.setValue(0.02)
        self.spin_tload = QDoubleSpinBox(); self.spin_tload.setRange(0.0, 5.0); self.spin_tload.setDecimals(4); self.spin_tload.setSingleStep(0.005); self.spin_tload.setValue(0.0)

        sel_box = QGroupBox("Selection"); sel_row = QHBoxLayout(sel_box)
        self.rb_pmsm = QtWidgets.QRadioButton("PMSM"); self.rb_bldc = QtWidgets.QRadioButton("BLDC"); self.rb_pmsm.setChecked(True)
        self.rb_vec = QtWidgets.QRadioButton("Vector (FOC)"); self.rb_trap = QtWidgets.QRadioButton("Trapezoidal"); self.rb_six = QtWidgets.QRadioButton("Six-step (3ph)"); self.rb_vec.setChecked(True)
        for w in (self.rb_pmsm, self.rb_bldc, self.rb_vec, self.rb_trap, self.rb_six): sel_row.addWidget(w)

        motor_box = QGroupBox("Motor Params"); mform = QFormLayout(motor_box)
        self.spin_R = QDoubleSpinBox(); self.spin_R.setRange(0.01, 50.0); self.spin_R.setDecimals(4); self.spin_R.setValue(0.5)
        self.spin_Ld = QDoubleSpinBox(); self.spin_Ld.setRange(1e-6, 1.0); self.spin_Ld.setDecimals(6); self.spin_Ld.setValue(0.0002)
        self.spin_Lq = QDoubleSpinBox(); self.spin_Lq.setRange(1e-6, 1.0); self.spin_Lq.setDecimals(6); self.spin_Lq.setValue(0.0002)
        self.spin_Kt = QDoubleSpinBox(); self.spin_Kt.setRange(1e-4, 1.0); self.spin_Kt.setDecimals(5); self.spin_Kt.setValue(0.06)
        self.spin_Ke_bldc = QDoubleSpinBox(); self.spin_Ke_bldc.setRange(1e-4, 1.0); self.spin_Ke_bldc.setDecimals(5); self.spin_Ke_bldc.setValue(0.06)
        self.spin_p = QSpinBox(); self.spin_p.setRange(1, 32); self.spin_p.setValue(4)
        self.spin_J = QDoubleSpinBox(); self.spin_J.setRange(1e-6, 1.0); self.spin_J.setDecimals(7); self.spin_J.setValue(2.0e-4)
        self.spin_B = QDoubleSpinBox(); self.spin_B.setRange(0.0, 0.1); self.spin_B.setDecimals(7); self.spin_B.setValue(1.0e-4)
        self.spin_V = QDoubleSpinBox(); self.spin_V.setRange(1.0, 1000.0); self.spin_V.setDecimals(1); self.spin_V.setValue(24.0)
        self.spin_Imax = QDoubleSpinBox(); self.spin_Imax.setRange(0.1, 200.0); self.spin_Imax.setDecimals(2); self.spin_Imax.setValue(10.0)
        for label, w in [("R [Ohm]", self.spin_R), ("Ld [H]", self.spin_Ld), ("Lq [H]", self.spin_Lq),
                         ("Kt [N*m/A]", self.spin_Kt), ("Ke [V*s/rad] (BLDC)", self.spin_Ke_bldc), ("Pole Pairs [p]", self.spin_p),
                         ("J [kg*m^2]", self.spin_J), ("B [N*m*s/rad]", self.spin_B), ("Vbus [V]", self.spin_V), ("Imax [A]", self.spin_Imax)]:
            mform.addRow(label, w)

        gains_box = QGroupBox("FOC Gains (PI)"); gform = QFormLayout(gains_box)
        self.spd_kp = QDoubleSpinBox(); self.spd_kp.setRange(0.0, 1e4); self.spd_kp.setDecimals(4); self.spd_kp.setValue(0.01)
        self.spd_ki = QDoubleSpinBox(); self.spd_ki.setRange(0.0, 1e5); self.spd_ki.setDecimals(4); self.spd_ki.setValue(2.0)
        self.id_kp = QDoubleSpinBox(); self.id_kp.setRange(0.0, 1e4); self.id_kp.setDecimals(4); self.id_kp.setValue(0.8)
        self.id_ki = QDoubleSpinBox(); self.id_ki.setRange(0.0, 1e5); self.id_ki.setDecimals(4); self.id_ki.setValue(120.0)
        self.iq_kp = QDoubleSpinBox(); self.iq_kp.setRange(0.0, 1e4); self.iq_kp.setDecimals(4); self.iq_kp.setValue(0.8)
        self.iq_ki = QDoubleSpinBox(); self.iq_ki.setRange(0.0, 1e5); self.iq_ki.setDecimals(4); self.iq_ki.setValue(120.0)
        for label, w in [("Speed Kp", self.spd_kp), ("Speed Ki", self.spd_ki), ("Id Kp", self.id_kp), ("Id Ki", self.id_ki), ("Iq Kp", self.iq_kp), ("Iq Ki", self.iq_ki)]:
            gform.addRow(label, w)

        self.btn_speed = QtWidgets.QRadioButton("Speed Mode"); self.btn_speed.setChecked(True)
        self.btn_torque = QtWidgets.QRadioButton("Torque Mode")
        form.addRow(self.btn_speed); form.addRow(self.btn_torque)
        form.addRow("Target Speed [rpm]", self.spin_rpm)
        form.addRow("Target Torque [N*m]", self.spin_torque)
        form.addRow("d-axis Current [A]", self.spin_idref)
        form.addRow("Plant dt [s]", self.spin_dt)
        form.addRow("UI period [s]", self.spin_vis)
        form.addRow("Load Torque [N*m]", self.spin_tload)

        self.btn_start = QPushButton("Start"); self.btn_pause = QPushButton("Pause"); self.btn_reset = QPushButton("Reset"); self.btn_export = QPushButton("Export CSV...")
        row = QHBoxLayout(); [row.addWidget(b) for b in (self.btn_start, self.btn_pause, self.btn_reset, self.btn_export)]
        self.lbl = QLabel("Idle")
        self.plot = LivePlot()

        self.btn_save_preset = QPushButton("Save Preset..."); self.btn_load_preset = QPushButton("Load Preset...")
        preset_row = QHBoxLayout(); preset_row.addWidget(self.btn_save_preset); preset_row.addWidget(self.btn_load_preset)

        left = QVBoxLayout(); left.addLayout(form); left.addWidget(sel_box); left.addWidget(motor_box); left.addWidget(gains_box); left.addLayout(row); left.addLayout(preset_row); left.addWidget(self.lbl)
        root = QHBoxLayout(cw); root.addLayout(left, 0); root.addWidget(self.plot, 1)

        menubar = self.menuBar(); m_file = menubar.addMenu("File")
        act_load = QtWidgets.QAction("Load Preset...", self); act_save = QtWidgets.QAction("Save Preset...", self)
        act_load_default = QtWidgets.QAction("Load Default Preset", self); act_exit = QtWidgets.QAction("Exit", self)
        for a in (act_load, act_save, act_load_default, act_exit): m_file.addAction(a)

        self.sim: Optional[object] = None; self.running = False; self.t = 0.0; self._csv = []
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.on_tick)

        self.btn_start.clicked.connect(self.on_start); self.btn_pause.clicked.connect(self.on_pause); self.btn_reset.clicked.connect(self.on_reset); self.btn_export.clicked.connect(self.on_export)
        self.btn_save_preset.clicked.connect(self.on_save_preset); self.btn_load_preset.clicked.connect(self.on_load_preset)
        act_save.triggered.connect(self.on_save_preset); act_load.triggered.connect(self.on_load_preset); act_load_default.triggered.connect(self.on_load_default_preset); act_exit.triggered.connect(self.close)

        # Plot toggles
        toggles = QHBoxLayout()
        self.chk_space = QPushButton("Space Vector"); self.chk_space.setCheckable(True)
        self.chk_pwm = QPushButton("PWM"); self.chk_pwm.setCheckable(True)
        self.chk_idiq = QPushButton("Id/Iq"); self.chk_idiq.setCheckable(True)
        self.chk_err = QPushButton("Id/Iq error"); self.chk_err.setCheckable(True)
        self.chk_coils = QPushButton("Phase Currents"); self.chk_coils.setCheckable(True)
        self.chk_duty = QPushButton("Gate Duty"); self.chk_duty.setCheckable(True)
        self.chk_torque = QPushButton("Torque"); self.chk_torque.setCheckable(True)
        self.chk_vmag = QPushButton("|V|"); self.chk_vmag.setCheckable(True)
        for b in (self.chk_space, self.chk_pwm, self.chk_idiq, self.chk_err, self.chk_coils, self.chk_duty, self.chk_torque, self.chk_vmag): toggles.addWidget(b)
        left.addLayout(toggles)

        self.chk_space.toggled.connect(lambda on: self.plot.toggle('space', on))
        self.chk_pwm.toggled.connect(lambda on: self.plot.toggle('p_pwm', on))
        self.chk_idiq.toggled.connect(lambda on: self.plot.toggle('p_idiq', on))
        self.chk_err.toggled.connect(lambda on: self.plot.toggle('p_err', on))
        self.chk_coils.toggled.connect(lambda on: self.plot.toggle('p_coils', on))
        self.chk_duty.toggled.connect(lambda on: self.plot.toggle('p_duty', on))
        self.chk_torque.toggled.connect(lambda on: self.plot.toggle('p_torque', on))
        self.chk_vmag.toggled.connect(lambda on: self.plot.toggle('p_vmag', on))

        # Autoload preset
        if preset_path:
            self._load_preset_from_path(preset_path)
        else:
            default_path = os.path.join(os.path.dirname(__file__), "presets", "default_spmsm.json")
            if os.path.exists(default_path): self._load_preset_from_path(default_path)

    def build_params(self):
        if self.rb_bldc.isChecked():
            p = MotorParamsBLDC(R=self.spin_R.value(), L=self.spin_Ld.value(), Kt=self.spin_Kt.value(), Ke=self.spin_Ke_bldc.value(), p=int(self.spin_p.value()), J=self.spin_J.value(), B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value())
            g = GainsBLDC(spd_kp=self.spd_kp.value(), spd_ki=self.spd_ki.value(), cur_kp=self.id_kp.value(), cur_ki=self.id_ki.value())
            return p, g
        else:
            p = PMSMParams(R=self.spin_R.value(), Ld=self.spin_Ld.value(), Lq=self.spin_Lq.value(), Kt=self.spin_Kt.value(), p=int(self.spin_p.value()), J=self.spin_J.value(), B=self.spin_B.value(), Vbus=self.spin_V.value(), Imax=self.spin_Imax.value())
            g = PMSMGains(spd_kp=self.spd_kp.value(), spd_ki=self.spd_ki.value(), id_kp=self.id_kp.value(), id_ki=self.id_ki.value(), iq_kp=self.iq_kp.value(), iq_ki=self.iq_ki.value())
            return p, g

    def on_start(self):
        if self.sim is None:
            p, g = self.build_params()
            if self.rb_bldc.isChecked():
                if self.rb_vec.isChecked(): self.sim = BLDCQSim(p, g)
                elif self.rb_trap.isChecked(): self.sim = BLDCTrapSim(p, g)
                else: self.sim = BLDCSixStepSim(p, g)
            else:
                self.sim = PMSMSim(p, g)
        if isinstance(self.sim, PMSMSim): self.sim.id_ref = self.spin_idref.value()
        self.running = True; self.timer.start(int(self.spin_vis.value() * 1000)); self.lbl.setText("Running...")

    def on_pause(self):
        self.running = False; self.timer.stop(); self.lbl.setText("Paused")

    def on_reset(self):
        if self.sim and hasattr(self.sim, 'reset'): self.sim.reset()
        self.t = 0.0; self._csv.clear(); self.plot.t.clear(); self.plot.speed.clear(); self.plot.iq.clear()
        self.lbl.setText("Reset")

    def on_export(self):
        if not self._csv:
            QMessageBox.information(self, "Export CSV", "Run the simulation first."); return
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "realtime.csv", "CSV Files (*.csv)")
        if not path: return
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s","omega_rad_s","speed_rpm","iq_a","id_a","vd_v","vq_v","Te_nm","v_mag_v"])
            w.writerows(self._csv)
        QMessageBox.information(self, "Export CSV", f"Saved: {path}")

    def _load_preset_from_path(self, path: str):
        with open(path, "r", encoding="utf-8") as f: d = json.load(f)
        m = d.get("motor", {})
        self.spin_R.setValue(float(m.get("R", self.spin_R.value())))
        self.spin_Ld.setValue(float(m.get("Ld", self.spin_Ld.value())))
        self.spin_Lq.setValue(float(m.get("Lq", self.spin_Lq.value())))
        self.spin_Kt.setValue(float(m.get("Kt", self.spin_Kt.value())))
        if "Ke" in m: self.spin_Ke_bldc.setValue(float(m.get("Ke")))
        self.spin_p.setValue(int(m.get("p", int(self.spin_p.value()))))
        self.spin_J.setValue(float(m.get("J", self.spin_J.value())))
        self.spin_B.setValue(float(m.get("B", self.spin_B.value())))
        self.spin_V.setValue(float(m.get("Vbus", self.spin_V.value())))
        self.spin_Imax.setValue(float(m.get("Imax", self.spin_Imax.value())))

        g = d.get("gains", {})
        self.spd_kp.setValue(float(g.get("spd_kp", self.spd_kp.value())))
        self.spd_ki.setValue(float(g.get("spd_ki", self.spd_ki.value())))
        self.id_kp.setValue(float(g.get("id_kp", self.id_kp.value())))
        self.id_ki.setValue(float(g.get("id_ki", self.id_ki.value())))
        self.iq_kp.setValue(float(g.get("iq_kp", self.iq_kp.value())))
        self.iq_ki.setValue(float(g.get("iq_ki", self.iq_ki.value())))

        t = d.get("targets", {})
        self.spin_rpm.setValue(float(t.get("rpm", self.spin_rpm.value())))
        self.spin_torque.setValue(float(t.get("torque", self.spin_torque.value())))
        self.spin_idref.setValue(float(t.get("id_ref", self.spin_idref.value())))

        sel = d.get("selection", {})
        (self.rb_bldc if sel.get("motor", "PMSM").upper() == "BLDC" else self.rb_pmsm).setChecked(True)
        ctrl = sel.get("control", "vector").lower()
        if ctrl == "trapezoidal": self.rb_trap.setChecked(True)
        elif ctrl == "sixstep": self.rb_six.setChecked(True)
        else: self.rb_vec.setChecked(True)

        plots = d.get("plots", {})
        def set_opt(btn, key):
            val = plots.get(key)
            if isinstance(val, bool): btn.setChecked(val)
        set_opt(self.chk_space, 'space'); set_opt(self.chk_pwm, 'pwm'); set_opt(self.chk_idiq, 'idiq'); set_opt(self.chk_err, 'err')
        set_opt(self.chk_coils, 'coils'); set_opt(self.chk_duty, 'duty'); set_opt(self.chk_torque, 'torque'); set_opt(self.chk_vmag, 'vmag')

    def _preset_dict(self):
        return {
            "motor": {
                "R": self.spin_R.value(), "Ld": self.spin_Ld.value(), "Lq": self.spin_Lq.value(),
                "Kt": self.spin_Kt.value(), "Ke": self.spin_Ke_bldc.value(), "p": int(self.spin_p.value()), "J": self.spin_J.value(),
                "B": self.spin_B.value(), "Vbus": self.spin_V.value(), "Imax": self.spin_Imax.value(),
            },
            "gains": {
                "spd_kp": self.spd_kp.value(), "spd_ki": self.spd_ki.value(),
                "id_kp": self.id_kp.value(), "id_ki": self.id_ki.value(),
                "iq_kp": self.iq_kp.value(), "iq_ki": self.iq_ki.value(),
            },
            "targets": {
                "rpm": self.spin_rpm.value(), "torque": self.spin_torque.value(), "id_ref": self.spin_idref.value(),
            },
            "selection": {
                "motor": "BLDC" if self.rb_bldc.isChecked() else "PMSM",
                "control": "sixstep" if self.rb_six.isChecked() else ("trapezoidal" if self.rb_trap.isChecked() else "vector"),
            },
            "plots": {
                "space": self.chk_space.isChecked(), "pwm": self.chk_pwm.isChecked(), "idiq": self.chk_idiq.isChecked(),
                "err": self.chk_err.isChecked(), "coils": self.chk_coils.isChecked(), "duty": self.chk_duty.isChecked(),
                "torque": self.chk_torque.isChecked(), "vmag": self.chk_vmag.isChecked(),
            },
        }

    def on_save_preset(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Preset", "motor_preset.json", "JSON Files (*.json)")
        if not path: return
        with open(path, "w", encoding="utf-8") as f: json.dump(self._preset_dict(), f, indent=2)

    def on_load_preset(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Preset", "", "JSON Files (*.json)")
        if not path: return
        try:
            self._load_preset_from_path(path)
        except Exception as e:
            QMessageBox.critical(self, "Load Preset", str(e))

    def on_load_default_preset(self):
        path = os.path.join(os.path.dirname(__file__), "presets", "default_spmsm.json")
        if not os.path.exists(path):
            QMessageBox.information(self, "Load Default Preset", "Default preset not found under presets/default_spmsm.json"); return
        self._load_preset_from_path(path)

    def on_tick(self):
        if not self.running or self.sim is None: return
        dt = self.spin_dt.value(); steps = max(1, int(self.spin_vis.value() / dt))
        mode = "speed" if self.btn_speed.isChecked() else "torque"; rpm_target = self.spin_rpm.value(); torque_ref = self.spin_torque.value(); tload = self.spin_tload.value()
        last = None
        for _ in range(steps):
            if isinstance(self.sim, PMSMSim):
                omega, id_, iq_, vd, vq, Te, vmag = self.sim.step(dt, mode, rpm_target, torque_ref, tload)
                self.t += dt; last = (self.t, omega, id_, iq_, vd, vq, Te, vmag)
                self._csv.append((self.t, omega, omega * 60.0 / (2.0 * math.pi), iq_, id_, vd, vq, Te, vmag))
            elif isinstance(self.sim, BLDCQSim):
                omega, i, v, Te = self.sim.step(dt, mode, rpm_target, torque_ref, tload)
                self.t += dt; rpm = omega * 60.0 / (2.0 * math.pi); last = (self.t, omega, 0.0, i, 0.0, v, Te, abs(v))
                self._csv.append((self.t, omega, rpm, i, 0.0, 0.0, v, Te, abs(v)))
            else:
                res = self.sim.step(dt, mode, rpm_target, torque_ref, tload)
                try:
                    omega, i, v, Te, duty = res; i_equiv = i; vmag = abs(v)
                except Exception:
                    omega, currents, vph, Te, duty = res
                    ia, ib, ic = currents; va, vb, vc = vph
                    i_equiv = (abs(ia) + abs(ib) + abs(ic)) / 3.0; vmag = (abs(va) + abs(vb) + abs(vc)) / 3.0
                self.t += dt; rpm = omega * 60.0 / (2.0 * math.pi); last = (self.t, omega, 0.0, i_equiv, 0.0, vmag, Te, vmag)
                self._csv.append((self.t, omega, rpm, i_equiv, 0.0, 0.0, vmag, Te, vmag))
        if last is None: return
        t, omega, id_or0, iq_or_i, vd_or0, vq_or_v, Te, vmag = last; rpm = omega * 60.0 / (2.0 * math.pi)
        extra = {}
        if isinstance(self.sim, PMSMSim) and hasattr(self.sim, 'extra'):
            ex = self.sim.extra; extra = {**ex, 'id': id_or0, 'iq': iq_or_i}
        self.plot.append(t, rpm, iq_or_i, Te, vmag, extra)
        self.lbl.setText(f"t={t:.2f}s rpm={rpm:.0f} I/iq={iq_or_i:.2f}A Te={Te:.3f}N*m |v|={vmag:.1f}V")


def main() -> int:
    parser = argparse.ArgumentParser(description="Realtime Motor Simulation UI")
    parser.add_argument("--preset", type=str, default=None, help="Path to preset JSON to load on startup")
    args = parser.parse_args()
    app = QApplication(sys.argv); w = MainWindow(preset_path=args.preset); w.resize(1100, 650); w.show(); return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

