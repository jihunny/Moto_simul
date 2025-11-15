# Motor Simulator (PMSM dq‑FOC)

A lightweight Python simulator for a Permanent Magnet Synchronous Motor (PMSM) with dq‑frame Field Oriented Control (FOC). Includes a Qt UI for interactive runs and plots.

## Features
- PMSM model: dq electrical dynamics, electromagnetic torque, mechanical load.
- FOC: speed PI → i_q,ref; current PIs for i_d/i_q with decoupling and bus voltage limit.
- Modes: speed control or torque control; optional CSV export.
- Qt UI (PySide6 or PyQt5) with optional matplotlib plots.

## Quickstart
- Python 3.9+ recommended.
- Create venv (Windows): `python -m venv .venv && . .venv/Scripts/activate`
- Install UI deps (optional): `pip install PySide6 matplotlib`

Run CLI examples:
- Speed: `python motor_sim.py --mode speed --rpm 2000 --duration 1.5 --dt 0.0005 --csv out.csv`
- Torque: `python motor_sim.py --mode torque --torque 0.2 --duration 1.0 --csv out_torque.csv`

Run Qt UI:
- `python qt_ui.py`
- Edit targets, motor params (R, L≈Ld=Lq, Kt, pole pairs p), and run.

Realtime UI:
- `python realtime_ui.py` for live simulation and plotting (Qt + Matplotlib).
  - Controls: start/pause/reset, speed/torque mode, d‑axis current, load torque.
  - Update rate set by "UI period"; plant integrates at smaller dt.

## Output
- CSV columns: `t_s, omega_rad_s, speed_rpm, iq_a, iq_ref_a, v_mag_v, t_load_nm, speed_err, iq_err, id_a, id_ref_a, vd_v, vq_v, Te_nm`.
- Console prints final speed, currents, voltage magnitude, and torque.

## Screenshots
- Main window: `docs/ui_main.png`
- Plot example: `docs/plot_example.png`
(Add PNGs at those paths to include them here.)

## Notes & Tuning
- Gains in `motor_sim.py` are conservative; tune `spd_pi`, `id_pi`, `iq_pi` for your motor.
- For SPMSM, Ld≈Lq; set `id_ref = 0`. For IPMSM, set Ld≠Lq and experiment with negative `id_ref`.
- Voltage is limited to `Vbus/√3` to mimic sine/SVPWM fundamental.

## Repo Structure
- `motor_sim.py` (offline sim), `qt_ui.py` (batch UI), `realtime_ui.py` (live UI)
- `control/` (PI, FOC helpers, PMSM model)
- `AGENTS.md` (contrib guide), `README.md`, CI workflow
