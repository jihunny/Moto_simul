# Repository Guidelines

## Project Structure & Module Organization
- Root-level Python modules: `config.py`, `foc.py`, `motor_model.py`, `motor_sim.py`, `plotter.py`, `utils.py`.
- Reference notes: `Motor_simul.md`.
- Suggested additions (when needed): `tests/` for automated tests, `assets/` for input data or figures.

## Build, Test, and Development Commands
- Run simulation: `python motor_sim.py`
- Plot/analysis: `python plotter.py`
- Lint/format (optional, recommended): `ruff check .`, `ruff format .` or `black .`
- Environment setup example:
  - Create venv: `python -m venv .venv && . .venv/Scripts/activate` (Windows)
  - Upgrade pip: `python -m pip install --upgrade pip`

## Coding Style & Naming Conventions
- Python 3.10+ recommended; follow PEP 8 with 4-space indentation.
- Functions/methods: `snake_case`; classes: `PascalCase`; constants: `UPPER_SNAKE_CASE`.
- Prefer type hints and concise docstrings (Google or NumPy style).
- Keep modules focused (one clear responsibility per file). Extract helpers to `utils.py`.

## Testing Guidelines
- Framework: `pytest` is recommended.
- Location: put tests under `tests/` mirroring module names.
- Naming: files `test_*.py`; functions `test_*`.
- Run tests: `pytest -q` (add `-k name` to filter). Aim for coverage of core math/model logic.

## Commit & Pull Request Guidelines
- Commits: use Conventional Commits (e.g., `feat: add FOC current loop`, `fix: handle zero-speed edge case`).
- Scope small and atomic; describe what and why.
- PRs: include a short description, screenshots/plots if relevant, reproduction or run steps, and link related issues.
- Checks: ensure code runs (`python motor_sim.py`) and plots generate without errors before requesting review.

## Security & Configuration Tips
- Do not commit secrets or large binary assets. Use `.gitignore` for local artifacts (venvs, `__pycache__`, data dumps).
- If adding dependencies, pin versions in `requirements.txt` and document install steps in `README.md`.
