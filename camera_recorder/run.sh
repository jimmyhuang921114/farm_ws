#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_DIR/.venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating virtual environment..."

    python3 -m venv "$VENV_DIR"

    source "$VENV_DIR/bin/activate"

    python -m pip install --upgrade pip
    python -m pip install -r "$PROJECT_DIR/requirements.txt"
else
    source "$VENV_DIR/bin/activate"
fi

exec python "$PROJECT_DIR/calibrate_charuco.py" "$@"