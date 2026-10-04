#!/usr/bin/env bash
# Run script for EV Motor Digital Twin Dashboard
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

# Check if virtualenv exists, otherwise create it
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv .venv
    source .venv/bin/activate
    echo "Installing dependencies..."
    pip install -r requirements.txt
else
    source .venv/bin/activate
fi

echo "Launching EV Motor Digital Twin Dashboard on http://localhost:8501..."
streamlit run app.py
