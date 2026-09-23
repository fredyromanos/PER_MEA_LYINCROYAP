#!/usr/bin/env bash
# Lanza la IHM de escritorio. Crea el venv la primera vez.
set -e
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
    echo "Création du venv et installation des dépendances…"
    python3 -m venv .venv
    .venv/bin/pip install --quiet --upgrade pip
    .venv/bin/pip install --quiet -r requirements.txt
fi

exec .venv/bin/python ihm_desktop.py "$@"
