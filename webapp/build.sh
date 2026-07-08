#!/usr/bin/env bash
# Build the static, serverless browser app (Shiny for Python -> shinylive/Pyodide).
#
# Rather than shipping a wheel (micropip local-wheel handling is brittle in shinylive),
# we vendor the pypowertrain source straight into the app dir so it lands on the
# Pyodide VFS sys.path and imports with no micropip step. The compute core needs only
# numpy + scipy, both built into Pyodide.
#
# Usage:  bash webapp/build.sh        # run from repo root
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
APPDIR="$REPO/webapp"
SITE="$REPO/webapp_site"

echo ">> vendoring pypowertrain source into $APPDIR/pypowertrain"
rm -rf "$APPDIR/pypowertrain"
rsync -a \
  --exclude '__pycache__' \
  --exclude '.ipynb_checkpoints' \
  --exclude '*.pyc' \
  --exclude 'test' --exclude 'tests' \
  --exclude 'app.py' --exclude 'app_odrive.py' --exclude 'launch.py' \
  "$REPO/pypowertrain/" "$APPDIR/pypowertrain/"

echo ">> exporting static site to $SITE"
shinylive export "$APPDIR" "$SITE"

echo ">> done. Serve with:"
echo "   python -m http.server --directory webapp_site --bind localhost 8008"