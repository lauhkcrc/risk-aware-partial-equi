#!/usr/bin/env bash
set -euo pipefail

CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
export PYTHONPATH="$CODEBASE_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" "$CODEBASE_ROOT/tests/test_r2_contract.py"
