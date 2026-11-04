#!/usr/bin/env bash
set -euo pipefail
compare_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
compare_python="$compare_root/../roban2-visual-console/.venv/bin/python"
if [[ ! -x "$compare_python" ]]; then
    compare_python="${ROBAN_COMPARE_PYTHON:-python3}"
fi
exec "$compare_python" "$compare_root/service.py" "${1:-start}"
