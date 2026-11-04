#!/usr/bin/env bash
set -euo pipefail
console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$console_root"
console_python="$console_root/.venv/bin/python"
console_pidfile="$console_root/state/server.pid"
mkdir -p state/records
case "${1:-start}" in
  start)
    "$console_python" tools/service.py start
    ;;
  stop)
    "$console_python" tools/service.py stop
    ;;
  status)
    "$console_python" tools/service.py status
    ;;
  foreground)
    shift
    exec "$console_python" server.py "$@"
    ;;
  camera-start|camera-stop|camera-status)
    exec "$console_python" tools/camera_service.py "${1#camera-}"
    ;;
  test)
    exec "$console_python" -m unittest discover -s tests -v
    ;;
  *)
    echo "Usage: $0 {start|stop|status|foreground|camera-start|camera-stop|camera-status|test}"
    exit 2
    ;;
esac
