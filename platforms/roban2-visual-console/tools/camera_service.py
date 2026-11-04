#!/usr/bin/env python3
"""Start/stop only the console's isolated camera launch, never robot control."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from server import ROOT, ssh_args


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['start', 'stop', 'status'])
    a = p.parse_args()
    c = json.loads((ROOT / 'config.json').read_text())
    # Existing, inspected, camera-only launch. Do not invoke a robot launch file.
    script = '''set -e
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/ros_ws/devel/setup.bash
camera_dir=/home/leju_kuavo/roban2-visual-console-camera
camera_launch=/home/leju_kuavo/roban2-stage0-test/launch/roban2_gemini_test.launch
mkdir -p "$camera_dir"
camera_pid=''
if [ -f "$camera_dir/owned.pid" ]; then camera_pid=$(cat "$camera_dir/owned.pid"); fi
camera_owned=false
if [[ "$camera_pid" =~ ^[0-9]+$ ]] && [ -r "/proc/$camera_pid/cmdline" ]; then
  if tr '\\0' ' ' < "/proc/$camera_pid/cmdline" | grep -Fq "$camera_launch"; then camera_owned=true; fi
fi
'''
    if a.action == 'start':
        script += '''if $camera_owned; then echo "CAMERA_ALREADY_OWNED pid=$camera_pid"; exit 0; fi
if rostopic list | grep -qE '/(color|rgb)/image_raw$'; then
  echo CAMERA_EXISTING_REUSED; exit 0
fi
setsid roslaunch "$camera_launch" </dev/null >>"$camera_dir/camera.log" 2>&1 &
camera_pid=$!
echo "$camera_pid" > "$camera_dir/owned.pid"
echo "CAMERA_STARTED pid=$camera_pid log=$camera_dir/camera.log"
'''
    elif a.action == 'stop':
        script += '''if $camera_owned; then kill -INT -- "-$camera_pid"; echo CAMERA_STOP_REQUESTED; else echo NO_OWNED_CAMERA; fi
'''
    else:
        script += '''echo "CAMERA_OWNED=$camera_owned pid=$camera_pid"
rostopic list | grep -E 'camera.*/(image_raw|camera_info)$' || true
tail -n 8 "$camera_dir/camera.log" 2>/dev/null || true
'''
    upper = ssh_args(c['upper_host'])
    upper[1:1] = ['-o', 'HostKeyAlias=' + c['upper_host_key_alias']]
    upper.append('bash -s')
    command = ssh_args(c['lower_host']) + [shlex.join(upper)] if c['upper_via_lower'] else upper
    r = subprocess.run(command, input=script, text=True, timeout=25)
    return r.returncode


if __name__ == '__main__':
    raise SystemExit(main())
