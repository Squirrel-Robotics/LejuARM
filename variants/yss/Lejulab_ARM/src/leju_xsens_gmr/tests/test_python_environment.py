"""Interpreter selection prevents the observed 3.8/3.14 setup failures."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

HELPER = Path(__file__).resolve().parents[2] / 'leju_launch/scripts/_gmr_python_env.sh'


def interpreter(path, minor):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('#!' + sys.executable + '\nimport subprocess, sys\n'
                    'code = "import sys; sys.version_info = (3, %d, 0); " + sys.argv[2]\n' % minor +
                    'sys.exit(subprocess.call([sys.executable, "-c", code]))\n')
    path.chmod(0o755)
    return str(path)


@pytest.mark.parametrize('minor,supported', [(8, False), (10, True), (12, True), (14, False)])
def test_explicit_interpreter_is_checked_before_creation(tmp_path, minor, supported):
    python = interpreter(tmp_path / 'python', minor)
    venv = tmp_path / 'venv'
    result = subprocess.run(['bash', '-c',
        'source "$1"; gmr_select_python "$2" "$3"', 'bash',
        str(HELPER), python, str(venv)], capture_output=True, text=True)
    assert (result.returncode == 0) == supported
    assert not venv.exists()
    if not supported:
        assert '3.10-3.12' in result.stderr


def test_old_container_python_uses_managed_runtime(tmp_path):
    bindir = tmp_path / 'bin'
    interpreter(bindir / 'python3', 8)
    managed = interpreter(tmp_path / 'managed-python', 10)
    uv = tmp_path / 'uv'
    uv.write_text('#!' + sys.executable + '\nimport sys\n'
                  'if sys.argv[1:3] == ["python", "find"]: print(' + repr(managed) + ')\n')
    uv.chmod(0o755)
    command = '''source "$1"
gmr_ensure_uv() { GMR_UV_BIN="$2"; }
gmr_select_python '' "$3"
printf '%s\n' "$GMR_PYTHON_BIN"
'''
    # Pass fake tool through a named variable, since function positional args differ.
    command = command.replace('GMR_UV_BIN="$2"', 'GMR_UV_BIN="$FAKE_UV"')
    env = dict(os.environ, PATH=str(bindir), FAKE_UV=str(uv))
    result = subprocess.run(['/bin/bash', '-c', command, 'bash', str(HELPER),
                             str(uv), str(tmp_path / 'venv')], env=env,
                            capture_output=True, text=True, check=True)
    assert result.stdout.splitlines()[-1] == managed
    assert 'managed Python 3.10' in result.stdout
