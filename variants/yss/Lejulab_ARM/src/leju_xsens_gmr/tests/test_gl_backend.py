"""MUJOCO_GL=osmesa from bashrc must not crash headless GMR imports."""
import os
from pathlib import Path
import subprocess
import sys

from leju_gmr_common.gl_backend import configure_mujoco_gl

HELPER = Path(__file__).resolve().parents[2] / "leju_launch/scripts/_gmr_python_env.sh"
SCRIPTS = HELPER.parent


def test_osmesa_headless_uses_disable():
    env = {"MUJOCO_GL": "osmesa"}
    assert configure_mujoco_gl(argv=["prog"], environ=env) == "disable"
    assert env["MUJOCO_GL"] == "disable"


def test_viewer_without_display_uses_egl():
    env = {"MUJOCO_GL": "osmesa"}
    assert configure_mujoco_gl(argv=["prog", "--viewer"], environ=env) == "egl"
    env["DISPLAY"] = ":1"
    env["MUJOCO_GL"] = "osmesa"
    assert configure_mujoco_gl(argv=["prog", "--viewer"], environ=env) == "glfw"


def test_explicit_egl_is_kept():
    env = {"MUJOCO_GL": "egl"}
    assert configure_mujoco_gl(argv=["prog"], environ=env) == "egl"


def test_shell_helper_overrides_osmesa():
    result = subprocess.run(
        ["bash", "-c", 'source "$1"; gmr_configure_mujoco_gl; printf %s "$MUJOCO_GL"',
         "bash", str(HELPER)],
        env={**os.environ, "MUJOCO_GL": "osmesa"},
        capture_output=True, text=True, check=True)
    assert result.stdout == "disable"


def test_stop_stale_is_noop_without_frontend():
    subprocess.run(
        ["bash", "-c", 'source "$1"; gmr_stop_stale_xsens', "bash", str(HELPER)],
        check=True)


def test_run_xsens_script_disables_osmesa(tmp_path):
    output = tmp_path / "gl.txt"
    python = tmp_path / "venv/bin/python3"
    python.parent.mkdir(parents=True)
    python.write_text(
        "#!/usr/bin/python3\nimport os\nopen(%r, 'w').write(os.environ.get('MUJOCO_GL', ''))\n"
        % str(output))
    python.chmod(0o755)
    env = dict(os.environ, XSENS_GMR_VENV=str(tmp_path / "venv"), MUJOCO_GL="osmesa",
               XSENS_REPLACE_STALE="0")
    subprocess.run(["bash", str(SCRIPTS / "run_xsens_gmr.sh")], env=env, check=True)
    assert output.read_text() == "disable"


def test_heft_extractor_imports_under_osmesa():
    env = dict(os.environ, MUJOCO_GL="osmesa")
    env.pop("DISPLAY", None)
    result = subprocess.run(
        [sys.executable, "-c", "from leju_gmr_common.heft_extractor import HeftJointExtractor"],
        env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
