"""Choose a MuJoCo GL backend before ``import mujoco``.

This robot's bashrc exports ``MUJOCO_GL=osmesa``. That path imports PyOpenGL
and crashes with ``AttributeError: 'NoneType' object has no attribute
'glGetError'``. GMR uses MuJoCo for IK, not rendering, so headless runs
disable GL. ``--viewer`` selects GLFW when ``DISPLAY`` is set, otherwise EGL.
"""
import os
import sys

_BROKEN = frozenset({"osmesa"})
_DISABLED = frozenset({"disable", "disabled", "off", "false", "0"})


def configure_mujoco_gl(argv=None, environ=None):
    """Return the backend after replacing a missing or unusable ``MUJOCO_GL``."""
    argv = sys.argv if argv is None else argv
    environ = os.environ if environ is None else environ
    wants_viewer = "--viewer" in argv
    current = environ.get("MUJOCO_GL", "").lower().strip()
    if wants_viewer:
        if current in _BROKEN | _DISABLED | {""}:
            environ["MUJOCO_GL"] = (
                "glfw" if environ.get("DISPLAY", "").strip() else "egl"
            )
        return environ["MUJOCO_GL"]
    if current in _BROKEN or current == "":
        environ["MUJOCO_GL"] = "disable"
    return environ.get("MUJOCO_GL", "disable")


configure_mujoco_gl()
