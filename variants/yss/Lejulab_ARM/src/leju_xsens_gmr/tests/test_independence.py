"""The source packages must remain independently importable and installable."""

import ast
import os
from pathlib import Path
import subprocess
import sys

import pytest

SRC = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('package,forbidden', [
    ('leju_xsens_gmr.main', ['leju_pico_gmr']),
    ('leju_pico_gmr.main', ['leju_xsens_gmr']),
    ('leju_gmr_common.app', ['leju_pico_gmr', 'leju_xsens_gmr']),
    ('leju_pico_gmr.joy_main', ['leju_xsens_gmr', 'leju_gmr_common.motion_retarget',
                                'leju_gmr_common.app']),
])
def test_import_does_not_require_other_frontend(package, forbidden):
    # A dependency regression fails even if both packages happen to be installed.
    script = '''import importlib, importlib.abc, sys
class RejectImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if any(fullname == p or fullname.startswith(p + '.') for p in BLOCKED):
            raise AssertionError('Forbidden cross-package dependency: ' + fullname)
sys.meta_path.insert(0, RejectImports())
importlib.import_module(PACKAGE)
'''
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run([sys.executable, '-c',
                             'BLOCKED = ' + repr(forbidden) + '\nPACKAGE = ' + repr(package) + '\n' + script],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_xsens_cli_owns_only_xsens_and_common_options():
    from leju_xsens_gmr.main import build_arg_parser
    parser = build_arg_parser()
    args = parser.parse_args([])
    assert args.port == 9763 and args.bvh_port == 9764
    assert not any('pico' in action.dest for action in parser._actions)
    assert not hasattr(args, 'bone_topic')
    assert not hasattr(args, 'cali_frame_count')
    assert not hasattr(args, 'broadcast_name')


def test_removed_xsens_modules_are_not_left_in_pico():
    package = SRC / 'leju_pico_gmr' / 'leju_pico_gmr'
    assert not list(package.glob('xsens*.py'))
    assert not (package / 'ik_configs/xsens_to_roban_s17.json').exists()


def test_ik_configs_belong_to_their_sources():
    from leju_xsens_gmr.params import IK_CONFIG_DICT as xsens
    from leju_pico_gmr.params import IK_CONFIG_DICT as pico
    from leju_gmr_common import params
    assert set(xsens) == {'bvh_xsense'}
    assert set(pico) == {'bvh_motive_streaming'}
    assert not hasattr(params, 'IK_CONFIG_DICT')
    assert 'leju_xsens_gmr' in str(xsens['bvh_xsense']['roban_s17'])
    assert 'leju_pico_gmr' in str(pico['bvh_motive_streaming']['roban_s17'])
