"""Launch contract tests: no ROS, hardware, DDS, or package installation needed."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / 'src/leju_launch/scripts'
LAUNCH = ROOT / 'src/leju_launch/launch'


def executable(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(0o755)
    return str(path)


@pytest.mark.parametrize('script,module,prefix', [
    ('run_xsens_gmr.sh', 'leju_xsens_gmr.main', 'XSENS'),
    ('run_pico_gmr.sh', 'leju_pico_gmr.main', 'PICO'),
    ('run_pico_joy.sh', 'leju_pico_gmr.joy_main', 'PICO'),
])
def test_runtime_environment_and_module(tmp_path, script, module, prefix):
    output = tmp_path / 'args.json'
    python = executable(tmp_path / 'venv/bin/python3',
        '#!/usr/bin/python3\nimport json, os, sys\n'
        'json.dump(sys.argv[1:], open(os.environ["CAPTURE"], "w"))\n')
    env = dict(os.environ, CAPTURE=str(output), PICO_GMR_VENV='/missing/pico',
               XSENS_GMR_VENV='/missing/xsens', XSENS_GMR_PYTHON='/unconfigured/system/python',
               XSENS_REPLACE_STALE='0')
    env[prefix + '_GMR_VENV'] = str(Path(python).parents[1])
    subprocess.run(['bash', str(SCRIPTS / script), '--host', '127.0.0.1',
                    '__name:=test', '__log:=/tmp/log', '__ns:=/test'], env=env, check=True)
    args = json.loads(output.read_text())
    assert args[:2] == ['-m', module]
    assert args[-2:] == ['--host', '127.0.0.1']
    assert not any(a.startswith('__') for a in args)
    assert ('--robot' in args) == (module == 'leju_pico_gmr.main')


def test_xsens_base_python_cannot_bypass_venv_or_fall_back_to_pico(tmp_path):
    env = dict(os.environ, XSENS_GMR_VENV='/missing/xsens', XSENS_GMR_PYTHON='',
               PICO_GMR_VENV=str(tmp_path))
    executable(tmp_path / 'bin/python3', '#!/bin/bash\nexit 0\n')
    result = subprocess.run(['bash', str(SCRIPTS / 'run_xsens_gmr.sh')], env=env, capture_output=True)
    assert result.returncode != 0
    env['XSENS_GMR_PYTHON'] = str(tmp_path / 'bin/python3')
    result = subprocess.run(['bash', str(SCRIPTS / 'run_xsens_gmr.sh')], env=env, capture_output=True)
    assert result.returncode != 0


def route(tmp_path, args, **environment):
    # Each child writes its own file so two optional processes cannot interleave.
    executable(tmp_path / 'start_node.sh', '''#!/usr/bin/python3
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
if args[0] == '--with-coredump': args.pop(0)
name = Path(args[0]).name
Path(os.environ['CAPTURE'], name).write_text(json.dumps({'args': args[1:],
 'xsens_env': os.environ.get('XSENS_GMR_VENV'), 'pico_env': os.environ.get('PICO_GMR_VENV'),
 'replace_stale': os.environ.get('XSENS_REPLACE_STALE')}))
''')
    command = f'''set -euo pipefail
SCRIPT_DIR={shlex.quote(str(SCRIPTS))}
source "$SCRIPT_DIR/_motion_teleop_common.sh"
parse_motion_args "$@"
SCRIPT_DIR={shlex.quote(str(tmp_path))}
ALL_PIDS=()
launch_motion_gmr
wait
'''
    env = dict(os.environ, CAPTURE=str(tmp_path), ROBOT_VERSION='17', COREDUMP='false',
               PICO_FPS='999', PICO_DOMAIN_ID='88', PICO_GMR_VENV='/pico',
               XSENS_GMR_VENV='/xsens', XSENS_FPS='101', XSENS_DOMAIN_ID='4')
    env.update(environment)
    subprocess.run(['bash', '-c', command, '--', *args], env=env, check=True)
    return {f.name: json.loads(f.read_text()) for f in tmp_path.glob('run_*.sh')}


@pytest.mark.parametrize('with_joy', [False, True])
def test_xsens_routes_only_body_and_optional_separate_joy(tmp_path, with_joy):
    args = ['--motion-source', 'xsens', '--xsens-fps', '120', '--xsens-domain=7',
            '--xsens-control-topic', '/control', '--xsens-venv', '/custom-xsens']
    if with_joy:
        args += ['--with-pico-joy', '--no-broadcast']
    calls = route(tmp_path, args)
    assert set(calls) == ({'run_xsens_gmr.sh', 'run_pico_joy.sh'} if with_joy else {'run_xsens_gmr.sh'})
    body = calls['run_xsens_gmr.sh']
    assert body['xsens_env'] == '/custom-xsens'
    assert body['replace_stale'] == '1'
    parsed = dict(zip(body['args'][::2], body['args'][1::2]))
    assert parsed == {'--host': '0.0.0.0', '--port': '9763', '--bvh-port': '9764',
        '--character-id': '0', '--fps': '120', '--domain-id': '7', '--control-topic': '/control'}
    if with_joy:
        joy = calls['run_pico_joy.sh']
        assert joy['pico_env'] == '/pico'
        assert joy['args'][-1] == '--no-broadcast'
        assert joy['args'][joy['args'].index('--domain-id') + 1] == '7'
        assert joy['args'][joy['args'].index('--joy-topic') + 1] == '/control'


def test_pico_default_preserves_body_arguments(tmp_path):
    calls = route(tmp_path, [])
    assert set(calls) == {'run_pico_gmr.sh'}
    args = calls['run_pico_gmr.sh']['args']
    assert args[args.index('--fps') + 1] == '999'
    assert args[args.index('--domain-id') + 1] == '88'
    assert '--robot' in args and '--broadcast-name' in args


@pytest.mark.parametrize('prefix,package', [('PICO', 'leju_pico_gmr'), ('XSENS', 'leju_xsens_gmr')])
def test_setup_installs_local_common_and_only_selected_frontend(tmp_path, prefix, package):
    repo = tmp_path / 'repo'
    scripts = repo / 'src/leju_launch/scripts'
    scripts.mkdir(parents=True)
    script = 'setup_' + prefix.lower() + '_gmr_env.sh'
    shutil.copy(SCRIPTS / script, scripts / script)
    shutil.copy(SCRIPTS / "_gmr_python_env.sh", scripts / "_gmr_python_env.sh")
    for name in ['leju_gmr_common', package]:
        path = repo / 'src' / name
        path.mkdir()
        (path / 'pyproject.toml').touch()
    dds = repo / 'src/lejusdk/3rd_party' / os.uname().machine / 'cyclonedds-0.10.2/lib'
    dds.mkdir(parents=True)
    (dds / 'libddsc.so').touch()
    # The bootstrap interpreter creates a fake venv whose pip records calls.
    bootstrap = executable(tmp_path / 'bootstrap', '''#!/usr/bin/python3
import os, pathlib, sys
if sys.argv[1] == "-c":
    if "print(" in sys.argv[2]: print("3.10")
    sys.exit(0)
venv = pathlib.Path(sys.argv[-1])
(venv / 'bin').mkdir(parents=True)
p = venv / 'bin/python3'
p.write_text('#!/usr/bin/python3\\nimport json, os, sys\\nwith open(os.environ["CAPTURE"], "a") as f: f.write(json.dumps(sys.argv[1:]) + "\\\\n")\\n')
p.chmod(0o755)
''')
    capture = tmp_path / 'pip.json'
    env = dict(os.environ, CAPTURE=str(capture))
    env[prefix + '_GMR_PYTHON'] = bootstrap
    env[prefix + '_GMR_VENV'] = str(tmp_path / 'isolated')
    subprocess.run(['bash', str(scripts / script)], env=env, check=True)
    calls = [json.loads(line) for line in capture.read_text().splitlines() if line]
    install = next(c for c in calls if c[:4] == ["-m", "pip", "install", "--no-binary=cyclonedds"])
    assert install == ['-m', 'pip', 'install', '--no-binary=cyclonedds',
                       '--editable', str(repo / 'src/leju_gmr_common'),
                       '--editable', str(repo / 'src' / package)]


def test_xml_frontends_independent():
    pico = ET.parse(LAUNCH / 'pico_teleop.launch').getroot()
    assert {n.get('type') for n in pico.findall('node')} == {'run_pico_gmr.sh'}
    assert not any('xsens' in a.get('name') for a in pico.findall('arg'))
    xsens = ET.parse(LAUNCH / 'xsens_teleop.launch').getroot()
    defaults = {a.get('name'): a.get('default') for a in xsens.findall('arg')}
    assert defaults['enable_pico_joy'] == 'false'
    assert defaults['xsens_fps'] == '100' and defaults['xsens_domain_id'] == '0'
    assert defaults['control_topic'] == '/rt/pico/joysticks'
    assert all('pico_teleop' not in n.get('file') for n in xsens.findall('include'))
    nodes = {n.get('type'): n for n in xsens.findall('node')}
    assert set(nodes) == {'run_xsens_gmr.sh', 'run_pico_joy.sh'}
    body = nodes['run_xsens_gmr.sh']
    assert {e.get('name') for e in body.findall('env')} == {
        'XSENS_GMR_VENV', 'XSENS_REPLACE_STALE'}
    assert 'pico_' not in body.get('args')
    flags = {a for a in body.get('args').split() if a.startswith('--')}
    assert not flags & {'--robot', '--pico-host', '--pico-port', '--no-pico-joy', '--no-broadcast', '--broadcast-name', '--bone-topic'}
    joy = nodes['run_pico_joy.sh']
    assert joy.get('if') == "$(eval arg('enable_pico_joy') and arg('control_topic') != '')"
    assert '--domain-id $(arg xsens_domain_id)' in joy.get('args')
    assert "--joy-topic '$(arg control_topic)'" in joy.get('args')


def test_shell_syntax():
    for script in SCRIPTS.glob('*.sh'):
        subprocess.run(['bash', '-n', str(script)], check=True)


@pytest.mark.parametrize('mode', ['sim', 'real'])
def test_dedicated_xsens_entry_forces_xsens_without_copying_platform(tmp_path, mode):
    wrapper = f'launch_xsens_{mode}.sh'
    shutil.copy(SCRIPTS / wrapper, tmp_path / wrapper)
    executable(tmp_path / f'launch_teleop_{mode}.sh',
               '#!/bin/bash\nprintf "%s\\n" "$@"\n')
    result = subprocess.run(['bash', str(tmp_path / wrapper), '--with-pico-joy',
                             '--motion-source', 'pico'], capture_output=True, text=True, check=True)
    assert result.stdout.splitlines() == ['--with-pico-joy', '--motion-source',
                                          'pico', '--motion-source', 'xsens']
    entry = (SCRIPTS / f'launch_teleop_{mode}.sh').read_text()
    assert 'source "${SCRIPT_DIR}/_launch_common.sh"' in entry
    assert 'parse_motion_args "$@"' in entry and '\nlaunch_motion_gmr\n' in entry


@pytest.mark.parametrize('args', [
    ['--motion-source', 'invalid'], ['--xsens-fps'],
    ['--motion-source', 'xsens', '--robot-version', '16'],
    ['--motion-source=xsens', '--robot-version=16'],
])
def test_invalid_motion_options_fail_before_platform_start(args):
    command = f'SCRIPT_DIR={shlex.quote(str(SCRIPTS))}; source "$SCRIPT_DIR/_motion_teleop_common.sh"; parse_motion_args "$@"'
    result = subprocess.run(['bash', '-c', command, '--', *args], capture_output=True)
    assert result.returncode != 0


def test_shell_empty_control_topic_suppresses_optional_joy(tmp_path):
    calls = route(tmp_path, ['--motion-source=xsens', '--with-pico-joy',
                            '--xsens-control-topic='])
    assert set(calls) == {'run_xsens_gmr.sh'}
    args = calls['run_xsens_gmr.sh']['args']
    assert args[args.index('--control-topic') + 1] == ''


@pytest.fixture
def noetic_resolver():
    # Run with ROS Noetic's Python packages on PYTHONPATH. Keep the remaining
    # shell/XML suite usable on machines that only have ROS 2 installed.
    return pytest.importorskip('roslaunch.substitution_args').resolve_args


@pytest.mark.parametrize('prefix', ['XSENS', 'PICO'])
@pytest.mark.parametrize('override', [None, '/tmp/custom venv'])
def test_noetic_venv_defaults_resolve(noetic_resolver, monkeypatch, prefix, override):
    monkeypatch.setenv('HOME', '/tmp/test home')
    variable = prefix + '_GMR_VENV'
    if override is None:
        monkeypatch.delenv(variable, raising=False)
    else:
        monkeypatch.setenv(variable, override)
    tree = ET.parse(LAUNCH / 'xsens_teleop.launch').getroot()
    expression = tree.find(f"arg[@name='{prefix.lower()}_gmr_venv']").get('default')
    assert noetic_resolver(expression) == (
        override if override is not None else
        f'/tmp/test home/.cache/leju_{prefix.lower()}_gmr/venv')


@pytest.mark.parametrize('control_topic', ['', '/rt/pico/joysticks'])
@pytest.mark.parametrize('head_topic', [None, '', '/custom/head'])
@pytest.mark.parametrize('enable_joy', ['false', 'true'])
def test_noetic_topic_arguments_and_optional_joy(
        noetic_resolver, control_topic, head_topic, enable_joy):
    tree = ET.parse(LAUNCH / 'xsens_teleop.launch').getroot()
    values = {'control_topic': control_topic, 'enable_pico_joy': enable_joy}
    if head_topic is not None:
        values['head_topic'] = head_topic
    context = {'arg': values}
    # Resolve the real frontend defaults and conditional switches in order.
    # Platform package lookup is deliberately excluded: no robot packages run.
    for arg in tree.findall('arg'):
        name = arg.get('name')
        expression = arg.get('default', arg.get('value', ''))
        if '$(find ' in expression:
            continue
        if arg.get('if') and noetic_resolver(arg.get('if'), context).lower() != 'true':
            continue
        if arg.get('unless') and noetic_resolver(arg.get('unless'), context).lower() == 'true':
            continue
        if name in values and arg.get('default') is not None:
            continue
        values[name] = noetic_resolver(expression, context)
    body = tree.find("node[@name='xsens_gmr']")
    argv = shlex.split(noetic_resolver(body.get('args'), context))
    assert argv[argv.index('--control-topic') + 1] == control_topic
    assert argv[argv.index('--head-topic') + 1] == (
        '/rt/head_trajectory' if head_topic is None else head_topic)
    joy = tree.find("node[@name='pico_joy']")
    enabled = noetic_resolver(joy.get('if'), context).lower() == 'true'
    assert enabled == (enable_joy == 'true' and control_topic != '')
    joy_argv = shlex.split(noetic_resolver(joy.get('args'), context))
    assert joy_argv[joy_argv.index('--joy-topic') + 1] == control_topic
