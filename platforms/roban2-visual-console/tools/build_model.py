#!/usr/bin/env python3
"""Vendor a reproducible S17 + two Revo-2 visualization model, offline only."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-project', required=True, type=Path)
    a = p.parse_args()
    source_root = a.source_project.resolve()
    assembled = source_root / 'outputs/model_alignment_20261021/lower_s17_revo2_v1'
    source = assembled / 'roban2_lower_revo2.urdf'
    provenance = json.loads((assembled / 'provenance.json').read_text())
    if sha(source) != provenance['files']['roban2_lower_revo2.urdf']:
        raise ValueError('Combined source URDF hash differs from its provenance')
    robot = ET.parse(source).getroot()
    robot.set('name', 'roban2_s17_revo2_visual_console')
    output = ROOT / 'assets/model'
    paths = {}
    for mesh in robot.iter('mesh'):
        original = Path(mesh.get('filename'))
        if '/biped_s17/meshes/' in str(original):
            relative = Path('meshes/biped_s17') / str(original).split('/biped_s17/meshes/', 1)[1]
            actual = source_root / 'assets/vendor/biped_s17/meshes' / str(original).split('/biped_s17/meshes/', 1)[1]
        elif '/revo2_system/meshes/' in str(original):
            suffix = str(original).split('/revo2_system/meshes/', 1)[1]
            relative = Path('meshes/revo2') / suffix
            actual = source_root / 'assets/vendor/brainco-description/revo2_system/meshes' / suffix
        else:
            raise ValueError('Unexpected mesh location: ' + str(original))
        if not actual.is_file():
            raise FileNotFoundError(actual)
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.is_file() or sha(destination) != sha(actual):
            shutil.copy2(actual, destination)
        mesh.set('filename', relative.as_posix())
        paths[relative.as_posix()] = sha(destination)
    backup = output / 'robot_body_only.urdf'
    if not backup.exists():
        shutil.copy2(output / 'robot.urdf', backup)
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(output / 'robot.urdf', encoding='utf-8', xml_declaration=True)
    movable = [j for j in robot.findall('joint') if j.get('type') != 'fixed']
    hands = [j for j in movable if j.get('name').startswith(('left_', 'right_'))]
    for side in ('left', 'right'):
        side_joints = [j for j in hands if j.get('name').startswith(side + '_')]
        assert len(side_joints) == 11
        assert len([j for j in side_joints if j.find('mimic') is None]) == 6
    manifest = {'schema': 'roban2_console_revo2_model_v1', 'source_project': str(source_root),
                'assembled_source': str(source), 'assembled_source_sha256': sha(source),
                'urdf_sha256': sha(output / 'robot.urdf'), 'body_joint_count': len(movable) - len(hands),
                'hand_kinematic_joints_per_side': 11, 'hand_active_channels_per_side': 6,
                'hand_mimic_joints_per_side': 5, 'total_kinematic_joint_count': len(movable),
                'mesh_file_count': len(paths), 'mounts': provenance['mounts'],
                'revo2_commit': provenance['revo2_commit'], 'hand_source_sha256': provenance['hand_source_sha256'],
                'hand_raw_to_rad_calibrated': False, 'hand_finger_pose': 'reference_open',
                'meshes': paths}
    (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in manifest.items() if k not in ('meshes', 'mounts')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
