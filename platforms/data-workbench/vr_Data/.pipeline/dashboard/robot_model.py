"""Whitelist-only URDF visual model for the local read-only review page."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from workbench import ACTION_ORDER

ASSETS = Path(__file__).resolve().parent.parent/'urdf'
URDF = 'biped_s17.urdf'
MESH_PREFIX = 'package://kuavo_assets/models/biped_s17/meshes/'


def vector(element, key, default):
    values = [float(v) for v in element.get(key,' '.join(map(str,default))).split()] if element is not None else list(default)
    if len(values)!=len(default) or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid URDF vector')
    return values


def origin(element):
    return dict(xyz=vector(element,'xyz',[0,0,0]),rpy=vector(element,'rpy',[0,0,0]))


def model_spec(assets=ASSETS):
    source = assets/URDF
    if source.is_symlink():
        raise ValueError('Invalid model path')
    robot = ET.parse(source).getroot()
    links,joints,meshes = [],[],set()
    for link in robot.findall('link'):
        visuals=[]
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Unsupported visual geometry')
            filename = mesh.get('filename','')
            if not filename.startswith(MESH_PREFIX):
                raise ValueError('Unrecognized mesh package')
            name = filename[len(MESH_PREFIX):]
            if Path(name).name!=name or not name.endswith('.STL'):
                raise ValueError('Invalid mesh name')
            meshes.add(name)
            visuals.append(dict(origin(visual.find('origin')),mesh=name,
                scale=vector(mesh,'scale',[1,1,1]),color=vector(visual.find('material/color'),'rgba',[.84,.88,.86,1])))
        links.append(dict(name=link.get('name'),visuals=visuals))
    for joint in robot.findall('joint'):
        kind=joint.get('type')
        if kind not in ('fixed','revolute','continuous','prismatic') or joint.find('mimic') is not None:
            raise ValueError('Unsupported joint type')
        axis=vector(joint.find('axis'),'xyz',[1,0,0])
        norm=math.sqrt(sum(v*v for v in axis))
        if not norm:
            raise ValueError('Zero joint axis')
        joints.append(dict(origin(joint.find('origin')),name=joint.get('name'),type=kind,
            parent=joint.find('parent').get('link'),child=joint.find('child').get('link'),axis=[v/norm for v in axis]))
    names={link['name'] for link in links}
    children={j['child'] for j in joints}
    roots=names-children
    if len(roots)!=1 or len(children)!=len(joints) or any(j['parent'] not in names or j['child'] not in names for j in joints):
        raise ValueError('URDF is not a connected tree')
    root_name=next(iter(roots))
    visited=set()
    def visit(name):
        if name in visited:
            raise ValueError('Cyclic model tree')
        visited.add(name)
        for j in joints:
            if j['parent']==name:
                visit(j['child'])
    visit(root_name)
    if visited!=names:
        raise ValueError('Disconnected model tree')
    if not set(ACTION_ORDER).issubset({j['name'] for j in joints if j['type']!='fixed'}):
        raise ValueError('URDF does not match 21 action joints')
    for name in meshes:
        mesh_path(name,assets)
    return dict(name=robot.get('name'),urdf=URDF,root=roots.pop(),links=links,joints=joints,
        meshes=sorted(meshes),mesh_base_url='/api/robot/mesh?name=',action_order=ACTION_ORDER,
        head_joints=['zhead_1_joint','zhead_2_joint'],base_pose='fixed',units='radians')


def mesh_path(name,assets=ASSETS):
    if not isinstance(name,str) or Path(name).name!=name or not name.endswith('.STL'):
        raise ValueError('Invalid mesh resource')
    path=assets/'meshes'/name
    if path.is_symlink() or path.parent.is_symlink() or not path.is_file():
        raise ValueError('Model mesh not found')
    if not path.resolve().is_relative_to(assets.resolve()):
        raise ValueError('Invalid model mesh path')
    return path
