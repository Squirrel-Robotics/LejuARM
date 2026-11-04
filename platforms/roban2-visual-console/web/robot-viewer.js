import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const vec = (text, fallback = '0 0 0') => (text || fallback).trim().split(/\s+/).map(Number);

export class RobotViewer {
  constructor(element) {
    this.element = element;
    this.joints = new Map();
    this.links = new Map();
    this.mimics = [];
    this.axes = [];
    this.meshes = [];
    this.pending = null;
    this.loaded = false;
    this.wireframe = false;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'low-power' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.renderer.setClearColor(0x191e22);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.35;
    element.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0x191e22, 4, 10);
    this.camera = new THREE.PerspectiveCamera(35, 1, .01, 50);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = .08;
    this.controls.minDistance = .18;
    this.controls.maxDistance = 6;
    this.controls.maxPolarAngle = Math.PI * .56;
    this.scene.add(new THREE.HemisphereLight(0xe9f4ff, 0x4b5343, 2.2));
    const key = new THREE.DirectionalLight(0xffffff, 3.1);
    key.position.set(2.5, -3, 5);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0xbcd5d9, 1.9);
    rim.position.set(-2, 3, 3);
    this.scene.add(rim);
    const grid = new THREE.GridHelper(8, 40, 0x48574e, 0x2c3839);
    grid.rotation.x = Math.PI / 2;
    grid.material.transparent = true;
    grid.material.opacity = .52;
    this.scene.add(grid);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.MeshStandardMaterial({ color: 0x1b2425, roughness: 1, transparent: true, opacity: .6, side: THREE.DoubleSide }));
    ground.position.z = -.004;
    this.scene.add(ground);
    const baseRing = new THREE.Mesh(new THREE.RingGeometry(.45, .453, 100), new THREE.MeshBasicMaterial({ color: 0x6a815a, transparent: true, opacity: .38, side: THREE.DoubleSide }));
    baseRing.position.z = .002;
    this.scene.add(baseRing);
    const axes = new THREE.AxesHelper(.23);
    axes.position.set(.6, -.55, .01);
    this.scene.add(axes);
    this.robot = new THREE.Group();
    this.scene.add(this.robot);
    this.focus = new THREE.Vector3(0, 0, .83);
    this.setView('perspective');
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(element);
    this.resize();
    let last = 0;
    this.renderer.setAnimationLoop((now) => {
      if (document.hidden || now - last < 1000 / 30) return;
      last = now;
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  resize() {
    const w = this.element.clientWidth, h = this.element.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  transform(group, origin) {
    if (!origin) return;
    group.position.fromArray(vec(origin.getAttribute('xyz')));
    const [r, p, y] = vec(origin.getAttribute('rpy'));
    group.quaternion.setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
  }

  async load(url, onProgress) {
    const response = await fetch(url);
    if (!response.ok) throw new Error('URDF HTTP ' + response.status);
    const xml = new DOMParser().parseFromString(await response.text(), 'application/xml');
    if (xml.querySelector('parsererror')) throw new Error('URDF XML 格式错误');
    const links = this.links;
    const children = new Set();
    const jobs = [];
    const stl = new STLLoader();
    const cache = new Map();
    for (const link of xml.querySelectorAll('robot > link')) {
      const name = link.getAttribute('name');
      const group = new THREE.Group();
      group.name = name;
      links.set(name, group);
      for (const visual of link.querySelectorAll(':scope > visual')) {
        const part = new THREE.Group();
        this.transform(part, visual.querySelector('origin'));
        group.add(part);
        const geometry = visual.querySelector('geometry');
        if (!geometry) continue;
        const isDark = /head|camera|bringo|radar/.test(name);
        const isArm = /zarm/.test(name);
        const isHand = /^(left|right)_/.test(name);
        const handColor = /base/.test(name) ? 0x424f54 : 0x85969e;
        const material = new THREE.MeshStandardMaterial({ color: isHand ? handColor : isDark ? 0x3c4c53 : isArm ? 0xa9bbc3 : 0xcdd4d6,
          roughness: .58, metalness: .25 });
        const addMesh = (geo) => {
          const mesh = new THREE.Mesh(geo, material);
          mesh.name = name;
          part.add(mesh);
          this.meshes.push(mesh);
        };
        const mesh = geometry.querySelector('mesh');
        if (mesh) {
          const filename = mesh.getAttribute('filename');
          const path = filename.startsWith('package://')
            ? '/model/meshes/' + encodeURIComponent(filename.split('/').pop())
            : new URL(filename, new URL(url, location.href)).href;
          if (!cache.has(path)) cache.set(path, stl.loadAsync(path));
          jobs.push(cache.get(path).then(geo => {
            addMesh(geo);
            if (mesh.getAttribute('scale')) part.scale.fromArray(vec(mesh.getAttribute('scale'), '1 1 1'));
          }));
        } else if (geometry.querySelector('box')) {
          addMesh(new THREE.BoxGeometry(...vec(geometry.querySelector('box').getAttribute('size'))));
        } else if (geometry.querySelector('sphere')) {
          addMesh(new THREE.SphereGeometry(Number(geometry.querySelector('sphere').getAttribute('radius')), 24, 16));
        } else if (geometry.querySelector('cylinder')) {
          const c = geometry.querySelector('cylinder');
          const geo = new THREE.CylinderGeometry(Number(c.getAttribute('radius')), Number(c.getAttribute('radius')), Number(c.getAttribute('length')), 32);
          geo.rotateX(Math.PI / 2);
          addMesh(geo);
        }
      }
    }
    for (const joint of xml.querySelectorAll('robot > joint')) {
      const origin = new THREE.Group();
      this.transform(origin, joint.querySelector('origin'));
      const motion = new THREE.Group();
      origin.add(motion);
      const childName = joint.querySelector('child').getAttribute('link');
      const parentName = joint.querySelector('parent').getAttribute('link');
      const child = links.get(childName), parent = links.get(parentName);
      if (!child || !parent) throw new Error('URDF 包含缺失连杆');
      parent.add(origin);
      motion.add(child);
      children.add(childName);
      const type = joint.getAttribute('type');
      if (type !== 'fixed') {
        const axis = new THREE.Vector3(...vec(joint.querySelector('axis')?.getAttribute('xyz'), '1 0 0')).normalize();
        this.joints.set(joint.getAttribute('name'), { motion, axis, type, value: 0 });
        const mimic = joint.querySelector('mimic');
        if (mimic) this.mimics.push({ name: joint.getAttribute('name'), parent: mimic.getAttribute('joint'),
          multiplier: Number(mimic.getAttribute('multiplier') || 1), offset: Number(mimic.getAttribute('offset') || 0) });
        const helper = new THREE.AxesHelper(.11);
        helper.visible = false;
        motion.add(helper);
        this.axes.push(helper);
      }
    }
    for (const [name, link] of links) if (!children.has(name)) this.robot.add(link);
    let done = 0;
    const results = await Promise.allSettled(jobs.map(promise => promise.finally(() => { onProgress?.(++done, jobs.length); })));
    const failures = results.filter(r => r.status === 'rejected');
    if (failures.length) throw new Error(`${failures.length} 个模型网格加载失败：${failures[0].reason.message}`);
    this.robot.updateMatrixWorld(true);
    const bounds = new THREE.Box3().setFromObject(this.robot);
    this.robot.position.z = -bounds.min.z + .012;
    this.focus.z = (bounds.max.z - bounds.min.z) * .52;
    this.loaded = true;
    if (this.pending) this.setJoints(...this.pending);
    this.setView('perspective');
    this.meshCount = jobs.length;
    return { meshes: jobs.length, joints: this.joints.size };
  }

  setJoints(names, q) {
    this.pending = [names, q];
    for (let i = 0; i < names.length; i++) {
      const joint = this.joints.get(names[i]);
      if (!joint || !Number.isFinite(q[i])) continue;
      if (joint.type === 'prismatic') joint.motion.position.copy(joint.axis).multiplyScalar(q[i]);
      else joint.motion.quaternion.setFromAxisAngle(joint.axis, q[i]);
      joint.value = q[i];
    }
    for (const mimic of this.mimics) {
      const joint = this.joints.get(mimic.name), parent = this.joints.get(mimic.parent);
      if (joint && parent) {
        joint.value = parent.value * mimic.multiplier + mimic.offset;
        joint.motion.quaternion.setFromAxisAngle(joint.axis, joint.value);
      }
    }
  }

  setView(view) {
    if (view === 'left-hand' || view === 'right-hand') {
      const side = view.startsWith('left') ? 'left' : 'right';
      const link = this.links.get(side + '_hand_base_link');
      if (link) {
        this.robot.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(link);
        const center = box.getCenter(new THREE.Vector3());
        this.controls.target.copy(center);
        this.camera.position.copy(center).add(new THREE.Vector3(.3, side === 'left' ? .45 : -.45, .07));
        this.controls.update();
        return;
      }
    }
    const offsets = { perspective: [2.05, -1.95, .9], front: [2.9, 0, .2], side: [0, -2.9, .2], top: [.001, 0, 3.3] };
    const v = offsets[view] || offsets.perspective;
    this.camera.position.copy(this.focus).add(new THREE.Vector3(...v));
    this.controls.target.copy(this.focus);
    this.controls.update();
  }

  toggleAxes() {
    const visible = !this.axes[0]?.visible;
    this.axes.forEach(a => { a.visible = visible; });
    return visible;
  }

  toggleWireframe() {
    this.wireframe = !this.wireframe;
    this.meshes.forEach(mesh => { mesh.material.wireframe = this.wireframe; });
    return this.wireframe;
  }
}
