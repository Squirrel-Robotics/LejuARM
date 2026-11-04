import * as THREE from './vendor/three.module.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {STLLoader} from './vendor/STLLoader.js';
import {buildRobot,applyJointAngles,samplePose} from './robot_pose.mjs';

export class RobotReview{
 constructor(host,signal){
  this.host=host;this.closed=false;this.mode='action';this.last=null;this.ready=false;
  this.geometries=new Map();this.materials=[];this.pendingRender=null;
  this.abort=new AbortController();signal.addEventListener('abort',()=>this.dispose(),{once:true});
  try{this.init();this.loading=this.load();}catch(error){this.fail(error);}
 }
 status(message,tone='muted'){
  const element=this.host.querySelector('[data-robot-status]');if(!element||this.closed)return;
  element.textContent=message;element.className='badge '+tone;
 }
 fail(error){if(this.closed)return;this.status('3D不可用','bad');const hint=this.host.querySelector('[data-robot-hint]');hint.textContent='模型加载失败：'+error.message;hint.hidden=false;}
 init(){
  this.stage=this.host.querySelector('[data-robot-stage]');
  this.renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,powerPreference:'low-power'});
  this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
  this.renderer.setClearColor(0xf0f5f2);this.renderer.outputColorSpace=THREE.SRGBColorSpace;
  this.renderer.domElement.setAttribute('aria-label','Roban S17 50Hz关节动作3D模型');this.renderer.domElement.setAttribute('role','img');this.stage.append(this.renderer.domElement);
  this.scene=new THREE.Scene();
  this.camera=new THREE.PerspectiveCamera(38,1,.01,100);this.camera.up.set(0,0,1);
  this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.enableDamping=false;
  this.controls.minDistance=.6;this.controls.maxDistance=8;this.controls.maxPolarAngle=Math.PI*.92;
  this.controls.addEventListener('change',()=>this.draw());
  this.scene.add(new THREE.HemisphereLight(0xffffff,0x719284,2.5));
  const key=new THREE.DirectionalLight(0xffffff,3);key.position.set(3,-2,5);this.scene.add(key);
  const fill=new THREE.DirectionalLight(0xe2eeff,1.5);fill.position.set(-2,3,2);this.scene.add(fill);
  this.ground=new THREE.GridHelper(2.4,24,0xb9cfc0,0xdce6df);this.ground.rotation.x=Math.PI/2;this.scene.add(this.ground);
  this.host.querySelector('[data-robot-mode]').onchange=event=>{this.mode=event.target.value;if(this.last)this.update(...this.last);};
  for(const button of this.host.querySelectorAll('[data-robot-view]'))button.onclick=()=>this.view(button.dataset.robotView);
  this.resize=new ResizeObserver(()=>this.resizeCanvas());this.resize.observe(this.stage);this.view('threequarter');
  this.renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();this.ready=false;this.status('3D绘图中断，请重新打开审核','bad');});
 }
 resizeCanvas(){if(this.closed||!this.renderer)return;const width=this.stage.clientWidth,height=this.stage.clientHeight;if(width<1||height<1)return;this.renderer.setSize(width,height,false);this.camera.aspect=width/height;this.camera.updateProjectionMatrix();this.draw();}
 view(name){
  const z=this.targetZ??.75;this.controls.target.set(0,0,z);
  if(name==='front')this.camera.position.set(2.1,0,z+.1);
  else if(name==='side')this.camera.position.set(0,-2.1,z+.1);
  else this.camera.position.set(1.45,-1.9,z+.65);
  this.controls.update();this.draw();
 }
 async load(){
  try{
   const response=await fetch('/api/robot/model',{signal:this.abort.signal});if(!response.ok)throw new Error((await response.json()).error||'URDF读取失败');this.spec=await response.json();
   const loader=new STLLoader(),names=[...this.spec.meshes];let done=0;
   const worker=async()=>{while(names.length&&!this.closed){const name=names.shift();const response=await fetch(this.spec.mesh_base_url+encodeURIComponent(name),{signal:this.abort.signal});if(!response.ok)throw new Error('缺少网格 '+name);const geometry=loader.parse(await response.arrayBuffer());if(this.closed){geometry.dispose();return;}geometry.computeBoundingBox();this.geometries.set(name,geometry);this.status(`模型加载 ${++done}/${this.spec.meshes.length}`,'busy');}};
   await Promise.all(Array.from({length:4},worker));if(this.closed)return;
   const reference=new THREE.MeshStandardMaterial({color:0x6c9d86,metalness:.15,roughness:.52});
   const measured=new THREE.MeshStandardMaterial({color:0x4084c8,metalness:.08,roughness:.55});
   this.materials.push(reference,measured);
   this.reference=buildRobot(this.spec,this.geometries,reference);this.measured=buildRobot(this.spec,this.geometries,measured);
   applyJointAngles(this.reference,{});const box=new THREE.Box3().setFromObject(this.reference.root);
   this.baseHeight=-box.min.z+.015;this.targetZ=(box.max.z-box.min.z)/2;
   for(const robot of [this.reference,this.measured]){robot.root.position.z=this.baseHeight;this.scene.add(robot.root);}
   this.ready=true;this.host.querySelector('[data-robot-hint]').hidden=true;this.view('threequarter');
   if(this.last)this.update(...this.last);else this.draw();
  }catch(error){if(error.name!=='AbortError')this.fail(error);}
 }
 update(row,schema){
  this.last=[row,schema];if(!this.ready||this.closed)return;
  const action=samplePose(row,schema,'action'),state=samplePose(row,schema,'state');
  const showAction=this.mode!=='state'&&!!action,showState=this.mode!=='action'&&!!state;
  this.reference.root.visible=showAction;this.measured.root.visible=showState;
  // Overlay uses a translucent blue actual pose; both retain exact source angles.
  const overlay=this.mode==='overlay';if(this.materials[1].transparent!==overlay){this.materials[1].transparent=overlay;this.materials[1].depthWrite=!overlay;this.materials[1].needsUpdate=true;}this.materials[1].opacity=overlay?.38:1;
  if(showAction)applyJointAngles(this.reference,action);if(showState)applyJointAngles(this.measured,state);
  const missing=this.mode==='action'?!action:this.mode==='state'?!state:!action||!state;
  this.status(`样本 ${row.sample_index} · ${missing?'源角度缺失':this.mode==='action'?'参考动作':this.mode==='state'?'实际反馈':'参考＋实际'}${row.matched?'':' · 时序不合格'}`,missing||!row.matched?'bad':'good');
  const note=this.host.querySelector('[data-robot-note]');note.textContent=this.mode==='action'?'绿：21关节参考 · 头部为中立位':this.mode==='state'?'蓝：23关节实际反馈 · 腰部已转换模型方向':'绿：参考 · 半透明蓝：实际反馈 · 参考头部为中立位';
  this.host.dataset.sampleIndex=row.sample_index;this.host.dataset.poseSource=this.mode;
  this.host.dataset.appliedAction=JSON.stringify(showAction?action:null);this.host.dataset.appliedState=JSON.stringify(showState?state:null);
  this.draw();
 }
 draw(){if(this.closed||this.pendingRender!==null||!this.renderer)return;this.pendingRender=requestAnimationFrame(()=>{this.pendingRender=null;if(!this.closed){this.renderer.render(this.scene,this.camera);if(this.ready&&this.last)this.host.dataset.renderedSampleIndex=this.last[0].sample_index;}});}
 dispose(){
  if(this.closed)return;this.closed=true;this.abort.abort();if(this.pendingRender!==null)cancelAnimationFrame(this.pendingRender);
  this.resize?.disconnect();this.controls?.dispose();for(const g of this.geometries.values())g.dispose();for(const m of this.materials)m.dispose();
  this.ground?.geometry.dispose();if(Array.isArray(this.ground?.material))this.ground.material.forEach(m=>m.dispose());else this.ground?.material.dispose();
  this.renderer?.dispose();this.renderer?.forceContextLoss();this.renderer?.domElement.remove();this.last=null;
 }
}
