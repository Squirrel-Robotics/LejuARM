import * as THREE from './vendor/three.module.js';

// URDF rpy means fixed-axis roll/pitch/yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
export function setOrigin(group,origin){
 group.position.fromArray(origin.xyz);
 group.rotation.set(...origin.rpy,'ZYX');
}
export function buildRobot(spec,geometries,material){
 const links=new Map(spec.links.map(link=>[link.name,new THREE.Group()])),joints=new Map();
 for(const link of spec.links){
  const group=links.get(link.name);group.name=link.name;
  for(const visual of link.visuals){
   const mesh=new THREE.Mesh(geometries.get(visual.mesh),material);
   setOrigin(mesh,visual);mesh.scale.fromArray(visual.scale);group.add(mesh);
  }
 }
 for(const joint of spec.joints){
  const origin=new THREE.Group(),motion=new THREE.Group();setOrigin(origin,joint);
  links.get(joint.parent).add(origin);origin.add(motion);motion.add(links.get(joint.child));
  joints.set(joint.name,{...joint,origin,motion,axisVector:new THREE.Vector3(...joint.axis).normalize()});
 }
 const root=links.get(spec.root);root.name=spec.name;
 return {root,links,joints};
}
export function applyJointAngles(robot,angles){
 for(const [name,joint] of robot.joints){
  const value=angles[name]??0;
  if(!Number.isFinite(value))throw new Error('关节数值无效：'+name);
  if(joint.type==='prismatic')joint.motion.position.copy(joint.axisVector).multiplyScalar(value);
  else if(joint.type!=='fixed')joint.motion.quaternion.setFromAxisAngle(joint.axisVector,value);
 }
 robot.root.updateMatrixWorld(true);
}
export function samplePose(row,schema,source){
 const names=schema.action_order;
 if(source==='action'){
  if(row.action?.length!==21||!row.action.every(Number.isFinite))return null;
  return Object.fromEntries([...names.map((name,i)=>[name,row.action[i]]),['zhead_1_joint',0],['zhead_2_joint',0]]);
 }
 if(row.observation_state?.length!==23||!row.observation_state.every(Number.isFinite))return null;
 const raw=row.observation_state;
 return Object.fromEntries([...names.map((name,i)=>[name,raw[schema.display_state_indices[i]]*(i===0?-1:1)]),
  ['zhead_1_joint',raw[21]],['zhead_2_joint',raw[22]]]);
}
