import * as THREE from 'three';
import { RobotViewer } from './robot-viewer.js';

const $ = id => document.getElementById(id);
const fmt = (n, digits=2) => Number.isFinite(n) ? n.toFixed(digits) : '—';
const deg = x => Number.isFinite(x) ? x * 180 / Math.PI : null;
const escape = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const names = ['waist_yaw_joint', ...['l','r'].flatMap(s => [1,2,3,4,5,6].map(i=>`leg_${s}${i}_joint`)), ...['l','r'].flatMap(s=>[1,2,3,4].map(i=>`zarm_${s}${i}_joint`))];
const active = [0,17,18,19,20], short = ['腰部','右臂 R1','右臂 R2','右臂 R3','右臂 R4'];
const phaseNames = {initial_settle:'初始姿态保持',inference_hold:'等待下一段模型推理',body_chunk:'执行身体轨迹',hand_boundary:'身体到位 / 手指动作'};
const sources = {model:['ACT 模型目标','模型计划姿态，可能尚未下发'],bounded:['上位机限速后候选','上位机候选姿态，不等于已执行'],ack:['下位机已接收目标','已获接收确认，不代表电机已到位'],native:['HEFT 已发布参考','下位机原生参考流记录']};
const clock = t => `${String(Math.floor(t/60)).padStart(2,'0')}:${(t%60).toFixed(2).padStart(5,'0')}`;
let runId='', frames=[], meta=null, cursor=0, playing=false, loaded=false, viewers=[], worldGroups=[];
let epoch=0, busy=false, catalogue=[], lastIndexSignature='', lastFrameIndex=-1, lastPaint=0, previousRAF=0;
let paths=[], pathEpoch=0, lastPathsCount=0, currentImage='', live=null, syncing=false, cameraBusy=false;

async function api(path) {
  const r=await fetch(path,{cache:'no-store'});
  const d=await r.json(); if(!r.ok)throw new Error(d.error || `HTTP ${r.status}`); return d;
}
function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toast.timer);toast.timer=setTimeout(()=>$('toast').hidden=true,6000);}
function searchIndex(t){let lo=0,hi=frames.length;while(lo<hi){const mid=(lo+hi)>>1;if(frames[mid].t<=t)lo=mid+1;else hi=mid;}return Math.max(0,lo-1);}
function chooseImage(t){const imgs=meta?.images||[];let choice=null;for(const img of imgs){if(img.t<=t)choice=img;else break;}return choice;}
function setFollow(value){$('follow').checked=value;if(value)playing=false;}

async function refreshIndex(initial=false){
  const items=await api('/api/runs');catalogue=items;
  const signature=JSON.stringify(items);
  if(signature!==lastIndexSignature){
    lastIndexSignature=signature;
    $('run-select').innerHTML=items.map(r=>`<option value="${r.id}">${r.id.slice(0,8)} · ${r.id.slice(9,11)}:${r.id.slice(11,13)}:${r.id.slice(13,15)}${r.speed!=null?` · ${r.speed}°/s`:''}${r.finished?'': ' · 记录中'}${r.has_motion_log?'':' · 无运动记录'}</option>`).join('');
    if(runId)$('run-select').value=runId;
  }
  const wanted=$('follow').checked ? items[0] : (items.find(r=>r.has_motion_log) || items[0]);
  if(wanted && (!runId || ($('follow').checked && wanted.id!==runId)))await selectRun(wanted.id);
}
async function selectRun(id){
  runId=id;epoch++;frames=[];meta=null;cursor=0;playing=false;lastFrameIndex=-1;lastPathsCount=0;paths=[];pathEpoch++;
  for(const v of viewers){v.robot.visible=false;if(v.pathLine)v.pathLine.visible=false;v.dirty=true;}
  $('run-select').value=id;$('download').href=`/api/run/${id}/csv`;
  $('camera-image').classList.remove('loaded');$('camera-empty').classList.remove('hidden');currentImage='';
  busy=false;await refreshData();
}
async function refreshData(){
  if(!runId || busy)return;
  busy=true;const id=runId,token=epoch;
  try{
    const data=await api(`/api/run/${id}/data?after=${frames.length}&version=${meta?.version??0}`);
    if(token!==epoch)return;
    if(data.reset)frames=[];
    frames.push(...data.frames);meta=data.meta;
    if($('follow').checked)cursor=meta.duration;
    if(!frames.length)cursor=0;
    $('service-status').textContent=meta.live?'推理记录更新中':$('follow').checked?'等待下一轮推理':'记录回放已连接';
    $('service-detail').textContent=meta.live?`日志落盘延迟约 ${fmt(meta.log_age_s,1)} s`:'本机数据 · 每秒检查更新';
    $('service-dot').style.background=meta.live?'#168d85':'#95aaa8';
    updateMeta();renderFrame(true);
    if(loaded && frames.length && (!paths.length || frames.length-lastPathsCount>=100 || data.reset))buildPaths();
  }catch(e){$('service-status').textContent='读取暂时中断';$('service-detail').textContent=e.message;$('service-dot').style.background='#d58265';}
  finally{if(token===epoch)busy=false;}
}
function updateMeta(){
  if(!meta)return;
  $('duration').innerHTML=`${fmt(meta.duration)}<small>s</small>`;
  $('policy-total').innerHTML=`${fmt(meta.policy_duration)}<small>/ 40 s</small>`;
  $('sample-count').textContent=`${meta.frames.toLocaleString()} 帧关节记录 · ${meta.acks.toLocaleString()} 条确认`;
  $('speed-cap').textContent=`本次有效限速 ${meta.speed??'—'}°/s`;
  $('peak').innerHTML=`${fmt(meta.peak?.error_deg)}<small>°</small>`;
  $('peak-joint').textContent=meta.peak?`${short[active.indexOf(names.indexOf(meta.peak.joint))]||meta.peak.joint} · ACK 同帧参考与反馈`:'暂无可配对的 HEFT 记录';
  $('run-status').textContent=meta.live?'推理进行中':meta.error?'运行已停止':meta.status?.startsWith('COMPLETE')?'记录已完成':meta.finished?'启动未完成':'等待记录';
  $('run-error').textContent=meta.error || (meta.finished?'抓取成败需现场确认':'自动读取新写入的日志');
  $('timeline').max=meta.duration||1;$('clock-end').textContent='/ '+clock(meta.duration);
  const canPlay=frames.length>1;for(const id of ['play','to-start','step-back','step-forward','to-error'])$(id).disabled=!canPlay;
  $('provenance').innerHTML=`<div>记录目录：<code>${escape(meta.source)}</code></div><div>模型 checkpoint：<code>${escape(meta.checkpoint||'暂无记录')}</code></div><div>参考与反馈按同一记录的实际时间轴显示；ACK 仅使用当时已经收到的样本，不引用未来确认。实际跟踪误差取同一 HEFT 样本内的已发布参考与实测值，ACK 采样间隙可能遗漏峰值。</div><div>模型目标、候选目标、已接收目标和已发布参考可以分别选择；未下发的未来轨迹不显示为实际运动。缺失值显示“—”。头部为本次初始编码器快照，重新设过零位后不能据此判断真实相机俯角。</div><div>历史图像为 ${meta.images.length} 张采样照片；${meta.parse_errors?`跳过 ${meta.parse_errors} 条无效记录。`:'无已检测到的无效日志行。'} 两侧显示的是运动学重建，物体接触与实际抓取成功未由本页面验证。</div>`;
}

function pose(viewer,q,h){
  if(!viewer?.loaded)return;
  viewer.robot.visible=!!q;if(!q)return;
  const n=[...names], values=[...q];
  for(const [name,value] of Object.entries(meta?.head||{})){n.push(name);values.push(value);}
  const suffix=['thumb_metacarpal_joint','thumb_proximal_joint','index_proximal_joint','middle_proximal_joint','ring_proximal_joint','pinky_proximal_joint'];
  // Existing simulation mapping only; channel display below preserves raw feedback.
  if(h)for(let s=0;s<2;s++){
    const raw=h.slice(s*6,s*6+6),f=[raw[1]/50,raw[0]/100,...raw.slice(2).map(v=>v/100)];
    for(let i=0;i<6;i++){n.push((s?'right_':'left_')+suffix[i]);values.push(Math.max(0,Math.min(1,f[i]))*[.785,1.03,1.41,1.41,1.41,1.41][i]);}
  }
  viewer.setJoints(n,values);viewer.robot.updateMatrixWorld(true);
}
function handPoint(viewer){const link=viewer.links.get('right_hand_base_link');return link?link.localToWorld(new THREE.Vector3(.035,.025,.075)):null;}
function handFor(frame,key){return key==='native'?frame.ack_hand:frame[key+'_hand'];}
function renderFrame(force=false){
  if(!meta || !frames.length){
    if(!force)return;
    $('phase').textContent='这条记录尚无可回放的运动帧';
    $('joint-table').innerHTML='';$('hand-bars').innerHTML='';
    $('ack-label').textContent='ACK —';$('frame-label').textContent='0 / 0 帧';
    $('wall-clock').textContent='00:00.00';$('policy-clock').textContent='策略 0.00 s';
    $('timeline').value=0;$('play').textContent='▶ 播放';
    for(const id of ['actual-position','target-position'])$(id).textContent='手部点 —';
    for(const v of viewers){v.robot.visible=false;v.dirty=true;}
    drawChart();if(meta)updateCamera();return;
  }
  cursor=Math.max(0,Math.min(cursor,meta.duration));const i=searchIndex(cursor),f=frames[i],key=$('source').value;
  if(!force && i===lastFrameIndex)return;lastFrameIndex=i;
  $('timeline').value=cursor;$('wall-clock').textContent=clock(cursor);$('policy-clock').textContent=`策略 ${fmt(f.p)} s`;
  $('phase').textContent=phaseNames[f.phase]||f.phase;$('ack-label').textContent=`ACK ${f.ack_seq??'—'}${f.ack_age_ms>1000?' · 旧样本':''}`;
  $('frame-label').textContent=`${i+1} / ${frames.length} 帧`;$('play').textContent=playing?'Ⅱ 暂停':'▶ 播放';
  $('target-title').textContent='仿真参考 · '+sources[key][0];$('target-caption').textContent=f[key]?sources[key][1]:'当前时刻尚无这类参考';
  $('actual-caption').textContent=`实测关节角驱动模型 · 策略 ${fmt(f.p)} s`;
  if(loaded){
    pose(viewers[0],f[key],handFor(f,key));pose(viewers[1],f.actual,f.actual_hand);
    for(let v=0;v<2;v++){
      const p=viewers[v].robot.visible?handPoint(viewers[v]):null;
      $(v?'actual-position':'target-position').textContent=p?`手部点 ${fmt(p.x,3)}, ${fmt(p.y,3)}, ${fmt(p.z,3)} m`:'手部点 —';
      const path=paths[v];if(path && viewers[v].pathLine){let count=0;while(count<path.indices.length && path.indices[count]<=i)count++;viewers[v].pathLine.geometry.setDrawRange(0,count);}
    }
  }
  $('joint-table').innerHTML=active.map((j,k)=>{const ref=deg(f[key]?.[j]),act=deg(f.actual[j]),delta=ref==null?null:ref-act,e=f.native_error?.[k];return `<tr data-joint="${j}"><td>${short[k]}</td><td>${fmt(ref)}°</td><td>${fmt(act)}°</td><td class="${Math.abs(delta)>12?'hot':''}">${fmt(delta)}°</td><td class="${Math.abs(e)>12?'hot':Math.abs(e)>8?'mild':''}">${fmt(e)}°</td></tr>`;}).join('');
  $('tracking-age').textContent=f.ack_seq==null?'尚无已收到的 HEFT 样本':`ACK ${f.ack_seq} · 接收后 ${fmt(f.ack_age_ms,0)} ms`;
  const labels=['Thumb','Aux','Index','Middle','Ring','Pinky'];
  $('hand-bars').innerHTML=labels.map((label,j)=>{const target=f.ack_hand?.[j+6],actual=f.actual_hand?.[j+6],max=j===1?50:100;return `<div class="hand-column"><div class="hand-meter"><i style="height:${Number.isFinite(target)?Math.min(100,Math.max(0,target/max*100)):0}%"></i><i style="height:${Number.isFinite(actual)?Math.min(100,Math.max(0,actual/max*100)):0}%"></i></div>${label}<b>${fmt(actual,0)}</b></div>`;}).join('');
  updateCamera(f);drawChart();
}

async function buildPaths(){
  if(!loaded||!frames.length)return;const token=++pathEpoch,key=$('source').value,rows=frames.slice(),count=rows.length;
  lastPathsCount=count;const result=[{points:[],indices:[]},{points:[],indices:[]}],stride=Math.max(1,Math.floor(count/420));
  const picks=[];for(let i=0;i<count;i+=stride)picks.push(i);if(picks.at(-1)!==count-1)picks.push(count-1);
  for(let k=0;k<picks.length;k++){
    if(token!==pathEpoch)return;const i=picks[k],f=rows[i];
    for(let v=0;v<2;v++){const q=v?f.actual:f[key];if(!q)continue;pose(viewers[v],q,v?f.actual_hand:handFor(f,key));const p=handPoint(viewers[v]);if(p){result[v].points.push(p);result[v].indices.push(i);}}
    if(k%35===34){renderFrame(true);await new Promise(requestAnimationFrame);}
  }
  if(token!==pathEpoch)return;paths=result;
  for(let v=0;v<2;v++){
    if(viewers[v].pathLine){viewers[v].scene.remove(viewers[v].pathLine);viewers[v].pathLine.geometry.dispose();viewers[v].pathLine.material.dispose();}
    const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints(paths[v].points),new THREE.LineBasicMaterial({color:v?0xd4704f:0x168d85,transparent:true,opacity:.9}));
    line.visible=$('show-path').checked;line.frustumCulled=false;viewers[v].pathLine=line;viewers[v].scene.add(line);
  }
  renderFrame(true);
}

function updateCamera(f){
  const isLive=$('follow').checked && meta?.live;
  $('camera-mode').textContent=isLive?'LIVE · 当前相机':'本次采样照片';
  if(isLive){$('camera-description').textContent='实时画面复用 8765 观察台缓存；轨迹来自已落盘日志，二者存在显示时差。';return;}
  const image=chooseImage(cursor);
  if(!image){$('camera-image').classList.remove('loaded');$('camera-empty').classList.remove('hidden');$('camera-empty').textContent='该时刻之前没有已保存的相机照片';$('camera-stamp').textContent='';return;}
  const url=`/api/run/${runId}/image/${image.file}`;
  if(currentImage!==url){currentImage=url;$('camera-image').src=url;}
  $('camera-stamp').textContent=`照片时刻 ${fmt(image.t)} s · 距当前 ${fmt(cursor-image.t)} s`;
  $('camera-description').textContent='历史记录显示当时保存的采样照片，保持上一张直到下一次采样；不是连续录像。';
}
async function refreshLive(){
  if(!$('follow').checked||!meta?.live||cameraBusy)return;cameraBusy=true;
  try{live=await api('/api/live');if(!$('follow').checked||!meta?.live)return;if(live.channels?.rgb?.status==='live'){
    $('camera-image').src=`/api/camera?t=${Date.now()}`;currentImage='live';
    $('camera-stamp').textContent=`实时 · 数据年龄 ${fmt(live.channels.rgb.source_age_ms,0)} ms`;
  }else{$('camera-image').classList.remove('loaded');$('camera-empty').classList.remove('hidden');$('camera-empty').textContent='实时相机未更新';}}
  catch(e){$('camera-empty').classList.remove('hidden');$('camera-empty').textContent='相机服务暂不可用：'+e.message;}
  finally{cameraBusy=false;}
}
$('camera-image').onload=()=>{$('camera-image').classList.add('loaded');$('camera-empty').classList.add('hidden');};
$('camera-image').onerror=()=>{$('camera-image').classList.remove('loaded');$('camera-empty').classList.remove('hidden');$('camera-empty').textContent='画面暂不可用';};

function drawChart(){
  const canvas=$('chart'),box=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio,2),w=box.width,h=box.height;
  if(!w||!h)return;canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);
  if(!frames.length)return;const j=Number($('chart-joint').value),k=active.indexOf(j),left=46,right=w-17,top=17,bottom=h*.61,errTop=h*.73,errBottom=h-24;
  const sampled=frames.filter((_,i)=>i%Math.max(1,Math.floor(frames.length/850))===0);if(sampled.at(-1)!==frames.at(-1))sampled.push(frames.at(-1));
  const values=sampled.flatMap(f=>[deg(f.model?.[j]),deg(f.native?.[j]),deg(f.actual?.[j])]).filter(Number.isFinite);
  let min=Math.min(...values),max=Math.max(...values);const margin=Math.max(3,(max-min)*.1);min-=margin;max+=margin;
  const x=t=>left+(right-left)*t/(meta?.duration||1),y=v=>bottom-(bottom-top)*(v-min)/(max-min),maxError=Math.max(15,...sampled.map(f=>Math.abs(f.native_error?.[k]||0)))*1.1,ey=v=>errBottom-(errBottom-errTop)*v/maxError;
  c.font='10px ui-monospace, monospace';c.lineWidth=1;
  for(let i=0;i<4;i++){const value=min+(max-min)*i/3,yy=y(value);c.strokeStyle='#edf1f2';c.beginPath();c.moveTo(left,yy);c.lineTo(right,yy);c.stroke();c.fillStyle='#92a0a8';c.fillText(value.toFixed(0)+'°',5,yy+3);}
  for(let i=0;i<=4;i++){const t=(meta?.duration||1)*i/4,xx=x(t);c.fillStyle='#92a0a8';c.fillText(t.toFixed(1)+'s',xx-12,h-6);}
  for(const [key,color] of [['model','#168d85'],['native','#638bc5'],['actual','#dc7956']]){
    c.strokeStyle=color;c.lineWidth=key==='actual'?1.8:1.3;c.beginPath();let pen=false;
    for(const f of sampled){const value=deg(f[key]?.[j]);if(value==null){pen=false;continue;}if(pen)c.lineTo(x(f.t),y(value));else c.moveTo(x(f.t),y(value));pen=true;}c.stroke();
  }
  c.fillStyle='#98a6ad';c.fillText('误差',5,errTop+5);c.setLineDash([4,3]);c.strokeStyle='#cc9a8c';c.beginPath();c.moveTo(left,ey(12));c.lineTo(right,ey(12));c.stroke();c.setLineDash([]);c.fillStyle='#b68577';c.fillText('12°',5,ey(12)+3);
  c.beginPath();c.strokeStyle='#ab6d57';c.lineWidth=1;let pen=false;for(const f of sampled){const v=f.native_error?.[k];if(v==null){pen=false;continue;}if(pen)c.lineTo(x(f.t),ey(Math.abs(v)));else c.moveTo(x(f.t),ey(Math.abs(v)));pen=true;}c.stroke();
  const xx=x(cursor);c.strokeStyle='#304c58';c.lineWidth=1;c.beginPath();c.moveTo(xx,top);c.lineTo(xx,errBottom);c.stroke();
  canvas.chartBounds={left,right,w};
}

async function setupModels(){
  try{
    viewers=[new RobotViewer($('target-view')),new RobotViewer($('actual-view'))];
    const world=await api('/api/workcell');
    await Promise.all(viewers.map(async(v,i)=>{
      await v.load('/model/robot.urdf',(n,total)=>$(i?'actual-loading':'target-loading').textContent=`加载模型 ${Math.round(n/total*100)}%`);
      v.focus.set(.20,0,.94);v.setView('perspective');v.camera.position.set(1.75,-1.9,1.72);v.controls.target.copy(v.focus);v.controls.update();
      const group=new THREE.Group();for(const g of world.geoms){const material=new THREE.MeshStandardMaterial({color:new THREE.Color(...g.rgba.slice(0,3)),roughness:.8});const mesh=new THREE.Mesh(new THREE.BoxGeometry(...g.size.map(s=>s*2)),material);mesh.position.fromArray(g.pos);group.add(mesh);}v.scene.add(group);worldGroups[i]=group;
      for(const mesh of v.meshes)if(/zarm_r/.test(mesh.name))mesh.material.color.lerp(new THREE.Color(i?0xe2a085:0x5ab9ad),.25);
      $(i?'actual-loading':'target-loading').classList.add('hidden');
      v.controls.addEventListener('change',()=>{if(!loaded||syncing||!$('sync-view').checked)return;syncing=true;const other=viewers[1-i];other.camera.position.copy(v.camera.position);other.camera.quaternion.copy(v.camera.quaternion);other.controls.target.copy(v.controls.target);other.controls.update();syncing=false;});
    }));
    loaded=true;renderFrame(true);buildPaths();
  }catch(e){for(const id of ['target-loading','actual-loading'])$(id).textContent='模型加载失败：'+e.message;toast(e.message);}
}

$('console-link').href=`http://${location.hostname}:8765/`;
$('run-select').onchange=()=>{setFollow(false);selectRun($('run-select').value).catch(e=>toast(e.message));};
$('refresh').onclick=()=>refreshIndex().then(refreshData).catch(e=>toast(e.message));
$('follow').onchange=()=>{if($('follow').checked){playing=false;refreshIndex().then(refreshData).then(refreshLive);}else renderFrame(true);};
$('play').onclick=()=>{setFollow(false);if(cursor>=(meta?.duration||0))cursor=0;playing=!playing;renderFrame(true);};
$('timeline').oninput=()=>{setFollow(false);playing=false;cursor=Number($('timeline').value);renderFrame(true);};
$('to-start').onclick=()=>{setFollow(false);playing=false;cursor=0;renderFrame(true);};
$('step-back').onclick=()=>{setFollow(false);playing=false;cursor=frames[Math.max(0,searchIndex(cursor)-1)]?.t||0;renderFrame(true);};
$('step-forward').onclick=()=>{setFollow(false);playing=false;cursor=frames[Math.min(frames.length-1,searchIndex(cursor)+1)]?.t||0;renderFrame(true);};
$('to-error').onclick=()=>{setFollow(false);playing=false;cursor=Math.max(0,(meta?.duration||0)-2);renderFrame(true);};
$('source').onchange=()=>{renderFrame(true);buildPaths();};
$('view').onchange=()=>{syncing=true;viewers.forEach(v=>v.setView($('view').value));syncing=false;};
$('show-path').onchange=()=>viewers.forEach(v=>{if(v.pathLine)v.pathLine.visible=$('show-path').checked;v.dirty=true;});
$('show-workcell').onchange=()=>{worldGroups.forEach(g=>g.visible=$('show-workcell').checked);viewers.forEach(v=>v.dirty=true);};
$('chart-joint').onchange=drawChart;
$('joint-table').onclick=e=>{const row=e.target.closest('[data-joint]');if(row){$('chart-joint').value=row.dataset.joint;drawChart();}};
$('chart').onclick=e=>{const bounds=$('chart').chartBounds;if(!bounds||!meta)return;const x=e.clientX-$('chart').getBoundingClientRect().left;setFollow(false);playing=false;cursor=Math.max(0,Math.min(1,(x-bounds.left)/(bounds.right-bounds.left)))*meta.duration;renderFrame(true);};
new ResizeObserver(()=>drawChart()).observe($('chart'));
function animate(now){const dt=previousRAF?Math.min(.15,(now-previousRAF)/1000):0;previousRAF=now;if(playing && meta){cursor+=dt*Number($('playback-speed').value);if(cursor>=meta.duration){cursor=meta.duration;playing=false;}}if(now-lastPaint>33){lastPaint=now;renderFrame();}requestAnimationFrame(animate);}
requestAnimationFrame(animate);
refreshIndex(true).catch(e=>toast('读取记录失败：'+e.message));setupModels();
setInterval(()=>refreshData(),1000);
setInterval(()=>refreshIndex().catch(()=>{}),2500);
setInterval(()=>refreshLive(),500);
