'use strict';
const $=id=>document.getElementById(id), esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fmt=(x,d=1)=>Number.isFinite(x)?x.toFixed(d):'—', num=x=>x===undefined||x===null?'—':Number(x).toLocaleString('zh-CN');
const phases={starting:'正在启动',recording:'正在采集',closing:'停止收尾',finalizing_vision:'封装原始数据',raw_ready:'原始数据就绪',transferring:'传输与校验',cleanup_pending:'等待缓存清理回执',received:'原始文件已校验',processing:'本机裁剪对齐',ready:'处理完成',processing_failed:'处理失败',error:'采集异常',interrupted:'收尾已中断',cache_deleting:'正在清理异常缓存',cache_delete_failed:'异常缓存清理失败',unknown:'待确认',purging:'永久删除中',purge_failed:'删除失败 · 可重试'};
const qualities={pass:'通过',attention:'通过 · 有提示',failed:'不合格',pending:'待检查'},results={success:'任务成功',failure:'任务失败',pending:'待标注'};
const tone=p=>p==='ready'?'good':['error','processing_failed','cache_delete_failed','interrupted','purge_failed'].includes(p)?'bad':['starting','closing','processing','transferring','finalizing_vision','cache_deleting','purging'].includes(p)?'busy':'muted';
let sessionRowsMarkup=null;
let state=null,apiBusy=false,lastSuccess=0,actionBusy=false,taskDirty=false,taskVersion=null,pendingTask=null,detailId=null,reviewData=null,reviewRow=null,reviewAbort=null;
const radiansToDegrees=value=>Number.isFinite(value)?value*180/Math.PI:null;
const text=(id,v)=>{if($(id))$(id).textContent=v??'—';};
function badge(id,label,color){text(id,label);$(id).className='badge '+color;}
function clock(ns,full=false){if(!ns)return '—';const d=new Date(Number(ns)/1e6);return d.toLocaleString('zh-CN',{...(full?{}:{hour:'2-digit',minute:'2-digit',second:'2-digit'}),hour12:false});}
function duration(s){if(s===null||s===undefined)return '—';s=Math.max(0,s);return `${String(Math.floor(s/60)).padStart(2,'0')}:${(s%60).toFixed(1).padStart(4,'0')}`;}
function toast(message,error=false){const t=document.createElement('div');t.className='toast'+(error?' error':'');t.textContent=message;$('toasts').append(t);setTimeout(()=>t.remove(),6500);}
function fileURL(path){return '/api/file?'+new URLSearchParams({path});}
function reviewURL(kind,session,index){return '/api/review/'+kind+'?'+new URLSearchParams({session,...(index===undefined?{}:{index})});}
async function post(path,body={}){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Roban-Token':state?.csrf_token||''},body:JSON.stringify(body),signal:AbortSignal.timeout(14000)});const data=await r.json();if(!r.ok||data.ok===false)throw new Error(data.error||'操作未完成');return data;}
function displayDuration(){if(!state)return;const c=state.current;const active=['starting','recording'].includes(state.current_state);let seconds=c?.start_button_ns?((c.stop_boundary_ns||state.lower_now_ns||state.now_ns)-c.start_button_ns)/1e9:null;if(active&&lastSuccess)seconds+=Math.min(3,(Date.now()-lastSuccess)/1000);text('duration',duration(seconds??0));text('usableDuration',duration(Math.max(0,(seconds??0)-10)));text('durationHint',seconds===null?'首尾各裁5秒 · 建议至少录制15秒':seconds<5?'前5秒将被移除，请先保持姿势':seconds<10?'不足10秒无法满足首尾裁剪，请继续录制':'结束前先保持姿势5秒，再按LT＋X');}
function renderCommunication(){
 const c=state?.feedback?.communication||{},fresh=!!c.fresh;
 badge('communicationSummary',c.summary||'通信接口状态待确认',fresh?c.state:'muted');
 text('communicationHint',!fresh?'连接中断或状态待确认，等待下位机重新取样':c.errors?.length?c.errors.join('；'):'分别查看接口是否开启、当前收发和总线错误计数。');
 text('communicationConfig',c.config_path?`部位对应来自当前配置：${c.config_path}`:'');
 const cans=(c.can||[]).map(row=>{
  const count=row.counters||{},delta=row.new_errors;
  const errors=delta?delta.rx_errors+delta.tx_errors+delta.bus_error:null,drops=delta?delta.rx_dropped+delta.tx_dropped:null;
  const status=fresh?row.status:'状态待确认',tone=fresh?row.state:'muted';
  return `<tr><td><strong>${esc(row.interface)}</strong><small>${esc(row.kind||'CAN')} · ${row.bitrate?fmt(row.bitrate/1e6,0)+' Mbit/s':'—'}${row.data_bitrate?' / '+fmt(row.data_bitrate/1e6,0)+' Mbit/s':''}</small></td><td>${esc(row.body)}</td><td><span class="badge ${esc(tone)}">${esc(status)}</span></td><td class="communication-rates">${fmt(fresh?row.rx_fps:null,0)} / ${fmt(fresh?row.tx_fps:null,0)}<small>本轮错误 +${fresh?num(errors):'—'} · 丢帧 +${fresh?num(drops):'—'}</small></td><td><span class="badge ${fresh&&row.kernel_warning?'warn':'muted'}">${esc(fresh?row.kernel_state||'UNKNOWN':'待确认')}</span><small>TEC / REC ${fresh?num(row.tec):'—'} / ${fresh?num(row.rec):'—'}</small><small>累计 warning ${num(count.error_warning)} · passive ${num(count.error_passive)} · bus-off ${num(count.bus_off)}</small></td></tr>`;
 });
 const serial=(c.serial||[]).map(row=>`<tr><td><strong>${esc(row.label)}</strong><small>${esc(row.path)}</small></td><td>${esc(row.body)}</td><td><span class="badge ${esc(fresh?row.state:'muted')}">${esc(fresh?row.status:'状态待确认')}</span></td><td>—<small>未读取串口数据流</small></td><td>${fresh&&row.owners?.length?row.owners.map(x=>`${esc(x.name)} · PID ${num(x.pid)}`).join('<br>'):fresh&&row.owners_known?'未占用':'待确认'}${row.alias?`<small>${esc(row.alias)}</small>`:''}</td></tr>`);
 $('communicationRows').innerHTML=[...cans,...serial].join('')||'<tr><td colspan="5" class="empty-text">等待下位机通信接口状态</td></tr>';
}
function renderFeedback(){
 const f=state.feedback||{},ok=state.feedback_ready,c=f.communication||{};
 $('feedbackIcon').parentElement.classList.toggle('ready',!!ok);text('feedbackIcon',ok?'✓':'◎');text('feedbackTitle',ok?'23个关节反馈检查通过':'关节反馈尚未通过检查');text('feedbackReason',f.reason||'等待下位机硬件反馈');text('feedbackHz',fmt(f.fresh?f.hz:null,0));text('feedbackDetailHint',`${f.reason||'等待反馈'} · 更新时间距源时间 ${fmt(f.age_ms)} ms · 最大间隔 ${fmt(f.max_interval_ms)} ms`);
 renderCommunication();
 $('feedbackRows').innerHTML=Array.from({length:23},(_,i)=>{
  const bus=(c.can||[]).find(row=>row.joint_indices?.includes(i)),busBad=c.fresh&&bus?.state==='bad';
  return `<tr><td>${i} · ${esc(f.joint_names?.[i]||'')}</td><td>${esc(bus?.interface||'待确认')}</td><td>${fmt(radiansToDegrees(f.q?.[i]),2)}</td><td>${fmt(f.v?.[i],5)}</td><td>${fmt(f.tau?.[i],4)}</td><td><span class="badge ${busBad?'bad':ok?'good':'muted'}">${busBad?'所属总线异常':ok?'汇总反馈更新':'待确认'}</span></td></tr>`;
 }).join('');
}
function taskOptions(id,placeholder){
 const el=$(id),options=`<option value="">${placeholder}</option>`+(state.task_catalog||[]).map(t=>`<option value="${esc(t.id)}">${esc(t.name)}</option>`).join('');
 if(el.dataset.options!==options){const old=el.value;el.innerHTML=options;el.value=old;el.dataset.options=options;}
}
function renderTask(){if(pendingTask&&(state.task?.version===pendingTask.task.version||Date.now()>pendingTask.expires))pendingTask=null;const task=pendingTask?.task||state.task||{};taskOptions('taskName','请选择自定义任务');text('taskSummary',[task.name||'未设置任务',task.hand==='R'?'R · 右手':task.hand==='L'?'L · 左手':'待分左右手',task.prompt].filter(Boolean).join(' · '));const versionKey=[task.version,task.task_id,task.hand].join('/');if(!taskDirty&&taskVersion!==versionKey){$('taskName').value=task.task_id||'';$('captureHand').value=task.hand||'';$('taskPrompt').value=task.prompt||'';$('operator').value=task.operator||'';taskVersion=versionKey;}
const locked=!!state.current||!state.remote_fresh||state.current_state==='offline';for(const el of $('taskForm').querySelectorAll('input,textarea,select,button:not([data-create-task])'))el.disabled=locked||actionBusy;badge('taskBadge',taskDirty?'尚未应用':task.version?'已应用':'未设置',taskDirty?'warn':task.version?'good':'muted');text('taskHint',locked?'下位机监听就绪、且上一段收尾完成后可应用设置；新建本机任务随时可用':'选择任务与左右手后应用，下条采集处理完成后自动归档。');}
function render(s){const old=state;state=s;lastSuccess=Date.now();$('connectionBanner').hidden=true;text('updatedAt',clock(s.now_ns));badge('connection',s.remote_fresh?'下位机已连接':'下位机待连接',s.remote_fresh?'good':'muted');text('diskFree',s.storage?.available?`${fmt(s.storage.free/1024**3,0)} GiB 可用`:'数据盘待检查');text('rootPath',s.root_path||s.storage?.path||'');renderFeedback();renderTask();text('headline',s.headline);text('heroHint',s.hint);text('currentSession',s.current?.session_id||(s.sessions[0]?.session_id?'当前空闲 · 最近 '+s.sessions[0].session_id:'尚未开始采集'));badge('phaseBadge',phases[s.current_state]||(s.can_start_next?'采集已就绪':'准备设备'),s.can_start_next?'good':tone(s.current_state));text('referenceHz',s.reference_fresh?`${fmt(s.telemetry?.reference_hz)} Hz`:'— Hz');text('referenceHint',s.reference_fresh?`最大间隔 ${fmt(s.telemetry.reference_interval_max_ms)} ms`:'目标50 Hz · 等待有效参考');text('queueCount',s.queue_count);text('queueHint',`${s.counts.processing} 段本机处理中`);text('nextTitle',s.can_start_next?'可以开始下一段':s.current_state==='recording'?'正在采集本段':'采集条件待就绪');text('nextReason',s.next_reason);text('shutdownTitle',s.safe_to_shutdown?(s.sessions.length?'原始数据已校验，可关闭下位机':'当前无待传数据'):'暂不可关闭下位机');text('shutdownReason',s.shutdown_reason);badge('nextBadge',s.can_start_next?'已就绪':'等待中',s.can_start_next?'good':'muted');badge('shutdownBadge',s.safe_to_shutdown?'已确认':'待完成',s.safe_to_shutdown?'good':'muted');$('nextDecision').classList.toggle('confirmed',s.can_start_next);$('shutdownDecision').classList.toggle('confirmed',s.safe_to_shutdown);updateButtons();renderServices();renderPipeline();renderSessions();renderEvents();displayDuration();
if(detailId){const item=s.sessions.find(x=>x.recording_id===detailId);if(!item){$('sessionDialog').close();}else if(($('sessionDialog').dataset.phase!==item.phase||$('sessionDialog').dataset.sessionPath!==(item.relative_path||item.session_id)))openDetail(detailId);}
if(old&&$('soundEnabled').checked&&((s.can_start_next&&!old.can_start_next)||(s.safe_to_shutdown&&!old.safe_to_shutdown)))beep();}
function updateButtons(){if(!state)return;$('startBtn').disabled=actionBusy||!state.can_start_next||taskDirty;$('stopBtn').disabled=actionBusy||!state.remote_fresh||!state.current||!['starting','recording'].includes(state.current_state);}
function renderServices(){const good=state.services.filter(x=>x.state==='good').length;badge('serviceCount',`${good} / ${state.services.length} 正常`,good===state.services.length?'good':'muted');text('healthSummary',`数据健康 ${good} / ${state.services.length}`);$('serviceList').innerHTML=state.services.map(x=>`<div class="service-row ${esc(x.state)}-state"><span class="service-dot"></span><div><strong>${esc(x.title)}</strong><p>${esc(x.detail)}</p></div></div>`).join('');$('startReceiverBtn').hidden=!!state.receiver.active;}
function renderPipeline(){const labels=['录制采集','下位机收尾','传输与校验','缓存清理','本机裁剪对齐','数据就绪'];const item=state.sessions[0];const p=item?.phase;const index={starting:0,recording:0,closing:1,finalizing_vision:1,raw_ready:2,transferring:2,cleanup_pending:3,received:4,processing:4,ready:6,processing_failed:4}[p]??-1;$('pipelineSteps').innerHTML=labels.map((x,i)=>`<li class="${i<index?'done':i===index?'current':''}">${x}</li>`).join('');text('pipelineHint',item?`${item.session_id||'本段'} · ${phases[p]||'等待状态'}`:'等待第一段采集');const transferring=state.sessions.find(x=>x.phase==='transferring');$('transferProgress').hidden=!transferring;if(transferring){text('transferLabel',`接收 ${transferring.session_id}`);text('transferPercent',`${fmt(transferring.progress)}%`);$('transferFill').style.width=(transferring.progress||0)+'%';}}
function qualityMessages(x){
 const capture=x.capture_health||{},raw=capture.raw||{},trimmed=capture.trimmed||{},report=x.report||{};
 const messages=(x.warnings||report.warnings||[]).map(w=>w==='Raw camera timestamp gaps exceed 1.5 frame periods'
  ?`原始相机有 ${num(raw.intervals_over_1_5_period??report.camera_timestamp_gap_events)} 处时间戳间隔超过 ${fmt(1500/(raw.expected_fps||60))} ms，最长 ${fmt(raw.interval_ms?.max??report.video_capture_interval_ms?.max,2)} ms；首尾裁剪后保留 ${num(trimmed.intervals_over_1_5_period)} 处，最长 ${fmt(trimmed.interval_ms?.max,2)} ms。`
  :w==='Required hand has invalid or stale 50Hz samples; check hand masks before training'?'所选手部存在无效或超龄点；训练时必须检查手部有效标记。':String(w));
 const hs=x.hand_alignment?.summary;
 if(hs&&!hs.training_complete_and_aligned)messages.push(`手部50Hz：${hs.required_sides.map(s=>s==='right'?'右手':'左手').join('、')}有效 ${num(hs.training_matched_samples)} / ${num(hs.alignment_samples)} 条，${num(hs.unmatched_required_samples)} 条无效或超龄；无效训练值为空，不能直接作为完整合格片段使用。`);
 if(x.hand_alignment_error)messages.push('手部导出读取失败：'+x.hand_alignment_error);
 if(!messages.length&&x.quality==='attention')messages.push(`相机连续性需要复核：序号缺失 ${num(raw.sequence_missing_frames)} 帧，长间隔 ${num(raw.intervals_over_1_5_period)} 处。`);
 return messages;
}
function qualityBadge(x,qc){
 if(qc!=='attention')return `<span class="badge ${qc==='pass'?'good':qc==='failed'?'bad':'muted'}">${qualities[qc]}</span>`;
 const raw=x.capture_health?.raw,trimmed=x.capture_health?.trimmed;
 const brief=raw?.intervals_over_1_5_period>0?`相机长间隔：原片 ${num(raw.intervals_over_1_5_period)} 处 · 裁后 ${num(trimmed?.intervals_over_1_5_period)} 处`:qualityMessages(x)[0];
 const handIssue=x.hand_alignment_error||x.hand_alignment&&!x.hand_alignment.training_complete_and_aligned;
 return `<button type="button" class="badge warn quality-badge" data-detail="${esc(x.recording_id)}" aria-label="${esc(x.session_id)} 查看质量提示" title="${esc(qualityMessages(x).join('；'))}">${handIssue?'手部待复核':'通过 · 有提示'} ⓘ</button><small class="quality-warning">${esc(brief)}</small>`;
}
function qualityNotice(x){
 if(x.quality!=='attention')return '';
 const capture=x.capture_health||{},raw=capture.raw,report=x.report||{},checks=report.checks||{};
 return `<section class="quality-notice" role="note" aria-label="本段质量提示"><h3>质量提示 · 身体关节与视频50Hz对齐已通过</h3><ul>${qualityMessages(x).map(message=>`<li>${esc(message)}</li>`).join('')}</ul>${raw?`<p>相机序号缺失 ${num(raw.sequence_missing_frames)} 帧，采集队列丢弃 ${num(capture.queue_dropped_frames)} 帧。长间隔按原始时间戳判断，不能直接等同于视频文件缺帧。</p>`:''}${checks.video_frame_count_matches_csv?`<p>裁后视频 ${num(report.video_frames)} 帧，逐帧时间戳 ${num(report.timestamp_rows)} 行，数量一致${checks.mcap_full_read_crc?'；MCAP完整性检查通过':''}。</p>`:''}<p><strong>身体关节与视频 ${num(x.samples)} 条50Hz样本，${num(x.unmatched)} 条未匹配。</strong>手部有效性单独检查，以手部标记为准。</p></section>`;
}
let selectedEpisodes=new Set(),archiveBusy=false,visibleArchiveIds=[],selectedTask='',taskNavigationMarkup=null;
const handName=x=>x==='R'?'R · 右手':x==='L'?'L · 左手':'待分左右手';
function chooseTask(id){selectedTask=id;selectedEpisodes.clear();$('filterHand').value=id==='trash'?'trash':'';$('assignmentTask').value=id.startsWith('task_')?id:'';renderSessions();}
function renderTaskNavigation(){
 const all=state.sessions||[],catalog=state.task_catalog||[];
 const groups=[{id:'',name:'全部任务',count:all.length},{id:'unassigned',name:'未分配任务',count:all.filter(x=>!x.task_id).length},...catalog.map(t=>({id:t.id,name:t.name,count:all.filter(x=>x.task_id===t.id).length})),{id:'trash',name:'回收站',count:state.trash?.length||0}];
 const markup=groups.map(t=>`<button type="button" data-task-category="${esc(t.id)}" class="task-category ${selectedTask===t.id?'active':''}" ${selectedTask===t.id?'aria-current="page"':''}><span>${esc(t.name)}</span><small>${t.count}</small></button>`).join('');
 if(markup!==taskNavigationMarkup){$('taskCategories').innerHTML=markup;taskNavigationMarkup=markup;}
 const current=catalog.find(t=>t.id===selectedTask),hand=$('filterHand').value;
 text('taskBreadcrumb',(current?.name||(selectedTask==='unassigned'?'未分配任务':selectedTask==='trash'?'回收站':'全部任务'))+(hand&&hand!=='trash'?' / '+handName(hand==='unfiled'?'':hand):''));
 text('taskFolderHint',current?`文件夹：${current.folder} / ${['R','L'].includes(hand)?hand:'L、R、unfiled'}`:selectedTask==='unassigned'?'勾选已有片段，选择目标任务后归类':'任务 → L / R → 采集片段');
}
function updateArchiveButtons(){
 const count=selectedEpisodes.size;text('archiveSelection',`已选择 ${count} 段`);
 for(const id of ['archiveR','archiveL','clearSelection','deleteSelected','restoreSelected','purgeSelected'])$(id).disabled=archiveBusy||!count;
 const inTrash=$('filterHand').value==='trash';for(const id of ['archiveR','archiveL','deleteSelected','assignTask','assignmentTask'])$(id).hidden=inTrash;for(const id of ['restoreSelected','purgeSelected','emptyTrash'])$(id).hidden=!inTrash;
 const chosenTrash=(state?.trash||[]).filter(x=>selectedEpisodes.has(x.recording_id));$('restoreSelected').disabled=archiveBusy||!count||chosenTrash.some(x=>!x.can_restore);$('purgeSelected').disabled=archiveBusy||!count||chosenTrash.some(x=>!x.can_purge);$('emptyTrash').disabled=archiveBusy||!(state?.trash||[]).length||(state?.trash||[]).some(x=>x.phase==='purging');
 text('archiveHelp',inTrash?'永久删除和清空回收站会先列出本次范围与预计释放空间，确认后无法恢复。清空只处理确认列表内的片段；删除中或失败待重试的片段不能恢复。':'先“归入任务”，再分 L / R；更换任务会保留已有左右手。删除移入回收站，可恢复到原目录。传输或处理中暂不可操作；未传到本机的异常片段可在行末清理对应缓存。');
 $('assignTask').disabled=archiveBusy||!count||!$('assignmentTask').value;$('assignmentTask').disabled=archiveBusy;
 const unassigned=[...(state?.sessions||[])].some(x=>selectedEpisodes.has(x.recording_id)&&!x.task_id);
 for(const id of ['archiveR','archiveL']){$(id).disabled=archiveBusy||!count||unassigned;$(id).title=unassigned?'请先将所选片段归入任务':'同步移动到所选任务下的左右手目录';}
 const checkbox=$('selectVisible');checkbox.disabled=archiveBusy||!visibleArchiveIds.length;
 const chosen=visibleArchiveIds.filter(id=>selectedEpisodes.has(id)).length;
 checkbox.checked=!!chosen&&chosen===visibleArchiveIds.length;checkbox.indeterminate=chosen>0&&chosen<visibleArchiveIds.length;
 text('archiveSummary',`R 右手 ${state?.sessions.filter(x=>x.hand==='R').length||0} · L 左手 ${state?.sessions.filter(x=>x.hand==='L').length||0} · 未归档 ${state?.sessions.filter(x=>x.local&&!x.hand).length||0} · 回收站 ${state?.trash?.length||0}`);
}
function renderSessions(){
 if(!state)return;
 taskOptions('assignmentTask','选择目标任务');renderTaskNavigation();
 const q=$('filterTask').value.toLowerCase(),date=$('filterDate').value,phase=$('filterPhase').value,quality=$('filterQuality').value,result=$('filterResult').value,hand=$('filterHand').value,all=hand==='trash'?(state.trash||[]):state.sessions;
 const eligible=new Set(all.filter(x=>x.can_archive||x.can_restore||x.can_purge).map(x=>x.recording_id));selectedEpisodes=new Set([...selectedEpisodes].filter(id=>eligible.has(id)));
 const items=all.filter(x=>{const a=x.annotation||{},task=x.task||{};return(!selectedTask||selectedTask==='trash'||(selectedTask==='unassigned'?!x.task_id:x.task_id===selectedTask))&&(!q||`${x.session_id} ${task.name||''} ${task.prompt||''}`.toLowerCase().includes(q))&&(!date||x.session_id?.startsWith(date.replaceAll('-','_')))&&(!phase||(phase==='pending'?!['ready','processing','processing_failed'].includes(x.phase):x.phase===phase))&&(!quality||(x.quality||'pending')===quality)&&(!result||(result==='excluded'?a.use_for_training===false:(a.result||'pending')===result))&&(!hand||(hand==='trash'?true:hand==='unfiled'?!x.hand:x.hand===hand));});
 const order=new Map((state.task_catalog||[]).map((t,i)=>[t.id,i]));items.sort((a,b)=>(order.get(a.task_id)??-1)-(order.get(b.task_id)??-1)||['R','L',''].indexOf(a.hand||'')-['R','L',''].indexOf(b.hand||''));
 visibleArchiveIds=items.filter(x=>x.can_archive||x.can_restore||x.can_purge).map(x=>x.recording_id);updateArchiveButtons();
 text('sessionCount',`${items.length} / ${all.length}`);text('navCount',state.counts.sessions);$('emptySessions').hidden=items.length>0;text('emptySessions',hand==='trash'?(all.length?'没有符合筛选条件的已删除记录':'回收站为空'):all.length?'当前分类没有记录，可将已有片段归入此任务':'还没有正式记录。服务就绪后按 LT＋Y 开始第一段。');
 let lastGroup=null;const rowsMarkup=items.map(x=>{const a=x.annotation||{},task=x.task||{},qc=x.quality||'pending',group=(x.task_id||'unassigned')+'/'+(x.hand||'unfiled');const heading=group!==lastGroup?`<tr class="record-group"><td colspan="9"><span>${esc(x.task_id?task.name:'未分配任务')}</span><i>/</i>${handName(x.hand)}<small>${items.filter(y=>(y.task_id||'unassigned')+'/'+(y.hand||'unfiled')===group).length} 段</small></td></tr>`:'';lastGroup=group;return heading+`<tr><td class="select-cell"><input type="checkbox" data-archive-select="${esc(x.recording_id)}" aria-label="选择 ${esc(x.session_id)}" ${selectedEpisodes.has(x.recording_id)?'checked':''} ${!(x.can_archive||x.can_restore||x.can_purge)||archiveBusy?'disabled':''} title="${x.can_purge&&!x.can_restore?'选择后可重试永久删除':x.can_restore?'选择后可恢复或永久删除':x.can_archive?'选择后可归档或删除':'传输校验及本机处理结束后可以操作'}"></td><td><strong>${esc(x.session_id||'当前采集')}</strong><small>${esc(task.name||'未命名任务')}</small><small>${clock(x.start_button_ns,true)}</small></td><td><span class="badge ${x.hand?'busy':'muted'}">${x.deleted?'回收站':x.hand==='R'?'R · 右手':x.hand==='L'?'L · 左手':'未归档'}</span></td><td>${duration(x.duration_s)}<small>保留 ${duration(x.retained_duration_s)}</small></td><td>${num(x.action_messages)} / ${num(x.samples)}<small>裁剪MCAP真实消息 / 对齐网格</small></td><td>${num(x.raw_frames)} / ${num(x.video_frames)}</td><td><span class="badge ${tone(x.phase)}">${x.purge_state?esc(phases[x.phase]):x.deleted?'已删除 · 可恢复':esc(phases[x.phase]||'待确认')}</span><small>${qualityBadge(x,qc)}</small>${(x.cache_deletion||x.purge_state)&&x.error?`<small class="cache-error">${esc(x.error)}</small>`:''}</td><td>${results[a.result||'pending']}<small>${a.use_for_training===false?'不用于训练':'保留'}</small></td><td>${x.deleted?`${x.can_restore?`<button class="text-button" data-restore-one="${esc(x.recording_id)}">恢复记录</button>`:''}<button class="text-button danger-text" data-purge-one="${esc(x.recording_id)}" ${!x.can_purge||archiveBusy?'disabled':''}>${x.purge_state==='failed'?'重试永久删除':x.purge_state?'删除中…':'永久删除'}</button>`:`<button class="text-button" data-detail="${esc(x.recording_id)}">查看 / 审核 →</button>${!x.local?`<button class="text-button danger-text" data-delete-cache="${esc(x.recording_id)}" ${!x.can_delete_cache||archiveBusy?'disabled':''} title="${esc(x.cache_delete_reason||'永久清理上下位机对应异常缓存')}">${x.phase==='cache_deleting'?'清理中…':x.phase==='cache_delete_failed'?'重试清理缓存':'删除异常缓存'}</button>`:''}`}</td></tr>`;}).join('');
 if(rowsMarkup!==sessionRowsMarkup){$('sessionRows').innerHTML=rowsMarkup;sessionRowsMarkup=rowsMarkup;}
}
async function archiveSelection(hand){
 if(archiveBusy||!selectedEpisodes.size)return;const ids=[...selectedEpisodes];archiveBusy=true;renderSessions();
 try{const result=await post('/api/archive',{recording_ids:ids,hand});selectedEpisodes.clear();toast(result.moved?`已将 ${result.moved} 段归到 ${hand}（${hand==='R'?'右手':'左手'}）`:`所选记录已在 ${hand} 目录中`);await poll();}
 catch(error){toast(error.message,true);}finally{archiveBusy=false;renderSessions();}
}
$('archiveR').onclick=()=>archiveSelection('R');$('archiveL').onclick=()=>archiveSelection('L');
$('taskCategories').addEventListener('click',e=>{const button=e.target.closest('[data-task-category]');if(button)chooseTask(button.dataset.taskCategory);});
$('assignmentTask').onchange=()=>updateArchiveButtons();
$('assignTask').onclick=async()=>{
 const taskId=$('assignmentTask').value;if(archiveBusy||!selectedEpisodes.size||!taskId)return;
 archiveBusy=true;renderSessions();
 try{const result=await post('/api/tasks/assign',{recording_ids:[...selectedEpisodes],task_id:taskId});selectedEpisodes.clear();selectedTask=taskId;$('filterHand').value='';toast(result.moved?`${result.moved} 段已归入任务，文件夹已同步移动`:'所选片段已在该任务下');await poll();}
 catch(error){toast(error.message,true);}finally{archiveBusy=false;renderSessions();}
};
for(const button of document.querySelectorAll('[data-create-task]'))button.onclick=()=>{
 $('createTaskError').hidden=true;$('newTaskName').value='';$('createTaskDialog').showModal();$('newTaskName').focus();
};
$('createTaskForm').onsubmit=async e=>{
 e.preventDefault();$('createTaskSubmit').disabled=true;$('createTaskError').hidden=true;
 try{const result=await post('/api/tasks/create',{name:$('newTaskName').value});state.task_catalog=[...(state.task_catalog||[]),result.task];$('createTaskDialog').close();renderTask();renderSessions();$('assignmentTask').value=result.task.id;
  if(!$('taskName').disabled){$('taskName').value=result.task.id;taskDirty=true;renderTask();}
  updateArchiveButtons();toast('任务已创建，可归类已有片段或在采集前应用');await poll();
 }catch(error){text('createTaskError',error.message);$('createTaskError').hidden=false;}finally{$('createTaskSubmit').disabled=false;}
};
$('clearSelection').onclick=()=>{selectedEpisodes.clear();renderSessions();};
$('selectVisible').onchange=e=>{for(const id of visibleArchiveIds){if(e.target.checked)selectedEpisodes.add(id);else selectedEpisodes.delete(id);}renderSessions();};
$('sessionRows').addEventListener('change',e=>{const box=e.target.closest('[data-archive-select]');if(!box)return;if(box.checked)selectedEpisodes.add(box.dataset.archiveSelect);else selectedEpisodes.delete(box.dataset.archiveSelect);renderSessions();});

let pendingDeleteIds=[],pendingDeleteKind='local';
$('deleteSelected').onclick=()=>{
 pendingDeleteKind='local';pendingDeleteIds=[...selectedEpisodes];if(!pendingDeleteIds.length)return;
 text('deleteDialogTitle','删除所选记录');text('confirmDelete','移到回收站');text('deleteWarning','整段原始数据、裁剪结果和标注会一起移入回收站，可恢复到原位置。此操作不会立即释放磁盘空间。');
 const items=state.sessions.filter(x=>pendingDeleteIds.includes(x.recording_id));
 text('deleteDescription',`将以下 ${items.length} 段数据移到回收站？`);
 $('deleteList').innerHTML=items.map(x=>`<li>${esc(x.session_id)} · ${esc(x.task?.name||'未命名任务')} · ${x.hand||'未归档'}</li>`).join('');
 $('deleteDialog').showModal();
};
async function changeTrash(ids,restore){
 if(archiveBusy||!ids.length)return;archiveBusy=true;renderSessions();
 try{const result=await post(restore?'/api/restore':'/api/trash',{recording_ids:ids});selectedEpisodes.clear();toast(result.moved?`${result.moved} 段数据${restore?'已恢复到原位置':'已移到回收站'}`:'记录状态已更新');await poll();}
 catch(error){toast(error.message,true);}finally{archiveBusy=false;renderSessions();}
}
function openCacheDelete(identity){
 const item=state.sessions.find(x=>x.recording_id===identity);if(!item?.can_delete_cache)return;
 pendingDeleteKind='cache';pendingDeleteIds=[identity];
 text('deleteDialogTitle','永久删除异常缓存');text('deleteDescription','确认放弃这段未传到本机的数据？');
 $('deleteList').innerHTML=`<li>${esc(item.session_id)} · ${esc(item.task?.name||'未命名任务')}<small>${esc(identity)}</small></li>`;
 text('deleteWarning','确认后将永久删除下位机对应的MCAP缓存，以及上位机对应的原始图片、视频和时间戳；全部清理完成后移除网页记录。此操作不可恢复，不进入本机回收站。');
 text('confirmDelete','永久清理上下位机缓存');$('deleteDialog').showModal();
}
async function deleteCache(identity){
 if(archiveBusy)return;archiveBusy=true;renderSessions();
 try{await post('/api/cache/delete',{recording_id:identity,confirm_permanent:true});toast('已提交异常缓存清理，完成后记录会自动移除');await poll();}
 catch(error){toast(error.message,true);}finally{archiveBusy=false;renderSessions();}
}
$('confirmDelete').onclick=()=>{const ids=[...pendingDeleteIds],kind=pendingDeleteKind;$('deleteDialog').close();if(kind==='cache')deleteCache(ids[0]);else changeTrash(ids,false);};
$('restoreSelected').onclick=()=>changeTrash([...selectedEpisodes],true);
let pendingPurgePlan=null,purgeSubmitting=false;
const byteSize=value=>Number.isFinite(value)?value>=1024**3?`${fmt(value/1024**3,2)} GiB`:value>=1024**2?`${fmt(value/1024**2,1)} MiB`:value>=1024?`${fmt(value/1024,1)} KiB`:`${value} B`:'—';
async function openPurge(scope,ids=[]){
 if(archiveBusy)return;archiveBusy=true;renderSessions();
 try{const plan=await post('/api/trash/purge/preview',{scope,recording_ids:ids});pendingPurgePlan=plan;
  text('purgeTitle',scope==='all'?'清空回收站':'永久删除回收站片段');text('purgeDescription',`本次将永久删除以下 ${plan.count} 段数据。请核对片段列表；确认后在后台清理。`);
  text('purgeBytes',byteSize(plan.estimated_reclaim_bytes));text('purgeFiles',`文件合计 ${byteSize(plan.logical_bytes)} · ${num(plan.file_count)} 个文件；共享文件按实际占用估算。`);
  $('purgeList').innerHTML=plan.items.map(x=>`<li>${esc(x.session_id)} · ${esc(x.task?.name||'未分配任务')} · ${handName(x.hand)}</li>`).join('');
  $('purgeLegacy').hidden=!plan.legacy_cache_count;text('purgeLegacy',`还将清理 ${plan.legacy_cache_count} 份无法区分所属片段的旧版审核视频缓存；其他片段需要时会自动重建审核预览。`);
  $('purgeConsent').checked=false;$('confirmPurge').disabled=true;$('purgeError').hidden=true;$('purgeDialog').showModal();
 }catch(error){toast(error.message,true);}finally{archiveBusy=false;renderSessions();}
}
$('purgeSelected').onclick=()=>openPurge('selected',[...selectedEpisodes]);$('emptyTrash').onclick=()=>openPurge('all');
$('purgeConsent').onchange=()=>{$('confirmPurge').disabled=purgeSubmitting||!$('purgeConsent').checked;};
$('confirmPurge').onclick=async()=>{
 if(purgeSubmitting||!pendingPurgePlan||!$('purgeConsent').checked)return;purgeSubmitting=true;$('confirmPurge').disabled=true;$('purgeError').hidden=true;
 try{await post('/api/trash/purge',{plan_id:pendingPurgePlan.plan_id,confirm_permanent:true});$('purgeDialog').close();selectedEpisodes.clear();toast('已提交永久删除，后台清理完成后记录自动移除；失败可重试');await poll();}
 catch(error){text('purgeError',error.message);$('purgeError').hidden=false;}finally{purgeSubmitting=false;$('confirmPurge').disabled=!$('purgeConsent').checked;}
};
$('sessionRows').addEventListener('click',e=>{const button=e.target.closest('[data-purge-one]');if(button)openPurge('selected',[button.dataset.purgeOne]);});
$('sessionRows').addEventListener('click',e=>{const button=e.target.closest('[data-restore-one]');if(button)changeTrash([button.dataset.restoreOne],true);const cache=e.target.closest('[data-delete-cache]');if(cache)openCacheDelete(cache.dataset.deleteCache);});
function renderEvents(){if(!state.events?.length)return;$('eventList').innerHTML=state.events.slice(0,35).map(e=>`<div class="event ${esc(e.level)}"><span class="event-dot"></span><div class="event-body"><strong>${esc(e.title)}</strong><p>${esc(e.detail)}</p></div><time>${clock(e.timestamp_ns)}</time></div>`).join('');}
async function refresh(){try{await post('/api/refresh');toast('已请求重新检查，等待实际数据更新');setTimeout(poll,500);}catch(e){toast(e.message,true);}}
$('refreshBtn').onclick=refresh;$('preflightBtn').onclick=()=>{refresh();document.getElementById('services').scrollIntoView({behavior:'smooth'});};$('feedbackDetailBtn').onclick=()=>$('feedbackDialog').showModal();
for(const id of ['filterTask','filterDate','filterPhase','filterQuality','filterResult','filterHand'])$(id).addEventListener('input',()=>{selectedEpisodes.clear();if(id==='filterHand'){if($('filterHand').value==='trash')selectedTask='trash';else if(selectedTask==='trash')selectedTask='';}renderSessions();});
$('taskForm').addEventListener('input',()=>{taskDirty=true;badge('taskBadge','尚未应用','warn');updateButtons();});
$('taskForm').onsubmit=async e=>{e.preventDefault();actionBusy=true;renderTask();try{const r=await post('/api/task',{task_id:$('taskName').value,hand:$('captureHand').value,task:{prompt:$('taskPrompt').value,operator:$('operator').value}});taskDirty=false;taskVersion=null;state.task=r.task;pendingTask={task:r.task,expires:Date.now()+10000};renderTask();toast('任务已应用，下条采集使用此设置');await poll();}catch(e){toast(e.message,true);}finally{actionBusy=false;renderTask();updateButtons();}};
async function record(action){if(actionBusy)return;actionBusy=true;updateButtons();try{await post('/api/record/'+action,{recording_id:state.current?.name});toast(action==='start'?'开始请求已确认，等待MCAP与RGB就绪':'停止请求已确认，正在封装原始数据');setTimeout(poll,300);}catch(e){toast(e.message,true);}finally{actionBusy=false;updateButtons();}}
$('startBtn').onclick=()=>record('start');$('stopBtn').onclick=()=>record('stop');$('startReceiverBtn').onclick=async()=>{try{await post('/api/receiver/start');toast('本机接收服务已启动');poll();}catch(e){toast(e.message,true);}};
async function copyText(value){try{await navigator.clipboard.writeText(value);toast('目录已复制');}catch{toast(value);}}$('copyRootBtn').onclick=()=>copyText(state?.root_path||state?.storage?.path||'');
function qualityCards(x){const r=x.report||{},capture=x.capture_health||{},ready=x.phase==='ready',h=x.hand_alignment?.summary;return [['文件完整性',r.complete_and_aligned?'MCAP CRC / 视频 / 哈希通过':x.received?'原始文件已校验':'待传输检查'],['参考实际更新',x.source_health?.action?`${fmt(x.source_health.action.observed_hz,3)} Hz · 最大间隔 ${fmt(x.source_health.action.max_interval_ms)} ms`:'等待源消息审计'],['相机连续性',capture.no_frame_loss_observed===true?'未发现采集丢帧':capture.raw?`序号缺失 ${capture.raw.sequence_missing_frames} · 长间隔 ${capture.raw.intervals_over_1_5_period}`:'待检查'],['身体与视频50Hz',ready?`${num(x.samples)} 样本 · ${num(x.unmatched)} 未匹配`:'待完成'],...['left','right'].map(side=>[`${side==='left'?'左':'右'}手6维 · 50Hz`,h?`${num(h.sides[side].matched_samples)} / ${num(h.alignment_samples)} 有效 · 反馈年龄上限 ${fmt(h.sides[side].age_upper_bound?.max_ms,2)} ms`:'尚未导出手部对齐'])].map(([a,b])=>`<div>${a}<strong>${esc(b)}</strong></div>`).join('');}
function reviewPanel(){return `<section class="alignment-review"><div class="panel-title"><div><span class="eyebrow">VIDEO × ROBOT · 20 ms / 样本</span><h3>50Hz 视频与机器人动作审核</h3></div><span id="reviewVerdict" class="badge busy">读取对齐数据</span></div><p id="reviewSummary">正在读取源时间戳与关节数据…</p><div id="reviewFailure" class="alert bad" hidden></div>
<div class="review-media"><div class="review-picture"><div class="media-title"><h4>RGB 视频</h4><small>真实源帧 · 每帧对应一个50Hz样本</small></div><video id="detailVideo" class="detail-video" controls preload="metadata" playsinline aria-label="50Hz对齐审核视频"></video><div id="reviewLoading" class="review-loading">首次打开会生成审核预览，请稍候…</div><div class="playback-toolbar"><span id="playbackState" class="badge muted">等待视频</span><label>播放速度 <select id="reviewSpeed" aria-label="审核播放速度"><option value="0.25">0.25×</option><option value="0.5">0.5×</option><option value="1" selected>1×</option></select></label></div></div>
<div class="robot-card" id="robotReview"><div class="media-title"><h4>Roban S17 · 3D</h4><select data-robot-mode aria-label="3D动作来源"><option value="action">参考动作 · 21关节</option><option value="state">实际反馈 · 23关节</option><option value="overlay">参考与实际叠加</option></select></div><div class="robot-stage" data-robot-stage><div class="robot-loading" data-robot-hint>正在加载URDF与模型网格…</div></div><div class="robot-toolbar"><span class="badge busy" data-robot-status>模型加载中</span><div><button type="button" class="text-button" data-robot-view="front">正面</button><button type="button" class="text-button" data-robot-view="side">侧面</button><button type="button" class="text-button" data-robot-view="threequarter">复位视角</button></div></div><p class="robot-note" data-robot-note>绿：21关节参考 · 头部为中立位</p><small>拖动旋转 · 滚轮缩放 · 基座固定，仅回放关节姿态</small></div></div>
<div class="sample-controls"><button class="button" id="samplePrev" disabled>← 上一条</button><input id="sampleSlider" aria-label="50Hz样本索引" type="range" min="0" max="0" value="0" disabled><button class="button" id="sampleNext" disabled>下一条 →</button><label>跳转样本<input id="sampleIndex" type="number" min="0" max="0" value="0" aria-label="跳转样本" disabled></label><button class="button" id="sampleGo" disabled>定位</button></div><div class="issue-navigation"><span id="issueCount">异常点检查中</span><button class="text-button" id="previousIssue" disabled>上一个异常</button><button class="text-button" id="nextIssue" disabled>下一个异常</button><small>参考≤50ms · 反馈≤50ms · RGB≤40ms</small></div>
<div class="review-data-grid"><div class="review-timing"><div class="current-sample"><div><small>视频与3D共同样本</small><strong id="sampleLabel">—</strong></div><span id="sampleVerdict" class="badge muted">待载入</span></div><div id="ageCards" class="age-cards"></div><details class="source-details"><summary>查看当前源帧与时间戳</summary><div id="sampleMeta" class="sample-meta">数据读取中</div></details></div><div class="review-numbers"><div class="curve-controls"><label>查看关节<select id="actionJoint" aria-label="审核关节"></select></label><span class="curve-legend"><i></i>参考角 <i></i>实际角</span></div><div id="selectedValues" class="selected-values"></div><svg class="curve" id="curve" viewBox="0 0 600 155" role="img" aria-label="参考角与实际角时间曲线"></svg><small>rad · 竖线为当前样本，红线为不合格点；点击曲线可定位。角度差反映跟踪状态，不能单独判断时间对齐。</small></div></div>
<details class="joint-details"><summary>查看全部21个关节参考角 / 实际角</summary><p class="subtle">按关节名称对应；腰部反馈源索引12，显示时乘−1转为模型方向，原始数值不变。</p><div class="joint-values"><table><thead><tr><th>关节 / 反馈源索引</th><th>参考 rad</th><th>实际 rad</th><th>实际−参考 rad</th></tr></thead><tbody id="jointRows"></tbody></table></div></details></section>`;}
let robotReview=null,robotModule=null;
async function startRobotReview(abort){
 const host=$('robotReview');if(!host)return;
 try{robotModule??=import('/robot3d.js');const {RobotReview}=await robotModule;
  if(abort.signal.aborted||!host.isConnected)return;
  robotReview=new RobotReview(host,abort.signal);if(reviewRow&&reviewData)robotReview.update(reviewRow,reviewData.schema);
 }catch(error){if(!abort.signal.aborted&&host.isConnected){const label=host.querySelector('[data-robot-hint]');label.textContent='3D加载失败：'+error.message;label.hidden=false;host.querySelector('[data-robot-status]').textContent='3D不可用';}}
}
function openDetail(id){
 const x=state?.sessions.find(x=>x.recording_id===id);if(!x)return;
 robotReview?.dispose();robotReview=null;
 $('detailVideo')?.pause();$('rawVideo')?.pause();reviewAbort?.abort();
 detailId=id;reviewData=null;reviewRow=null;reviewSeekIndex=null;reviewVideoReady=false;
 text('dialogTitle',`${x.session_id||'当前采集'} · ${x.task?.name||'未命名任务'}`);$('sessionDialog').dataset.phase=x.phase;$('sessionDialog').dataset.sessionPath=x.relative_path||x.session_id;
 const ready=x.phase==='ready',available=x.local&&x.received,reviewable=available&&['ready','processing_failed'].includes(x.phase),base=x.relative_path||x.session_id,raw=`${base}/raw`,processed=`${base}/processed`,a=x.annotation||{};
 const stats=[['原始动作参考',x.source_health?.action?.messages],['裁剪MCAP参考',x.action_messages],['裁剪MCAP状态',x.state_messages],['已保存50Hz样本',x.samples],['原始RGB帧',x.raw_frames],['已保存裁剪RGB帧',x.video_frames],['已审计时间戳行数',x.report?.timestamp_rows],['已审计未匹配',x.unmatched]];
 const links=available?[[`${raw}/Action_ObservationState/${id}.mcap`,'原始MCAP'],[`${raw}/Action_ObservationState/${id}.json`,'采集元数据'],[`${raw}/vision/rgb.mp4`,'原始60FPS视频'],[`${raw}/vision/frame_timestamps.csv`,'原始时间戳'],...(ready?[[`${processed}/Action_ObservationState/${id}.mcap`,'裁剪MCAP'],[`${processed}/vision/rgb.mp4`,'裁剪60FPS视频'],[`${processed}/vision/frame_timestamps.csv`,'裁剪时间戳'],[`${processed}/vision/alignment.csv`,'对齐表'],[`${processed}/vision/aligned_50hz.jsonl`,'对齐数值JSONL'],[`${processed}/vision/aligned_50hz_schema.json`,'字段说明'],[`${processed}/vision/audit_report.json`,'检查报告']]:[]),...(x.hand_alignment?Object.entries({'aligned_50hz_with_hands.jsonl':'含双手6维的50Hz数据','alignment.csv':'含手部有效标记的对齐表','schema.json':'手部字段与单位说明','summary.json':'手部检查统计','audit_report.json':'手部独立检查报告'}).map(([name,label])=>[`${processed}/vision/${x.hand_alignment.files[name]}`,label]):[])].map(([p,n])=>`<a class="button" download href="${fileURL(p)}">${n}</a>`).join(''):'';
 $('dialogContent').innerHTML=`<div class="review-heading"><span class="badge ${tone(x.phase)}">${esc(phases[x.phase]||'待确认')}</span><span class="badge ${x.hand?'busy':'muted'}">${x.hand==='R'?'R · 右手':x.hand==='L'?'L · 左手':'未归档'}</span><span>${esc(x.task?.prompt||'未填写任务描述')} · 采集人 ${esc(x.task?.operator||'未填写')}</span></div>${qualityNotice(x)}${reviewable?reviewPanel():`<p class="loading">完整传到本机并完成处理后，可进行动作与视频审核。</p>`}<details class="detail-section"><summary>采集统计与处理报告</summary><div class="detail-stats">${stats.map(([n,v])=>`<div>${n}<strong>${num(v)}</strong></div>`).join('')}</div><div class="quality-grid">${qualityCards(x)}</div>${x.error?`<div class="alert bad">${esc(x.error)}</div>`:''}</details>${available?`<details class="detail-section" id="nativePlayback"><summary>独立回看原始 / 裁剪60FPS视频</summary><p>原始 ${duration(x.duration_s)} · ${ready?'已在本机首尾各裁5秒':'尚未生成合格裁剪结果'}。此处用于查看完整上下文；上方50Hz审核预览与动作逐帧联动。</p><div class="curve-controls"><label>视频版本 <select id="videoVersion"><option value="raw">原始60FPS视频</option>${ready?'<option value="processed">裁剪后60FPS视频</option>':''}</select></label></div><video id="rawVideo" class="detail-video" controls preload="none" src="${fileURL(raw+'/vision/rgb.mp4')}"></video></details><section class="detail-section"><h3>人工验收</h3><form id="annotationForm" class="annotation"><label>任务结果<select id="annotationResult">${Object.entries(results).map(([v,n])=>`<option value="${v}" ${a.result===v?'selected':''}>${n}</option>`).join('')}</select></label><label class="checkbox"><input id="annotationUse" type="checkbox" ${a.use_for_training!==false?'checked':''}>用于训练（取消勾选可恢复）</label><textarea id="annotationNote" maxlength="4000" rows="2" placeholder="备注 / 失败原因">${esc(a.note||'')}</textarea><div class="wide form-footer"><small>标注不改变技术检查结论或原始文件。</small><button class="button primary" type="submit">保存标注</button></div></form></section>`:''}<section class="detail-section"><h3>文件与时间</h3><p>开始 ${clock(x.start_button_ns,true)} · 结束 ${clock(x.stop_boundary_ns,true)}</p><div class="detail-actions">${links}${x.path?`<button class="button" data-copy="${esc(x.path)}">复制会话目录</button>`:''}</div><div class="detail-path">${esc(x.path||'')}</div></section>`;
 if(!$('sessionDialog').open)$('sessionDialog').showModal();
 if(available){$('videoVersion').onchange=()=>{$('rawVideo').src=fileURL(`${base}/${$('videoVersion').value}/vision/rgb.mp4`);};$('rawVideo').onplay=()=>$('detailVideo')?.pause();$('annotationForm').onsubmit=async e=>{e.preventDefault();try{await post('/api/annotate',{session:x.session_id,result:$('annotationResult').value,use_for_training:$('annotationUse').checked,note:$('annotationNote').value});toast('人工标注已保存');poll();}catch(e){toast(e.message,true);}};}
 if(reviewable)loadReview(x);
}
let reviewSeekIndex=null,reviewVideoReady=false;
const jointLabel=name=>name==='waist_yaw_joint'?'腰部旋转':name?.startsWith('leg_l')?'左腿 '+name.match(/\d+/)?.[0]:name?.startsWith('leg_r')?'右腿 '+name.match(/\d+/)?.[0]:name?.startsWith('zarm_l')?'左臂 '+name.match(/\d+/)?.[0]:name?.startsWith('zarm_r')?'右臂 '+name.match(/\d+/)?.[0]:name;
const reasonText=reason=>String(reason||'').split(';').filter(Boolean).map(r=>({stale_action:'参考角更新超时',stale_state:'实际角反馈超时',stale_rgb:'RGB图像超时',missing_or_invalid_action:'参考缺失或无效',missing_or_invalid_state:'反馈缺失或无效',missing_or_invalid_rgb:'图像缺失'}[r]||r)).join('、');
async function loadReview(item){
 const abort=new AbortController();reviewAbort=abort;
 try{
  const response=await fetch(reviewURL('curve',item.session_id),{signal:abort.signal});const data=await response.json();if(!response.ok)throw new Error(data.error||'审核数据加载失败');
  if(abort.signal.aborted||detailId!==item.recording_id)return;
  reviewData=data;startRobotReview(abort);const diagnostic=data.mode==='diagnostic',bad=data.bad_indices.length,clockOK=data.summary.clock_quality_accepted===true;
  badge('reviewVerdict',diagnostic?'诊断查看 · 数据未通过':bad||!clockOK?'存在对齐问题':'对齐检查通过',diagnostic||bad||!clockOK?'bad':'good');
  text('reviewSummary',`${diagnostic?'原始数据诊断视图':'使用已保存对齐表'} · 首尾各裁 ${data.trim_start_seconds}s / ${data.trim_end_seconds}s · ${num(data.total_samples)}个网格 · ${num(bad)}个不合格点 · 原始参考实际 ${Number.isFinite(data.source_action_health?.observed_hz)?fmt(data.source_action_health.observed_hz,3)+'Hz':'未审计'}。`);
  if(diagnostic||!clockOK){$('reviewFailure').hidden=false;text('reviewFailure',`${diagnostic?'片段未通过，红色样本仅供诊断。 ':''}${clockOK?'时钟检查通过。':'时钟质量未通过或不可确认。'} 原始文件与训练数据结论不变。`);}
  const names=data.schema.action_order;$('actionJoint').innerHTML=names.map((n,i)=>`<option value="${i}">${esc(jointLabel(n))} · ${esc(n)}</option>`).join('');
  $('actionJoint').onchange=()=>{drawCurve();if(reviewRow)renderSample(reviewRow.sample_index);};
  $('sampleSlider').max=data.total_samples-1;$('sampleIndex').max=data.total_samples-1;
  text('issueCount',bad?`${bad} 个不合格点，红色标记可定位`:'全段无不合格网格点');
  $('previousIssue').disabled=$('nextIssue').disabled=!bad;
  $('previousIssue').onclick=()=>jumpIssue(-1);$('nextIssue').onclick=()=>jumpIssue(1);
  $('sampleSlider').oninput=()=>setSample(Number($('sampleSlider').value));
  $('sampleIndex').onchange=()=>setSample(Number($('sampleIndex').value));
  $('sampleIndex').onfocus=()=>$('detailVideo')?.pause();
  $('sampleIndex').onkeydown=e=>{if(e.key==='Enter'){e.preventDefault();setSample(Number(e.currentTarget.value));}};
  $('sampleGo').onclick=()=>setSample(Number($('sampleIndex').value));
  $('samplePrev').onclick=()=>setSample((reviewRow?.sample_index||0)-1);$('sampleNext').onclick=()=>setSample((reviewRow?.sample_index||0)+1);
  $('curve').onclick=e=>{const rect=e.currentTarget.getBoundingClientRect();setSample(Math.round(Math.max(0,Math.min(1,((e.clientX-rect.left)/rect.width*600-30)/555))*(data.total_samples-1)));};
  drawCurve();renderSample(0);
  const video=$('detailVideo');
  $('reviewSpeed').onchange=()=>{video.playbackRate=Number($('reviewSpeed').value);};
  video.onloadedmetadata=()=>{if(video!==$('detailVideo'))return;reviewVideoReady=true;$('reviewLoading').hidden=true;for(const id of ['samplePrev','sampleNext','sampleSlider','sampleIndex','sampleGo'])$(id).disabled=false;badge('playbackState','已暂停 · 画面与样本逐帧对应','muted');};
  video.onerror=()=>{if(video!==$('detailVideo'))return;reviewVideoReady=false;text('reviewLoading','审核预览生成或加载失败，请重新打开该记录；原片可在下方回看。');$('reviewLoading').hidden=false;badge('playbackState','审核视频未就绪','bad');};
  video.onplay=()=>{$('rawVideo')?.pause();badge('playbackState','播放中 · 50Hz逐帧联动','good');};
  video.onpause=()=>{if(reviewSeekIndex===null&&!video.seeking)syncVideo(video.currentTime);if(reviewSeekIndex===null)badge('playbackState','已暂停 · 画面与样本逐帧对应','muted');};
  video.onseeking=()=>badge('playbackState','画面定位中','busy');
  video.onseeked=()=>{reviewSeekIndex=null;syncVideo(video.currentTime);badge('playbackState',video.paused?'已暂停 · 画面与样本逐帧对应':'播放中 · 50Hz逐帧联动',video.paused?'muted':'good');};
  if(video.requestVideoFrameCallback){const update=(_,meta)=>{if(video!==$('detailVideo')||abort.signal.aborted)return;if(!video.seeking&&reviewSeekIndex===null){video.dataset.presentedMediaTime=meta.mediaTime;video.dataset.reviewFrame=Math.floor(meta.mediaTime*50+1e-5);syncVideo(meta.mediaTime);}video.requestVideoFrameCallback(update);};video.requestVideoFrameCallback(update);}else{video.ontimeupdate=()=>{if(!video.seeking&&reviewSeekIndex===null)syncVideo(video.currentTime);};}
  video.src=reviewURL('video',item.session_id);
 }catch(e){if(e.name!=='AbortError'&&detailId===item.recording_id){badge('reviewVerdict','审核不可用','bad');text('reviewSummary',e.message);text('reviewLoading','无法加载审核，请查看采集统计与处理报告。');}}
}
function syncVideo(seconds){if(!reviewData)return;const n=Math.max(0,Math.min(reviewData.total_samples-1,Math.floor(seconds*50+1e-5)));if(reviewRow?.sample_index!==n)renderSample(n);}
function setSample(n){if(!reviewData||!Number.isFinite(n))return;n=Math.max(0,Math.min(reviewData.total_samples-1,Math.round(n)));const video=$('detailVideo');if(!video||!reviewVideoReady)return;video.pause();reviewSeekIndex=n;badge('playbackState','画面定位中','busy');renderSample(n);video.currentTime=(n+.25)/50;}
function jumpIssue(direction){if(!reviewData)return;const bad=reviewData.bad_indices,n=reviewRow?.sample_index??-1;const target=direction>0?bad.find(i=>i>n):bad.findLast(i=>i<n);setSample(target??(direction>0?bad[0]:bad.at(-1)));}
function renderSample(n){
 if(!reviewData)return;const row=reviewData.samples[n];if(!row)return;reviewRow=row;robotReview?.update(row,reviewData.schema);$('sampleSlider').value=n;if(document.activeElement!==$('sampleIndex'))$('sampleIndex').value=n;
 text('sampleLabel',`${n} / ${reviewData.total_samples-1} · ${fmt(row.timestamp_s,3)}s`);
 const reason=reasonText(row.timing.reason);badge('sampleVerdict',row.matched?'时序合格':reason||'不合格',row.matched?'good':'bad');
 const previous=reviewData.samples[n-1],reused=previous&&previous.action_index===row.action_index;
 $('ageCards').innerHTML=['action','state','rgb'].map((key,i)=>{const value=row.timing[key+'_age_ns'],ms=value===''?null:Number(value)/1e6,limit=reviewData.thresholds_ms[key],good=ms!==null&&ms>=0&&ms<=limit&&!row.timing.reason?.includes('invalid_'+key);return `<div class="${good?'age-good':'age-bad'}"><small>${['参考','实际反馈','RGB'][i]}距网格</small><strong>${fmt(ms,2)} <em>ms</em></strong><small>阈值 ≤${limit} ms</small></div>`;}).join('');
 const ai=Number($('actionJoint').value),si=reviewData.schema.display_state_indices[ai],actual=row.observation_state?.[si],measured=Number.isFinite(actual)?actual*(ai===0?-1:1):null,command=row.action?.[ai],delta=Number.isFinite(measured)&&Number.isFinite(command)?measured-command:null;
 $('selectedValues').innerHTML=`<span>参考角 <strong>${fmt(command,5)}</strong></span><span>实际角 <strong>${fmt(measured,5)}</strong></span><span>实际−参考 <strong>${fmt(delta,5)}</strong></span>`;
 text('sampleMeta',`审核帧 ${n} ↔ 50Hz样本 ${n}\n${reviewData.mode==='diagnostic'?'原始':'裁剪'}RGB源帧 ${row.rgb_frame_index} · 原片帧 ${row.original_rgb_frame_index}\n参考源索引 ${row.action_index} · 反馈源索引 ${row.state_index}${reused?' · 沿用上一条真实参考':''}\n网格源时间 ${row.timestamp_ns} ns${reason?'\n不合格原因：'+reason:''}`);
 $('jointRows').innerHTML=reviewData.schema.action_order.map((name,i)=>{const j=reviewData.schema.display_state_indices[i],a=row.action?.[i],raw=row.observation_state?.[j],s=Number.isFinite(raw)?raw*(i===0?-1:1):null;return `<tr class="${i===ai?'selected-joint':''}"><td>${esc(jointLabel(name))} · ${esc(name)} / ${j}</td><td>${fmt(a,5)}</td><td>${fmt(s,5)}</td><td>${fmt(Number.isFinite(a)&&Number.isFinite(s)?s-a:null,5)}</td></tr>`;}).join('');
 const marker=$('curveMarker');if(marker){const x=30+n/(reviewData.total_samples-1||1)*555;marker.setAttribute('x1',x);marker.setAttribute('x2',x);}
}
function drawCurve(){
 if(!reviewData||!$('curve'))return;const points=reviewData.points,ai=Number($('actionJoint').value),si=reviewData.schema.display_state_indices[ai],a=points.map(x=>x.action?.[ai]),b=points.map(x=>{const v=x.observation_state?.[si];return Number.isFinite(v)?v*(ai===0?-1:1):null;}),finite=[...a,...b].filter(Number.isFinite);if(!finite.length){$('curve').innerHTML='';return;}
 let low=Math.min(...finite),high=Math.max(...finite),span=Math.max(.01,high-low);low-=span*.1;high+=span*.1;
 const X=i=>30+i/(reviewData.total_samples-1||1)*555,Y=v=>130-(v-low)/(high-low)*110;
 const path=values=>{let active=false;return values.map((v,i)=>{if(!Number.isFinite(v)){active=false;return '';}const cmd=active?'L':'M';active=true;return cmd+fmt(X(points[i].sample_index),2)+' '+fmt(Y(v),2);}).join(' ');};
 const marker=X(reviewRow?.sample_index||0),issues=reviewData.bad_indices.map(n=>`<line x1="${X(n)}" x2="${X(n)}" y1="18" y2="133" stroke="#cc6666" opacity=".35"/>`).join('');
 $('curve').innerHTML=`<path d="M30 20V130H585" fill="none" stroke="#dae4dc"/><text x="2" y="23" font-size="9" fill="#819689">${fmt(high,2)}</text><text x="2" y="130" font-size="9" fill="#819689">${fmt(low,2)}</text><text x="30" y="149" font-size="9" fill="#819689">0s</text><text x="550" y="149" font-size="9" fill="#819689">${fmt(reviewData.samples.at(-1).timestamp_s)}s</text>${issues}<path d="${path(a)}" fill="none" stroke="#187955" stroke-width="1.8"/><path d="${path(b)}" fill="none" stroke="#4d81ac" stroke-width="1.5"/><line id="curveMarker" x1="${marker}" x2="${marker}" y1="15" y2="135" stroke="#9e741f" stroke-width="1.5"/>`;
}
document.addEventListener('click',e=>{const d=e.target.closest('[data-detail]');if(d)openDetail(d.dataset.detail);const c=e.target.closest('[data-copy]');if(c)copyText(c.dataset.copy);const close=e.target.closest('[data-close]');if(close)$(close.dataset.close).close();});$('sessionDialog').addEventListener('close',()=>{robotReview?.dispose();robotReview=null;detailId=null;reviewData=null;reviewVideoReady=false;reviewAbort?.abort();$('detailVideo')?.pause();$('rawVideo')?.pause();});
function disconnected(){ if(state){state.can_start_next=false;state.safe_to_shutdown=false;state.remote_fresh=false;state.feedback_ready=false;state.feedback={ready:false,fresh:false,reason:'关节反馈状态连接中断，等待重新确认'};renderFeedback();} $('connectionBanner').hidden=false;text('headline','页面状态连接中断');text('heroHint','正在恢复实时状态，采集与关闭条件待确认');for(const id of ['startBtn','stopBtn','applyTask'])$(id).disabled=true;text('nextTitle','采集条件待确认');text('shutdownTitle','关闭条件待确认');for(const id of ['nextDecision','shutdownDecision'])$(id).classList.remove('confirmed');badge('nextBadge','待确认','muted');badge('shutdownBadge','待确认','muted');}
async function poll(){if(apiBusy)return;apiBusy=true;try{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(4000)});if(!r.ok)throw new Error();render(await r.json());}catch{disconnected();}finally{apiBusy=false;}}
let painted=[],lastPaint=0,previewFrames=0,rgbMeta={};
function previewFresh(meta){return !!meta?.fresh&&Number.isFinite(meta.received_ns)&&(meta.latency_bound_ms||0)+Math.max(0,Date.now()-meta.received_ns/1e6)<700;}
async function preview(){for(;;){try{const r=await fetch('/api/preview',{cache:'no-store'});if(!r.ok)throw new Error('预览连接未就绪');const reader=r.body.getReader();let buf=new Uint8Array(0);for(;;){const v=await reader.read();if(v.done)throw new Error('预览连接中断');const joined=new Uint8Array(buf.length+v.value.length);joined.set(buf);joined.set(v.value,buf.length);buf=joined;while(buf.length>=4){const length=new DataView(buf.buffer,buf.byteOffset,4).getUint32(0);if(length>16384)throw new Error('无效预览头');if(buf.length<4+length)break;const meta=JSON.parse(new TextDecoder().decode(buf.subarray(4,4+length)));if(meta.size>4*1024*1024)throw new Error('无效预览帧');const end=4+length+meta.size;if(buf.length<end)break;if(meta.size&&previewFresh(meta)){const bytes=buf.slice(4+length,end);const bitmap=await createImageBitmap(new Blob([bytes],{type:'image/jpeg'}));if(!previewFresh(meta)){bitmap.close();buf=buf.slice(end);continue;}rgbMeta=meta;const canvas=$('previewCanvas');if(canvas.width!==bitmap.width||canvas.height!==bitmap.height){canvas.width=bitmap.width;canvas.height=bitmap.height;}canvas.getContext('2d',{alpha:false}).drawImage(bitmap,0,0);bitmap.close();lastPaint=performance.now();painted.push(lastPaint);painted=painted.filter(t=>lastPaint-t<3000);previewFrames++;canvas.dataset.stampNs=meta.stamp_ns||'';$('previewEmpty').hidden=true;}buf=buf.slice(end);}}}catch(e){text('previewEmptyHint',e.message||'等待上位机RGB图像');await new Promise(r=>setTimeout(r,2000));}}}
setInterval(()=>{const now=performance.now();if(now-lastPaint>700||!previewFresh(rgbMeta)){$('previewEmpty').hidden=false;text('previewEmptyHint',lastPaint?'画面已过期，正在重新连接':'等待相机图像，请检查相机服务');}const p=state?.preview||{},fresh=now-lastPaint<700&&previewFresh(rgbMeta);const displayed=fresh&&painted.length>1&&painted.at(-1)-painted[0]>=1500?(painted.length-1)*1000/(painted.at(-1)-painted[0]):null;text('sourceFps',`${fmt(p.fresh?p.source_fps:null)} FPS`);text('previewFps',`接收 ${fmt(p.fresh?p.relay_fps:null)} · 显示 ${fmt(displayed)} FPS · 时延 ≤ ${fmt(fresh?rgbMeta.latency_bound_ms:null)} ms`);$('previewCanvas').dataset.displayedFps=displayed??'';$('previewCanvas').dataset.frames=previewFrames;if(lastSuccess&&Date.now()-lastSuccess>6500)disconnected();else displayDuration();},250);
function beep(){try{const ctx=new AudioContext(),o=ctx.createOscillator(),g=ctx.createGain();o.connect(g);g.connect(ctx.destination);g.gain.value=.06;o.frequency.value=660;o.start();o.stop(ctx.currentTime+.12);o.onended=()=>ctx.close();}catch{}}
$('soundEnabled').checked=localStorage.getItem('robanSound')==='1';$('soundEnabled').onchange=()=>{localStorage.setItem('robanSound',$('soundEnabled').checked?'1':'0');if($('soundEnabled').checked)beep();};setInterval(poll,1000);poll();preview();
