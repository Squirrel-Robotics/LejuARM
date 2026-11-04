import { RobotViewer } from './robot-viewer.js';

const $ = (id) => document.getElementById(id);
const all = (selector) => [...document.querySelectorAll(selector)];
const escape = (value) => String(value ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const number = (v, digits = 1) => typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '--';
const pretty = (obj) => obj == null ? '尚未收到反馈' : JSON.stringify(obj, null, 2);
const statusName = (channel) => ({ live: '在线', stale: '已过期', waiting: '等待数据', clock_error: '时钟异常' }[channel?.status] || '未连接');
const statusClass = (channel) => channel?.status === 'live' ? 'live' : channel?.status === 'waiting' ? '' : 'warn';
const timeString = (date = Date.now()) => new Date(date).toLocaleTimeString('zh-CN', { timeZone: 'Asia/Shanghai', hour12: false });
const byteSize = (bytes) => bytes > 1024 * 1024 ? (bytes / 1024 / 1024).toFixed(1) + ' MB' : Math.round(bytes / 1024) + ' KB';
let state = null, model = null, config = null, viewer = null;
let detail = 'joints', group = 'all', radians = false, paused = false, selected = 'zarm_l1_joint';
let camera = 'rgb', connected = false, lastMessage = 0, lastJointStamp = null, lastDetailRender = 0;
let lastEventKey = '', lastRecordKey = '', recordBusy = false, lastTableKey = '';
let cachedTableState = null, disconnectTimer = null;
const history = new Map();
const receiveHistory = [];

function toast(message, error = false) {
  $('toast').textContent = message;
  $('toast').classList.remove('hidden');
  $('toast').classList.toggle('error', error);
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => $('toast').classList.add('hidden'), 4500);
}

async function jsonFetch(url, options) {
  const response = await fetch(url, options);
  if (!response.ok) {
    let text = await response.text();
    try { text = JSON.parse(text).error || text; } catch {}
    throw new Error(text || ('HTTP ' + response.status));
  }
  return response.json();
}

async function action(name) {
  return jsonFetch('/api/action/' + name, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
}

function label(name) {
  const side = name.includes('_l') ? '左' : '右';
  let match;
  if ((match = name.match(/^zarm_[lr](\d)/))) return side + '臂 · ' + match[1] + ' 轴';
  if ((match = name.match(/^leg_[lr](\d)/))) return side + '腿 · ' + match[1] + ' 轴';
  if (name === 'waist_yaw_joint') return '腰部 · 偏航';
  if (name === 'zhead_1_joint') return '头部 · 偏航';
  if (name === 'zhead_2_joint') return '头部 · 俯仰';
  return name;
}

function effective(channel) {
  if (!channel) return { status: 'waiting', data: null, hz: 0, age_ms: null, source_age_ms: null };
  const elapsed = Math.max(0, performance.now() - lastMessage);
  if (!connected || elapsed > 1800) return { ...channel, status: channel.data ? 'stale' : 'waiting', age_ms: channel.age_ms == null ? null : channel.age_ms + elapsed, hz: 0 };
  return channel;
}

function dot(id, channel) { $(id).className = ($(id).classList.contains('small-dot') ? 'small-dot ' : 'dot ') + statusClass(channel); }

function ingest(data) {
  state = data;
  lastMessage = performance.now();
  const joint = data.channels.joint;
  if (joint.status === 'live' && joint.stamp !== lastJointStamp) {
    lastJointStamp = joint.stamp;
    const q = joint.data;
    viewer?.setJoints(q.names, q.q);
    q.names.forEach((name, i) => {
      const points = history.get(name) || [];
      points.push({ t: data.server_time, q: q.q[i] });
      while (points.length && points[0].t < data.server_time - 31) points.shift();
      history.set(name, points);
    });
    receiveHistory.push(joint.age_ms);
    while (receiveHistory.length > 40) receiveHistory.shift();
  }
  render();
}

function render() {
  if (!state) return;
  const channels = state.channels;
  const joint = effective(channels.joint), image = effective(channels[camera]), hand = effective(channels.hand);
  const colorLive = effective(channels.rgb).status === 'live';
  const active = joint.status === 'live';
  const anyLive = active || colorLive;
  $('connection-label').textContent = !connected ? '服务断开' : active && colorLive ? '实时在线' : anyLive ? '部分在线' : '服务在线';
  $('connection-chip').querySelector('.dot').className = 'dot ' + (!connected ? 'bad' : anyLive ? 'live' : 'warn');
  dot('overall-dot', { status: active && colorLive ? 'live' : 'stale' });
  $('overall-state').textContent = !connected ? '观察台连接已断开' : active && colorLive ? '机器人反馈与画面在线' : active ? '关节反馈在线 · 等待画面' : colorLive ? '画面在线 · 等待关节反馈' : '采集器在线 · 等待数据';
  $('strip-detail').textContent = active ? `${joint.data.names.length} 轴反馈 · ${number(joint.hz)} Hz · 固定底座显示` : '请按现有操作流程启动机器人反馈，观察台将自动接入';
  $('host-label').textContent = location.host;
  $('camera-rate').textContent = number(image.hz) + ' fps';
  $('camera-age').textContent = image.age_ms == null ? '等待数据' : '接收间隔 ' + Math.round(image.age_ms) + ' ms';
  dot('camera-dot', image);
  const hasImage = image.data != null;
  $('camera-image').classList.toggle('loaded', hasImage);
  $('camera-empty').classList.toggle('hidden', hasImage);
  $('camera-stale').classList.toggle('hidden', !hasImage || image.status === 'live');
  $('snapshot-button').disabled = image.status !== 'live';
  $('camera-live').innerHTML = `<i class="dot ${statusClass(image)}"></i>${image.status === 'live' ? 'LIVE' : hasImage ? 'STALE' : 'WAITING'}`;
  $('camera-time').textContent = image.stamp > 0 ? timeString(image.stamp * 1000) : '--';
  $('camera-frame').textContent = camera === 'rgb' ? 'HEAD / COLOR' : 'DEPTH / AUTO RANGE · 原始尺度未标定';
  $('camera-size').textContent = image.data ? image.data.width + ' × ' + image.data.height : '-- × --';
  $('joint-rate').textContent = number(joint.hz) + ' Hz';
  $('joint-age').textContent = joint.age_ms == null ? '等待反馈' : '接收间隔 ' + Math.round(joint.age_ms) + ' ms';
  dot('joint-dot', joint);
  $('model-status-label').textContent = active ? '关节反馈同步中' : joint.data ? '反馈已过期 · 保留姿态' : '参考姿态 · 未接收关节';
  $('model-status-detail').textContent = active ? '23 轴反馈 · Revo-2 手指参考姿态' : 'S17 + Revo-2 × 2 / 手指参考姿态';
  $('metric-joints').innerHTML = (joint.data ? joint.data.names.length : '--') + '<small> / 23</small>';
  $('metric-joints-detail').textContent = active ? '原生 DDS · ' + number(joint.hz) + ' Hz' : '原生 DDS · ' + statusName(joint);
  $('metric-age').innerHTML = (joint.age_ms == null ? '--' : Math.round(joint.age_ms)) + '<small> ms</small>';
  $('metric-link-detail').textContent = joint.source_age_ms == null ? '末次关节反馈在本机的接收间隔' : '源数据年龄 ' + Math.round(joint.source_age_ms) + ' ms · 采集端时钟';
  for (const side of ['left', 'right']) {
    const live = hand.status === 'live';
    const valid = live && hand.data?.[side + '_valid'] && hand.data?.[side + '_sample_age_ms'] <= state.limits.joint_stale_ms;
    const span = $(side + '-hand');
    span.classList.toggle('valid', valid);
    span.classList.toggle('invalid', !!hand.data && !valid);
    span.querySelector('b').textContent = !hand.data ? '--' : valid ? '有效' : live ? '无效 / 旧' : '过期';
  }
  const controller = effective(channels.controller), hardware = effective(channels.hardware);
  const c = controller.data || {};
  const mode = c.current_controller || c.controller_name || c.controller || c.state;
  $('metric-controller').textContent = typeof mode === 'string' || typeof mode === 'number' ? String(mode).slice(0, 22) : hardware.data ? '已接收硬件状态' : '未收到';
  $('controller-detail').textContent = '控制器 ' + statusName(controller) + ' / 硬件 ' + statusName(hardware);
  $('footer-viewers').textContent = state.viewer_count + ' 个观察端';
  const rec = state.recording;
  $('record-label').textContent = rec.active ? '结束记录' : '记录反馈';
  $('record-button').classList.toggle('recording', rec.active);
  $('record-indicator').textContent = rec.active ? `REC ${Math.floor(rec.duration_s / 60).toString().padStart(2, '0')}:${(rec.duration_s % 60).toString().padStart(2, '0')} · ${byteSize(rec.bytes)}` : 'LOCAL NETWORK';
  const eventKey = state.events[0]?.time + state.events[0]?.title;
  if (eventKey !== lastEventKey) {
    lastEventKey = eventKey;
    $('event-list').innerHTML = state.events.slice(0, 4).map(e => `<div class="event-row ${escape(e.level)}"><time>${escape(e.time.slice(11, 19))}</time><span class="event-indicator"></span><span>${escape(e.title)}</span><small>${escape(e.detail)}</small></div>`).join('');
  }
  if (performance.now() - lastDetailRender > 250) {
    renderDetails();
    lastDetailRender = performance.now();
  }
}

function renderDetails(force = false) {
  if (!state || !model) return;
  const joint = effective(state.channels.joint);
  dot('data-live-dot', paused && detail === 'joints' ? { status: 'stale' } : joint);
  $('data-live-label').textContent = paused && detail === 'joints' ? '表格已暂停 · 采集继续' : statusName(joint);
  if (detail === 'joints') {
    if (!paused || !cachedTableState) cachedTableState = joint;
    renderJoints(cachedTableState, force);
    renderTrend();
  } else if (detail === 'hands') renderHands();
  else if (detail === 'controller') {
    $('hardware-raw').textContent = pretty(state.channels.hardware.data);
    $('controller-raw').textContent = pretty(state.channels.controller.data);
  } else if (detail === 'sources') renderSources();
  else if (detail === 'records') {
    const key = state.recording.file + state.recording.active;
    if (key !== lastRecordKey) { lastRecordKey = key; loadRecords(); }
  }
}

function renderJoints(joint, force) {
  const key = [joint.stamp, joint.status, group, selected, radians].join('|');
  if (!force && key === lastTableKey) return;
  lastTableKey = key;
  const rows = model.joints.filter(j => group === 'all' || (group === 'arm' ? j.name.startsWith('zarm') : group === 'leg' ? j.name.startsWith('leg_') : /zhead|waist/.test(j.name)));
  const values = joint.data;
  const indices = new Map((values?.names || []).map((n, i) => [n, i]));
  $('angle-column').textContent = radians ? '角度 rad' : '角度 °';
  $('joint-table').innerHTML = rows.map(j => {
    const i = indices.get(j.name), q = values?.q[i], velocity = values?.v[i], tau = values?.tau[i];
    const present = Number.isFinite(q);
    const ratio = present && j.upper != null && j.lower != null ? (q - j.lower) / (j.upper - j.lower) : null;
    const out = ratio != null && (ratio < 0 || ratio > 1);
    return `<tr data-joint="${escape(j.name)}" class="${selected === j.name ? 'selected' : ''}" tabindex="0" aria-label="${escape(label(j.name))}，点击查看曲线"><td>${escape(label(j.name))}<small>${escape(j.name)}</small></td><td class="${present ? 'numeric' : 'missing'}">${number(present ? (radians ? q : q * 180 / Math.PI) : undefined, radians ? 4 : 2)}</td><td>${number(velocity, 3)}</td><td>${number(tau, 2)}</td><td><span class="limit-track"><span class="limit-fill ${out ? 'outside' : ''}" style="width:${ratio == null ? 0 : Math.min(100, Math.max(0, ratio * 100))}%"></span></span><span class="limit-text">${!present ? '--' : out ? '超模型限位' : '范围内'}</span></td></tr>`;
  }).join('');
}

function renderTrend() {
  const points = history.get(selected) || [];
  const now = state.server_time + Math.max(0, performance.now() - lastMessage) / 1000;
  const source = effective(state.channels.joint);
  const i = source.data?.names.indexOf(selected) ?? -1;
  const lastQ = i >= 0 ? source.data.q[i] : undefined;
  const factor = radians ? 1 : 180 / Math.PI;
  $('trend-name').textContent = label(selected);
  $('selected-native').textContent = selected;
  $('trend-value').innerHTML = number(lastQ == null ? undefined : lastQ * factor, radians ? 4 : 2) + '<small>' + (radians ? 'rad' : '°') + '</small>';
  const visible = points.filter(p => now - p.t <= 30);
  if (!visible.length) {
    $('trend-line').setAttribute('d', ''); $('trend-area').setAttribute('d', '');
    $('trend-min').textContent = '--'; $('trend-max').textContent = '--';
    return;
  }
  let low = Math.min(...visible.map(p => p.q * factor)), high = Math.max(...visible.map(p => p.q * factor));
  const padding = Math.max((high - low) * .2, radians ? .005 : .3);
  low -= padding; high += padding;
  let path = '', fill = '', segment = [];
  const flush = () => {
    if (!segment.length) return;
    path += 'M' + segment.map(p => p.join(',')).join('L');
    fill += 'M' + segment[0][0] + ',119L' + segment.map(p => p.join(',')).join('L') + 'L' + segment.at(-1)[0] + ',119Z';
    segment = [];
  };
  visible.forEach((p, index) => {
    if (index && p.t - visible[index - 1].t > .5) flush();
    segment.push([Math.max(0, 300 * (p.t - (now - 30)) / 30).toFixed(2), (110 - 100 * (p.q * factor - low) / (high - low)).toFixed(2)]);
  });
  flush();
  $('trend-line').setAttribute('d', path); $('trend-area').setAttribute('d', fill);
  $('trend-min').textContent = number(low, radians ? 3 : 1); $('trend-max').textContent = number(high, radians ? 3 : 1);
}

function renderHands() {
  const channel = effective(state.channels.hand), h = channel.data;
  const names = ['Thumb', 'ThumbAux', 'Index', 'Middle', 'Ring', 'Pinky'];
  $('hands-content').innerHTML = ['left', 'right'].map((side, s) => `<div class="hand-table"><h4>${s ? '右手 / RIGHT' : '左手 / LEFT'}<span>${escape(statusName(channel))} · 样本年龄 ${number(h?.[side + '_sample_age_ms'], 0)} ms</span></h4><table><thead><tr><th>通道</th><th>原始位置</th><th>原始速度</th><th>原始电流</th><th>状态码</th></tr></thead><tbody>${names.map((name, i) => { const n = s * 6 + i; return `<tr><td>${name}</td><td>${number(h?.position[n], 2)}</td><td>${number(h?.velocity[n], 2)}</td><td>${number(h?.current[n], 2)}</td><td>${h?.state[n] ?? '--'}</td></tr>`; }).join('')}</tbody></table></div>`).join('');
}

function renderSources() {
  const rows = [['joint', '原生关节反馈', '/rt/joint_state'], ['rgb', '头部 RGB', config?.color_topic || '--'], ['depth', '深度伪彩', config?.depth_topic || '--'], ['hand', '灵巧手反馈', '/rt/hand_state'], ['hardware', '下位机状态', '/rt/hardware/state'], ['controller', '上位机状态', '/controller_state']];
  $('source-cards').innerHTML = rows.map(([key, title, topic]) => {
    const c = effective(state.channels[key]);
    const bridge = state.bridges[key === 'joint' || key === 'hand' || key === 'hardware' ? 'lower' : 'upper'];
    const selectedTopic = state.channels.ros_status.data?.selected?.[key];
    return `<div class="source-card"><h4>${title}<span class="source-badge ${c.status === 'live' ? '' : 'warn'}">${statusName(c)}</span></h4><div class="source-value">${number(c.hz)} <small>Hz</small></div><p>${escape(c.topic || selectedTopic || topic)}<br>本机接收间隔：${number(c.age_ms, 0)} ms<br>源数据年龄：${number(c.source_age_ms, 0)} ms<br>采集器：${bridge.connected ? '已连接' : '未连接'} / 重连 ${bridge.restarts} 次</p>${bridge.error ? `<p class="error">${escape(bridge.error)}</p>` : ''}${key === 'joint' && !c.data ? '<p class="error">尚无原生反馈；请按现有流程启动机器人。本页面不启动控制器。</p>' : ''}</div>`;
  }).join('');
  $('camera-info-raw').textContent = pretty(state.channels.camera_info.data);
  $('config-raw').textContent = pretty(config);
  $('topics-raw').textContent = pretty(state.channels.ros_status.data?.topics);
  $('model-disclaimer').textContent = 'URDF SHA256: ' + model.sha256 + '\nS17 身体 23 轴 + Revo-2 左右各 11 个运动学关节（各 6 个主动通道、5 个 mimic）。双手继承现有工程安装坐标并随手臂运动。手指保持张开参考姿态；raw→rad 与物理安装方向未经本次实机标定。固定底座显示，未显示世界坐标位移。';
}

async function loadRecords() {
  try {
    const rows = await jsonFetch('/api/records');
    $('record-list').innerHTML = rows.length ? rows.map(r => `<div class="record-entry"><span>${escape(r.name)}</span><span>${byteSize(r.bytes)}${r.active ? ' · 正在记录' : ''}</span><a href="/api/records/${encodeURIComponent(r.name)}" download>下载 JSONL ↓</a></div>`).join('') : '<p class="empty-records">尚无反馈记录。点击右上角“记录反馈”开始一个新会话。</p>';
  } catch (e) { toast(e.message, true); }
}

function showDetail(name, scroll = true) {
  detail = name;
  for (const [key, id] of Object.entries({ joints: 'joints-detail', hands: 'hands-detail', controller: 'controller-detail-view', sources: 'sources-detail', records: 'records-detail' })) $(id).classList.toggle('hidden', key !== name);
  all('.data-tabs button').forEach(b => { b.classList.toggle('active', b.dataset.detail === name); b.setAttribute('aria-selected', String(b.dataset.detail === name)); });
  all('.rail-button').forEach(b => b.classList.toggle('active', b.dataset.section === name));
  renderDetails(true);
  if (name === 'records') loadRecords();
  if (scroll) $('data-panel').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function connect() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/api/ws');
  ws.onopen = () => { connected = true; clearTimeout(disconnectTimer); };
  ws.onmessage = (event) => { try { ingest(JSON.parse(event.data)); } catch (e) { console.error('Render update failed', e); } };
  ws.onclose = () => { connected = false; render(); disconnectTimer = setTimeout(connect, 1800); };
  ws.onerror = () => ws.close();
}

async function initialize() {
  setInterval(() => { $('clock').textContent = timeString(); if (state && performance.now() - lastMessage > 1200) render(); }, 500);
  all('[data-detail]').forEach(button => button.addEventListener('click', () => showDetail(button.dataset.detail, !button.closest('.data-tabs'))));
  all('.rail-button').forEach(button => button.addEventListener('click', () => {
    if (button.dataset.section === 'overview') {
      window.scrollTo({ top: 0, behavior: 'smooth' });
      all('.rail-button').forEach(b => b.classList.toggle('active', b === button));
    } else showDetail(button.dataset.section);
  }));
  all('[data-group]').forEach(button => button.addEventListener('click', () => { group = button.dataset.group; all('[data-group]').forEach(b => b.classList.toggle('active', b === button)); renderDetails(true); }));
  $('unit-toggle').addEventListener('click', () => { radians = !radians; $('unit-toggle').innerHTML = radians ? 'RAD <span>⇄</span> DEG' : 'DEG <span>⇄</span> RAD'; renderDetails(true); });
  $('pause-table').addEventListener('click', () => { paused = !paused; $('pause-table').textContent = paused ? '继续显示' : '暂停显示'; $('pause-table').classList.toggle('paused-indicator', paused); renderDetails(true); });
  const selectJoint = event => { const row = event.target.closest('[data-joint]'); if (row) { selected = row.dataset.joint; renderDetails(true); } };
  $('joint-table').addEventListener('click', selectJoint);
  $('joint-table').addEventListener('keydown', event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); selectJoint(event); } });
  all('[data-camera]').forEach(button => button.addEventListener('click', () => {
    camera = button.dataset.camera;
    all('[data-camera]').forEach(b => { b.classList.toggle('selected', b === button); b.setAttribute('aria-pressed', String(b === button)); });
    $('camera-image').src = '/api/stream/' + camera + '?session=' + Date.now();
    render();
  }));
  $('camera-image').src = '/api/stream/rgb';
  $('camera-image').addEventListener('error', () => { setTimeout(() => { $('camera-image').src = '/api/stream/' + camera + '?retry=' + Date.now(); }, 2000); });
  $('snapshot-button').addEventListener('click', async () => {
    try {
      const response = await fetch('/api/snapshot/' + camera);
      if (!response.ok) throw new Error(await response.text());
      const blob = await response.blob();
      const url = URL.createObjectURL(blob), a = document.createElement('a');
      a.href = url; a.download = 'roban2_' + camera + '_' + new Date().toISOString().replace(/[:.]/g, '-') + '.jpg'; a.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      toast('已保存当前画面');
    } catch (e) { toast(e.message, true); }
  });
  $('fullscreen-camera').addEventListener('click', async () => { try { document.fullscreenElement ? await document.exitFullscreen() : await $('camera-stage').requestFullscreen(); } catch { toast('当前浏览器暂不支持全屏', true); } });
  $('export-state').addEventListener('click', () => { const a = document.createElement('a'); a.href = '/api/export'; a.download = 'roban2_state.json'; a.click(); });
  $('record-button').addEventListener('click', async () => {
    if (recordBusy) return;
    recordBusy = true; $('record-button').disabled = true;
    try {
      const result = await action(state?.recording.active ? 'record-stop' : 'record-start');
      toast(result.recording.active ? '开始记录反馈；图像仅记录时间戳，视频不写入文件' : '反馈记录已保存，可在“反馈记录”中下载');
      lastRecordKey = ''; if (detail === 'records') loadRecords();
    } catch (e) { toast(e.message, true); }
    finally { recordBusy = false; $('record-button').disabled = false; }
  });
  $('refresh-records').addEventListener('click', loadRecords);
  $('reconnect').addEventListener('click', async () => { $('reconnect').disabled = true; try { await action('reconnect'); toast('正在重建反馈订阅'); } catch (e) { toast(e.message, true); } finally { $('reconnect').disabled = false; } });
  $('open-joints').addEventListener('click', () => showDetail('joints'));
  const results = await Promise.allSettled([jsonFetch('/api/model'), jsonFetch('/api/config')]);
  if (results[0].status === 'fulfilled') model = results[0].value;
  if (results[1].status === 'fulfilled') config = results[1].value;
  if (!model) toast('无法获取 URDF 元数据，请查看服务日志', true);
  try {
    viewer = new RobotViewer($('model-canvas'));
    viewer.load('/model/robot.urdf', (n, total) => { $('model-load-text').textContent = `加载模型网格 ${n} / ${total}`; }).then(result => {
      $('model-loading').classList.add('hidden');
      $('model-canvas').dataset.loaded = String(result.joints);
    }).catch(e => { $('model-load-text').textContent = e.message; toast(e.message, true); console.error(e); });
    all('[data-view]').forEach(button => button.addEventListener('click', () => { viewer.setView(button.dataset.view); all('[data-view]').forEach(b => b.classList.toggle('selected', b === button)); }));
    $('toggle-axes').addEventListener('click', () => $('toggle-axes').setAttribute('aria-pressed', String(viewer.toggleAxes())));
    $('toggle-wire').addEventListener('click', () => $('toggle-wire').setAttribute('aria-pressed', String(viewer.toggleWireframe())));
    $('reset-view').addEventListener('click', () => { viewer.setView('perspective'); all('[data-view]').forEach(b => b.classList.toggle('selected', b.dataset.view === 'perspective')); });
  } catch (e) {
    $('model-load-text').textContent = '3D 初始化失败：' + e.message;
    toast('浏览器 3D 不可用；画面与数据面板仍可使用', true);
  }
  connect();
}

initialize().catch(e => { console.error(e); toast('初始化失败：' + e.message, true); });
