// Live depth only. Original uint16 millimetres stay available for pixel readout.
export function decodeDepth(raw, width, height) {
  if (!(raw instanceof ArrayBuffer) || !Number.isInteger(width) || !Number.isInteger(height) ||
      width < 1 || height < 1 || width > 1920 || height > 1080 || raw.byteLength !== width * height * 2) {
    throw new Error('深度图尺寸与数据长度不一致');
  }
  if (new Uint8Array(new Uint16Array([1]).buffer)[0] === 1) return new Uint16Array(raw);
  const source = new DataView(raw), values = new Uint16Array(width * height);
  for (let i = 0; i < values.length; i++) values[i] = source.getUint16(i * 2, true);
  return values;
}

export function depthMeters(values, x, y, width, height) {
  if (!values || x < 0 || y < 0 || x >= width || y >= height) return null;
  const value = values[y * width + x];
  return value > 0 ? value / 1000 : null;
}

export function previewFresh(meta, now = Date.now()) {
  return !!meta?.fresh && Number.isFinite(meta.received_ns) &&
    (meta.latency_bound_ms || 0) + Math.max(0, now - meta.received_ns / 1e6) < 700;
}

export function autoRange(values) {
  // Ignore invalid pixels and isolated extremes; retain the actual uint16 values.
  const histogram = new Uint32Array(65536);
  let count = 0;
  for (const value of values) if (value > 0) { histogram[value]++; count++; }
  if (!count) return null;
  const lowRank = Math.max(1, Math.ceil(count * .02)), highRank = Math.ceil(count * .98);
  let cumulative = 0, low = 0, high = 0;
  for (let value = 1; value < histogram.length; value++) {
    cumulative += histogram[value];
    if (!low && cumulative >= lowRank) low = value;
    if (cumulative >= highRank) { high = value; break; }
  }
  // A flat scene must not turn millimetre noise into a full rainbow.
  let near = low / 1000, far = high / 1000;
  if (far - near < .1) {
    near = Math.max(0, Math.min(65.435, (near + far) / 2 - .05));
    far = Math.min(65.535, near + .1);
  }
  return [near, far];
}

export function smoothRange(previous, target, elapsedMs) {
  if (!previous) return [...target];
  const weight = 1 - Math.exp(-Math.max(0, Math.min(1000, elapsedMs)) / 650);
  return previous.map((value, i) => value + (target[i] - value) * weight);
}

export function makePalette(near, far, inverseDepth = false) {
  if (!Number.isFinite(near) || !Number.isFinite(far) || near < 0 || far <= near || far > 65.535) {
    throw new Error('请设置 0 ≤ 近端 < 远端 ≤ 65.535 米');
  }
  const palette = new Uint8ClampedArray(65536 * 4);
  const stops = [[239, 92, 66], [246, 211, 96], [75, 184, 164], [55, 106, 180]];
  // Values outside the display range share endpoint colours. Fill those in
  // bulk, instead of recalculating 65,536 colours on every AUTO update.
  const packed = new Uint32Array(palette.buffer);
  palette.set([...stops[3], 255], 0); packed.fill(packed[0]);
  palette.set([...stops[0], 255], 0);
  const first = Math.max(1, Math.floor(near * 1000) + 1), end = Math.min(65536, Math.ceil(far * 1000));
  packed.fill(packed[0], 1, Math.min(65536, first));
  const inverseNear = near > 0 ? 1 / near : 0, inverseSpan = inverseNear - 1 / far;
  for (let value = first; value < end; value++) {
    const offset = value * 4;
    // In AUTO, inverse depth gives nearby objects useful colour contrast even
    // when a small part of the view contains a distant wall. Readings stay metric.
    const fraction = inverseDepth && near > 0
      ? (inverseNear - 1000 / value) / inverseSpan
      : (value / 1000 - near) / (far - near);
    const t = Math.max(0, Math.min(1, fraction)) * 3;
    const index = Math.min(2, Math.floor(t)), alpha = t - index;
    for (let channel = 0; channel < 3; channel++) palette[offset + channel] = Math.round(
      stops[index][channel] * (1 - alpha) + stops[index + 1][channel] * alpha);
    palette[offset + 3] = 255;
  }
  palette.set([20, 37, 30, 255], 0);
  return palette;
}

function startDepthView() {
  const $ = id => document.getElementById(id), canvas = $('depthCanvas'), context = canvas.getContext('2d', { alpha: false });
  let values = null, image = null, palette = makePalette(.2, 3), packedPalette = new Uint32Array(palette.buffer);
  let meta = {}, probe = null, lastPaint = 0, painted = [], frames = 0, rangeError = '';
  let manualRange = [.2, 3], automaticRange = null, lastAutoUpdate = 0;
  const isAuto = () => $('depthRangeMode').value === 'auto';
  const format = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  const set = (id, value) => { $(id).textContent = value; };

  function readProbe() {
    const point = probe || [Math.floor(canvas.width / 2), Math.floor(canvas.height / 2)];
    const fresh = !!lastPaint && performance.now() - lastPaint < 700 && previewFresh(meta);
    const metres = fresh ? depthMeters(values, point[0], point[1], canvas.width, canvas.height) : null;
    set('depthProbeLabel', probe ? '指向像素深度' : '中心像素深度');
    set('depthDistance', !fresh ? '—' : metres === null ? '无有效深度' : `${format(metres, 3)} m`);
    set('depthCoordinate', `像素 ${point[0]}, ${point[1]}`);
  }

  function draw() {
    if (!values) return;
    const pixels = new Uint32Array(image.data.buffer);
    // ImageData and packedPalette share the platform's byte order.
    for (let i = 0; i < values.length; i++) pixels[i] = packedPalette[values[i]];
    context.putImageData(image, 0, 0);
    const point = probe || [Math.floor(canvas.width / 2), Math.floor(canvas.height / 2)];
    context.strokeStyle = '#ffffff'; context.lineWidth = 1.5; context.beginPath();
    context.moveTo(point[0] - 7, point[1]); context.lineTo(point[0] + 7, point[1]);
    context.moveTo(point[0], point[1] - 7); context.lineTo(point[0], point[1] + 7); context.stroke();
    readProbe();
  }

  function point(event) {
    const rect = canvas.getBoundingClientRect();
    probe = [Math.max(0, Math.min(canvas.width - 1, Math.floor((event.clientX - rect.left) / rect.width * canvas.width))),
      Math.max(0, Math.min(canvas.height - 1, Math.floor((event.clientY - rect.top) / rect.height * canvas.height)))];
    draw();
  }
  canvas.addEventListener('pointermove', point);
  canvas.addEventListener('pointerdown', point);
  canvas.addEventListener('pointerleave', () => { probe = null; draw(); });
  function rangeHint(message = '') {
    set('depthRangeHint', message || (isAuto()
      ? '自动色阶增强近处层次；颜色随场景调整，黑色表示无有效深度。'
      : '手动色阶；黑色表示无有效深度，范围外使用端点颜色。'));
  }
  function applyRange([near, far]) {
    palette = makePalette(near, far, isAuto()); packedPalette = new Uint32Array(palette.buffer);
    set('depthNearLabel', `${format(near, 2)} m · 近`); set('depthFarLabel', `${format(far, 2)} m · 远`);
    canvas.dataset.rangeNear = near; canvas.dataset.rangeFar = far;
    if (isAuto()) {
      $('depthNear').value = near.toFixed(3); $('depthFar').value = far.toFixed(3);
    }
  }
  function updateAutoRange(now) {
    if (!isAuto() || !values || now - lastAutoUpdate < 200) return;
    const target = autoRange(values);
    if (target) {
      automaticRange = smoothRange(automaticRange, target, now - lastAutoUpdate);
      applyRange(automaticRange); rangeHint();
    } else {
      automaticRange = null;
      rangeHint('当前没有有效深度，等待有效画面后自动调整色阶。');
    }
    lastAutoUpdate = now;
  }
  function updateRange() {
    if (isAuto()) return;
    try {
      const near = $('depthNear').valueAsNumber, far = $('depthFar').valueAsNumber;
      applyRange([near, far]); manualRange = [near, far]; rangeError = ''; draw();
    } catch (error) { rangeError = error.message; }
    rangeHint(rangeError);
  }
  for (const id of ['depthNear', 'depthFar']) {
    $(id).addEventListener('input', updateRange);
    $(id).addEventListener('change', updateRange);
  }
  $('depthRangeMode').addEventListener('change', () => {
    const automatic = isAuto();
    for (const id of ['depthNear', 'depthFar']) $(id).disabled = automatic;
    canvas.dataset.rangeMode = automatic ? 'auto' : 'manual';
    set('depthModeLabel', automatic ? '02 / DEPTH · AUTO RANGE' : '02 / DEPTH · MANUAL');
    automaticRange = null; lastAutoUpdate = 0; rangeError = ''; rangeHint();
    if (automatic) {
      if (lastPaint && performance.now() - lastPaint < 700) updateAutoRange(performance.now());
    } else {
      $('depthNear').value = manualRange[0]; $('depthFar').value = manualRange[1]; applyRange(manualRange);
    }
    draw();
  });
  canvas.dataset.rangeMode = 'auto';

  async function preview() {
    const browserInflates = typeof DecompressionStream === 'function';
    for (;;) {
      try {
        const response = await fetch('/api/depth/preview' + (browserInflates ? '' : '?raw=1'), { cache: 'no-store' });
        if (!response.ok) throw new Error('深度预览连接未就绪');
        const reader = response.body.getReader();
        let buffer = new Uint8Array(0);
        for (;;) {
          const result = await reader.read();
          if (result.done) throw new Error('深度流已断开，正在重连');
          const joined = new Uint8Array(buffer.length + result.value.length); joined.set(buffer); joined.set(result.value, buffer.length); buffer = joined;
          while (buffer.length >= 4) {
            const headerLength = new DataView(buffer.buffer, buffer.byteOffset, 4).getUint32(0);
            if (!headerLength || headerLength >= 16384) throw new Error('深度流头无效');
            if (buffer.length < 4 + headerLength) break;
            const header = JSON.parse(new TextDecoder().decode(buffer.subarray(4, 4 + headerLength)));
            if (!Number.isInteger(header.size) || header.size < 0 || header.size > 4 * 1024 * 1024) throw new Error('深度帧长度无效');
            const end = 4 + headerLength + header.size;
            if (buffer.length < end) break;
            const bytes = buffer.slice(4 + headerLength, end); buffer = buffer.slice(end);
            if (!header.size || !previewFresh(header)) continue;
            if (header.unit_m !== .001 || header.raw_size !== header.width * header.height * 2) throw new Error('深度单位或尺寸不符');
            let raw;
            if (header.codec === 'zlib-u16le' && browserInflates) {
              raw = await new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'))).arrayBuffer();
            } else if (header.codec === 'raw-u16le') raw = bytes.buffer;
            else throw new Error('不支持的深度流编码');
            if (!previewFresh(header)) continue;
            values = decodeDepth(raw, header.width, header.height); meta = header;
            if (canvas.width !== header.width || canvas.height !== header.height || !image) {
              canvas.width = header.width; canvas.height = header.height; image = context.createImageData(header.width, header.height); probe = null;
            }
            const now = performance.now();
            if (!lastPaint || now - lastPaint >= 700) { automaticRange = null; lastAutoUpdate = 0; }
            updateAutoRange(now);
            lastPaint = now; frames++; painted.push(lastPaint); painted = painted.filter(t => lastPaint - t < 3000);
            canvas.dataset.frames = frames; canvas.dataset.stampNs = header.stamp_ns; draw(); $('depthEmpty').hidden = true;
          }
        }
      } catch (error) {
        set('depthEmptyHint', error.message || '等待上位机深度图');
        await new Promise(resolve => setTimeout(resolve, 2000));
      }
    }
  }

  setInterval(() => {
    const now = performance.now(), fresh = !!lastPaint && now - lastPaint < 700 && previewFresh(meta);
    $('depthEmpty').hidden = fresh;
    if (!fresh) set('depthEmptyHint', lastPaint ? '深度画面已过期，正在重连' : '正在连接上位机深度图');
    const displayed = fresh && painted.length > 1 && painted.at(-1) - painted[0] >= 1500
      ? (painted.length - 1) * 1000 / (painted.at(-1) - painted[0]) : null;
    set('depthFps', `${format(fresh ? meta.source_fps : null)} FPS`);
    set('depthPreviewFps', `接收 ${format(fresh ? meta.relay_fps : null)} · 显示 ${format(displayed)} FPS · 时延 ≤ ${format(fresh ? meta.latency_bound_ms : null)} ms`);
    set('depthSourceHint', fresh ? `${meta.width} × ${meta.height} · ${meta.frame_id?.includes('color') ? 'RGB视角配准' : '深度视角'} · 仅实时查看` : 'Gemini 335 · 等待深度数据');
    const rgbStamp = $('previewCanvas').dataset.stampNs;
    const delta = fresh && rgbStamp && meta.stamp_ns ? Math.abs(Number(BigInt(meta.stamp_ns) - BigInt(rgbStamp))) / 1e6 : null;
    set('depthSync', delta === null ? '与RGB时间差 —' : `最近显示帧时间差 ${format(delta)} ms`);
    canvas.dataset.displayedFps = displayed ?? ''; readProbe();
  }, 250);
  preview();
}

if (typeof document !== 'undefined' && document.getElementById('depthCanvas')) startDepthView();
