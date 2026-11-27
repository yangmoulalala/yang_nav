(() => {
  const workspace = element('terrain-workspace');
  const statusLine = element('terrain-status');
  const canvas = element('terrain-canvas');
  const context = canvas.getContext('2d');
  const tool = element('surface-tool');
  const documentButtons = [...element('terrain-documents').querySelectorAll('[data-frame]')];
  const referenceInput = element('terrain-reference-source');
  const registerForm = element('terrain-register');
  const guessFields = ['x', 'y', 'z', 'yaw'];
  const selectionLayer = document.createElement('canvas');
  const selectionContext = selectionLayer.getContext('2d');
  let state;
  let image;
  let owners;
  let selection = new Set();
  let details;
  let busy = false;
  let gesture;
  let pendingMapSource = false;
  let view = {x: 0, y: 0, scale: 1};

  const project = () => state?.project;
  const patchPath = (part) => `terrain/${part}?revision=${encodeURIComponent(state.revision)}`;
  const readNumbers = (form) => Object.fromEntries([...new FormData(form)].filter(([, value]) => value !== '')
    .map(([key, value]) => [key, Number(value)]));

  function setGuess(guess) {
    guessFields.forEach((name, index) => { registerForm.elements[name].value = guess?.[index] ?? 0; });
  }

  function showReferences() {
    const {projects = [], models = []} = config?.terrain_references || {};
    const loaded = project()?.reference;
    const placeholder = new Option('选择场地工程或模型', '');
    const archives = document.createElement('optgroup');
    archives.label = '场地标注工程';
    archives.replaceChildren(...projects.map(({path, annotated}) => new Option(`${path} · ${annotated} 个已标注`, path)));
    const meshes = document.createElement('optgroup');
    meshes.label = '原始场地模型';
    meshes.replaceChildren(...models.map((path) => new Option(path, path)));
    const selected = referenceInput.value;
    referenceInput.replaceChildren(placeholder, ...[archives, meshes].filter((group) => group.children.length));
    if (loaded && ![...referenceInput.options].some((option) => option.value === loaded)) {
      referenceInput.add(new Option(loaded, loaded));
    }
    referenceInput.value = selected && [...referenceInput.options].some((option) => option.value === selected)
      ? selected : (loaded || '');
    controls();
  }

  function controls() {
    element('terrain').setAttribute('aria-busy', String(busy));
    element('terrain-reload').disabled = busy;
    for (const button of element('terrain').querySelectorAll('form button')) button.disabled = busy;
    for (const button of documentButtons) button.disabled = busy;
    const selected = selection.size;
    element('patch-type').disabled = element('patch-passable').disabled = busy || !selected;
    element('patch-direction').querySelector('button').disabled = busy || !selected;
    const referenceChanged = referenceInput.value && referenceInput.value !== project()?.reference;
    element('terrain-reference').querySelector('button').disabled = busy || !referenceChanged;
    registerForm.querySelector('[type="submit"]').disabled = busy || !project()?.reference || !!referenceChanged;
    element('terrain-auto-seed').disabled = busy || !project()?.registration_seed;
    for (const button of workspace.querySelectorAll('[data-edit]')) {
      const action = button.dataset.edit;
      button.disabled = busy || (!['undo', 'redo', 'adopt'].includes(action) && !selected) ||
        (action === 'merge' && selected < 2) || (['grow', 'shrink'].includes(action) && selected !== 1) ||
        (action === 'adopt' && (!project()?.can_adopt || !!referenceChanged));
    }
  }

  function sizeCanvas() {
    const bounds = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    const width = Math.round(bounds.width * ratio);
    const height = Math.round(bounds.height * ratio);
    if (width && height && (canvas.width !== width || canvas.height !== height)) {
      canvas.width = width;
      canvas.height = height;
    }
    return {width: bounds.width, height: bounds.height, ratio};
  }

  function fit() {
    if (!image || canvas.clientWidth === 0) return;
    const {width, height} = sizeCanvas();
    const scale = Math.min((width - 24) / image.width, (height - 24) / image.height);
    view = {scale, x: (width - image.width * scale) / 2, y: (height - image.height * scale) / 2};
    draw();
  }

  function zoom(factor, x = canvas.clientWidth / 2, y = canvas.clientHeight / 2) {
    if (!image) return;
    const minimum = Math.min(canvas.clientWidth / image.width, canvas.clientHeight / image.height) / 3;
    const scale = Math.min(48, Math.max(minimum, view.scale * factor));
    const change = scale / view.scale;
    view = {scale, x: x - (x - view.x) * change, y: y - (y - view.y) * change};
    draw();
  }

  function draw() {
    if (!image || !canvas.clientWidth) return;
    const {width, height, ratio} = sizeCanvas();
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    context.imageSmoothingEnabled = false;
    const dimensions = [view.x, view.y, image.width * view.scale, image.height * view.scale];
    context.drawImage(image, ...dimensions);
    if (selection.size) context.drawImage(selectionLayer, ...dimensions);
    if (details?.arrows) {
      const map = project();
      for (const [x, y, angle] of details.arrows) {
        const px = view.x + (x - map.origin[0]) / map.resolution * view.scale;
        const py = view.y + (map.height - (y - map.origin[1]) / map.resolution) * view.scale;
        context.save();
        context.translate(px, py);
        context.rotate(-angle * Math.PI / 180);
        context.beginPath();
        context.moveTo(-17, 0);
        context.lineTo(17, 0);
        context.lineTo(8, -6);
        context.moveTo(17, 0);
        context.lineTo(8, 6);
        context.strokeStyle = '#17201f';
        context.lineWidth = 5;
        context.stroke();
        context.strokeStyle = '#fff5bc';
        context.lineWidth = 2;
        context.stroke();
        context.restore();
      }
    }
    if (!gesture || gesture.kind === 'pan' || gesture.kind === 'select') return;
    const {points, kind} = gesture;
    const [first, last] = [points[0], points.at(-1)];
    const point = ([col, row]) => [view.x + (col + 0.5) * view.scale,
      view.y + (image.height - row - 0.5) * view.scale];
    const [left, top] = point(first);
    const [right, bottom] = point(last);
    context.strokeStyle = '#d6fff1';
    context.fillStyle = '#76e8ca30';
    context.lineWidth = kind === 'brush' ? Math.max(2, element('brush-radius').valueAsNumber * 2 * view.scale) : 1.5;
    context.lineCap = context.lineJoin = 'round';
    context.beginPath();
    if (['box', 'rectangle'].includes(kind)) {
      context.rect(left, top, right - left, bottom - top);
      context.fill();
    } else {
      context.moveTo(left, top);
      for (const cell of points.slice(1)) context.lineTo(...point(cell));
      if (kind === 'polygon') { context.closePath(); context.fill(); }
    }
    context.stroke();
  }

  function drawSelection() {
    if (!owners) return;
    selectionLayer.width = image.width;
    selectionLayer.height = image.height;
    if (selection.size) {
      const mask = selectionContext.createImageData(image.width, image.height);
      for (let index = 0; index < owners.length; index++) {
        const id = owners[index];
        if (!selection.has(id)) continue;
        const edge = index % image.width === 0 || owners[index - 1] !== id || owners[index - image.width] !== id;
        const offset = 4 * index;
        mask.data[offset] = 255;
        mask.data[offset + 1] = 229;
        mask.data[offset + 2] = 133;
        mask.data[offset + 3] = edge ? 245 : 78;
      }
      selectionContext.putImageData(mask, 0, 0);
    }
    draw();
  }

  function patchList() {
    const container = element('patch-list');
    const fragment = document.createDocumentFragment();
    const unmarkedOnly = element('patch-unmarked').checked;
    for (const patch of (project()?.patches || []).filter((row) => !unmarkedOnly || row.type === -1)) {
      const item = document.createElement('button');
      const label = document.createElement('span');
      const area = document.createElement('small');
      item.type = 'button';
      item.dataset.patch = patch.id;
      item.setAttribute('aria-pressed', String(selection.has(patch.id)));
      label.textContent = `#${patch.id} · ${state.types[patch.type] || '保留类型'}${patch.passable ? '' : ' · 禁行'}`;
      area.textContent = `${patch.area.toFixed(2)} m²`;
      item.append(label, area);
      fragment.append(item);
    }
    container.replaceChildren(fragment);
  }

  async function select(ids) {
    selection = new Set(ids);
    details = undefined;
    const chosen = project().patches.filter((row) => selection.has(row.id));
    const [first] = chosen;
    const same = (key) => first && chosen.every((row) => row[key] === first[key]) ? String(first[key]) : '';
    element('terrain-selection').textContent = chosen.length ? `已选 ${chosen.length} 个` : '未选择';
    element('patch-type').value = same('type');
    element('patch-passable').value = same('passable');
    element('patch-direction').elements.angle.value = first?.dir ?? '';
    element('patch-info').textContent = chosen.length ? `选中面积 ${chosen.reduce((sum, row) => sum + row.area, 0).toFixed(2)} m²` : '';
    for (const item of element('patch-list').children) {
      item.setAttribute('aria-pressed', String(selection.has(Number(item.dataset.patch))));
    }
    drawSelection();
    controls();
    if (chosen.length !== 1) return;
    const request = selection;
    try {
      const result = await api(`${patchPath('patch')}&id=${first.id}`);
      if (request !== selection) return;
      details = result;
      const {incline, area, length, width, zmin, zmax, clearance} = result.info;
      const meters = (value) => Number.isFinite(value) ? `${value.toFixed(2)} m` : '—';
      const directions = result.arrows.map((arrow) => `${arrow[2].toFixed(1)}°`).join(' / ');
      element('patch-info').textContent = `${area.toFixed(2)} m² · 坡度 ${incline.toFixed(1)}° · 方向 ${directions}\n` +
        `尺寸 ${meters(length)} × ${meters(width)} · 高程 ${meters(zmin)} ～ ${meters(zmax)}\n净空 ${meters(clearance)}`;
      draw();
    } catch (error) {
      if (request === selection) element('patch-info').textContent = error.message;
    }
  }

  async function load(recenter = false, ids = [...selection]) {
    const next = await api('terrain');
    const fresh = !state?.project || state.active !== next.active || state.project.source !== next.project?.source || !image;
    for (const button of documentButtons) {
      const frame = button.dataset.frame;
      const current = next.documents[frame];
      const name = current?.source.split('/').at(-1);
      button.setAttribute('aria-pressed', String(frame === next.active));
      button.querySelector('small').textContent = current ? `${name} · ${current.patches} 个面片${current.dirty ? ' · 未保存' : ''}` : '未打开';
      button.title = current?.source || '';
      element(`terrain-open-${frame}`).hidden = frame !== next.active;
    }
    element('terrain-discard-hint').hidden = !next.documents[next.active]?.dirty;
    if (!next.project) {
      state = next;
      selection = new Set();
      details = undefined;
      image?.close();
      image = owners = undefined;
      workspace.hidden = true;
      statusLine.textContent = next.active === 'field' ? '打开 STEP 或已保存的场地标注工程。' : '打开建图目录、PCD 或已保存的点云标注工程。';
      return;
    }
    if (state?.revision !== next.revision || !image) {
      const read = async (part, method) => {
        const response = await fetch(`/api/terrain/${part}?revision=${encodeURIComponent(next.revision)}`);
        if (!response.ok) throw new Error((await response.json()).error);
        return response[method]();
      };
      const [preview, data] = await Promise.all([read('preview', 'blob'), read('owners', 'arrayBuffer')]);
      const bitmap = await createImageBitmap(preview);
      if (data.byteLength !== bitmap.width * bitmap.height * 4) {
        bitmap.close();
        throw new Error('地图栅格尺寸不一致，请重新读取工程');
      }
      image?.close();
      image = bitmap;
      owners = new Uint32Array(data);
    }
    const referenceChanged = state?.project?.reference !== next.project.reference;
    state = next;
    workspace.hidden = false;
    const map = project();
    const marked = map.patches.filter((row) => row.type >= 0).length;
    statusLine.textContent = `${map.frame === 'map' ? '地图坐标' : '场地坐标'} · ${map.resolution.toFixed(3)} m/格 · ` +
      `${marked}/${map.patches.length} 个面片已标注${map.dirty ? ' · 有未保存修改' : ''}`;
    if (recenter || fresh) {
      const source = map.source.replace(/\.surf\.npz$|\.(pcd|stl|step|stp|npz)$/i, '');
      const base = source.startsWith('maps/') ? source : `maps/${source.split('/').at(-1) || 'terrain'}`;
      element('terrain-save').elements.output.value = map.archive || (map.directory ? `${base}/annotation.surf.npz` : `${base}.surf.npz`);
      element('terrain-export').elements.output.value = map.directory ? `${base}/terrain_web` : `${base}_terrain`;
      fit();
    }
    if (map.frame === 'map' && (recenter || fresh || referenceChanged)) setGuess(map.registration_seed);
    if (referenceChanged) referenceInput.value = '';
    showReferences();
    element('patch-type').replaceChildren(new Option('批量设置', ''),
      ...Object.entries(state.types).sort(([left], [right]) => (+left === -1 ? 99 : +left) - (+right === -1 ? 99 : +right))
        .map(([id, name]) => new Option(name, id)));
    const segment = element('terrain-segment');
    for (const input of segment.querySelectorAll('input')) input.value = map.params[input.name];
    element('terrain-registration').hidden = map.frame !== 'map';
    const reg = map.registration;
    const reference = map.reference
      ? `参考 ${map.reference.split('/').at(-1)} · ${state.documents.field.annotated} 个已标注`
      : '请选择已标注的场地工程，再载入参考。';
    const indicator = element('terrain-registration-result');
    const goodFit = reg && reg.residual_p50 < 0.06 && reg.inlier_ratio_8cm > 0.6;
    indicator.classList.toggle('warn', !!reg && !goodFit);
    const residuals = reg ? `${goodFit ? '配准符合参考指标' : '配准可疑'} · ` +
      `P50 ${reg.residual_p50.toFixed(3)} m / P90 ${reg.residual_p90.toFixed(3)} m · ` +
      `内点 ${(100 * reg.inlier_ratio_8cm).toFixed(0)}% · ` +
      `平移 (${reg.t_world_map.map((value) => value.toFixed(2)).join(', ')}) m · 航向 ${reg.rpy_deg[2].toFixed(1)}°` : '';
    indicator.textContent = [reference, map.can_adopt ? `青色为参考边界 · ${residuals}` : (reg && `上次配准 · ${residuals}`)]
      .filter(Boolean).join(' · ');
    patchList();
    const present = new Set(map.patches.map((row) => row.id));
    await select(fresh ? [] : ids.filter((pid) => present.has(pid)));
  }

  async function run(operation, payload = {}, recenter = false) {
    if (busy) return;
    busy = true;
    controls();
    if (['open', 'reference'].includes(operation) && /\.st(?:ep|p)$/i.test(payload.source.trim())) {
      message('正在识别 STEP 场地。大型文件首次打开可能需要几分钟。', false, true);
    } else if (operation && operation !== 'switch') {
      message('正在处理地形工程…', false, true);
    }
    try {
      if (!state) state = await api('terrain');
      const result = operation ? await api(`terrain/${operation}`, {selected: [...selection], ...payload,
        revision: state.revision}) : {};
      if (operation === 'connected') await select(result.selected);
      else if (operation === 'seed') setGuess(result.guess);
      else await load(recenter, result.selected ?? (recenter ? [] : [...selection]));
      if (result.message) message(result.message);
      if (result.output || (operation === 'save' && state.active === 'field')) await loadConfig();
    } catch (error) {
      message(error.message, true);
    } finally {
      busy = false;
      controls();
    }
  }

  function cellAt(x, y, clamp = false) {
    const col = Math.floor((x - view.x) / view.scale);
    const row = image.height - 1 - Math.floor((y - view.y) / view.scale);
    if (clamp) return [Math.max(0, Math.min(image.width - 1, col)), Math.max(0, Math.min(image.height - 1, row))];
    return col >= 0 && row >= 0 && col < image.width && row < image.height ? [col, row] : null;
  }

  const ownerAt = ([col, row]) => owners[(image.height - 1 - row) * image.width + col];

  function pick(ids, toggle) {
    const result = new Set(toggle ? selection : []);
    for (const id of ids) {
      if (toggle && result.has(id)) result.delete(id);
      else if (id) result.add(id);
    }
    select(result);
  }

  function pointer(event) {
    const bounds = canvas.getBoundingClientRect();
    return [event.clientX - bounds.left, event.clientY - bounds.top];
  }

  canvas.addEventListener('pointerdown', (event) => {
    if (!image || busy || gesture || ![0, 1, 2].includes(event.button)) return;
    const cursor = pointer(event);
    const pan = event.button !== 0 || tool.value === 'pan';
    const kind = pan ? 'pan' : (tool.value === 'select' && event.shiftKey ? 'box' : tool.value);
    const cell = cellAt(...cursor);
    if (!pan && !cell) return;
    event.preventDefault();
    canvas.setPointerCapture(event.pointerId);
    gesture = {id: event.pointerId, kind, points: [cell], cursor, toggle: event.ctrlKey || event.metaKey};
    draw();
  });
  canvas.addEventListener('pointermove', (event) => {
    if (!image) return;
    const cursor = pointer(event);
    const hovered = cellAt(...cursor);
    const map = project();
    element('terrain-cursor').textContent = hovered ? `${map.frame} · ` +
      `X ${(map.origin[0] + (hovered[0] + 0.5) * map.resolution).toFixed(2)} m · ` +
      `Y ${(map.origin[1] + (hovered[1] + 0.5) * map.resolution).toFixed(2)} m` : '滚轮缩放 · 中键或右键拖动平移';
    if (gesture?.id !== event.pointerId) return;
    if (gesture.kind === 'pan') {
      view.x += cursor[0] - gesture.cursor[0];
      view.y += cursor[1] - gesture.cursor[1];
      gesture.cursor = cursor;
    } else if (gesture.kind !== 'select') {
      const point = cellAt(...cursor, true);
      if (['box', 'rectangle', 'line'].includes(gesture.kind)) gesture.points[1] = point;
      else if (gesture.points.length < 499 && point.some((value, axis) => value !== gesture.points.at(-1)[axis])) gesture.points.push(point);
    }
    draw();
  });
  canvas.addEventListener('pointerup', (event) => {
    if (gesture?.id !== event.pointerId) return;
    const {kind, points, toggle} = gesture;
    gesture = undefined;
    draw();
    if (kind === 'pan') return;
    const end = cellAt(...pointer(event), true);
    if (kind === 'select') return pick([ownerAt(end)], toggle);
    if (kind === 'box') {
      const [[startCol, startRow], [endCol, endRow]] = [points[0], end];
      const ids = new Set();
      for (let row = Math.min(startRow, endRow); row <= Math.max(startRow, endRow); row++) {
        for (let col = Math.min(startCol, endCol); col <= Math.max(startCol, endCol); col++) ids.add(ownerAt([col, row]));
      }
      return pick(ids, toggle);
    }
    if (['rectangle', 'line'].includes(kind)) points[1] = end;
    else if (end.some((value, axis) => value !== points.at(-1)[axis])) points.push(end);
    run('region', {tool: kind, points, mode: element('region-mode').value,
      radius: element('brush-radius').valueAsNumber, clip: element('clip-selection').checked});
  });
  canvas.addEventListener('lostpointercapture', () => { gesture = undefined; draw(); });
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());
  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    zoom(event.deltaY < 0 ? 1.18 : 1 / 1.18, ...pointer(event));
  }, {passive: false});
  new ResizeObserver(draw).observe(canvas);
  element('terrain-fit').addEventListener('click', fit);
  element('terrain-zoom-in').addEventListener('click', () => zoom(1.35));
  element('terrain-zoom-out').addEventListener('click', () => zoom(1 / 1.35));
  tool.addEventListener('change', () => {
    for (const id of ['region-mode-field', 'clip-field']) element(id).hidden = !['brush', 'rectangle', 'polygon'].includes(tool.value);
    element('brush-radius-field').hidden = tool.value !== 'brush';
    canvas.dataset.tool = tool.value;
  });
  element('patch-list').addEventListener('click', (event) => {
    const item = event.target.closest('[data-patch]');
    if (item && !busy) pick([Number(item.dataset.patch)], event.ctrlKey || event.metaKey);
  });
  element('patch-list').addEventListener('dblclick', (event) => {
    const id = Number(event.target.closest('[data-patch]')?.dataset.patch);
    const row = project()?.patches.find((patch) => patch.id === id);
    if (!row || busy) return;
    const map = project();
    view.x = canvas.clientWidth / 2 - (row.x - map.origin[0]) / map.resolution * view.scale;
    view.y = canvas.clientHeight / 2 - (map.height - (row.y - map.origin[1]) / map.resolution) * view.scale;
    draw();
  });
  element('patch-unmarked').addEventListener('change', patchList);
  element('patch-type').addEventListener('change', (event) => {
    if (event.target.value !== '') run('annotate', {type: Number(event.target.value)});
  });
  element('patch-passable').addEventListener('change', (event) => {
    if (event.target.value !== '') run('annotate', {passable: event.target.value === 'true'});
  });
  workspace.addEventListener('click', (event) => {
    const command = event.target.closest('[data-edit]')?.dataset.edit;
    if (!command) return;
    run(command);
  });
  function onSubmit(form, handler) {
    element(form).addEventListener('submit', (event) => { event.preventDefault(); handler(event.target); });
  }
  for (const button of documentButtons) {
    const frame = button.dataset.frame;
    button.addEventListener('click', () => { if (state?.active !== frame) run('switch', {frame}, true); });
    onSubmit(`terrain-open-${frame}`, (form) => {
      const {source, offset_x, offset_y, offset_z, ...params} = Object.fromEntries(new FormData(form));
      let offset;
      if (frame === 'field' && /\.st(?:ep|p)$/i.test(source.trim())) {
        const coordinates = [offset_x, offset_y, offset_z];
        if (coordinates.some((value) => value !== '')) {
          if (coordinates.includes('')) return message('请填写 STEP 原点的全部三个分量，或全部留空。', true);
          offset = coordinates.map(Number);
        }
      }
      const discard = state?.documents[frame]?.dirty === true;
      run('open', {source, target: frame, discard, offset,
        params: Object.fromEntries(Object.entries(params).filter(([, value]) => value !== '').map(([key, value]) => [key, Number(value)]))}, true);
    });
  }
  onSubmit('terrain-save', (form) => {
    const output = form.elements.output.value;
    run('save', {output});
  });
  onSubmit('terrain-export', (form) => {
    const {output, ...options} = Object.fromEntries(new FormData(form));
    run('export', {output, ...Object.fromEntries(Object.entries(options).map(([key, value]) => [key, Number(value)]))});
  });
  onSubmit('terrain-segment', (form) => run('segment', {params: readNumbers(form)}));
  referenceInput.addEventListener('change', controls);
  document.addEventListener('terrain-sources', showReferences);
  onSubmit('terrain-reference', (form) => {
    const source = form.elements.source.value;
    if (!source || source === project()?.reference) return;
    const discard = state?.documents.field?.dirty === true;
    run('reference', {source, discard});
  });
  onSubmit('terrain-seed', (form) => run('seed', {source: form.elements.source.value}));
  element('terrain-auto-seed').addEventListener('click', () => setGuess(project().registration_seed));
  onSubmit('terrain-register', (form) => run('register', {guess: guessFields.map((name) => form.elements[name].valueAsNumber)}));
  onSubmit('patch-direction', (form) => run('annotate', {dir: form.elements.angle.valueAsNumber}));
  element('terrain-reload').addEventListener('click', () => run());
  document.addEventListener('terrain-map-source', (event) => {
    element('terrain-open-map').elements.source.value = event.detail;
    pendingMapSource = true;
  });
  document.addEventListener('nav-page', (event) => {
    if (event.detail !== 'terrain') return;
    if (pendingMapSource) run('switch', {frame: 'map'}, true);
    else run();
    pendingMapSource = false;
  });
  if (location.hash === '#terrain') run();
})();
