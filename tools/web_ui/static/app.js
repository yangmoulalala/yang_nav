const element = (id) => document.getElementById(id);
const setup = element('setup-form');
const tuning = element('tuning-form');
const bag = element('bag-form');
const optimizer = element('optimizer-form');
const processes = {gazebo: 'Gazebo', localization: '定位 / 建图', navigation: 'Nav2'};
const processLabels = {running: '运行中', stopped: '已停止', failed: '异常退出',
  stopping: '正在关闭', external: '外部运行', cancelled: '已取消'};
const locationLabels = {NORMAL: '正常', OBS_DEGRADED: '观测退化', SUSPECT: '可疑',
  LOST: '已丢失', INITIALIZING: '初始化'};
const locationReasons = { 'operator requested global relocalization': '等待新点云进行全图匹配',
  'global relocalization failed': '全图候选未通过验证，可在特征更明显处重试' };
const profileLabels = {sim: '仿真', real: '真车'};
const terrainLabels = {normal: '普通地形', tunnel: '隧道', ramp: '坡道', recovery: '恢复'};
let config;
let status;
let busy = false;
let noticeTimeout;

function message(content, error = false, persistent = false) {
  const notice = element('notice');
  notice.hidden = false;
  notice.textContent = content;
  notice.classList.toggle('bad', error);
  clearTimeout(noticeTimeout);
  if (!persistent) noticeTimeout = setTimeout(() => { notice.hidden = true; }, 6500);
}

async function api(route, body) {
  const options = body === undefined ? {} : {method: 'POST', body: JSON.stringify(body),
    headers: {'Content-Type': 'application/json', 'X-Nav-Console': '1'}};
  const response = await fetch(`/api/${route}`, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

async function operate(route, body = {}, reloadConfig = false) {
  if (busy) return;
  busy = true;
  updateControls();
  message(route === 'stack/stop' ? '正在关闭，等待录包收尾和进程退出…' : '正在执行…', false, true);
  try {
    const result = await api(route, body);
    if (reloadConfig) await loadConfig();
    message(result.message);
  } catch (error) {
    message(error.message, true);
  } finally {
    busy = false;
    updateControls();
    await updateStatus();
  }
}

function paintSetup(profile) {
  const values = config[profile];
  setup.elements.profile.value = profile;
  for (const [name, value] of Object.entries(values)) {
    const field = setup.elements.namedItem(name);
    if (!field) continue;
    if (field.type === 'checkbox') field.checked = value;
    else field.value = value;
  }
  updateSetupVisibility();
}

function updateSetupVisibility() {
  for (const label of setup.querySelectorAll('[data-profile], [data-mode], [data-lidar]')) {
    label.hidden = ['profile', 'mode', 'lidar'].some((name) =>
      label.dataset[name] && label.dataset[name] !== setup.elements[name].value);
  }
}

async function loadConfig() {
  config = await api('config');
  paintSetup(config.profile);
  element('map-options').replaceChildren(...config.maps.map((path) => new Option(path, path)));
  element('terrain-options').replaceChildren(...config.terrain.map((path) => new Option(path, path)));
  const sources = config.terrain_sources || [];
  const isStep = (path) => /\.st(?:ep|p)$/i.test(path);
  const fieldSources = [...sources.filter(isStep), ...sources.filter((path) => !isStep(path) && !/\.pcd$/i.test(path))];
  element('surface-field-options').replaceChildren(...fieldSources.map((path) => new Option(path, path)));
  element('surface-map-options').replaceChildren(...[...config.maps, ...sources.filter((path) => /\.(pcd|surf\.npz)$/i.test(path))]
    .map((path) => new Option(path, path)));
  element('terrain-seed-options').replaceChildren(...config.terrain.map((path) => new Option(`${path}/terrain.json`)));
  document.dispatchEvent(new Event('terrain-sources'));
  const sourceInput = optimizer.elements.source;
  if (!sourceInput.value || sourceInput.value === sourceInput.dataset.suggested) {
    const current = config[config.profile];
    sourceInput.value = current.mode === 'mapping' && config.maps.includes(current.map_dir)
      ? current.map_dir : current.prior_map;
    sourceInput.dataset.suggested = sourceInput.value;
    suggestOptimizerOutput();
  }
  element('tuning-profile').textContent = `当前调参配置 · ${profileLabels[config.profile]}`;
  for (const component of ['navigation', 'localization']) {
    const labels = config.fields[config.profile].filter((field) => field.component === component).map((field) => {
      const label = document.createElement('label');
      label.textContent = `${field.label} · ${field.unit}`;
      const input = document.createElement('input');
      input.type = 'number';
      input.name = field.key;
      input.min = field.min;
      input.max = field.max;
      input.step = field.step === 1 ? 1 : 'any';
      input.placeholder = `默认 ${field.default}`;
      input.value = config[config.profile].tuning[field.key] ?? '';
      label.append(input);
      return label;
    });
    element(`tuning-${component}`).replaceChildren(...labels);
  }
  updateControls();
}

function processPill(id, row) {
  const pill = element(id);
  pill.dataset.state = row.state;
  pill.textContent = processLabels[row.state];
}

function setupProcessRows() {
  for (const [name, title] of Object.entries(processes)) {
    const row = document.createElement('div');
    row.className = 'process';
    row.id = `row-${name}`;
    const info = document.createElement('div');
    const heading = document.createElement('b');
    heading.textContent = title;
    const badge = document.createElement('span');
    badge.id = `badge-${name}`;
    badge.className = 'pill';
    const detail = document.createElement('small');
    detail.id = `detail-${name}`;
    info.append(heading, ' ', badge, detail);
    const actions = document.createElement('div');
    actions.className = 'actions';
    for (const [action, label] of [['start', '启动'], ['stop', '停止'], ['restart', '重启']]) {
      const button = document.createElement('button');
      button.textContent = label;
      button.dataset.action = `process/${name}/${action}`;
      actions.append(button);
    }
    row.append(info, actions);
    element('processes').append(row);
  }
}

function hasExtrapolatedPose(localization) {
  return localization?.status === 'SUSPECT' && localization.pose_extrapolated === 'true';
}

function displayStatus() {
  const {ros, profile, processes: rows} = status;
  const loc = ros.localization;
  const ready = loc?.pose_trusted === 'true';
  const extrapolated = hasExtrapolatedPose(loc);
  element('domain').textContent = `ROS 域 ${status.domain}`;
  element('active-profile').textContent = `${profileLabels[profile]} · 运行控制`;
  element('loc-status').textContent = locationLabels[loc?.status] || loc?.status || '—';
  element('loc-status').className = loc ? (ready ? 'good' : 'warn') : '';
  const reason = locationReasons[loc?.reason] || loc?.reason;
  const trust = ready ? '位姿可信' : extrapolated ? '里程计前推 · 限速通行' : '位姿未可信';
  element('loc-trust').textContent = loc ? `${trust}${reason ? ` · ${reason}` : ''}` : '等待定位数据';
  const format = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  element('pose').textContent = ros.pose ? `${format(ros.pose.x)} / ${format(ros.pose.y)}` : '—';
  element('yaw').textContent = ros.pose ? `${ros.pose.frame} · 航向 ${format(ros.pose.yaw, 1)}°` : 'x / y · m';
  element('speed').textContent = `${format(ros.measured?.speed)} / ${format(ros.command?.speed)}`;
  element('velocity').textContent = `m/s${ros.mode ? ` · ${terrainLabels[ros.mode] || ros.mode}` : ''}`;
  element('nav2').textContent = ros.nav2 === 'active' ? '已激活' : ros.nav2 || '—';
  element('nav2').className = ros.nav2 === 'active' ? 'good' : '';
  element('chassis').textContent = ros.chassis === undefined ? '底盘反馈 —' : `底盘反馈${ros.chassis ? '在线' : '离线'}`;
  element('row-gazebo').hidden = profile !== 'sim' && rows.gazebo.state !== 'external';
  for (const name of ['gazebo', 'localization', 'navigation']) {
    const row = rows[name];
    processPill(`badge-${name}`, row);
    const mode = row.mode ? `${row.mode === 'mapping' ? '建图' : '先验匹配'} · ` : '';
    const path = row.state === 'running' && row.output ? `${mode}${row.output}` : '';
    element(`detail-${name}`).textContent = row.state === 'failed' ? `退出码 ${row.code} · 查看日志` : path;
  }
  processPill('bag-status', rows.bag);
  const foxStatus = element('fox-status');
  foxStatus.dataset.state = status.foxglove_online ? 'running' : 'stopped';
  foxStatus.textContent = status.foxglove_online ? '已发现 Bridge' : '当前 ROS 域未发现';
  element('bag-output').textContent = rows.bag.output ?? '';
  if (rows.optimizer) {
    const task = rows.optimizer;
    processPill('optimizer-state', task);
    if (task.code === 0 && task.output) element('optimizer-state').textContent = '已完成';
    element('optimizer-output').textContent = task.output ?? '';
    element('optimizer-annotate').hidden = !(task.code === 0 && task.output);
  } else {
    element('optimizer-state').textContent = '等待后台更新';
  }
  element('fox-address').value = `ws://${location.hostname}:${status.foxglove_port}`;
}

function updateControls() {
  const rows = status?.processes;
  const ros = status?.ros;
  const mode = rows?.localization.mode || config?.[status?.profile]?.mode;
  const unavailable = busy || !rows;
  for (const button of document.querySelectorAll('button[data-action]')) {
    const action = button.dataset.action;
    let enabled = !unavailable;
    if (enabled && action.startsWith('process/')) {
      const [, name, command] = action.split('/');
      const row = rows[name];
      const running = row?.state === 'running';
      enabled = command === 'start' ? ['stopped', 'failed'].includes(row?.state) : running;
      if (name === 'navigation' && mode === 'mapping' && command === 'start') enabled = false;
    }
    if (enabled && action === 'save_map') enabled = !!ros.services.save_map && (mode === 'mapping' || rows.localization.state === 'external');
    if (enabled && action === 'relocalize') enabled = !!ros.services.relocalize &&
      (mode !== 'mapping' || rows.localization.state === 'external') &&
      !['running', 'external', 'stopping'].includes(rows.navigation.state);
    if (enabled && action === 'cancel_goal') enabled = !!ros.services.cancel_goal || rows.goal.state === 'running';
    button.disabled = !enabled;
  }
  element('bag-start').disabled = unavailable || !['stopped', 'failed'].includes(rows.bag.state);
  element('optimizer-start').disabled = unavailable || !['stopped', 'failed', 'cancelled'].includes(rows?.optimizer?.state);
  const poseAvailable = ros?.localization?.pose_trusted === 'true' || hasExtrapolatedPose(ros?.localization);
  element('send-goal').disabled = unavailable || ros.nav2 !== 'active' || !poseAvailable || rows.goal.state === 'running';
  for (const form of [setup, tuning]) form.querySelector('[type="submit"]').disabled = busy || !config;
}

function confirmAction(text) {
  const dialog = element('confirmation');
  element('confirm-text').textContent = text;
  dialog.returnValue = 'cancel';
  return new Promise((resolve) => {
    dialog.addEventListener('close', () => resolve(dialog.returnValue === 'yes'), {once: true});
    dialog.showModal();
  });
}

document.addEventListener('click', async (event) => {
  const button = event.target.closest('button[data-action]');
  if (!button || button.disabled) return;
  const action = button.dataset.action;
  if (action === 'relocalize' && !await confirmAction('确认 Nav2 已停止、车辆已停稳。触发后定位将进入未可信状态，直到匹配通过。')) return;
  if (action === 'stack/stop' && !await confirmAction('确认车辆已停稳。录包收尾后，将关闭此工作台启动的进程。')) return;
  if (action === 'process/gazebo/stop' && status?.processes.localization.state === 'running' &&
      !await confirmAction('停止 Gazebo 会同时关闭定位和 Nav2。建图时请先保存地图。')) return;
  if (status?.profile === 'real' && (action === 'stack/start' || /process\/(localization|navigation)\/(start|restart|stop)$/.test(action)) &&
      !await confirmAction('即将操作真车进程，请确认车辆周围安全。')) return;
  await operate(action, {}, action === 'save_map');
});

setup.addEventListener('change', (event) => {
  if (event.target.name === 'profile') paintSetup(event.target.value);
  else updateSetupVisibility();
});
setup.addEventListener('submit', (event) => {
  event.preventDefault();
  const body = Object.fromEntries(new FormData(setup));
  body.gui = setup.elements.gui.checked;
  body.uwb = setup.elements.uwb.checked;
  operate('setup', body, true);
});
tuning.addEventListener('submit', (event) => {
  event.preventDefault();
  const values = Object.fromEntries(Array.from(new FormData(tuning))
    .filter(([, value]) => value !== '').map(([name, value]) => [name, Number(value)]));
  operate('tune', {profile: config.profile, values}, true);
});
bag.addEventListener('submit', (event) => {
  event.preventDefault();
  const body = Object.fromEntries(new FormData(bag));
  body.duration = Number(body.duration);
  body.all = bag.elements.all.checked;
  operate('process/bag/start', body);
});
function suggestOptimizerOutput() {
  const form = optimizer.elements;
  if (!form.output.value || form.output.dataset.suggested === form.output.value) {
    const source = form.source.value.trim().replace(/\/+$/, '');
    form.output.value = source ? `${source}_optimized` : '';
    form.output.dataset.suggested = form.output.value;
  }
}
optimizer.elements.source.addEventListener('input', suggestOptimizerOutput);
optimizer.elements.raycasting.addEventListener('change', () => {
  element('optimizer-origin').hidden = !optimizer.elements.raycasting.checked;
});
optimizer.addEventListener('submit', (event) => {
  event.preventDefault();
  const form = optimizer.elements;
  operate('process/optimizer/start', {source: form.source.value, output: form.output.value,
    threads: Number(form.threads.value), mls: form.mls.checked, raycasting: form.raycasting.checked,
    origin: [form.origin_x, form.origin_y, form.origin_z].map((input) => Number(input.value))});
});
element('optimizer-annotate').addEventListener('click', async () => {
  const output = status?.processes.optimizer?.output;
  if (!output) return;
  document.dispatchEvent(new CustomEvent('terrain-map-source', {detail: output}));
  location.hash = 'terrain';
  await loadConfig();
});
element('goal-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  const {x, y} = Object.fromEntries(new FormData(event.target));
  if (!await confirmAction(`发送地图目标 (${x}, ${y}) m，确认行驶区域安全。`)) return;
  operate('goal', {x: Number(x), y: Number(y)});
});
element('copy-fox').addEventListener('click', async () => {
  const input = element('fox-address');
  if (navigator.clipboard) {
    try {
      await navigator.clipboard.writeText(input.value);
      message('地址已复制');
      return;
    } catch (_) {
      input.focus();
    }
  }
  input.select();
  message(document.execCommand('copy') ? '地址已复制' : '地址已选中，可手动复制');
});

async function updateStatus() {
  const badge = element('connection');
  try {
    status = await api('status');
    displayStatus();
    badge.dataset.state = 'running';
    badge.textContent = '已连接';
  } catch (_) {
    status = undefined;
    badge.dataset.state = 'failed';
    badge.textContent = '连接中断';
  }
  updateControls();
}

async function updateLog() {
  const optimizing = location.hash === '#optimizer';
  if (!optimizing && location.hash !== '#logs') return;
  const name = optimizing ? 'optimizer' : element('log-name').value;
  const log = element(optimizing ? 'optimizer-log' : 'log-text');
  try {
    const data = await api(`log/${name}`);
    if (!optimizing && name !== element('log-name').value) return;
    const follow = log.scrollHeight - log.scrollTop - log.clientHeight < 25 || log.dataset.name !== name;
    const text = data.text.replace(/\x1b\[[0-9;]*[a-z]/gi, '') || '暂无日志';
    if (log.textContent !== text) log.textContent = text;
    if (follow) log.scrollTop = log.scrollHeight;
    log.dataset.name = name;
  } catch (_) {
    log.textContent = '日志暂时不可用';
  }
}

function showPage() {
  const selected = [...document.querySelectorAll('.sidebar nav a')].find((link) => link.hash === location.hash)?.hash || '#overview';
  for (const page of document.querySelectorAll('main > section')) page.hidden = `#${page.id}` !== selected;
  for (const link of document.querySelectorAll('.sidebar nav a')) {
    link.classList.toggle('selected', link.hash === selected);
    if (link.hash === selected) link.setAttribute('aria-current', 'page');
    else link.removeAttribute('aria-current');
  }
  window.scrollTo(0, 0);
  updateLog();
  document.dispatchEvent(new CustomEvent('nav-page', {detail: selected.slice(1)}));
}

setupProcessRows();
showPage();
window.addEventListener('hashchange', showPage);
element('log-name').addEventListener('change', updateLog);
updateControls();
loadConfig().catch((error) => message(error.message, true));
updateStatus();
setInterval(updateStatus, 1000);
setInterval(updateLog, 3000);
