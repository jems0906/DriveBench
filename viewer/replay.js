const TICK_MS = 100;
const cases = {
  cyclist_lane_entry: { number: '01', title: 'Cyclist lane entry', actor: 'Cyclist' },
  pedestrian_crossing: { number: '02', title: 'Pedestrian crossing', actor: 'Pedestrian' },
};
const byId = (id) => document.getElementById(id);
const panels = byId('panels');
const timeline = byId('timeline');
const play = byId('play');
const previous = byId('previous');
const next = byId('next');
const rate = byId('rate');
const switches = [...document.querySelectorAll('[data-case]')];
const reference = new URL('../reports/reference/', import.meta.url);
const cache = new Map();
let report;
let active;
let selected;
let tick = 0;
let playback = null;

const seconds = (value) => `${(value * TICK_MS / 1000).toFixed(1)} s`;
const terminal = (frames) => frames.at(-1).tick;

async function json(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Could not load ${url.pathname} (${response.status})`);
  return response.json();
}

async function load(id) {
  if (cache.has(id)) return cache.get(id);
  report ??= json(new URL('comparison.json', reference)).catch((error) => {
    report = undefined;
    throw error;
  });
  const { pairs } = await report;
  const pair = pairs.find((entry) => entry.input === `${id}.json` && entry.seed === 42);
  if (!pair?.replay) throw new Error('Missing reference replay');
  const [replay, scenario] = await Promise.all([
    json(new URL(pair.replay, reference)),
    json(new URL(`../scenarios/${id}.json`, import.meta.url)),
  ]);
  const runs = ['baseline', 'candidate'].map((key) => ({ key, summary: pair[key], frames: replay[key] }));
  const last = Math.max(...runs.map(({ frames }) => terminal(frames)));
  const points = [...scenario.road.lanes.flat(), ...scenario.route];
  for (const { frames } of runs) {
    for (const frame of frames) {
      for (const box of [frame.ego, ...frame.actors]) points.push(box.position);
    }
  }
  const xs = points.map(([east]) => east);
  const ys = points.map(([, north]) => north);
  const bounds = [Math.min(...xs) - 3, Math.max(...xs) + 3, Math.min(...ys) - 3, Math.max(...ys) + 3];
  const result = { runs, scenario, last, bounds };
  cache.set(id, result);
  return result;
}

function makePanel(run) {
  const card = byId('panel-template').content.firstElementChild.cloneNode(true);
  card.dataset.run = run.key;
  const field = (name) => card.querySelector(`.${name}`);
  field('version-role').textContent = run.key.toUpperCase();
  field('version').textContent = run.summary.driver_version;
  const verdict = field('verdict');
  verdict.dataset.result = run.summary.passed ? 'pass' : 'fail';
  verdict.textContent = run.summary.passed ? 'PASS' : run.summary.failures.includes('collision') ? 'FAIL: COLLISION' : 'FAIL';
  verdict.title = run.summary.failures.join(', ').replaceAll('_', ' ');
  const canvas = card.querySelector('canvas');
  const event = field('map-event');
  event.dataset.result = verdict.dataset.result;
  panels.append(card);
  return { card, canvas, event, speed: field('speed'), accel: field('accel'), time: field('run-time') };
}

function draw(canvas, model, frame, history, collision) {
  const context = canvas.getContext('2d');
  const { width, height } = canvas;
  const [left, right, bottom, top] = model.bounds;
  const scale = Math.min((width - 72) / (right - left), (height - 72) / (top - bottom));
  const project = ([east, north]) => [width / 2 + (east - (left + right) / 2) * scale,
    height / 2 - (north - (bottom + top) / 2) * scale];
  function line(points, color, size, dash = []) {
    context.beginPath();
    points.forEach((point, index) => {
      const position = project(point);
      if (index) context.lineTo(...position);
      else context.moveTo(...position);
    });
    context.strokeStyle = color;
    context.lineWidth = size;
    context.setLineDash(dash);
    context.stroke();
    context.setLineDash([]);
  }
  context.fillStyle = '#141f21';
  context.fillRect(0, 0, width, height);
  for (let east = Math.floor(left / 5) * 5; east <= right + 5; east += 5) {
    line([[east, bottom - 30], [east, top + 30]], '#243436', 1);
  }
  for (let north = Math.floor((bottom - 20) / 5) * 5; north <= top + 20; north += 5) {
    line([[left - 10, north], [right + 10, north]], '#243436', 1);
  }
  for (const lane of model.scenario.road.lanes) {
    context.beginPath();
    lane.forEach((point, index) => {
      if (index) context.lineTo(...project(point));
      else context.moveTo(...project(point));
    });
    context.closePath();
    context.fillStyle = '#293a3d';
    context.fill();
    context.strokeStyle = '#607173';
    context.lineWidth = 2;
    context.stroke();
  }
  line(model.scenario.route, '#a4b8ae', 2, [10, 9]);
  line(history.map((sample) => sample.ego.position), '#65dec0', 3);
  const goal = project(model.scenario.route.at(-1));
  context.beginPath();
  context.arc(...goal, scale * model.scenario.goal_radius, 0, Math.PI * 2);
  context.fillStyle = '#5da98425';
  context.fill();
  context.strokeStyle = '#80bb94';
  context.lineWidth = 2;
  context.stroke();
  context.fillStyle = '#b7d7c3';
  context.font = '500 16px monospace';
  context.textAlign = 'center';
  context.fillText('GOAL', goal[0], goal[1] - model.scenario.goal_radius * scale - 9);
  box(frame.ego, collision ? '#ff8e72' : '#65dec0', true);
  for (const actor of frame.actors) box(actor, '#ffbe76', false);
  const scaleY = height - 23;
  const scaleStart = width - 25 - scale * 5;
  context.fillStyle = '#9fb5af';
  context.fillRect(scaleStart, scaleY, scale * 5, 2);
  context.fillRect(scaleStart, scaleY - 5, 2, 7);
  context.fillRect(width - 27, scaleY - 5, 2, 7);
  context.fillText('5 m', scaleStart + scale * 2.5, scaleY - 8);

  function box(actor, color, ego) {
    const length = actor.length * scale;
    const width = actor.width * scale;
    context.save();
    context.translate(...project(actor.position));
    context.rotate(-actor.heading);
    context.fillStyle = color;
    context.strokeStyle = ego && collision ? '#ffd4c3' : '#102421';
    context.lineWidth = ego && collision ? 3 : 1.5;
    context.beginPath();
    context.roundRect(-length / 2, -width / 2, length, width, Math.min(5, width / 4));
    context.fill();
    context.stroke();
    if (ego) {
      context.fillStyle = collision ? '#6d3224' : '#256052';
      context.fillRect(length * .14, -width * .34, length * .15, width * .68);
    } else {
      context.beginPath();
      context.arc(0, 0, Math.max(12, Math.hypot(length, width) / 2 + 3), 0, Math.PI * 2);
      context.strokeStyle = '#ffbe768c';
      context.lineWidth = 1.5;
      context.stroke();
    }
    context.restore();
  }
}

function render() {
  if (!active) return;
  timeline.value = tick;
  timeline.setAttribute('aria-valuetext', seconds(tick));
  byId('clock').value = seconds(tick);
  previous.disabled = tick === 0;
  next.disabled = tick === active.last;
  for (const run of active.runs) {
    const last = terminal(run.frames);
    const frame = run.frames[Math.min(tick, last)];
    const ended = tick >= last;
    const collision = ended && run.summary.failures.includes('collision');
    const { card, canvas, event, speed, accel, time } = run.view;
    card.dataset.tick = frame.tick;
    speed.textContent = frame.ego.speed.toFixed(1);
    accel.textContent = ended ? '\u2014' : frame.action.acceleration.toFixed(1);
    accel.dataset.braking = !ended && frame.action.acceleration < -0.05;
    time.textContent = (frame.tick * TICK_MS / 1000).toFixed(1);
    event.hidden = !ended;
    event.textContent = `${collision ? 'COLLISION' : frame.goal ? 'GOAL REACHED' : 'RUN ENDED'} / ${seconds(frame.tick)}`;
    canvas.setAttribute('aria-label', `${run.summary.driver_version} map at ${seconds(frame.tick)}${ended ? `, ${event.textContent}` : ''}`);
    draw(canvas, active, frame, run.frames.slice(0, frame.tick + 1), collision);
  }
}

function setPlaying(playing) {
  play.querySelector('.play-symbol').hidden = playing;
  play.querySelector('.pause-symbol').hidden = !playing;
  play.title = playing ? 'Pause' : 'Play';
  play.setAttribute('aria-label', play.title);
}

function pause() {
  if (playback) cancelAnimationFrame(playback.request);
  playback = null;
  setPlaying(false);
}

function seek(position) {
  if (!active) return;
  pause();
  tick = Math.max(0, Math.min(active.last, position));
  render();
}

function start() {
  if (!active || playback) return;
  if (tick >= active.last) seek(0);
  setPlaying(true);
  playback = { time: performance.now(), tick, rate: Number(rate.value), request: requestAnimationFrame(advance) };
}

function advance(now) {
  if (!playback) return;
  const position = Math.min(active.last, playback.tick + Math.floor((now - playback.time) * playback.rate / TICK_MS));
  if (position > tick) {
    tick = position;
    render();
  }
  if (tick >= active.last) pause();
  else playback.request = requestAnimationFrame(advance);
}

async function select(id) {
  pause();
  selected = id;
  active = undefined;
  tick = 0;
  const config = cases[id];
  for (const button of switches) button.setAttribute('aria-pressed', button.dataset.case === id);
  byId('case-number').textContent = config.number;
  byId('case-title').textContent = config.title;
  byId('actor-label').textContent = config.actor;
  byId('finding').textContent = 'Loading replay';
  byId('finding').dataset.tone = 'neutral';
  byId('load-error').hidden = true;
  panels.setAttribute('aria-busy', 'true');
  panels.replaceChildren();
  for (const control of [play, previous, next, timeline]) control.disabled = true;
  try {
    const model = await load(id);
    if (selected !== id || active) return;
    active = { ...model, runs: model.runs.map((run) => ({ ...run, view: makePanel(run) })) };
    const candidatePass = model.runs[1].summary.passed;
    byId('finding').dataset.tone = candidatePass ? 'positive' : 'negative';
    byId('finding').textContent = candidatePass ? 'Improvement / v2 avoids collision' : 'Regression / v2 collides';
    const collision = model.runs.find((run) => run.summary.failures.includes('collision'));
    byId('marker').textContent = collision ? `Collision at ${seconds(terminal(collision.frames))}` : '';
    byId('duration').textContent = seconds(model.last);
    byId('end-label').textContent = seconds(model.last);
    timeline.max = model.last;
    timeline.disabled = play.disabled = false;
    render();
  } catch (error) {
    if (selected !== id || active) return;
    byId('finding').textContent = 'Replay unavailable';
    byId('load-error').textContent = error.message;
    byId('load-error').hidden = false;
  } finally {
    if (selected === id) panels.setAttribute('aria-busy', 'false');
  }
}

for (const button of switches) button.addEventListener('click', () => select(button.dataset.case));
play.addEventListener('click', () => playback ? pause() : start());
previous.addEventListener('click', () => seek(tick - 1));
next.addEventListener('click', () => seek(tick + 1));
timeline.addEventListener('input', () => seek(Number(timeline.value)));
rate.addEventListener('change', () => {
  if (playback) {
    pause();
    start();
  }
});
window.addEventListener('DOMContentLoaded', () => window.lucide?.createIcons());
select('cyclist_lane_entry');