// LawnBot dashboard — vanilla JS + Canvas + one WebSocket.
//
// State arrives every 100 ms (10 Hz). We render the map on every state and
// send commands either via the WebSocket (cmd/payload JSON) or POST /cmd/<name>.

(() => {
  const canvas = document.getElementById('map');
  const ctx = canvas.getContext('2d');
  const evList = document.getElementById('events');
  const teachHint = document.getElementById('teach-hint');
  const breakdown = document.getElementById('pid-breakdown');
  const stateChip = document.getElementById('state-chip');
  const conn = document.getElementById('conn');

  let state = null;
  let view = { scale: 30, ox: 0, oy: 0, follow: true };  // pixels per meter
  let teleopActive = false;
  let teleopHB = 0;

  // ---- WebSocket -----------------------------------------------------
  let ws;
  function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => { conn.textContent = 'connected'; conn.className = 'chip ok'; };
    ws.onclose = () => { conn.textContent = 'disconnected'; conn.className = 'chip warn'; setTimeout(connect, 1000); };
    ws.onmessage = (m) => { state = JSON.parse(m.data); render(); };
  }
  connect();

  function send(cmd, payload) {
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify({ cmd, payload: payload || {} }));
    }
  }

  // ---- canvas sizing -------------------------------------------------
  function resize() {
    const r = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = r.width * dpr;
    canvas.height = r.height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  window.addEventListener('resize', resize);
  resize();

  // World → screen transform (north up, east right).
  function w2s(x, y) {
    const r = canvas.getBoundingClientRect();
    const px = r.width / 2 + (x - view.ox) * view.scale;
    const py = r.height / 2 - (y - view.oy) * view.scale;
    return [px, py];
  }

  // ---- render --------------------------------------------------------
  function render() {
    if (!state) return;
    stateChip.textContent = state.mission?.state ?? 'IDLE';
    const r = canvas.getBoundingClientRect();
    ctx.clearRect(0, 0, r.width, r.height);

    if (view.follow && state.pose) {
      view.ox = state.pose.x;
      view.oy = state.pose.y;
    }

    drawGrid(r);
    if (state.boundary) drawPolygon(state.boundary, '#1f3a23', '#39613f', 2);
    if (state.keepouts) state.keepouts.forEach(k => drawPolygon(k.points, '#3a1f1f', '#874141', 2));
    if (document.getElementById('layer-path').checked && state.path) drawPath(state.path, '#3d9ec6');
    if (document.getElementById('layer-coverage').checked && state.covered) drawPath(state.covered, '#2c6b4d', 4);
    if (document.getElementById('layer-teach').checked && state.teach && state.teach.track) drawPath(state.teach.track, '#e0a338', 2, true);
    if (document.getElementById('layer-gps').checked && state.gps_xy) drawCross(state.gps_xy[0], state.gps_xy[1], '#e89052');
    if (state.pose) drawRover(state.pose);
    if (state.target) drawReach(state.target, state.reach ?? 0.22);
    drawHUD();
  }

  function drawGrid(r) {
    ctx.strokeStyle = '#141a22';
    ctx.lineWidth = 1;
    const step = 1; // 1 m
    const wMin = view.ox - (r.width / 2) / view.scale;
    const wMax = view.ox + (r.width / 2) / view.scale;
    const hMin = view.oy - (r.height / 2) / view.scale;
    const hMax = view.oy + (r.height / 2) / view.scale;
    ctx.beginPath();
    for (let x = Math.floor(wMin); x <= Math.ceil(wMax); x += step) {
      const [px] = w2s(x, 0);
      ctx.moveTo(px, 0); ctx.lineTo(px, r.height);
    }
    for (let y = Math.floor(hMin); y <= Math.ceil(hMax); y += step) {
      const [, py] = w2s(0, y);
      ctx.moveTo(0, py); ctx.lineTo(r.width, py);
    }
    ctx.stroke();
  }

  function drawPolygon(pts, fill, stroke, w) {
    if (!pts || pts.length < 2) return;
    ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = w || 1;
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) {
      const [px, py] = w2s(pts[i][0], pts[i][1]);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.closePath(); ctx.fill(); ctx.stroke();
  }

  function drawPath(pts, color, width, dashed) {
    if (!pts || pts.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = width || 1.5;
    if (dashed) ctx.setLineDash([5, 4]); else ctx.setLineDash([]);
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) {
      const [px, py] = w2s(pts[i][0], pts[i][1]);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function drawCross(x, y, color) {
    const [px, py] = w2s(x, y);
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(px - 6, py); ctx.lineTo(px + 6, py);
    ctx.moveTo(px, py - 6); ctx.lineTo(px, py + 6);
    ctx.stroke();
  }

  function drawRover(pose) {
    const [px, py] = w2s(pose.x, pose.y);
    const len = 14;
    const dx = Math.cos(pose.theta) * len;
    const dy = -Math.sin(pose.theta) * len;
    ctx.fillStyle = '#67d3f5'; ctx.strokeStyle = '#0c1014';
    ctx.beginPath();
    ctx.moveTo(px + dx, py + dy);
    ctx.lineTo(px - dx * 0.5 + dy * 0.4, py - dy * 0.5 - dx * 0.4);
    ctx.lineTo(px - dx * 0.5 - dy * 0.4, py - dy * 0.5 + dx * 0.4);
    ctx.closePath();
    ctx.fill(); ctx.stroke();
  }

  function drawReach(target, reach) {
    const [px, py] = w2s(target[0], target[1]);
    ctx.strokeStyle = '#b5d33b'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(px, py, reach * view.scale, 0, Math.PI * 2);
    ctx.stroke();
  }

  function drawHUD() {
    const q = ['invalid', 'single', 'DGPS', 'PPS', 'RTK-fixed', 'RTK-float'];
    const s = state;
    document.getElementById('hud-fix').textContent = q[s.gps?.quality ?? 0] || '-';
    document.getElementById('hud-age').textContent = (s.gps?.age_s ?? 0).toFixed(2);
    document.getElementById('hud-sats').textContent = s.gps?.sats ?? '-';
    document.getElementById('hud-batt').textContent = (s.battery?.percent ?? -1).toFixed(0);
    document.getElementById('hud-heading').textContent = ((s.pose?.theta ?? 0) * 180 / Math.PI).toFixed(0);
    document.getElementById('hud-err').textContent = ((s.control?.heading_err ?? 0) * 180 / Math.PI).toFixed(1);
    document.getElementById('hud-cross').textContent = (s.control?.cross_track ?? 0).toFixed(2);
    document.getElementById('hud-wp').textContent = `${s.mission?.waypoint_idx ?? 0}/${s.mission?.n_waypoints ?? 0}`;
    document.getElementById('hud-cov').textContent = (s.mission?.coverage_pct ?? 0).toFixed(0);

    if (s.teach && s.teach.distance_to_close != null) {
      teachHint.textContent = `${s.teach.distance_to_close.toFixed(2)} m to close`;
    } else if (s.teach && s.teach.mode && s.teach.mode !== 'idle') {
      teachHint.textContent = `recording: ${s.teach.mode}`;
    } else { teachHint.textContent = ''; }

    if (s.control?.pid) {
      breakdown.textContent = `P ${s.control.pid.p.toFixed(2)}  I ${s.control.pid.i.toFixed(2)}  D ${s.control.pid.d.toFixed(2)}`;
    }

    // Events feed
    if (s.events && s.events.length) {
      evList.innerHTML = '';
      s.events.slice(-8).reverse().forEach(e => {
        const li = document.createElement('li');
        li.textContent = e;
        evList.appendChild(li);
      });
    }
  }

  // ---- panel buttons → commands -------------------------------------
  document.querySelectorAll('[data-cmd]').forEach(b => {
    b.addEventListener('click', () => send(b.dataset.cmd));
  });

  // ---- live tuning sliders -------------------------------------------
  function tuningHandler() {
    send('control.tune', {
      kp: parseFloat(document.getElementById('kp').value),
      ki: parseFloat(document.getElementById('ki').value),
      kd: parseFloat(document.getElementById('kd').value),
      v_nominal: parseFloat(document.getElementById('vnom').value),
      lookahead_m: parseFloat(document.getElementById('ld').value),
    });
  }
  ['kp', 'ki', 'kd', 'vnom', 'ld'].forEach(id => {
    document.getElementById(id).addEventListener('input', tuningHandler);
  });

  // ---- pan/zoom on the map ------------------------------------------
  let drag = null;
  canvas.addEventListener('mousedown', e => { drag = [e.clientX, e.clientY]; view.follow = false; });
  canvas.addEventListener('mouseup', () => drag = null);
  canvas.addEventListener('mouseleave', () => drag = null);
  canvas.addEventListener('mousemove', e => {
    if (!drag) return;
    view.ox -= (e.clientX - drag[0]) / view.scale;
    view.oy += (e.clientY - drag[1]) / view.scale;
    drag = [e.clientX, e.clientY];
  });
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const k = Math.exp(-e.deltaY * 0.001);
    view.scale = Math.max(3, Math.min(200, view.scale * k));
  }, { passive: false });

  // ---- joystick (touch + mouse) → teleop -----------------------------
  const joy = document.getElementById('joy');
  const stick = document.getElementById('stick');
  let joyDrag = false;
  function joyEvent(clientX, clientY) {
    const r = joy.getBoundingClientRect();
    const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
    const dx = clientX - cx, dy = clientY - cy;
    const max = r.width / 2 - 22;
    const m = Math.hypot(dx, dy);
    const s = m > max ? max / m : 1;
    const x = dx * s, y = dy * s;
    stick.style.left = (48 + x) + 'px';
    stick.style.top = (48 + y) + 'px';
    // Map: up (negative y) = forward speed; x = steering
    const v = -y / max;
    const delta = (x / max);
    send('teleop', { v, delta });
    teleopHB = performance.now();
  }
  function resetStick() { stick.style.left = '48px'; stick.style.top = '48px'; send('teleop', { v: 0, delta: 0 }); }
  joy.addEventListener('pointerdown', e => { joyDrag = true; joy.setPointerCapture(e.pointerId); joyEvent(e.clientX, e.clientY); });
  joy.addEventListener('pointermove', e => { if (joyDrag) joyEvent(e.clientX, e.clientY); });
  joy.addEventListener('pointerup', () => { joyDrag = false; resetStick(); });
  joy.addEventListener('pointercancel', () => { joyDrag = false; resetStick(); });

  // ---- keyboard teleop (arrows / WASD) -------------------------------
  const keys = { v: 0, d: 0 };
  function pushKey() { send('teleop', { v: keys.v, delta: keys.d }); teleopHB = performance.now(); }
  document.addEventListener('keydown', e => {
    if (e.repeat) return;
    if (e.key === 'ArrowUp' || e.key === 'w') keys.v = 0.4;
    else if (e.key === 'ArrowDown' || e.key === 's') keys.v = -0.3;
    else if (e.key === 'ArrowLeft' || e.key === 'a') keys.d = -0.5;
    else if (e.key === 'ArrowRight' || e.key === 'd') keys.d = 0.5;
    else return;
    pushKey();
  });
  document.addEventListener('keyup', e => {
    if (e.key === 'ArrowUp' || e.key === 'ArrowDown' || e.key === 'w' || e.key === 's') keys.v = 0;
    else if (e.key === 'ArrowLeft' || e.key === 'ArrowRight' || e.key === 'a' || e.key === 'd') keys.d = 0;
    else return;
    pushKey();
  });

  // Heartbeat repeater: send the current teleop command at 10 Hz while a key/stick is engaged.
  setInterval(() => {
    if ((keys.v !== 0 || keys.d !== 0) || joyDrag) pushKey();
  }, 100);

  // Double-click recenters and re-engages follow mode.
  canvas.addEventListener('dblclick', () => { view.follow = true; });
})();
