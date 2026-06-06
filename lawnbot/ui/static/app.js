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

  // ---- Backend connection (Sim local ↔ Real Pi on network) -----------
  //
  // backendBase = '' → relative to whichever server delivered this page
  //                    (typically the local sim on Windows).
  // backendBase = 'host[:port]' → connect to a specific lawnbot server on
  //                    the network — usually the Pi at one of the candidate
  //                    addresses below.
  const PI_CANDIDATES = [
    'lawnbot.local:8080',
    'raspberrypi.local:8080',
    '192.168.4.1:8080',
    '10.42.0.1:8080',
  ];

  let backendBase = '';
  let backendMode = 'sim';
  let backendBusy = false;            // true while probing / connecting; suppresses snapshot mirror
  let reconnectTimer = null;

  try {
    const saved = JSON.parse(localStorage.getItem('lawnbot_backend') || '{}');
    if (saved.mode === 'real' && saved.host) {
      backendBase = saved.host;
      backendMode = 'real';
    } else if (saved.mode === 'sim') {
      backendMode = 'sim';
    }
  } catch (e) {}

  function wsBaseUrl() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const host = backendBase || location.host;
    return `${proto}://${host}/ws`;
  }
  function backendLabel() {
    return backendBase ? backendBase : (location.host || 'local');
  }

  // ---- WebSocket -----------------------------------------------------
  let ws;
  function connect() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    try { if (ws) ws.close(); } catch (e) {}
    ws = new WebSocket(wsBaseUrl());
    ws.onopen = () => {
      conn.textContent = `connected · ${backendLabel()}`; conn.className = 'chip ok';
    };
    ws.onclose = () => {
      conn.textContent = `disconnected · ${backendLabel()}`; conn.className = 'chip warn';
      reconnectTimer = setTimeout(connect, 1000);
    };
    ws.onerror = () => {
      // Trigger a label change so the user sees the bad host immediately.
      conn.textContent = `error · ${backendLabel()}`; conn.className = 'chip bad';
    };
    ws.onmessage = (m) => { state = JSON.parse(m.data); render(); };
  }
  connect();

  function send(cmd, payload) {
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify({ cmd, payload: payload || {} }));
    }
  }

  // ---- Backend switcher UI -------------------------------------------
  const backendMode_el = document.getElementById('backend-mode');
  const backendHost_el = document.getElementById('backend-host');
  const backendRealRow = document.getElementById('backend-real-row');
  const backendConnect = document.getElementById('backend-connect');
  const backendStatus  = document.getElementById('backend-status');
  const backendSwap    = document.getElementById('backend-swap');

  function refreshBackendUi() {
    if (!backendMode_el) return;
    backendMode_el.value = backendMode;
    if (backendHost_el && backendMode === 'real') backendHost_el.value = backendBase;
    if (backendRealRow) backendRealRow.style.display = (backendMode === 'real') ? '' : 'none';
  }
  refreshBackendUi();

  if (backendMode_el) {
    backendMode_el.addEventListener('change', () => {
      backendMode = backendMode_el.value;
      refreshBackendUi();
    });
  }

  function normalizeHost(h) {
    h = (h || '').trim();
    if (h.startsWith('http://'))  h = h.slice('http://'.length);
    if (h.startsWith('https://')) h = h.slice('https://'.length);
    h = h.replace(/^\/+|\/+$/g, '');
    if (h && !h.includes(':')) h += ':8080';
    return h;
  }

  // Probe a host by opening a WebSocket and waiting for the lawnbot snapshot
  // shape. Resolves with the host on success, rejects on timeout / wrong
  // protocol / connection error.
  function probeHost(host, timeoutMs = 2000) {
    return new Promise((resolve, reject) => {
      let settled = false;
      let probe;
      const finish = (fn, val) => { if (settled) return; settled = true;
                                    try { probe && probe.close(); } catch (e) {}
                                    fn(val); };
      const timer = setTimeout(() => finish(reject, new Error('timeout')), timeoutMs);
      try {
        probe = new WebSocket(`ws://${host}/ws`);
      } catch (e) {
        clearTimeout(timer);
        return reject(e);
      }
      probe.onerror = () => { clearTimeout(timer); finish(reject, new Error('error')); };
      probe.onopen  = () => {
        // Got a successful handshake — wait for the first JSON frame and
        // confirm it carries the lawnbot snapshot shape.
        probe.onmessage = (m) => {
          try {
            const s = JSON.parse(m.data);
            if (s && (s.mission !== undefined || s.pose !== undefined || s.gps !== undefined)) {
              clearTimeout(timer);
              finish(resolve, host);
            } else {
              clearTimeout(timer);
              finish(reject, new Error('not lawnbot'));
            }
          } catch (e) {
            clearTimeout(timer);
            finish(reject, e);
          }
        };
      };
    });
  }

  // Race a list of probes in parallel; resolve with the first one that hits.
  async function discoverPi(extraHost) {
    const list = [];
    if (extraHost) list.push(normalizeHost(extraHost));
    for (const c of PI_CANDIDATES) list.push(c);
    // Dedupe + drop empties.
    const seen = new Set();
    const probes = list.filter(h => h && !seen.has(h) && (seen.add(h), true));
    backendStatus.innerHTML = 'searching network…<br>' +
        probes.map(h => `• ${h}`).join('<br>');
    try {
      return await Promise.any(probes.map(h => probeHost(h, 2500)));
    } catch (e) {
      return null;   // AggregateError → none responded
    }
  }

  if (backendConnect) {
    backendConnect.addEventListener('click', async () => {
      if (backendMode === 'sim') {
        backendBase = '';
        localStorage.setItem('lawnbot_backend', JSON.stringify({ mode: 'sim' }));
        backendBusy = true;
        backendStatus.textContent = `connecting → local …`;
        connect();
        // Let the snapshot mirror take over after ~1s of stable connection.
        setTimeout(() => { backendBusy = false; }, 1500);
        return;
      }
      // Real mode: use the manual host if provided, else auto-discover.
      const manual = normalizeHost(backendHost_el.value);
      backendConnect.disabled = true;
      backendBusy = true;
      let connectedNew = false;
      try {
        let host = manual;
        if (!host) {
          host = await discoverPi();
          if (!host) {
            backendStatus.textContent =
              '✗ no Pi found on lawnbot.local / raspberrypi.local / 192.168.4.1 / 10.42.0.1. ' +
              'Enter the Pi’s host:port manually.';
            return;
          }
        } else {
          backendStatus.textContent = `probing ${host} …`;
          try { await probeHost(host, 3000); }
          catch (e) {
            backendStatus.textContent = `✗ ${host} did not respond (${e.message || e}).`;
            return;
          }
        }
        backendBase = host;
        if (backendHost_el) backendHost_el.value = host;
        localStorage.setItem('lawnbot_backend', JSON.stringify({ mode: 'real', host }));
        backendStatus.textContent = `connecting → ${host} …`;
        connect();
        connectedNew = true;
      } finally {
        backendConnect.disabled = false;
        if (connectedNew) {
          setTimeout(() => { backendBusy = false; }, 1500);
        } else {
          // Probe failed — let the snapshot mirror resume showing the still-
          // active old connection (or stay paused if there isn't one).
          backendBusy = false;
        }
      }
    });
  }

  if (backendSwap) {
    backendSwap.addEventListener('click', () => {
      const choice = prompt(
        'Hot-swap the CONNECTED server\'s hardware between sim and real.\n' +
        'Type "sim" or "real" (only works when the connected machine can open real I/O — i.e. on the Pi):',
        'real'
      );
      if (!choice) return;
      const tgt = choice.trim().toLowerCase();
      if (tgt !== 'sim' && tgt !== 'real') {
        alert('Type "sim" or "real" (got: ' + choice + ')');
        return;
      }
      send('backend.swap', { target: tgt });
    });
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

    renderHardware(s);

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

  // ---- keyboard teleop (WASD / arrows + Shift modifier + Space) -------
  // Track key state so each modifier change re-pushes the current command.
  const keysDown = { fwd: false, back: false, left: false, right: false, slow: false };

  // Don't steal keys while the user is typing into an input/textarea or
  // dragging a slider — that would block normal page interaction.
  function isTypingTarget(el) {
    if (!el) return false;
    const tag = el.tagName;
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') {
      const t = (el.type || '').toLowerCase();
      // Range sliders still want to ignore drive keys — return true so we skip.
      return true;
    }
    return el.isContentEditable === true;
  }

  function buildCmd() {
    // Magnitudes — fast by default; Shift halves them for fine positioning.
    const fast = !keysDown.slow;
    const fwdMag = fast ? 0.9 : 0.4;
    const backMag = fast ? 0.6 : 0.3;
    const steerMag = fast ? 0.9 : 0.4;
    let v = 0, d = 0;
    if (keysDown.fwd) v += fwdMag;
    if (keysDown.back) v -= backMag;
    if (keysDown.left) d -= steerMag;
    if (keysDown.right) d += steerMag;
    return { v, delta: d };
  }

  function pushKey() {
    const c = buildCmd();
    send('teleop', c);
    teleopHB = performance.now();
  }

  // Decode key → boolean state delta. Returns true if we handled it.
  function decodeKey(e, down) {
    if (e.repeat) return false;
    const k = e.key;
    const code = e.code;
    switch (k) {
      case 'ArrowUp':    case 'w': case 'W': keysDown.fwd = down; return true;
      case 'ArrowDown':  case 's': case 'S': keysDown.back = down; return true;
      case 'ArrowLeft':  case 'a': case 'A': keysDown.left = down; return true;
      case 'ArrowRight': case 'd': case 'D': keysDown.right = down; return true;
      case 'Shift':                          keysDown.slow = down; return true;
      case ' ':                              // Space = panic brake
        if (down) {
          keysDown.fwd = keysDown.back = keysDown.left = keysDown.right = false;
        }
        return true;
    }
    if (code === 'Space') {
      if (down) {
        keysDown.fwd = keysDown.back = keysDown.left = keysDown.right = false;
      }
      return true;
    }
    return false;
  }

  document.addEventListener('keydown', e => {
    if (isTypingTarget(e.target)) return;
    if (decodeKey(e, true)) {
      e.preventDefault();      // stop Space from scrolling the page
      pushKey();
    }
  });
  document.addEventListener('keyup', e => {
    if (isTypingTarget(e.target)) return;
    if (decodeKey(e, false)) {
      e.preventDefault();
      pushKey();
    }
  });
  // If the window loses focus mid-drive, clear keys so the rover doesn't keep
  // executing the last command past the deadman.
  window.addEventListener('blur', () => {
    let any = false;
    for (const k of Object.keys(keysDown)) {
      if (keysDown[k]) any = true;
      keysDown[k] = false;
    }
    if (any) pushKey();
  });

  // Heartbeat repeater: keep the deadman happy while ANY direction is held.
  setInterval(() => {
    const moving = keysDown.fwd || keysDown.back || keysDown.left || keysDown.right;
    if (moving || joyDrag) pushKey();
  }, 100);

  // Double-click recenters and re-engages follow mode.
  canvas.addEventListener('dblclick', () => { view.follow = true; });

  // ---- Sim speed slider ------------------------------------------------
  const simSpeed = document.getElementById('sim-speed');
  const simSpeedVal = document.getElementById('sim-speed-val');
  if (simSpeed) {
    let lastSent = 1.0;
    simSpeed.addEventListener('input', () => {
      simSpeedVal.textContent = parseFloat(simSpeed.value).toFixed(2);
    });
    simSpeed.addEventListener('change', () => {
      const v = parseFloat(simSpeed.value);
      if (Math.abs(v - lastSent) > 1e-6) {
        lastSent = v;
        send('sim.speed', { scale: v });
      }
    });
  }

  // ---- Cut pattern controls -------------------------------------------
  const patPreset = document.getElementById('pat-preset');
  const patDeck = document.getElementById('pat-deck');
  const patOverlap = document.getElementById('pat-overlap');
  const patHeadland = document.getElementById('pat-headland');
  const patAxis = document.getElementById('pat-axis');
  const patCross = document.getElementById('pat-cross');
  const patApply = document.getElementById('pat-apply');
  const patStatus = document.getElementById('pat-status');
  function bindRange(input, valSpan, digits) {
    if (!input || !valSpan) return;
    input.addEventListener('input', () => { valSpan.textContent = parseFloat(input.value).toFixed(digits); });
  }
  bindRange(patDeck, document.getElementById('pat-deck-val'), 2);
  bindRange(patOverlap, document.getElementById('pat-overlap-val'), 0);
  bindRange(patHeadland, document.getElementById('pat-headland-val'), 2);

  // Some presets ignore "axis" and "crosscut" — grey them out for clarity.
  function refreshPatternEnable() {
    const preset = patPreset ? patPreset.value : 'boustrophedon';
    const axisRelevant = (preset === 'boustrophedon' || preset === 'wave');
    const crossRelevant = (preset === 'boustrophedon');
    if (patAxis)  patAxis.disabled  = !axisRelevant;
    if (patCross) patCross.disabled = !crossRelevant;
  }
  if (patPreset) {
    patPreset.addEventListener('change', refreshPatternEnable);
    refreshPatternEnable();
  }

  if (patApply) {
    patApply.addEventListener('click', () => {
      const payload = {
        pattern: patPreset ? patPreset.value : 'boustrophedon',
        deck_m: parseFloat(patDeck.value),
        overlap_pct: parseFloat(patOverlap.value) / 100,
        headland_m: parseFloat(patHeadland.value),
        primary_axis: patAxis.value,
        crosscut: patCross.checked,
      };
      send('mission.plan', payload);          // re-plan with these params
      patStatus.textContent = 'planning…';
    });
  }

  // Keep pattern + sim-speed + auto-tune UI mirrored from server snapshots.
  let patSyncedFromServer = false;
  let speedSyncedFromServer = false;
  function mirrorAuxFromState(s) {
    // Pattern sliders — only sync once initially (so user input isn't yanked).
    if (s.pattern && !patSyncedFromServer && patDeck) {
      const p = s.pattern;
      patDeck.value = p.deck_m;
      document.getElementById('pat-deck-val').textContent = (+p.deck_m).toFixed(2);
      patOverlap.value = Math.round((p.overlap_pct || 0) * 100);
      document.getElementById('pat-overlap-val').textContent = patOverlap.value;
      patHeadland.value = p.headland_m;
      document.getElementById('pat-headland-val').textContent = (+p.headland_m).toFixed(2);
      patAxis.value = p.primary_axis || 'h';
      patCross.checked = !!p.crosscut;
      if (patPreset) {
        patPreset.value = p.pattern || 'boustrophedon';
        refreshPatternEnable();
      }
      patSyncedFromServer = true;
    }
    if (s.sim && !speedSyncedFromServer && simSpeed) {
      simSpeed.value = s.sim.time_scale;
      simSpeedVal.textContent = (+s.sim.time_scale).toFixed(2);
      speedSyncedFromServer = true;
    }
    // Show the server's current hardware backend in the status line, but
    // don't clobber an in-flight discovery / "connecting" message.
    if (s.backend && backendStatus && !backendBusy) {
      const label = backendBase ? backendBase : 'local';
      backendStatus.textContent = `connected · ${label} · server backend: ${s.backend.toUpperCase()}`;
    }
    // Pattern status from waypoint count.
    if (patStatus && s.mission) {
      const n = s.mission.n_waypoints;
      if (n > 0) patStatus.textContent = `${n} waypoints planned`;
    }
    // Auto-tune status — live.
    const ats = document.getElementById('autotune-status');
    if (ats && s.autotune) {
      const a = s.autotune;
      if (a.running) {
        const g = a.gains || {};
        const dp = a.dp || {};
        const lastTxt = (a.last_cost == null) ? '—' : a.last_cost.toExponential(3);
        const bestTxt = (a.best_cost == null) ? '—' : a.best_cost.toExponential(3);
        const gainsLine = ['kp','ki','kd','lookahead_m']
          .map(k => `${k}=${(g[k] ?? 0).toFixed(2)}`).join('  ');
        const dpLine = ['kp','ki','kd','lookahead_m']
          .map(k => `Δ${k}=${(dp[k] ?? 0).toFixed(3)}`).join('  ');
        ats.innerHTML =
          `iter ${a.iteration}  ${a.gain_under_test || ''} ${a.direction || ''}<br>` +
          `cost: best ${bestTxt} · last ${lastTxt}<br>` +
          `gains: ${gainsLine}<br>` +
          `steps: ${dpLine}`;
        // Mirror best gains into the live PID/lookahead sliders.
        const apply = (id, val) => {
          const el = document.getElementById(id);
          if (el && val != null) el.value = +val;
        };
        apply('kp', g.kp); apply('ki', g.ki); apply('kd', g.kd); apply('ld', g.lookahead_m);
      } else if (a.iteration > 0 && a.best_cost != null) {
        const g = a.gains || {};
        ats.textContent =
          `done (${a.iteration} iters). Final cost ${a.best_cost.toExponential(3)} — ` +
          `kp ${(+g.kp).toFixed(2)} ki ${(+g.ki).toFixed(2)} ` +
          `kd ${(+g.kd).toFixed(2)} ld ${(+g.lookahead_m).toFixed(2)}m`;
      } else {
        ats.textContent = 'idle';
      }
    }
  }

  // ---- Floating panel system ---------------------------------------------
  // Every former sidebar group is now a draggable, dockable window. The dock
  // along the bottom of the page has one toggle button per panel. Position +
  // visibility persist per panel in localStorage.
  //
  // Default positions are picked so the panels stack neatly on the right edge
  // of the screen on first run; users can drag them anywhere and the new
  // position survives reloads.
  const PANELS = [
    { id: 'panel-mission', initial: { right: 16,  top: 60  }, openByDefault: true  },
    { id: 'panel-mode',    initial: { right: 16,  top: 280 }, openByDefault: true  },
    { id: 'panel-backend', initial: { right: 16,  top: 360 }, openByDefault: false },
    { id: 'panel-sim',     initial: { right: 16,  top: 540 }, openByDefault: false },
    { id: 'panel-pattern', initial: { right: 290, top: 60  }, openByDefault: false },
    { id: 'panel-tune',    initial: { right: 290, top: 360 }, openByDefault: false },
    { id: 'panel-teach',   initial: { right: 290, top: 480 }, openByDefault: false },
    { id: 'panel-pid',     initial: { right: 290, top: 620 }, openByDefault: false },
    { id: 'panel-manual',  initial: { right: 564, top: 60  }, openByDefault: false },
    { id: 'panel-layers',  initial: { right: 564, top: 320 }, openByDefault: true  },
    { id: 'panel-events',  initial: { right: 564, top: 470 }, openByDefault: true  },
    { id: 'gnss-panel',    initial: { left: 16,   bottom: 110 }, openByDefault: false },
  ];

  const PANEL_STORAGE_KEY = (id, kind) => `lawnbot_panel_${id}_${kind}`;
  let panelZ = 100;
  function bringToFront(panel) { panel.style.zIndex = ++panelZ; }

  function clampToViewport(left, top, w, h) {
    const vw = window.innerWidth, vh = window.innerHeight;
    left = Math.max(0, Math.min(left, vw - 40));     // keep dragbar grabbable
    top  = Math.max(36, Math.min(top, vh - 40));     // header is 36px
    return [left, top];
  }

  function setPanelPosition(panel, pos) {
    panel.style.left   = (pos.left   != null) ? pos.left   + 'px' : 'auto';
    panel.style.top    = (pos.top    != null) ? pos.top    + 'px' : 'auto';
    panel.style.right  = (pos.right  != null) ? pos.right  + 'px' : 'auto';
    panel.style.bottom = (pos.bottom != null) ? pos.bottom + 'px' : 'auto';
  }

  function setupPanel(spec) {
    const panel = document.getElementById(spec.id);
    if (!panel) return null;

    // Restore saved position, else use initial.
    let savedPos = null;
    try { savedPos = JSON.parse(localStorage.getItem(PANEL_STORAGE_KEY(spec.id, 'pos')) || 'null'); } catch (e) {}
    if (savedPos && savedPos.left != null && savedPos.top != null) {
      setPanelPosition(panel, { left: savedPos.left, top: savedPos.top });
    } else {
      setPanelPosition(panel, spec.initial);
    }

    // Restore visibility (default to spec.openByDefault).
    const savedVis = localStorage.getItem(PANEL_STORAGE_KEY(spec.id, 'visible'));
    const visible = (savedVis === null) ? spec.openByDefault : (savedVis === 'true');
    panel.style.display = visible ? '' : 'none';

    // Drag from the .dragbar header.
    const handle = panel.querySelector('.dragbar');
    if (handle) {
      let drag = null;
      handle.addEventListener('pointerdown', (e) => {
        if (e.target.tagName === 'BUTTON' || e.target.closest('button')) return;
        bringToFront(panel);
        const r = panel.getBoundingClientRect();
        drag = { dx: e.clientX - r.left, dy: e.clientY - r.top, id: e.pointerId };
        handle.setPointerCapture(e.pointerId);
      });
      handle.addEventListener('pointermove', (e) => {
        if (!drag || drag.id !== e.pointerId) return;
        let left = e.clientX - drag.dx;
        let top  = e.clientY - drag.dy;
        const r = panel.getBoundingClientRect();
        [left, top] = clampToViewport(left, top, r.width, r.height);
        setPanelPosition(panel, { left, top });
      });
      const release = (e) => {
        if (!drag) return;
        drag = null;
        try { handle.releasePointerCapture(e.pointerId); } catch (_) {}
        const r = panel.getBoundingClientRect();
        localStorage.setItem(
          PANEL_STORAGE_KEY(spec.id, 'pos'),
          JSON.stringify({ left: r.left, top: r.top })
        );
      };
      handle.addEventListener('pointerup', release);
      handle.addEventListener('pointercancel', release);
    }
    // Bring to front when clicking anywhere in the panel body, so the active
    // panel is always above the others.
    panel.addEventListener('pointerdown', () => bringToFront(panel));

    // Close button (×) in the header
    const closeBtn = panel.querySelector('.dragbar .close-btn');
    if (closeBtn) {
      closeBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        setPanelVisible(spec.id, false);
      });
    }
    return panel;
  }

  function setPanelVisible(id, visible) {
    const panel = document.getElementById(id);
    if (!panel) return;
    panel.style.display = visible ? '' : 'none';
    localStorage.setItem(PANEL_STORAGE_KEY(id, 'visible'), String(!!visible));
    if (visible) bringToFront(panel);
    updateDockButton(id);
  }

  function updateDockButton(id) {
    const btn = document.querySelector(`#dock button[data-panel="${id}"]`);
    const panel = document.getElementById(id);
    if (!btn || !panel) return;
    const visible = panel.style.display !== 'none';
    btn.classList.toggle('active', visible);
  }

  function buildDock() {
    const dock = document.getElementById('dock');
    if (!dock) return;
    dock.innerHTML = '';
    for (const spec of PANELS) {
      const panel = document.getElementById(spec.id);
      if (!panel) continue;
      const label = panel.dataset.panelLabel || spec.id;
      const btn = document.createElement('button');
      btn.textContent = label;
      btn.dataset.panel = spec.id;
      btn.addEventListener('click', () => {
        const isVisible = panel.style.display !== 'none';
        setPanelVisible(spec.id, !isVisible);
      });
      dock.appendChild(btn);
    }
    // Tiny "reset" button to recover from off-screen panels.
    const reset = document.createElement('button');
    reset.textContent = 'reset layout';
    reset.title = 'Restore default positions for every panel';
    reset.style.marginLeft = 'auto';
    reset.addEventListener('click', () => {
      for (const spec of PANELS) {
        localStorage.removeItem(PANEL_STORAGE_KEY(spec.id, 'pos'));
        localStorage.removeItem(PANEL_STORAGE_KEY(spec.id, 'visible'));
        const panel = document.getElementById(spec.id);
        if (!panel) continue;
        setPanelPosition(panel, spec.initial);
        panel.style.display = spec.openByDefault ? '' : 'none';
        updateDockButton(spec.id);
      }
    });
    dock.appendChild(reset);
  }

  for (const spec of PANELS) setupPanel(spec);
  buildDock();
  for (const spec of PANELS) updateDockButton(spec.id);

  // Keep references to the canvases used by the GNSS panel rendering below.
  const gnssPanel = document.getElementById('gnss-panel');
  const skyCanvas = document.getElementById('gnss-sky');
  const sphereCanvas = document.getElementById('gnss-sphere');
  const barsCanvas = document.getElementById('gnss-bars');
  const rawPre = document.getElementById('gnss-raw');
  const legend = document.getElementById('gnss-legend');

  // Color per constellation. Matches what u-center / SwiftNav typically use.
  const CONSTELLATION_COLOR = {
    'GPS':     '#67d3f5',
    'GLONASS': '#e89052',
    'Galileo': '#b5d33b',
    'BeiDou':  '#d567b6',
    'QZSS':    '#67e8a2',
    'IRNSS':   '#c188ff',
    'SBAS':    '#aaaaaa',
    'Mixed':   '#dddddd',
  };
  const colorFor = (c) => CONSTELLATION_COLOR[c] || '#cccccc';

  function renderGnss(s) {
    const gps = s.gps || {};
    const sats = gps.satellites || [];
    drawSkyplot(sats);
    drawSphere(sats);
    drawBars(sats);
    renderLegend(sats);
    renderRaw(s, gps, sats);
  }

  function drawSkyplot(sats) {
    const c = skyCanvas;
    const cx = c.width / 2, cy = c.height / 2;
    const R = Math.min(cx, cy) - 14;
    const ctx2 = c.getContext('2d');
    ctx2.clearRect(0, 0, c.width, c.height);

    // Background disc
    ctx2.fillStyle = '#0a0d12';
    ctx2.beginPath(); ctx2.arc(cx, cy, R + 8, 0, Math.PI * 2); ctx2.fill();

    // Elevation rings: 0°, 30°, 60°
    ctx2.strokeStyle = '#1d2530';
    ctx2.lineWidth = 1;
    for (const el of [0, 30, 60]) {
      const r = R * (90 - el) / 90;
      ctx2.beginPath(); ctx2.arc(cx, cy, r, 0, Math.PI * 2); ctx2.stroke();
    }
    // Cardinal lines
    ctx2.beginPath();
    ctx2.moveTo(cx - R, cy); ctx2.lineTo(cx + R, cy);
    ctx2.moveTo(cx, cy - R); ctx2.lineTo(cx, cy + R);
    ctx2.stroke();

    // Labels (N/E/S/W)
    ctx2.fillStyle = '#8a96a3';
    ctx2.font = '10px system-ui';
    ctx2.textAlign = 'center'; ctx2.textBaseline = 'middle';
    ctx2.fillText('N', cx, cy - R - 7);
    ctx2.fillText('S', cx, cy + R + 7);
    ctx2.fillText('E', cx + R + 7, cy);
    ctx2.fillText('W', cx - R - 7, cy);
    ctx2.fillText('90°', cx + 4, cy - 4);
    ctx2.fillText('60°', cx + R / 3 + 6, cy - 4);
    ctx2.fillText('30°', cx + 2 * R / 3 + 6, cy - 4);

    // Satellites
    for (const s of sats) {
      if (s.el < 0 || s.snr <= 0) continue;
      const az = s.az * Math.PI / 180;
      const r = R * (90 - s.el) / 90;
      const px = cx + r * Math.sin(az);
      const py = cy - r * Math.cos(az);
      const fill = colorFor(s.constellation);
      const radius = 4 + Math.min(8, Math.max(0, (s.snr - 20)) * 0.25);
      ctx2.fillStyle = s.used ? fill : '#33424f';
      ctx2.strokeStyle = fill;
      ctx2.lineWidth = s.used ? 0 : 1.2;
      ctx2.beginPath(); ctx2.arc(px, py, radius, 0, Math.PI * 2);
      ctx2.fill(); if (!s.used) ctx2.stroke();
      ctx2.fillStyle = '#0c1014';
      ctx2.font = '9px ui-monospace, Consolas, monospace';
      ctx2.fillText(String(s.prn), px, py + 0.5);
    }
  }

  // Tiny isometric sphere — orthographic projection tilted ~35° from zenith
  // so the user sees a hemisphere from above with depth cueing.
  function drawSphere(sats) {
    const c = sphereCanvas;
    const W = c.width, H = c.height;
    const cx = W / 2, cy = H / 2 + 4;
    const R = Math.min(W, H) / 2 - 8;
    const ctx2 = c.getContext('2d');
    ctx2.clearRect(0, 0, W, H);

    const tilt = 35 * Math.PI / 180;  // viewer tilt off zenith
    const cosT = Math.cos(tilt), sinT = Math.sin(tilt);

    // Background sphere outline (great circle as seen from this angle = ellipse).
    ctx2.fillStyle = '#0a0d12';
    ctx2.beginPath();
    ctx2.ellipse(cx, cy, R, R * cosT + (R - R * cosT) * 0.0, 0, 0, Math.PI * 2);
    ctx2.fill();

    // Latitude rings (elevation lines)
    ctx2.strokeStyle = '#1d2530'; ctx2.lineWidth = 1;
    for (const el of [30, 60]) {
      const z = Math.sin(el * Math.PI / 180);
      const r2 = Math.cos(el * Math.PI / 180);
      // Project the ring of radius r2 at height z under our tilt.
      ctx2.beginPath();
      for (let k = 0; k <= 64; k++) {
        const t = (k / 64) * Math.PI * 2;
        const X = r2 * Math.sin(t);
        const Y3 = r2 * Math.cos(t);
        const Z = z;
        const yp = Y3 * cosT - Z * sinT;
        const px = cx + X * R;
        const py = cy - yp * R;
        if (k === 0) ctx2.moveTo(px, py); else ctx2.lineTo(px, py);
      }
      ctx2.stroke();
    }
    // Outline (horizon great circle)
    ctx2.strokeStyle = '#2a3a4a';
    ctx2.beginPath();
    for (let k = 0; k <= 96; k++) {
      const t = (k / 96) * Math.PI * 2;
      const X = Math.sin(t);
      const Y3 = Math.cos(t);
      const Z = 0;
      const yp = Y3 * cosT - Z * sinT;
      const px = cx + X * R;
      const py = cy - yp * R;
      if (k === 0) ctx2.moveTo(px, py); else ctx2.lineTo(px, py);
    }
    ctx2.stroke();

    // Zenith marker
    ctx2.fillStyle = '#33424f';
    const zX = 0, zY3 = 0, zZ = 1;
    const zyp = zY3 * cosT - zZ * sinT;
    ctx2.beginPath(); ctx2.arc(cx + zX * R, cy - zyp * R, 1.5, 0, Math.PI * 2); ctx2.fill();

    // Satellites — back-to-front depth ordering so near sats overdraw far ones.
    const projected = sats.filter(s => s.snr > 0 && s.el >= 0).map(s => {
      const el = s.el * Math.PI / 180;
      const az = s.az * Math.PI / 180;
      const X = Math.cos(el) * Math.sin(az);   // east
      const Y3 = Math.cos(el) * Math.cos(az);  // north
      const Z = Math.sin(el);                  // up
      const yp = Y3 * cosT - Z * sinT;
      const depth = Y3 * sinT + Z * cosT;       // toward viewer
      return { s, px: cx + X * R, py: cy - yp * R, depth };
    });
    projected.sort((a, b) => a.depth - b.depth);
    for (const p of projected) {
      const fill = colorFor(p.s.constellation);
      const r = 3 + Math.min(5, Math.max(0, p.s.snr - 20) * 0.18);
      // Fade with depth (rear sats slightly dim).
      const alpha = 0.55 + 0.45 * ((p.depth + 1) / 2);
      ctx2.globalAlpha = alpha;
      ctx2.fillStyle = p.s.used ? fill : '#33424f';
      ctx2.strokeStyle = fill; ctx2.lineWidth = p.s.used ? 0 : 1;
      ctx2.beginPath(); ctx2.arc(p.px, p.py, r, 0, Math.PI * 2);
      ctx2.fill(); if (!p.s.used) ctx2.stroke();
      ctx2.globalAlpha = 1;
    }
  }

  function drawBars(sats) {
    const c = barsCanvas;
    const W = c.width, H = c.height;
    const ctx2 = c.getContext('2d');
    ctx2.clearRect(0, 0, W, H);

    const padL = 28, padR = 6, padT = 6, padB = 22;
    const innerW = W - padL - padR;
    const innerH = H - padT - padB;
    const sorted = [...sats].sort((a, b) => {
      if (a.constellation === b.constellation) return a.prn - b.prn;
      return a.constellation.localeCompare(b.constellation);
    });

    // Y-axis grid: 0..55 dB-Hz
    const SNR_MAX = 55;
    ctx2.strokeStyle = '#1d2530'; ctx2.lineWidth = 1;
    ctx2.fillStyle = '#8a96a3';
    ctx2.font = '10px ui-monospace, Consolas, monospace';
    ctx2.textAlign = 'right'; ctx2.textBaseline = 'middle';
    for (const v of [0, 15, 30, 45]) {
      const y = padT + innerH * (1 - v / SNR_MAX);
      ctx2.beginPath(); ctx2.moveTo(padL, y); ctx2.lineTo(W - padR, y); ctx2.stroke();
      ctx2.fillText(String(v), padL - 4, y);
    }

    if (!sorted.length) {
      ctx2.fillStyle = '#8a96a3';
      ctx2.textAlign = 'center'; ctx2.textBaseline = 'middle';
      ctx2.fillText('no satellites tracked', W / 2, H / 2);
      return;
    }

    const slot = innerW / sorted.length;
    const barW = Math.min(20, slot * 0.7);
    ctx2.textAlign = 'center'; ctx2.textBaseline = 'top';
    for (let i = 0; i < sorted.length; i++) {
      const s = sorted[i];
      const x = padL + i * slot + (slot - barW) / 2;
      const h = innerH * Math.min(1, Math.max(0, s.snr) / SNR_MAX);
      const y = padT + innerH - h;
      const color = colorFor(s.constellation);
      ctx2.fillStyle = s.used ? color : '#33424f';
      ctx2.strokeStyle = color; ctx2.lineWidth = 1;
      ctx2.fillRect(x, y, barW, h);
      if (!s.used) ctx2.strokeRect(x + 0.5, y + 0.5, barW - 1, h - 1);
      // PRN label
      ctx2.fillStyle = '#8a96a3';
      ctx2.fillText(String(s.prn), x + barW / 2, padT + innerH + 2);
    }
  }

  function renderLegend(sats) {
    const counts = {};
    for (const s of sats) counts[s.constellation] = (counts[s.constellation] || 0) + 1;
    const usedCounts = {};
    for (const s of sats) if (s.used) usedCounts[s.constellation] = (usedCounts[s.constellation] || 0) + 1;
    const parts = Object.entries(counts).sort().map(([k, v]) =>
      `<span><span class="sw" style="background:${colorFor(k)}"></span>${k} ${usedCounts[k]||0}/${v}</span>`
    );
    legend.innerHTML = parts.join('');
  }

  function renderRaw(s, gps, sats) {
    const q = ['invalid', 'single', 'DGPS', 'PPS', 'RTK-fixed', 'RTK-float'];
    const fixLabel = q[gps.quality ?? 0] || 'unknown';
    const used = sats.filter(x => x.used).length;
    const tracked = sats.filter(x => x.snr > 0).length;
    const ageS = (gps.age_s == null) ? '∞' : Number(gps.age_s).toFixed(2);
    const latS = (gps.lat == null) ? '—' : Number(gps.lat).toFixed(7);
    const lonS = (gps.lon == null) ? '—' : Number(gps.lon).toFixed(7);
    const altS = (gps.alt_m == null) ? '—' : Number(gps.alt_m).toFixed(2);
    const xy = s.gps_xy;
    const xyS = xy ? `(${xy[0].toFixed(3)}, ${xy[1].toFixed(3)})` : '—';
    const pose = s.pose ? `(${s.pose.x.toFixed(3)}, ${s.pose.y.toFixed(3)}, θ=${(s.pose.theta * 180 / Math.PI).toFixed(1)}°)` : '—';
    const lines = [
      `fix          ${fixLabel}  (q=${gps.quality ?? 0})  age=${ageS}s`,
      `sats         ${used} used / ${tracked} tracked / ${sats.length} reported   HDOP ${gps.hdop ?? '—'}`,
      `lat/lon/alt  ${latS}, ${lonS}, ${altS} m`,
      `ENU (raw)    ${xyS}`,
      `pose (est)   ${pose}`,
    ];
    rawPre.textContent = lines.join('\n');
  }

  // Hook into the existing render() — render GNSS + pattern/sim/autotune mirrors.
  const origRender = render;
  render = function patchedRender() {     // eslint-disable-line no-func-assign
    origRender();
    if (state) {
      renderGnss(state);
      mirrorAuxFromState(state);
    }
  };

  // ---- Hardware status panel ----------------------------------------
  function renderHardware(s) {
    const hw = s.hardware || {};
    const sys = s.system || {};
    const batt = s.battery || {};

    // Device list
    const devUl = document.getElementById('hw-devices');
    if (devUl) {
      const items = [];
      // Each I2C address from the probe
      (hw.devices || []).forEach(d => {
        const dot = d.present ? 'ok' : 'bad';
        const val = d.present ? 'present' : 'absent';
        items.push(`<li><span class="hw-name"><span class="hw-dot ${dot}"></span>${d.addr} · ${d.label}</span><span class="hw-val">${val}</span></li>`);
      });
      // Serial0 (GPS UART)
      const s0 = hw.serial0 || {};
      const s0dot = s0.present ? 'ok' : 'bad';
      const s0val = s0.present ? (s0.target || 'ok') : 'missing';
      items.push(`<li><span class="hw-name"><span class="hw-dot ${s0dot}"></span>${s0.path || '/dev/serial0'} · GPS UART</span><span class="hw-val">${s0val}</span></li>`);
      // GPS fix as a separate health signal
      const q = ['invalid', 'single', 'DGPS', 'PPS', 'RTK-fixed', 'RTK-float'];
      const gpsq = s.gps?.quality ?? 0;
      const gpsDot = gpsq >= 4 ? 'ok' : (gpsq >= 1 ? 'warn' : 'bad');
      items.push(`<li><span class="hw-name"><span class="hw-dot ${gpsDot}"></span>GPS fix · ${q[gpsq] || '—'}</span><span class="hw-val">${s.gps?.sats ?? 0} sats</span></li>`);
      // GPIO chip
      const gp = hw.gpiochip || {};
      const gpDot = gp.accessible ? 'ok' : 'bad';
      items.push(`<li><span class="hw-name"><span class="hw-dot ${gpDot}"></span>${gp.path || '/dev/gpiochip*'} · Servo GPIO</span><span class="hw-val">${gp.accessible ? 'rw' : 'no access'}</span></li>`);
      // PiSugar socket
      const psSockDot = hw.pisugar_socket ? 'ok' : 'bad';
      items.push(`<li><span class="hw-name"><span class="hw-dot ${psSockDot}"></span>pisugar-server · /tmp/pisugar-server.sock</span><span class="hw-val">${hw.pisugar_socket ? 'up' : 'down'}</span></li>`);
      devUl.innerHTML = items.join('');
    }

    // Battery block
    const battEl = document.getElementById('hw-battery');
    if (battEl) {
      if (!batt.available) {
        battEl.innerHTML = '<span style="color:#8a96a3">no PiSugar detected</span>';
      } else {
        const pct = Math.max(0, Math.min(100, batt.percent || 0));
        const mode = batt.charging ? 'charging' : (batt.plugged ? 'plugged · idle' : 'on battery');
        const curr_mA = (batt.current_a || 0) * 1000;
        const sign = curr_mA >= 0 ? '+' : '';
        battEl.innerHTML = `
          <div class="hw-row"><b style="color:#e7ecef">${batt.model || 'PiSugar'}</b><span>${mode}</span></div>
          <div class="hw-bar"><div class="hw-bar-fill" style="width:${pct}%"></div></div>
          <div class="hw-row"><span>${pct.toFixed(0)}%</span><span>${(batt.voltage_v || 0).toFixed(2)} V · ${sign}${curr_mA.toFixed(0)} mA</span></div>
        `;
      }
    }

    // Host metrics (RAM/CPU/temp/uptime)
    const hostUl = document.getElementById('hw-host');
    if (hostUl) {
      const uptime = formatUptime(sys.uptime_s || 0);
      const cpuDot = (sys.cpu_pct ?? 0) > 85 ? 'warn' : 'ok';
      const tempDot = (sys.temp_c ?? 0) > 75 ? 'warn' : ((sys.temp_c ?? 0) > 0 ? 'ok' : 'bad');
      const memDot = (sys.mem_pct ?? 0) > 85 ? 'warn' : 'ok';
      hostUl.innerHTML = [
        `<li><span class="hw-name"><span class="hw-dot ${cpuDot}"></span>CPU</span><span class="hw-val">${(sys.cpu_pct ?? 0).toFixed(0)}%  ld ${(sys.load_1 ?? 0).toFixed(2)}</span></li>`,
        `<li><span class="hw-name"><span class="hw-dot ${memDot}"></span>RAM</span><span class="hw-val">${(sys.mem_used_mb ?? 0).toFixed(0)} / ${(sys.mem_total_mb ?? 0).toFixed(0)} MB (${(sys.mem_pct ?? 0).toFixed(0)}%)</span></li>`,
        `<li><span class="hw-name"><span class="hw-dot ${tempDot}"></span>Temp</span><span class="hw-val">${(sys.temp_c ?? 0).toFixed(1)} °C</span></li>`,
        `<li><span class="hw-name"><span class="hw-dot ok"></span>Uptime</span><span class="hw-val">${uptime}</span></li>`,
      ].join('');
    }

    const ageEl = document.getElementById('hw-scan-age');
    if (ageEl) ageEl.textContent = `scan age ${(hw.last_scan_age_s ?? 0).toFixed(1)}s`;
  }

  function formatUptime(s) {
    s = Math.floor(s);
    const d = Math.floor(s / 86400); s -= d * 86400;
    const h = Math.floor(s / 3600); s -= h * 3600;
    const m = Math.floor(s / 60); s -= m * 60;
    if (d) return `${d}d ${h}h`;
    if (h) return `${h}h ${m}m`;
    if (m) return `${m}m ${s}s`;
    return `${s}s`;
  }
})();
