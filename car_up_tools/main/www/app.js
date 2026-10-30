(() => {
  "use strict";

  const FIELD = 4.0;
  const field = document.getElementById("field");
  const ctx = field.getContext("2d");
  const W = field.width, H = field.height;
  const M = 28;
  const S = (W - 2 * M) / FIELD;

  const joy = document.getElementById("joy");
  const jctx = joy.getContext("2d");

  let state = null;
  let tool = "none";
  let clickPoints = [];
  let freePoints = [];
  let drawing = false;
  let preview = null;
  let joyActive = false;
  let joyTimer = null;
  let joyVec = { x: 0, y: 0 };
  let history = { x: [], y: [], th: [], tof0: [], tof1: [] };

  const $ = (id) => document.getElementById(id);
  const debugEl = $("debugLog");
  let lastFaults = -1;
  let lastMotion = -1;
  let lastLink = true;
  let lastLostLog = 0;
  let lastParamKey = "";
  let lastCalibKey = "";

  function dbg(msg, level = "") {
    if (!debugEl) return;
    const line = document.createElement("div");
    if (level) line.className = level;
    const t = new Date().toLocaleTimeString("zh-CN", { hour12: false });
    line.textContent = `[${t}] ${msg}`;
    debugEl.appendChild(line);
    while (debugEl.childElementCount > 120) debugEl.removeChild(debugEl.firstChild);
    debugEl.scrollTop = debugEl.scrollHeight;
  }
  window.addEventListener("error", (e) => dbg(`JS: ${e.message}`, "err"));

  function w2c(x, y) {
    return { x: M + x * S, y: H - M - y * S };
  }
  function c2w(cx, cy) {
    return {
      x: Math.max(0, Math.min(FIELD, (cx - M) / S)),
      y: Math.max(0, Math.min(FIELD, (H - M - cy) / S)),
    };
  }
  function evPos(e) {
    const r = field.getBoundingClientRect();
    const cx = (e.clientX - r.left) * (W / r.width);
    const cy = (e.clientY - r.top) * (H / r.height);
    return c2w(cx, cy);
  }
  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
  function fmt(v, n = 2) { return Number(v).toFixed(n); }

  async function post(obj) {
    try {
      const r = await fetch("/api/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(obj),
      });
      if (!r.ok) throw new Error("http " + r.status);
      const j = await r.json();
      if (!j.ok) {
        $("statusLine").textContent = "命令被拒绝: " + obj.cmd;
        dbg(`命令失败: ${obj.cmd}`, "err");
      }
      return j;
    } catch (e) {
      $("conn").textContent = "断开";
      $("conn").className = "bad";
      dbg(`请求失败: ${obj.cmd} (${e.message})`, "err");
      return { ok: false };
    }
  }

  async function poll() {
    try {
      const r = await fetch("/api/state", { cache: "no-store" });
      if (!r.ok) throw new Error("http");
      state = await r.json();
      $("conn").textContent = "已连接";
      $("conn").className = "good";
      if (!lastLink) dbg("已连接 ESP32", "ok");
      lastLink = true;
      updateUI();
      drawField();
      drawCharts();
    } catch (e) {
      $("conn").textContent = "未连接";
      $("conn").className = "bad";
      const now = Date.now();
      if (lastLink || now - lastLostLog > 5000) {
        dbg("未连接 ESP32 / HTTP 超时", "err");
        lastLostLog = now;
      }
      lastLink = false;
    }
  }

  function motionName(s) {
    return ["IDLE", "RUN", "PAUSE", "ERROR"][s] || "?";
  }

  function updateUI() {
    const c = state.chassis || {};
    const n = state.nav || {};
    const m = state.motion || {};
    const p = state.param || {};
    const cal = state.calib || {};

    $("navMode").textContent = (n.mode === 1) ? "比赛" : "日常";
    $("enabled").textContent = c.enabled ? "是" : "否";
    $("motionState").textContent = motionName(m.state);
    $("progress").textContent = (m.completed || 0) + "/" + (m.total || 0);
    $("crossTrack").textContent = fmt((m.cross || 0) * 1000, 0) + " mm";
    $("faults").textContent = "0x" + (c.faults || 0).toString(16).padStart(4, "0");
    if (n.ok) {
      $("poseText").textContent = `x ${fmt(n.x)}  y ${fmt(n.y)}  θ ${fmt(n.theta)}`;
      history.x.push(n.x); history.y.push(n.y); history.th.push(n.theta);
      history.tof0.push(n.tof ? n.tof[0] : 0);
      history.tof1.push(n.tof ? n.tof[1] : 0);
      for (const k of Object.keys(history)) {
        if (history[k].length > 240) history[k].shift();
      }
    }

    $("calCount").textContent = cal.count || 0;
    $("calRaw").textContent = cal.have ? (cal.raw + " mm") : "--";
    $("calGain").textContent = cal.have
      ? `${fmt(cal.gain, 3)} / ${fmt(cal.offset, 0)} mm`
      : "--";

    const pv = p.values || [];
    $("pMaxV").textContent = pv[0] != null ? (pv[0] / 1000).toFixed(2) + " m/s" : "--";
    $("pMaxW").textContent = pv[1] != null ? (pv[1] / 1000).toFixed(2) + " rad/s" : "--";
    $("pMode").textContent = pv[2] === 1 ? "比赛" : (pv[2] === 0 ? "日常" : "--");
    $("pFg").textContent = pv[3] != null ? (pv[3] / 1000).toFixed(3) : "--";
    $("pFo").textContent = pv[4] != null ? pv[4] + " mm" : "--";
    $("pRg").textContent = pv[5] != null ? (pv[5] / 1000).toFixed(3) : "--";
    $("pRo").textContent = pv[6] != null ? pv[6] + " mm" : "--";

    document.querySelectorAll(".mode").forEach((b) => b.classList.remove("active"));
    if (n.mode === 1) $("btnComp").classList.add("active");
    else $("btnDaily").classList.add("active");

    $("statusLine").textContent =
      `v ${fmt(n.v || 0, 2)} m/s  ω ${fmt(n.w || 0, 2)} rad/s  ` +
      `ToF ${n.tof ? n.tof[0] : 0}/${n.tof ? n.tof[1] : 0} mm  ` +
      `P ${n.pxx != null ? n.pxx.toFixed(3) : "--"}/${n.pyy != null ? n.pyy.toFixed(3) : "--"}`;

    if (c.faults !== lastFaults) {
      if (c.faults) dbg(`底盘故障 0x${c.faults.toString(16).padStart(4, "0")}`, "err");
      else if (lastFaults > 0) dbg("底盘故障已清除", "ok");
      lastFaults = c.faults || 0;
    }
    if (m.state !== lastMotion) {
      dbg(`运动状态: ${motionName(m.state)}${m.error ? " err=" + m.error : ""}`,
          m.state === 3 ? "err" : "ok");
      lastMotion = m.state;
    }
    const pk = `${p.id}:${p.status}:${p.value}`;
    if (p.have && p.status !== 0 && pk !== lastParamKey) {
      dbg(`参数 ${p.id} 操作失败 (status=${p.status})`, "err");
    }
    lastParamKey = pk;
    const ck = `${cal.action}:${cal.sensor}:${cal.status}:${cal.count}`;
    if (cal.have && cal.status !== 0 && ck !== lastCalibKey) {
      dbg(`标定失败 sensor=${cal.sensor} status=${cal.status}`, "err");
    }
    lastCalibKey = ck;
  }

  function drawField() {
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0b0f14";
    ctx.fillRect(0, 0, W, H);

    // 网格
    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 8; i++) {
      const gx = M + (i / 8) * (W - 2 * M);
      const gy = H - M - (i / 8) * (H - 2 * M);
      ctx.beginPath(); ctx.moveTo(gx, M); ctx.lineTo(gx, H - M); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(M, gy); ctx.lineTo(W - M, gy); ctx.stroke();
    }

    // 围挡
    ctx.strokeStyle = "#c9d1d9";
    ctx.lineWidth = 3;
    ctx.strokeRect(M, M, W - 2 * M, H - 2 * M);
    ctx.fillStyle = "#8b949e";
    ctx.font = "12px monospace";
    ctx.fillText("0,0", M + 4, H - M - 6);
    ctx.fillText("4,4", W - M - 36, M + 16);

    if (!state) return;
    const n = state.nav || {};
    const m = state.motion || {};

    // 规划路径
    const plan = state.plan || [];
    if (plan.length > 1) {
      ctx.strokeStyle = "#58a6ff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      plan.forEach((p, i) => {
        const q = w2c(p[0], p[1]);
        if (i === 0) ctx.moveTo(q.x, q.y); else ctx.lineTo(q.x, q.y);
      });
      ctx.stroke();
      plan.forEach((p) => {
        const q = w2c(p[0], p[1]);
        ctx.fillStyle = "#58a6ff";
        ctx.beginPath(); ctx.arc(q.x, q.y, 3, 0, Math.PI * 2); ctx.fill();
      });
    }

    // 实际轨迹
    const trail = state.trail || [];
    if (trail.length > 1) {
      ctx.strokeStyle = "#d29922";
      ctx.lineWidth = 2;
      ctx.beginPath();
      trail.forEach((p, i) => {
        const q = w2c(p[0], p[1]);
        if (i === 0) ctx.moveTo(q.x, q.y); else ctx.lineTo(q.x, q.y);
      });
      ctx.stroke();
    }

    // 当前目标
    if (m.target_x || m.target_y) {
      const q = w2c(m.target_x || 0, m.target_y || 0);
      ctx.strokeStyle = "#f85149";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(q.x, q.y, 7, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(q.x - 10, q.y); ctx.lineTo(q.x + 10, q.y); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(q.x, q.y - 10); ctx.lineTo(q.x, q.y + 10); ctx.stroke();
    }

    // 预览
    if (preview) {
      ctx.strokeStyle = "#bc8cff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      preview.forEach((p, i) => {
        const q = w2c(p.x, p.y);
        if (i === 0) ctx.moveTo(q.x, q.y); else ctx.lineTo(q.x, q.y);
      });
      ctx.stroke();
    }

    // 小车箭头
    if (n.ok) {
      const q = w2c(n.x, n.y);
      ctx.save();
      ctx.translate(q.x, q.y);
      ctx.rotate(-n.theta);
      ctx.fillStyle = "#3fb950";
      ctx.beginPath();
      ctx.moveTo(14, 0);
      ctx.lineTo(-9, 8);
      ctx.lineTo(-5, 0);
      ctx.lineTo(-9, -8);
      ctx.closePath();
      ctx.fill();
      ctx.strokeStyle = "#0d1117";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = "#3fb950";
      ctx.beginPath(); ctx.arc(q.x, q.y, 3, 0, Math.PI * 2); ctx.fill();
    }
  }

  function drawCharts() {
    drawChart($("chartXY"), [
      { data: history.x, color: "#58a6ff", min: 0, max: 4 },
      { data: history.y, color: "#3fb950", min: 0, max: 4 },
    ], "x/y (m)");
    drawChart($("chartTof"), [
      { data: history.tof0, color: "#d29922", min: 0, max: 4000 },
      { data: history.tof1, color: "#bc8cff", min: 0, max: 4000 },
    ], "ToF (mm)");
  }

  function drawChart(canvas, series, label) {
    const c = canvas.getContext("2d");
    const w = canvas.width, h = canvas.height;
    c.clearRect(0, 0, w, h);
    c.fillStyle = "#0b0f14"; c.fillRect(0, 0, w, h);
    c.strokeStyle = "#1f2937"; c.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
      const y = (h * i) / 4;
      c.beginPath(); c.moveTo(0, y); c.lineTo(w, y); c.stroke();
    }
    c.fillStyle = "#8b949e"; c.font = "11px monospace"; c.fillText(label, 8, 14);

    const maxN = Math.max(2, ...series.map(s => s.data.length));
    for (const s of series) {
      if (s.data.length < 2) continue;
      c.strokeStyle = s.color; c.lineWidth = 1.5;
      c.beginPath();
      for (let i = 0; i < s.data.length; i++) {
        const x = (i / (maxN - 1)) * (w - 10) + 5;
        const v = clamp((s.data[i] - s.min) / (s.max - s.min), 0, 1);
        const y = h - 8 - v * (h - 24);
        if (i === 0) c.moveTo(x, y); else c.lineTo(x, y);
      }
      c.stroke();
    }
  }

  function drawJoy() {
    const w = joy.width, h = joy.height;
    const cx = w / 2, cy = h / 2, r = 70;
    jctx.clearRect(0, 0, w, h);
    jctx.fillStyle = "#0b0f14"; jctx.fillRect(0, 0, w, h);
    jctx.strokeStyle = "#30363d"; jctx.lineWidth = 2;
    jctx.beginPath(); jctx.arc(cx, cy, r, 0, Math.PI * 2); jctx.stroke();
    jctx.beginPath(); jctx.moveTo(cx - r, cy); jctx.lineTo(cx + r, cy); jctx.stroke();
    jctx.beginPath(); jctx.moveTo(cx, cy - r); jctx.lineTo(cx, cy + r); jctx.stroke();
    const kx = cx + joyVec.x * r;
    const ky = cy + joyVec.y * r;
    jctx.fillStyle = joyActive ? "#1f6feb" : "#30363d";
    jctx.beginPath(); jctx.arc(kx, ky, 16, 0, Math.PI * 2); jctx.fill();
    jctx.strokeStyle = "#e6edf3"; jctx.lineWidth = 1; jctx.stroke();
  }

  // 工具选择
  document.querySelectorAll(".tool").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".tool").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      tool = btn.dataset.tool;
      clickPoints = [];
      preview = null;
      $("hint").textContent = {
        none: "观察模式",
        point: "点击场地发送 GOTO",
        line: "点击两个点画直线",
        rect: "点击两个对角点画矩形",
        circle: "先点圆心，再点半径",
        free: "按住拖拽手绘轨迹",
      }[tool] || "";
    });
  });

  field.addEventListener("click", (e) => {
    if (tool === "none" || tool === "free") return;
    const p = evPos(e);
    const speed = Number($("pathSpeed").value) / 100;
    if (tool === "point") {
      post({ cmd: "goto", x: p.x, y: p.y, speed });
      return;
    }
    clickPoints.push(p);
    if (tool === "line" && clickPoints.length === 2) {
      post({ cmd: "line", points: clickPoints, speed });
      clickPoints = []; preview = null;
    } else if (tool === "rect" && clickPoints.length === 2) {
      const [a, b] = clickPoints;
      post({ cmd: "rect", x1: a.x, y1: a.y, x2: b.x, y2: b.y, speed });
      clickPoints = []; preview = null;
    } else if (tool === "circle" && clickPoints.length === 2) {
      const [a, b] = clickPoints;
      const r = Math.hypot(a.x - b.x, a.y - b.y);
      post({ cmd: "circle", cx: a.x, cy: a.y, r, ccw: 1, speed });
      clickPoints = []; preview = null;
    } else {
      preview = clickPoints.slice();
      drawField();
    }
  });

  field.addEventListener("pointerdown", (e) => {
    if (tool !== "free") return;
    drawing = true;
    freePoints = [evPos(e)];
    preview = freePoints.slice();
    field.setPointerCapture(e.pointerId);
  });
  field.addEventListener("pointermove", (e) => {
    if (!drawing || tool !== "free") return;
    const p = evPos(e);
    const last = freePoints[freePoints.length - 1];
    if (Math.hypot(p.x - last.x, p.y - last.y) > 0.04) {
      freePoints.push(p);
      preview = freePoints.slice();
      drawField();
    }
  });
  field.addEventListener("pointerup", (e) => {
    if (!drawing) return;
    drawing = false;
    if (freePoints.length > 1) {
      post({ cmd: "polyline", points: freePoints, speed: Number($("pathSpeed").value) / 100 });
    }
    freePoints = []; preview = null;
    drawField();
  });
  field.addEventListener("pointercancel", () => {
    drawing = false; freePoints = []; preview = null; drawField();
  });

  // 摇杆
  function joyPos(e) {
    const r = joy.getBoundingClientRect();
    const x = (e.clientX - r.left) * (joy.width / r.width);
    const y = (e.clientY - r.top) * (joy.height / r.height);
    const cx = joy.width / 2, cy = joy.height / 2;
    let dx = (x - cx) / 70, dy = (y - cy) / 70;
    const n = Math.hypot(dx, dy);
    if (n > 1) { dx /= n; dy /= n; }
    return { x: dx, y: dy };
  }
  function startJoy(e) {
    joyActive = true;
    joyVec = joyPos(e);
    drawJoy();
    if (joyTimer) clearInterval(joyTimer);
    joyTimer = setInterval(sendJoy, 50);
    joy.setPointerCapture(e.pointerId);
    sendJoy();
  }
  function moveJoy(e) {
    if (!joyActive) return;
    joyVec = joyPos(e);
    drawJoy();
  }
  function endJoy() {
    if (!joyActive) return;
    joyActive = false;
    joyVec = { x: 0, y: 0 };
    drawJoy();
    if (joyTimer) { clearInterval(joyTimer); joyTimer = null; }
    post({ cmd: "velocity", v: 0, w: 0 });
    $("joyOut").textContent = "v 0.00 / w 0.00";
  }
  function sendJoy() {
    if (!joyActive) return;
    const maxV = Number($("maxV").value) || 0.6;
    const maxW = Number($("maxW").value) || 2.0;
    const v = -joyVec.y * maxV;
    const w = -joyVec.x * maxW;
    post({ cmd: "velocity", v, w });
    $("joyOut").textContent = `v ${v.toFixed(2)} / w ${w.toFixed(2)}`;
  }
  joy.addEventListener("pointerdown", startJoy);
  joy.addEventListener("pointermove", moveJoy);
  joy.addEventListener("pointerup", endJoy);
  joy.addEventListener("pointercancel", endJoy);

  // 按钮
  $("btnDaily").onclick = () => post({ cmd: "mode", mode: 0 });
  $("btnComp").onclick = () => post({ cmd: "mode", mode: 1 });
  $("btnEnable").onclick = () => post({ cmd: "enable" });
  $("btnDisable").onclick = () => post({ cmd: "disable" });
  $("btnStop").onclick = () => post({ cmd: "stop" });
  $("btnSpeed").onclick = () => post({
    cmd: "set_speed",
    max_v: Number($("maxV").value),
    max_w: Number($("maxW").value),
  });
  $("btnPid").onclick = () => post({
    cmd: "pid", wheel: 255,
    kp: Number($("kp").value), ki: Number($("ki").value), kd: Number($("kd").value),
  });
  $("btnTurn").onclick = () => post({
    cmd: "turn",
    theta: Number($("turnDeg").value) * Math.PI / 180,
  });
  $("btnSave").onclick = () => post({ cmd: "save" });
  $("btnLoad").onclick = () => post({ cmd: "load" });
  $("btnReset").onclick = () => post({ cmd: "reset" });
  $("btnBackup").onclick = () => post({ cmd: "backup" });
  $("btnRestore").onclick = () => post({ cmd: "restore" });
  $("btnClearLog").onclick = () => { if (debugEl) debugEl.textContent = ""; };

  $("calStart").onclick = () => post({ cmd: "calib", action: 0, sensor: Number($("calSensor").value) });
  $("calSample").onclick = () => post({
    cmd: "calib", action: 1, sensor: Number($("calSensor").value),
    known: Number($("calKnown").value),
  });
  $("calFit").onclick = () => post({ cmd: "calib", action: 2, sensor: Number($("calSensor").value) });
  $("calSave").onclick = () => post({ cmd: "calib", action: 3, sensor: Number($("calSensor").value) });
  $("calReset").onclick = () => post({ cmd: "calib", action: 5, sensor: Number($("calSensor").value) });

  $("btnReadParams").onclick = async () => {
    for (let id = 0; id <= 6; id++) {
      await post({ cmd: "param", action: 0, id, value: 0 });
    }
  };

  $("pathSpeed").oninput = () => {
    $("pathSpeedVal").textContent = (Number($("pathSpeed").value) / 100).toFixed(2) + " m/s";
  };

  drawJoy();
  drawField();
  poll();
  setInterval(poll, 100);
})();
