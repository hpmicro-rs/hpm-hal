import { WebUsbMonitorTransport, readWebUsbMonitor } from "./monitor_webusb.js";

const $ = (id) => document.getElementById(id);

const elements = {
  port: $("port"),
  transport: $("transport"),
  elf: $("elf"),
  interval: $("interval"),
  status: $("status"),
  state: $("connection-state"),
  lastSample: $("last-sample"),
  refresh: $("refresh"),
  poll: $("poll"),
  webusb: $("webusb"),
  webusbLog: $("webusb-log"),
};

const rawGroups = {
  "sensor-raw": [
    "MON_SENSOR_CYCLE_COUNT",
    "MON_SENSOR_READY_MASK",
    "MON_SENSOR_LAST_ERROR",
    "MON_BH1750_SAMPLE_COUNT",
    "MON_BH1750_RAW",
    "MON_BH1750_LUX_X100",
    "MON_APDS9930_SAMPLE_COUNT",
    "MON_APDS9930_ID",
    "MON_APDS9930_STATUS",
    "MON_APDS9930_CLEAR_RAW",
    "MON_APDS9930_IR_RAW",
    "MON_APDS9930_PROXIMITY_RAW",
    "MON_BME680_SAMPLE_COUNT",
    "MON_BME680_TEMP_C_X100",
    "MON_BME680_HUMIDITY_RH_X100",
    "MON_BME680_PRESSURE_PA",
    "MON_BME680_GAS_OHM",
    "MON_BME680_GAS_VALID",
    "MON_PMS7003_STATUS",
    "MON_PMS7003_VALID_FRAMES",
    "MON_PMS7003_INVALID_FRAMES",
    "MON_PMS7003_UART_ERRORS",
    "MON_PMS7003_PM1_0_UG_M3",
    "MON_PMS7003_PM2_5_UG_M3",
    "MON_PMS7003_PM10_UG_M3",
    "MON_PMS7003_VERSION",
    "MON_PMS7003_SENSOR_ERROR",
    "MON_PMS7003_LAST_UPDATE_MS",
  ],
  "runtime-raw": [
    "MON_LOOP_COUNT",
    "MON_LAST_ERROR",
    "MON_RX_PACKETS",
    "MON_PANIC_LINE",
    "MON_PANIC_COLUMN",
    "MON_UPTIME_US",
    "MON_CSR_MCAUSE",
    "MON_CSR_MEPC",
    "MON_CSR_MTVAL",
    "MON_CSR_MSTATUS",
    "MON_CSR_MIE",
    "MON_CSR_MIP",
    "MON_CSR_MDCAUSE",
  ],
  "usb-raw": [
    "MON_USB_STAGE",
    "MON_USB_USBSTS",
    "MON_USB_USBINTR",
    "MON_USB_PORTSC1",
    "MON_USB_OTGSC",
    "MON_USB_PHY_CTRL0",
    "MON_USB_USBCMD",
    "MON_USB_USBMODE",
    "MON_USB_IRQ_COUNT",
    "MON_USB_QTD_ADDR",
    "MON_USB_QTD_DATA_PTR",
    "MON_USB_QTD_LEN",
    "MON_USB_QTD_SYS_ADDR",
  ],
  "exception-raw": ["EXCEPTION_MCAUSE", "EXCEPTION_MEPC"],
};

const history = {
  light: [],
  temperature: [],
  humidity: [],
  pm1: [],
  pm25: [],
  pm10: [],
  proximity: [],
};

const HISTORY_LIMIT = 60;
const CHART_TRANSITION_MS = 420;
const PM25_BANDS = [
  {
    maximum: 15,
    level: "good",
    badge: "健康目标",
    badgeState: "ok",
    detail: "处于 WHO 24 小时健康指导值内",
  },
  {
    maximum: 35,
    level: "elevated",
    badge: "数值升高",
    badgeState: "warn",
    detail: "瞬时值高于 WHO 24 小时指导线 15 µg/m³",
  },
  {
    maximum: 50,
    level: "attention",
    badge: "需要关注",
    badgeState: "attention",
    detail: "瞬时值高于 EPA 24 小时标准线 35 µg/m³",
  },
  {
    maximum: 100,
    level: "high",
    badge: "室内超标",
    badgeState: "error",
    detail: "瞬时值高于中国室内 24 小时限值线 50 µg/m³",
  },
  {
    maximum: Number.POSITIVE_INFINITY,
    level: "severe",
    badge: "严重告警",
    badgeState: "critical",
    detail: "优先排查颗粒物来源并启用 HEPA 净化",
  },
];
let polling = false;
let pollTimer = 0;
let reading = false;
let chartAnimationFrame = 0;
let latestValues = null;
let webUsbTransport = null;
let webUsbSymbols = null;
let webUsbHeartbeat = 0;
const webUsbOwner = globalThis.crypto?.randomUUID?.() ?? `hpm-webusb-${Date.now()}-${Math.random()}`;

function value(values, name, fallback = 0) {
  return Number(values[name] ?? fallback);
}

function text(id, content) {
  $(id).textContent = String(content);
}

function hex(number, width = 8) {
  return `0x${(number >>> 0).toString(16).padStart(width, "0")}`;
}

function setConnection(state, title, detail) {
  elements.state.dataset.state = state;
  elements.status.textContent = title;
  elements.lastSample.textContent = detail;
}

function setBadge(id, label, state) {
  const badge = $(id);
  badge.textContent = label;
  badge.className = `sensor-badge ${state}`;
}

function setHealth(id, title, state) {
  text(`health-${id}`, title);
  $(`health-${id}-dot`).dataset.state = state;
}

function renderPm25Advisory(pm25, ready) {
  const card = document.querySelector("[data-testid='pms7003-card']");
  const advisory = $("pm-advisory");
  document.querySelectorAll(".pm-thresholds span").forEach((item) => item.classList.remove("active"));

  if (!ready) {
    card.dataset.airLevel = "idle";
    advisory.dataset.level = "idle";
    text("pm-advisory-label", "等待数据");
    text("pm-advisory-detail", "24 小时达标以长期平均值判定");
    setBadge("pms-status", "等待帧", "warn");
    return;
  }

  const band = PM25_BANDS.find(({ maximum }) => pm25 <= maximum);
  card.dataset.airLevel = band.level;
  advisory.dataset.level = band.level;
  text("pm-advisory-label", `${band.badge} · ${pm25} µg/m³`);
  text("pm-advisory-detail", `${band.detail}；24 小时达标以长期平均值判定`);
  setBadge("pms-status", band.badge, band.badgeState);
  document.querySelector(`.pm-thresholds span[data-level="${band.level}"]`)?.classList.add("active");
}

function pushHistory(name, sample) {
  history[name].push(sample);
  if (history[name].length > HISTORY_LIMIT) history[name].shift();
}

function snapshotHistory() {
  return Object.fromEntries(Object.entries(history).map(([name, values]) => [name, [...values]]));
}

function monotoneTangents(points) {
  const count = points.length;
  const slopes = Array.from({ length: count - 1 }, (_, index) => {
    const current = points[index];
    const next = points[index + 1];
    return (next.y - current.y) / (next.x - current.x);
  });
  const tangents = Array(count).fill(0);
  tangents[0] = slopes[0];
  tangents[count - 1] = slopes[count - 2];

  for (let index = 1; index < count - 1; index += 1) {
    const before = slopes[index - 1];
    const after = slopes[index];
    tangents[index] = before * after <= 0 ? 0 : (before + after) / 2;
  }

  for (let index = 0; index < slopes.length; index += 1) {
    const slope = slopes[index];
    if (slope === 0) {
      tangents[index] = 0;
      tangents[index + 1] = 0;
      continue;
    }
    const startRatio = tangents[index] / slope;
    const endRatio = tangents[index + 1] / slope;
    const magnitude = Math.hypot(startRatio, endRatio);
    if (magnitude > 3) {
      const scale = 3 / magnitude;
      tangents[index] = scale * startRatio * slope;
      tangents[index + 1] = scale * endRatio * slope;
    }
  }

  return tangents;
}

function traceMonotoneCurve(ctx, points) {
  const tangents = monotoneTangents(points);
  ctx.moveTo(points[0].x, points[0].y);
  for (let index = 0; index < points.length - 1; index += 1) {
    const current = points[index];
    const next = points[index + 1];
    const width = next.x - current.x;
    ctx.bezierCurveTo(
      current.x + width / 3,
      current.y + (tangents[index] * width) / 3,
      next.x - width / 3,
      next.y - (tangents[index + 1] * width) / 3,
      next.x,
      next.y,
    );
  }
}

function paddedRange(values) {
  let min = Math.min(...values);
  let max = Math.max(...values);
  const pad = Math.max((max - min) * 0.25, Math.abs(max) * 0.015, 1);
  return { min: min - pad, max: max + pad };
}

function sameValues(left, right) {
  return left.length === right.length && left.every((value, index) => value === right[index]);
}

function animatedPoints(dataset, width, height, progress, rangeValues, previousRangeValues) {
  const current = dataset.values;
  const previous = dataset.previousValues ?? [];
  const currentRange = paddedRange(rangeValues ?? current);
  const previousRange =
    (previousRangeValues ?? previous).length >= 2
      ? paddedRange(previousRangeValues ?? previous)
      : currentRange;
  const min = previousRange.min + (currentRange.min - previousRange.min) * progress;
  const max = previousRange.max + (currentRange.max - previousRange.max) * progress;
  const toPoint = (value, x) => ({
    x,
    y: height - ((value - min) / (max - min)) * height,
  });

  if (progress >= 1 || previous.length < 2) {
    return current.map((value, index) => toPoint(value, (index / (current.length - 1)) * width));
  }

  const appended =
    current.length === previous.length + 1 && sameValues(previous, current.slice(0, -1));
  if (appended) {
    const slots =
      previous.length - 1 + (current.length - previous.length) * progress;
    return current.map((value, index) => toPoint(value, (index / slots) * width));
  }

  const shifted =
    current.length === previous.length &&
    current.length === HISTORY_LIMIT &&
    sameValues(previous.slice(1), current.slice(0, -1));
  if (shifted) {
    const movingWindow = [...previous, current[current.length - 1]];
    return movingWindow.map((value, index) =>
      toPoint(value, ((index - progress) / (current.length - 1)) * width),
    );
  }

  return current.map((value, index) => {
    const oldIndex = Math.min(index, previous.length - 1);
    const oldValue = previous[oldIndex] ?? value;
    const animatedValue = oldValue + (value - oldValue) * progress;
    return toPoint(animatedValue, (index / (current.length - 1)) * width);
  });
}

function drawChart(canvasId, series, progress = 1) {
  const canvas = $(canvasId);
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return;
  const ratio = window.devicePixelRatio || 1;
  const pixelWidth = Math.round(rect.width * ratio);
  const pixelHeight = Math.round(rect.height * ratio);
  if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
  if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);

  ctx.strokeStyle = "rgba(148, 163, 184, 0.12)";
  ctx.lineWidth = 1;
  for (let line = 1; line < 4; line += 1) {
    const y = (rect.height / 4) * line;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(rect.width, y);
    ctx.stroke();
  }

  const visibleSeries = series.filter((dataset) => dataset.values.length >= 2);
  const layered = visibleSeries.length > 1;
  const rangeValues = layered ? visibleSeries.flatMap((dataset) => dataset.values) : null;
  const previousRangeValues = layered
    ? visibleSeries.flatMap((dataset) => dataset.previousValues ?? [])
    : null;
  const datasets = visibleSeries
    .map((dataset) => ({
      ...dataset,
      points: animatedPoints(
        dataset,
        rect.width,
        rect.height,
        progress,
        rangeValues,
        previousRangeValues,
      ),
    }));

  if (layered) {
    ctx.save();
    ctx.globalCompositeOperation = "screen";
    for (const dataset of datasets) {
      ctx.beginPath();
      traceMonotoneCurve(ctx, dataset.points);
      ctx.lineTo(dataset.points[dataset.points.length - 1].x, rect.height);
      ctx.lineTo(dataset.points[0].x, rect.height);
      ctx.closePath();
      ctx.fillStyle = dataset.color;
      ctx.globalAlpha = 0.025;
      ctx.fill();
    }
    ctx.restore();

    ctx.save();
    ctx.globalCompositeOperation = "screen";
    for (const dataset of datasets) {
      ctx.beginPath();
      traceMonotoneCurve(ctx, dataset.points);
      ctx.strokeStyle = dataset.color;
      ctx.globalAlpha = 0.075;
      ctx.lineWidth = 6;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.shadowColor = dataset.color;
      ctx.shadowBlur = 7;
      ctx.shadowOffsetY = 3;
      ctx.stroke();
    }
    ctx.restore();
  }

  for (const dataset of datasets) {
    ctx.save();
    ctx.beginPath();
    ctx.strokeStyle = dataset.color;
    ctx.globalAlpha = layered ? 0.62 : 0.92;
    ctx.globalCompositeOperation = layered ? "screen" : "source-over";
    ctx.lineWidth = 2.2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    traceMonotoneCurve(ctx, dataset.points);
    ctx.stroke();
    ctx.restore();
  }

  for (const dataset of datasets) {
    ctx.save();
    ctx.fillStyle = dataset.color;
    ctx.globalAlpha = layered ? 0.26 : 0.5;
    ctx.globalCompositeOperation = layered ? "screen" : "source-over";
    for (const point of dataset.points) {
      if (point.x < -2 || point.x > rect.width + 2) continue;
      ctx.beginPath();
      ctx.arc(point.x, point.y, 1.35, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = layered ? 0.72 : 1;
    const latest = dataset.points[dataset.points.length - 1];
    ctx.beginPath();
    ctx.arc(latest.x, latest.y, 2.6, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }
}

function drawAllCharts(previous = null, progress = 1) {
  drawChart("chart-light", [
    { values: history.light, previousValues: previous?.light, color: "#f5c451" },
  ], progress);
  drawChart("chart-climate", [
    { values: history.temperature, previousValues: previous?.temperature, color: "#fb7185" },
    { values: history.humidity, previousValues: previous?.humidity, color: "#4fd1c5" },
  ], progress);
  drawChart("chart-pm", [
    { values: history.pm1, previousValues: previous?.pm1, color: "#60a5fa" },
    { values: history.pm25, previousValues: previous?.pm25, color: "#a78bfa" },
    { values: history.pm10, previousValues: previous?.pm10, color: "#f472b6" },
  ], progress);
  drawChart("chart-proximity", [
    { values: history.proximity, previousValues: previous?.proximity, color: "#818cf8" },
  ], progress);
}

function animateAllCharts(previous) {
  window.cancelAnimationFrame(chartAnimationFrame);
  const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  if (reducedMotion || Object.values(previous).every((values) => values.length < 2)) {
    drawAllCharts();
    return;
  }

  const startedAt = performance.now();
  const frame = (now) => {
    const elapsed = Math.min(1, (now - startedAt) / CHART_TRANSITION_MS);
    const progress = 1 - (1 - elapsed) ** 3;
    drawAllCharts(previous, progress);
    if (elapsed < 1) chartAnimationFrame = window.requestAnimationFrame(frame);
  };
  chartAnimationFrame = window.requestAnimationFrame(frame);
}

function setBar(id, current, maximum) {
  const percent = Math.max(2, Math.min(100, (current / maximum) * 100));
  $(id).style.width = `${percent}%`;
}

function renderRawTable(id, values) {
  const table = $(id);
  table.replaceChildren();
  for (const name of rawGroups[id]) {
    if (!(name in values)) continue;
    const row = document.createElement("tr");
    const key = document.createElement("td");
    const raw = document.createElement("td");
    key.textContent = name;
    raw.textContent = `${values[name]} · ${hex(values[name])}`;
    row.append(key, raw);
    table.append(row);
  }
}

function renderHealth(values) {
  const readyMask = value(values, "MON_SENSOR_READY_MASK");
  const sensorError = value(values, "MON_SENSOR_LAST_ERROR");
  const i2cHealthy = (readyMask & 0x7) === 0x7 && sensorError === 0;
  setHealth("i2c", i2cHealthy ? "3/3 设备就绪" : `ready=${hex(readyMask)} err=${hex(sensorError)}`, i2cHealthy ? "ok" : "error");

  const pmsHealthy =
    value(values, "MON_PMS7003_STATUS") === 2 &&
    value(values, "MON_PMS7003_UART_ERRORS") === 0 &&
    value(values, "MON_PMS7003_SENSOR_ERROR") === 0;
  setHealth(
    "uart",
    pmsHealthy ? `${value(values, "MON_PMS7003_VALID_FRAMES").toLocaleString()} 个有效帧` : "串口或传感器告警",
    pmsHealthy ? "ok" : "error",
  );

  const usbHealthy = value(values, "MON_USB_IRQ_COUNT") > 0;
  setHealth("usb", usbHealthy ? `IRQ ${value(values, "MON_USB_IRQ_COUNT").toLocaleString()}` : "等待 USB IRQ", usbHealthy ? "ok" : "warn");

  const runtimeHealthy =
    value(values, "EXCEPTION_MCAUSE") === 0 &&
    value(values, "MON_PANIC_LINE") === 0;
  setHealth("runtime", runtimeHealthy ? "异常与 panic 记录清零" : "检测到异常记录", runtimeHealthy ? "ok" : "error");

  const allHealthy = i2cHealthy && pmsHealthy && usbHealthy && runtimeHealthy;
  const summary = $("overall-health");
  summary.textContent = allHealthy ? "全部系统正常" : "存在待检查状态";
  summary.dataset.state = allHealthy ? "ok" : "error";
}

function renderSensors(values) {
  const previousHistory = snapshotHistory();
  const lux = value(values, "MON_BH1750_LUX_X100") / 100;
  const temperature = value(values, "MON_BME680_TEMP_C_X100") / 100;
  const humidity = value(values, "MON_BME680_HUMIDITY_RH_X100") / 100;
  const pressure = value(values, "MON_BME680_PRESSURE_PA") / 100;
  const gas = value(values, "MON_BME680_GAS_OHM") / 1000;
  const pm1 = value(values, "MON_PMS7003_PM1_0_UG_M3");
  const pm25 = value(values, "MON_PMS7003_PM2_5_UG_M3");
  const pm10 = value(values, "MON_PMS7003_PM10_UG_M3");
  const proximity = value(values, "MON_APDS9930_PROXIMITY_RAW");
  const clear = value(values, "MON_APDS9930_CLEAR_RAW");
  const ir = value(values, "MON_APDS9930_IR_RAW");

  text("bh-lux", lux.toFixed(2));
  text("bh-raw", value(values, "MON_BH1750_RAW"));
  text("bh-count", value(values, "MON_BH1750_SAMPLE_COUNT").toLocaleString());
  setBadge("bh-status", value(values, "MON_BH1750_SAMPLE_COUNT") > 0 ? "在线" : "等待", value(values, "MON_BH1750_SAMPLE_COUNT") > 0 ? "ok" : "warn");

  text("temperature", temperature.toFixed(2));
  text("humidity", humidity.toFixed(2));
  text("legend-temperature", `${temperature.toFixed(1)} °C`);
  text("legend-humidity", `${humidity.toFixed(1)} %`);
  text("pressure", pressure.toFixed(2));
  text("gas", gas.toFixed(2));
  const gasValid = value(values, "MON_BME680_GAS_VALID") === 1;
  setBadge("gas-status", gasValid ? "气体有效" : "气体预热", gasValid ? "ok" : "warn");

  text("pm25", pm25);
  text("pm1", pm1);
  text("pm25-small", pm25);
  text("pm10", pm10);
  text("legend-pm1", pm1);
  text("legend-pm25", pm25);
  text("legend-pm10", pm10);
  text("pms-frames", value(values, "MON_PMS7003_VALID_FRAMES").toLocaleString());
  text("pms-version", hex(value(values, "MON_PMS7003_VERSION"), 2));
  const pmsReady = value(values, "MON_PMS7003_STATUS") === 2;
  renderPm25Advisory(pm25, pmsReady);
  setBar("pm1-bar", pm1, 75);
  setBar("pm25-bar", pm25, 75);
  setBar("pm10-bar", pm10, 75);

  text("proximity", proximity);
  text("clear", clear);
  text("ir", ir);
  text("apds-id", hex(value(values, "MON_APDS9930_ID"), 2));
  text("apds-count", value(values, "MON_APDS9930_SAMPLE_COUNT").toLocaleString());
  const apdsReady = value(values, "MON_APDS9930_ID") === 0x39;
  setBadge("apds-status", apdsReady ? "ID 已确认" : "检查设备", apdsReady ? "ok" : "error");
  setBar("clear-bar", clear, 1024);
  setBar("ir-bar", ir, 1024);

  pushHistory("light", lux);
  pushHistory("temperature", temperature);
  pushHistory("humidity", humidity);
  pushHistory("pm1", pm1);
  pushHistory("pm25", pm25);
  pushHistory("pm10", pm10);
  pushHistory("proximity", proximity);
  animateAllCharts(previousHistory);
}

function renderSnapshot(data) {
  latestValues = data.values;
  const { info, values } = data;
  text("vidpid", `${info.vid.toString(16).padStart(4, "0")}:${info.pid.toString(16).padStart(4, "0")}`);
  text("version", `v${info.version}`);
  text("latency", `${data.duration_ms} ms`);
  text("cycle", value(values, "MON_SENSOR_CYCLE_COUNT").toLocaleString());
  text("usb-irq", value(values, "MON_USB_IRQ_COUNT").toLocaleString());
  text("packet", `${info.maxsize} B packet · ${info.max_count} values`);
  renderSensors(values);
  renderHealth(values);
  Object.keys(rawGroups).forEach((id) => renderRawTable(id, values));
}

async function loadPorts() {
  const response = await fetch("/api/ports", { cache: "no-store" });
  if (!response.ok) throw new Error(`端口列表读取失败：HTTP ${response.status}`);
  const data = await response.json();
  elements.port.replaceChildren();
  if (data.ports.length === 0) {
    const option = document.createElement("option");
    option.textContent = "未发现 CDC 串口";
    option.value = "";
    elements.port.append(option);
  } else {
    for (const port of data.ports) {
      const option = document.createElement("option");
      option.value = port;
      option.textContent = port;
      elements.port.append(option);
    }
  }
  elements.elf.value = data.default_elf;
  syncTransportControls();
}

function syncTransportControls() {
  elements.port.disabled = elements.transport.value !== "serial";
  elements.webusb.textContent = webUsbTransport ? "断开 WebUSB" : "连接 WebUSB";
}

async function setBackendUsbMode(mode, { keepalive = false, clean = true } = {}) {
  const response = await fetch("/api/usb-mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode, owner: webUsbOwner, clean }),
    cache: "no-store",
    keepalive,
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `USB ownership HTTP ${response.status}`);
  return data;
}

async function loadMonitorSymbols() {
  const params = new URLSearchParams({ elf: elements.elf.value });
  const response = await fetch(`/api/symbols?${params}`, { cache: "no-store" });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `ELF 符号读取失败：HTTP ${response.status}`);
  if (Object.keys(data.symbols).length === 0) throw new Error("ELF 中没有 Monitor 符号");
  return data.symbols;
}

function startWebUsbHeartbeat() {
  window.clearInterval(webUsbHeartbeat);
  webUsbHeartbeat = window.setInterval(() => {
    setBackendUsbMode("webusb").catch((error) => {
      elements.webusbLog.textContent = `WebUSB 租约续期失败\n${String(error.message || error)}`;
    });
  }, 3000);
}

async function disconnectWebUsb({ resumeBackend = true, selectBackend = true, clean = true } = {}) {
  window.clearInterval(webUsbHeartbeat);
  webUsbHeartbeat = 0;
  const transport = webUsbTransport;
  webUsbTransport = null;
  webUsbSymbols = null;
  if (transport) await transport.close();
  if (resumeBackend) await setBackendUsbMode("backend", { clean });
  if (selectBackend) elements.transport.value = "bulk";
  elements.webusbLog.textContent = "WebUSB 已断开，backend USB 已恢复";
  syncTransportControls();
}

async function connectWebUsb({ resumePolling = true } = {}) {
  const previousTransport = elements.transport.value === "webusb" ? "bulk" : elements.transport.value;
  if (!("usb" in navigator)) {
    elements.webusbLog.textContent = "当前浏览器未开放 navigator.usb";
    elements.transport.value = previousTransport;
    syncTransportControls();
    throw new Error("当前浏览器未开放 WebUSB");
  }

  stopPolling();
  let device = null;
  let backendPaused = false;
  try {
    const grantedDevices = await navigator.usb.getDevices();
    device =
      grantedDevices.find((candidate) => candidate.vendorId === 0x1209 && candidate.productId === 0x5e31) ??
      (await navigator.usb.requestDevice({ filters: [{ vendorId: 0x1209, productId: 0x5e31 }] }));
    webUsbSymbols = await loadMonitorSymbols();
    await setBackendUsbMode("webusb");
    backendPaused = true;
    webUsbTransport = await WebUsbMonitorTransport.open(device);
    elements.transport.value = "webusb";
    elements.webusbLog.textContent =
      `${device.productName || "HPM5E31"}\n` +
      `interface ${webUsbTransport.descriptor.interfaceNumber} · ` +
      `bulk OUT ${webUsbTransport.descriptor.endpointOut} / IN ${webUsbTransport.descriptor.endpointIn}\n` +
      "backend USB 已暂停，WebUSB Monitor 已连接";
    startWebUsbHeartbeat();
    syncTransportControls();
    if (resumePolling) startPolling();
  } catch (error) {
    if (webUsbTransport) await webUsbTransport.close();
    else if (device?.opened) await device.close();
    webUsbTransport = null;
    webUsbSymbols = null;
    if (backendPaused) await setBackendUsbMode("backend", { clean: false }).catch(() => {});
    elements.transport.value = previousTransport;
    elements.webusbLog.textContent = String(error.message || error);
    syncTransportControls();
    if (resumePolling) startPolling();
    throw error;
  }
}

async function readMonitor() {
  if (reading) return;
  reading = true;
  elements.refresh.disabled = true;
  setConnection("reading", "正在读取", `${elements.transport.value.toUpperCase()} · 请求设备数据`);
  try {
    let data;
    if (elements.transport.value === "webusb") {
      if (!webUsbTransport || !webUsbSymbols) throw new Error("WebUSB Monitor 尚未连接");
      data = await readWebUsbMonitor(webUsbTransport, Object.keys(webUsbSymbols), webUsbSymbols, 4000);
    } else {
      const params = new URLSearchParams({
        transport: elements.transport.value,
        port: elements.port.value,
        elf: elements.elf.value,
        timeout: "4.0",
      });
      const response = await fetch(`/api/read?${params}`, { cache: "no-store" });
      data = await response.json();
      if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    }
    renderSnapshot(data);
    const sampled = new Date(data.sampled_at);
    setConnection("online", "设备在线", `${data.transport.toUpperCase()} · ${sampled.toLocaleTimeString()}`);
  } catch (error) {
    setConnection("error", "读取失败", String(error.message || error));
    if (elements.transport.value === "webusb" && webUsbTransport && !webUsbTransport.connected) {
      await disconnectWebUsb({ clean: false }).catch(() => {});
    }
  } finally {
    reading = false;
    elements.refresh.disabled = false;
  }
}

async function pollTick() {
  if (!polling) return;
  await readMonitor();
  if (polling) pollTimer = window.setTimeout(pollTick, Number(elements.interval.value));
}

function startPolling() {
  if (polling) return;
  polling = true;
  elements.poll.textContent = "暂停实时监控";
  elements.poll.classList.add("active");
  pollTick();
}

function stopPolling() {
  polling = false;
  window.clearTimeout(pollTimer);
  pollTimer = 0;
  elements.poll.textContent = "开始实时监控";
  elements.poll.classList.remove("active");
  const detail = latestValues ? "数据保持在最后一次成功采样" : "尚未采样";
  setConnection("idle", "实时监控已暂停", detail);
}

async function waitForReadIdle(timeoutMs = 5000) {
  const deadline = performance.now() + timeoutMs;
  while (reading && performance.now() < deadline) {
    await new Promise((resolve) => window.setTimeout(resolve, 10));
  }
  if (reading) throw new Error("等待当前 Monitor 读取结束超时");
}

function togglePolling() {
  if (polling) stopPolling();
  else startPolling();
}

async function handleTransportChange() {
  const resumePolling = polling;
  if (elements.transport.value === "webusb") {
    await connectWebUsb({ resumePolling }).catch((error) => {
      setConnection("error", "WebUSB 连接失败", String(error.message || error));
    });
    return;
  }
  stopPolling();
  await waitForReadIdle();
  if (webUsbTransport) await disconnectWebUsb({ selectBackend: false });
  else await setBackendUsbMode("backend").catch(() => {});
  syncTransportControls();
  if (resumePolling) startPolling();
}

async function toggleWebUsb() {
  const resumePolling = polling;
  if (webUsbTransport) {
    stopPolling();
    await waitForReadIdle();
    await disconnectWebUsb();
    if (resumePolling) startPolling();
    return;
  }
  await connectWebUsb({ resumePolling: true }).catch((error) => {
    setConnection("error", "WebUSB 连接失败", String(error.message || error));
  });
}

elements.refresh.addEventListener("click", readMonitor);
elements.poll.addEventListener("click", togglePolling);
elements.transport.addEventListener("change", handleTransportChange);
elements.interval.addEventListener("change", () => {
  if (polling) {
    window.clearTimeout(pollTimer);
    pollTimer = window.setTimeout(pollTick, Number(elements.interval.value));
  }
});
elements.webusb.addEventListener("click", toggleWebUsb);
window.addEventListener("resize", () => {
  window.cancelAnimationFrame(chartAnimationFrame);
  drawAllCharts();
});
window.addEventListener("pagehide", () => {
  window.clearInterval(webUsbHeartbeat);
  if (webUsbTransport) {
    fetch("/api/usb-mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode: "backend", owner: webUsbOwner, clean: !reading }),
      keepalive: true,
    }).catch(() => {});
  }
});
if ("usb" in navigator) {
  navigator.usb.addEventListener("disconnect", (event) => {
    if (webUsbTransport?.device === event.device) {
      webUsbTransport.connected = false;
      disconnectWebUsb().catch(() => {});
    }
  });
}

loadPorts()
  .then(startPolling)
  .catch((error) => setConnection("error", "初始化失败", String(error.message || error)));
