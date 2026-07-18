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
let polling = false;
let pollTimer = 0;
let reading = false;
let latestValues = null;

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

function pushHistory(name, sample) {
  history[name].push(sample);
  if (history[name].length > HISTORY_LIMIT) history[name].shift();
}

function drawChart(canvasId, series) {
  const canvas = $(canvasId);
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
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

  for (const dataset of series) {
    if (dataset.values.length < 2) continue;
    let min = Math.min(...dataset.values);
    let max = Math.max(...dataset.values);
    const pad = Math.max((max - min) * 0.25, Math.abs(max) * 0.015, 1);
    min -= pad;
    max += pad;
    ctx.beginPath();
    ctx.strokeStyle = dataset.color;
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    dataset.values.forEach((point, index) => {
      const x = (index / (dataset.values.length - 1)) * rect.width;
      const y = rect.height - ((point - min) / (max - min)) * rect.height;
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}

function drawAllCharts() {
  drawChart("chart-light", [{ values: history.light, color: "#f5c451" }]);
  drawChart("chart-climate", [
    { values: history.temperature, color: "#fb7185" },
    { values: history.humidity, color: "#4fd1c5" },
  ]);
  drawChart("chart-pm", [
    { values: history.pm1, color: "#60a5fa" },
    { values: history.pm25, color: "#a78bfa" },
    { values: history.pm10, color: "#f472b6" },
  ]);
  drawChart("chart-proximity", [{ values: history.proximity, color: "#818cf8" }]);
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
  text("pressure", pressure.toFixed(2));
  text("gas", gas.toFixed(2));
  const gasValid = value(values, "MON_BME680_GAS_VALID") === 1;
  setBadge("gas-status", gasValid ? "气体有效" : "气体预热", gasValid ? "ok" : "warn");

  text("pm25", pm25);
  text("pm1", pm1);
  text("pm25-small", pm25);
  text("pm10", pm10);
  text("pms-frames", value(values, "MON_PMS7003_VALID_FRAMES").toLocaleString());
  text("pms-version", hex(value(values, "MON_PMS7003_VERSION"), 2));
  const pmsReady = value(values, "MON_PMS7003_STATUS") === 2;
  setBadge("pms-status", pmsReady ? "数据有效" : "等待帧", pmsReady ? "ok" : "warn");
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
  drawAllCharts();
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
  elements.port.disabled = elements.transport.value === "bulk";
}

async function readMonitor() {
  if (reading) return;
  reading = true;
  elements.refresh.disabled = true;
  setConnection("reading", "正在读取", `${elements.transport.value.toUpperCase()} · 请求设备数据`);
  const params = new URLSearchParams({
    transport: elements.transport.value,
    port: elements.port.value,
    elf: elements.elf.value,
    timeout: "4.0",
  });

  try {
    const response = await fetch(`/api/read?${params}`, { cache: "no-store" });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    renderSnapshot(data);
    const sampled = new Date(data.sampled_at);
    setConnection("online", "设备在线", `${data.transport.toUpperCase()} · ${sampled.toLocaleTimeString()}`);
  } catch (error) {
    setConnection("error", "读取失败", String(error.message || error));
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

function togglePolling() {
  if (polling) stopPolling();
  else startPolling();
}

async function probeWebUsb() {
  if (!("usb" in navigator)) {
    elements.webusbLog.textContent = "当前浏览器未开放 navigator.usb";
    return;
  }
  try {
    const device = await navigator.usb.requestDevice({ filters: [{ vendorId: 0x1209, productId: 0x5e31 }] });
    await device.open();
    if (device.configuration === null) await device.selectConfiguration(1);
    const vendorInterface = device.configuration.interfaces.find((iface) =>
      iface.alternates.some((alternate) => alternate.interfaceClass === 0xff),
    );
    elements.webusbLog.textContent = vendorInterface
      ? `${device.productName || "HPM5E31"}\nvendor interface ${vendorInterface.interfaceNumber}`
      : `${device.productName || "HPM5E31"}\nvendor interface 未发现`;
  } catch (error) {
    elements.webusbLog.textContent = String(error);
  }
}

elements.refresh.addEventListener("click", readMonitor);
elements.poll.addEventListener("click", togglePolling);
elements.transport.addEventListener("change", syncTransportControls);
elements.interval.addEventListener("change", () => {
  if (polling) {
    window.clearTimeout(pollTimer);
    pollTimer = window.setTimeout(pollTick, Number(elements.interval.value));
  }
});
elements.webusb.addEventListener("click", probeWebUsb);
window.addEventListener("resize", drawAllCharts);

loadPorts()
  .then(startPolling)
  .catch((error) => setConnection("error", "初始化失败", String(error.message || error)));
