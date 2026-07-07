const portEl = document.querySelector("#port");
const elfEl = document.querySelector("#elf");
const statusEl = document.querySelector("#status");
const refreshEl = document.querySelector("#refresh");
const pollEl = document.querySelector("#poll");
const webusbEl = document.querySelector("#webusb");
const webusbLogEl = document.querySelector("#webusb-log");

const groups = {
  blinky: [
    "MON_LOOP_COUNT",
    "MON_BLINK_TICKS",
    "MON_BLINK_PERIOD_MS",
    "MON_GPIO_PD06_LEVEL",
    "MON_GPIO_PD07_LEVEL",
    "MON_LED_STATE",
    "MON_LAST_ERROR",
    "MON_RX_PACKETS",
    "MON_UPTIME_US",
  ],
  csr: [
    "MON_CSR_MCAUSE",
    "MON_CSR_MEPC",
    "MON_CSR_MTVAL",
    "MON_CSR_MSTATUS",
    "MON_CSR_MIE",
    "MON_CSR_MIP",
    "MON_CSR_MDCAUSE",
    "MON_CSR_PMPCFG0",
    "MON_CSR_PMPADDR0",
    "MON_CSR_PMPADDR1",
    "MON_CSR_PMACFG0",
    "MON_CSR_PMAADDR1",
  ],
  usb: [
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
  exception: ["EXCEPTION_MCAUSE", "EXCEPTION_MEPC"],
};

let pollTimer = 0;

function hex(value) {
  return `0x${(value >>> 0).toString(16).padStart(8, "0")}`;
}

function setStatus(text) {
  statusEl.textContent = text;
}

function renderTable(id, values) {
  const table = document.querySelector(`#${id}`);
  table.replaceChildren();
  for (const name of groups[id]) {
    if (!(name in values)) continue;
    const row = document.createElement("tr");
    const key = document.createElement("td");
    const val = document.createElement("td");
    key.textContent = name;
    val.textContent = `${hex(values[name])} (${values[name]})`;
    row.append(key, val);
    table.append(row);
  }
}

async function loadPorts() {
  const resp = await fetch("/api/ports");
  const data = await resp.json();
  portEl.replaceChildren();
  for (const port of data.ports) {
    const opt = document.createElement("option");
    opt.value = port;
    opt.textContent = port;
    portEl.append(opt);
  }
  const hpmPort = data.ports.find((port) => port.includes("HPM"));
  if (hpmPort) {
    portEl.value = hpmPort;
  }
  elfEl.value = data.default_elf;
}

async function readMonitor() {
  const params = new URLSearchParams({
    port: portEl.value,
    elf: elfEl.value,
    timeout: "2.0",
  });
  setStatus("reading");
  const resp = await fetch(`/api/read?${params}`);
  const data = await resp.json();
  if (!resp.ok) {
    setStatus(data.error || "error");
    return;
  }
  const info = data.info;
  document.querySelector("#vidpid").textContent = `${info.vid.toString(16).padStart(4, "0")}:${info.pid
    .toString(16)
    .padStart(4, "0")}`;
  document.querySelector("#version").textContent = String(info.version);
  document.querySelector("#packet").textContent = `${info.maxsize} bytes / ${info.max_count} values`;
  renderTable("blinky", data.values);
  renderTable("csr", data.values);
  renderTable("usb", data.values);
  renderTable("exception", data.values);
  setStatus(`ok ${new Date().toLocaleTimeString()}`);
}

function togglePoll() {
  if (pollTimer) {
    clearInterval(pollTimer);
    pollTimer = 0;
    pollEl.textContent = "Start Poll";
    setStatus("poll stopped");
    return;
  }
  readMonitor();
  pollTimer = window.setInterval(readMonitor, 1000);
  pollEl.textContent = "Stop Poll";
}

async function probeWebUsb() {
  if (!("usb" in navigator)) {
    webusbLogEl.textContent = "navigator.usb unavailable";
    return;
  }
  try {
    const device = await navigator.usb.requestDevice({
      filters: [{ vendorId: 0x1209, productId: 0x5e31 }],
    });
    webusbLogEl.textContent = `selected ${device.productName || "HPM device"}\ninterfaces require vendor firmware`;
  } catch (err) {
    webusbLogEl.textContent = String(err);
  }
}

refreshEl.addEventListener("click", readMonitor);
pollEl.addEventListener("click", togglePoll);
webusbEl.addEventListener("click", probeWebUsb);

loadPorts().then(readMonitor).catch((err) => setStatus(String(err)));
