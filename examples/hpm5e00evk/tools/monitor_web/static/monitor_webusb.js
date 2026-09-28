const START = 0x4850;
const MAGIC = 0x4d44;
const VERSION = 2;
const END = 0xa55a;
const HOST_MASK = 0x8000;
const ACK_MASK = 0x8000;

const CMD_CONNECT = 0x0001;
const CMD_GETINFO = 0x0002;
const CMD_GETVALUE = 0x00a0;
const TYPE_UINT32 = 0x05;
const RESULT_SUCCESS = 0x00ff;

const PROFILE_MIN_SIZE = 34;
const PAYLOAD_OFFSET = 32;

export function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit += 1) {
      const mask = -(crc & 1);
      crc = ((crc >>> 1) ^ (0xbeaf5aa5 & mask)) >>> 0;
    }
  }
  return crc >>> 0;
}

export function makePacket(index, command, payload = new Uint8Array()) {
  const frame = new Uint8Array(PROFILE_MIN_SIZE + payload.byteLength);
  const view = new DataView(frame.buffer);
  view.setUint16(0, START, true);
  view.setUint16(2, MAGIC, true);
  view.setUint16(4, VERSION, true);
  view.setUint32(6, 0, true);
  view.setUint16(14, index, true);
  view.setUint16(16, command | HOST_MASK, true);
  view.setUint16(18, 0, true);
  view.setUint32(20, 0, true);
  view.setUint32(24, 0, true);
  view.setUint32(28, payload.byteLength, true);
  frame.set(payload, PAYLOAD_OFFSET);
  view.setUint16(frame.byteLength - 2, END, true);
  view.setUint32(10, crc32(frame.subarray(14)), true);
  return frame;
}

export function parsePacket(frame) {
  if (!(frame instanceof Uint8Array) || frame.byteLength < PROFILE_MIN_SIZE) {
    throw new Error("Monitor 响应长度不足");
  }
  const view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
  if (view.getUint16(0, true) !== START || view.getUint16(2, true) !== MAGIC || view.getUint16(4, true) !== VERSION) {
    throw new Error("Monitor 响应头无效");
  }
  const payloadLength = view.getUint32(28, true);
  const totalLength = PROFILE_MIN_SIZE + payloadLength;
  if (frame.byteLength !== totalLength) throw new Error(`Monitor 响应长度异常：${frame.byteLength}/${totalLength}`);
  if (view.getUint16(totalLength - 2, true) !== END) throw new Error("Monitor 响应尾无效");
  const expectedCrc = view.getUint32(10, true);
  const actualCrc = crc32(frame.subarray(14));
  if (expectedCrc !== actualCrc) throw new Error("Monitor 响应 CRC 错误");
  return {
    index: view.getUint16(14, true),
    command: view.getUint16(16, true),
    result: view.getUint16(18, true),
    timing: view.getUint32(20, true),
    payload: frame.slice(PAYLOAD_OFFSET, PAYLOAD_OFFSET + payloadLength),
  };
}

function requireSuccess(packet, index, command) {
  if (packet.index !== index) throw new Error(`Monitor packet index 异常：${packet.index}/${index}`);
  if (packet.command !== command) throw new Error(`Monitor command 异常：0x${packet.command.toString(16)}`);
  if (packet.result !== (ACK_MASK | RESULT_SUCCESS)) {
    throw new Error(`Monitor command 失败：0x${packet.result.toString(16)}`);
  }
}

export function makeGetValuePayload(names, symbols) {
  const payload = new Uint8Array(12 + names.length * 12);
  const view = new DataView(payload.buffer);
  view.setUint16(0, names.length, true);
  view.setUint8(2, 0);
  view.setUint8(3, 0);
  view.setUint32(4, names.length, true);
  view.setUint32(8, 0, true);
  names.forEach((name, item) => {
    if (!(name in symbols)) throw new Error(`ELF 缺少符号：${name}`);
    const offset = 12 + item * 12;
    view.setUint8(offset, TYPE_UINT32);
    view.setUint32(offset + 4, Number(symbols[name]) >>> 0, true);
    view.setUint32(offset + 8, 0, true);
  });
  return payload;
}

export function parseGetValuePayload(names, payload) {
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const count = view.getUint16(0, true);
  if (count !== names.length || payload.byteLength !== 12 + count * 12) {
    throw new Error(`Monitor value count 异常：${count}/${names.length}`);
  }
  const values = {};
  names.forEach((name, item) => {
    const offset = 12 + item * 12;
    if (view.getUint8(offset) !== TYPE_UINT32) throw new Error(`${name} 数据类型异常`);
    values[name] = view.getUint32(offset + 8, true);
  });
  return values;
}

export function findVendorBulkInterface(device) {
  if (!device.configuration) throw new Error("WebUSB 设备尚未配置");
  for (const iface of device.configuration.interfaces) {
    for (const alternate of iface.alternates) {
      if (alternate.interfaceClass !== 0xff) continue;
      const endpointIn = alternate.endpoints.find((endpoint) => endpoint.type === "bulk" && endpoint.direction === "in");
      const endpointOut = alternate.endpoints.find((endpoint) => endpoint.type === "bulk" && endpoint.direction === "out");
      if (endpointIn && endpointOut) {
        return {
          interfaceNumber: iface.interfaceNumber,
          alternateSetting: alternate.alternateSetting,
          endpointIn: endpointIn.endpointNumber,
          endpointOut: endpointOut.endpointNumber,
          packetSize: Math.max(endpointIn.packetSize, endpointOut.packetSize),
        };
      }
    }
  }
  throw new Error("未找到 vendor bulk IN/OUT 接口");
}

export class WebUsbMonitorTransport {
  constructor(device, descriptor) {
    this.device = device;
    this.descriptor = descriptor;
    this.connected = true;
  }

  static async open(device) {
    await device.open();
    if (!device.configuration) await device.selectConfiguration(1);
    const descriptor = findVendorBulkInterface(device);
    await device.claimInterface(descriptor.interfaceNumber);
    const iface = device.configuration.interfaces.find((candidate) => candidate.interfaceNumber === descriptor.interfaceNumber);
    if (iface?.alternate?.alternateSetting !== descriptor.alternateSetting) {
      await device.selectAlternateInterface(descriptor.interfaceNumber, descriptor.alternateSetting);
    }
    return new WebUsbMonitorTransport(device, descriptor);
  }

  async close() {
    if (!this.connected) return;
    this.connected = false;
    try {
      await this.device.releaseInterface(this.descriptor.interfaceNumber);
    } catch (_error) {
      // The browser may already have released the interface after disconnect.
    }
    try {
      await this.device.close();
    } catch (_error) {
      // Closing an unplugged device is already complete from the host's view.
    }
  }

  async recoverFromTimeout() {
    this.connected = false;
    try {
      await this.device.reset();
    } catch (_error) {
      // A browser-side reset may report disconnect after the bus reset succeeds.
    }
    try {
      await this.device.close();
    } catch (_error) {
      // The reset can close the device handle as part of re-enumeration.
    }
  }

  async exchange(frame, timeoutMs) {
    if (!this.connected) throw new Error("WebUSB 设备未连接");
    let recovery = null;
    let timer = 0;
    const timeout = new Promise((_, reject) => {
      timer = globalThis.setTimeout(() => {
        recovery = this.recoverFromTimeout();
        reject(new Error("WebUSB Monitor 传输超时，已尝试重置 USB 设备"));
      }, timeoutMs);
    });
    try {
      const written = await Promise.race([
        this.device.transferOut(this.descriptor.endpointOut, frame),
        timeout,
      ]);
      if (written.status !== "ok" || written.bytesWritten !== frame.byteLength) {
        throw new Error(`WebUSB 写入失败：${written.status}`);
      }

      const received = await Promise.race([
        this.device.transferIn(this.descriptor.endpointIn, 512),
        timeout,
      ]);
      if (received.status !== "ok" || !received.data) throw new Error(`WebUSB 读取失败：${received.status}`);
      return parsePacket(
        new Uint8Array(received.data.buffer, received.data.byteOffset, received.data.byteLength),
      );
    } catch (error) {
      if (recovery) await recovery;
      else await this.close();
      throw error;
    } finally {
      globalThis.clearTimeout(timer);
    }
  }
}

export async function readWebUsbMonitor(transport, names, symbols, timeoutMs = 4000) {
  const started = performance.now();
  let index = 1;

  let packet = await transport.exchange(makePacket(index, CMD_CONNECT), timeoutMs);
  requireSuccess(packet, index, CMD_CONNECT);
  index += 1;

  packet = await transport.exchange(makePacket(index, CMD_GETINFO), timeoutMs);
  requireSuccess(packet, index, CMD_GETINFO);
  if (packet.payload.byteLength !== 24) throw new Error(`Monitor GETINFO 长度异常：${packet.payload.byteLength}`);
  const infoView = new DataView(packet.payload.buffer, packet.payload.byteOffset, packet.payload.byteLength);
  const info = {
    vid: infoView.getUint16(0, true),
    pid: infoView.getUint16(2, true),
    version: Number(infoView.getBigUint64(4, true)),
    maxsize: infoView.getUint32(12, true),
    memsize: infoView.getUint32(16, true),
    max_count: infoView.getUint16(20, true),
    threshold: infoView.getUint16(22, true),
  };
  if (info.max_count < 1) throw new Error("Monitor GETINFO max_count 无效");
  index += 1;

  const availableNames = names.filter((name) => name in symbols);
  if (availableNames.length === 0) throw new Error("ELF 中没有可读取的 Monitor 符号");
  const values = {};
  for (let start = 0; start < availableNames.length; start += info.max_count) {
    const chunk = availableNames.slice(start, start + info.max_count);
    packet = await transport.exchange(makePacket(index, CMD_GETVALUE, makeGetValuePayload(chunk, symbols)), timeoutMs);
    requireSuccess(packet, index, CMD_GETVALUE);
    Object.assign(values, parseGetValuePayload(chunk, packet.payload));
    index += 1;
  }

  return {
    info,
    values,
    symbols: Object.fromEntries(availableNames.map((name) => [name, symbols[name]])),
    transport: "webusb",
    port: null,
    sampled_at: new Date().toISOString(),
    duration_ms: Math.round((performance.now() - started) * 10) / 10,
  };
}
