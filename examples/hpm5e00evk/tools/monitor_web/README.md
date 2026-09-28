# HPM sensor monitor web dashboard

This local dashboard reads HPM Monitor V2 over backend USB vendor bulk,
browser-direct WebUSB, or CDC serial and visualizes the live `monitor_sensors`
values.

## Run

From `hpm-hal/examples/hpm5e00evk`:

```sh
uv run --with pyusb --with pyserial \
  python tools/monitor_web/server.py \
  --host 127.0.0.1 --port 8765 \
  --elf /path/to/the/programmed/monitor_sensors
```

Open <http://127.0.0.1:8765>. The page defaults to USB bulk, the release
`monitor_sensors` ELF, and a one-second refresh period.

## Dashboard

- BH1750 lux, raw reading, sample count, and 60-point trend.
- BME680 temperature, humidity, pressure, gas resistance, and trend.
- PMS7003 atmospheric PM1.0/PM2.5/PM10, frame status, and trend.
- APDS-9930 clear/IR/proximity raw values, device ID, and trend.
- I2C, UART3, USB Monitor, runtime, panic, and exception health.
- Expandable raw sensor, CSR, USB, and exception values.

The PMS7003 card applies an instantaneous engineering alert scale at 15, 35,
50, and 100 µg/m³. The active band, card border, primary value, and badge update
together; the page labels this as an instantaneous alert because formal indoor
air compliance uses a 24-hour mean. Climate and particle charts include
color-matched legends with the latest value for every series.

Trend charts use a local monotone cubic Hermite curve rendered as Canvas
Bezier segments. The slope limiter prevents overshoot between adjacent sensor
samples, while small point markers preserve the measured values and emphasize
the newest sample. This avoids the endpoint oscillation and whole-window drift
of a high-degree polynomial fit. Multiple-series charts render all low-opacity
area shadows first and all main strokes last. Screen blending mixes coincident
colors without letting a later series hide an earlier one. All series in a
chart share one vertical range, so stable readings remain visibly separated and
only genuine crossings overlap. Each new sample uses a 420 ms ease-out
transition: growing
histories compress smoothly, and a full 60-point window slides left as the new
point enters from the right. Reduced-motion browser preferences render the new
state immediately.

Polling uses a recursive timer. The next read begins after the previous read
finishes, so a slow or timed-out USB request cannot overlap a second request.
The page preserves the most recent successful snapshot during transport errors.

## WebUSB ownership

Select **WebUSB direct** or press **Connect WebUSB** to claim the HPM5E31 vendor
bulk interface from the browser. The page pauses backend bulk access before the
claim and renews a short ownership lease while connected. A clean switch waits
for the current Monitor frame before releasing WebUSB and resuming backend USB.
Lease expiry and interrupted transfers trigger a USB bus reset before backend
takeover.
