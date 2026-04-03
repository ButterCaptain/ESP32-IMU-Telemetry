# MPU6050 Real-Time Attitude Visualizer — Zero-Latency Binary Telemetry Pipeline

![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Protocol](https://img.shields.io/badge/protocol-ESP--NOW-orange)
![Filter](https://img.shields.io/badge/filter-Madgwick%20AHRS-green)
![Status](https://img.shields.io/badge/status-active%20development-yellow)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> A full attitude estimation and real-time 3D visualization pipeline built on two ESP32s, an MPU6050 IMU, and a browser-based WebGL dashboard. Orientation data is fused using the Madgwick AHRS filter, packed into a 16-byte binary struct, and transmitted peer-to-peer via ESP-NOW — bypassing TCP/IP entirely to achieve low-latency telemetry that mirrors real-world aerospace ground station architecture.

![Demo GIF](docs/demo.gif)

---

## Table of Contents
- [Overview](#overview)
- [Why This Architecture?](#why-this-architecture)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Theory](#theory)
  - [Sensor Fusion — Why Two Sensors?](#sensor-fusion--why-two-sensors)
  - [Accelerometer Calibration](#accelerometer-calibration)
  - [The Madgwick Filter](#the-madgwick-filter)
  - [Quaternions vs. Euler Angles](#quaternions-vs-euler-angles)
  - [The β Parameter](#the-β-parameter)
- [Implementation](#implementation)
  - [Flight Node — Firmware](#flight-node--firmware)
  - [Ground Station — Firmware](#ground-station--firmware)
  - [Ground Control — Node.js Server](#ground-control--nodejs-server)
  - [Dashboard — Frontend Visualizer](#dashboard--frontend-visualizer)
- [Schematic](#schematic)
- [Getting Started](#getting-started)
- [Engineering Log — Design Decisions & Dead Ends](#engineering-log--design-decisions--dead-ends)
- [Known Issues & Future Work](#known-issues--future-work)
- [References](#references)

---

## Overview

This project implements a complete attitude estimation and telemetry pipeline modeled on real aerospace ground station architecture. An MPU6050 6-DOF IMU is read by a **Flight Node ESP32**, which applies a static calibration offset, runs the Madgwick AHRS filter to compute a stable quaternion orientation, packs the result into a 16-byte binary struct, and transmits it peer-to-peer to a **Ground Station ESP32** via ESP-NOW — a connectionless radio protocol that bypasses the Wi-Fi stack entirely.

The Ground Station receives the binary packet over the air, prepends a sync header, and blasts the raw bytes over USB serial to a **Node.js server** running on the host computer. The server hunts for the sync header, slices clean 18-byte packets from the serial stream, and rebroadcasts them over a local WebSocket. A **browser-based dashboard** (Three.js / WebGL) connects to the WebSocket, unpacks the binary floats using a `DataView`, and applies the quaternion directly to a 3D F-22 model in real time.

The result is a low-latency, network-independent telemetry link that works anywhere — no router required. This sensor module and architecture form the foundation for future projects with UNLV's AIAA rocketry and aerospace club.

---

## Why This Architecture?

The naive approach — HTTP `fetch()` from a browser to an ESP32 over Wi-Fi — was the starting point. It failed. Update rates topped out at **2–4 Hz**, producing a visualization that was unusable for real attitude tracking. The root causes were systematic, and solving them required rethinking every layer of the stack.

| Problem | Root Cause | Solution |
|---------|-----------|----------|
| 2–4 Hz update rate | HTTP TCP handshake overhead on every request | Replaced with ESP-NOW (connectionless) + WebSocket (persistent connection) |
| 85-byte packets for 16 bytes of data | JSON text serialization overhead | Raw binary struct with `#pragma pack(push, 1)` — 16 bytes flat |
| Campus network blocking WebSockets | Institutional firewall (eduroam) | Eliminated Wi-Fi entirely — ESP-NOW operates below the IP stack |
| Serial stream misalignment | No packet framing on raw binary | `0xAA 0xBB` sync header — parser hunts for beacon before reading payload |
| Gimbal lock in visualization | Euler angle representation | Switched to native quaternion transmission and Three.js `Quaternion` object |

Each fix is the same decision a working aerospace engineer would make. The architecture that emerged — sensor node, radio link, hardware bridge, serial parser, WebSocket relay, binary visualizer — is a functional scaled-down version of how real flight telemetry systems are designed.

---

## System Architecture

```
┌─────────────────────────────────────────────┐
│            FLIGHT NODE (ESP32 #1)           │
│                                             │
│  MPU6050 ──I2C──► Calibration Offset        │
│                        │                    │
│                        ▼                    │
│                  Madgwick Filter            │
│                        │                    │
│                        ▼                    │
│             Quaternion [W, X, Y, Z]         │
│                        │                    │
│              #pragma pack(push, 1)          │
│                  16-byte struct             │
│                        │                    │
│                   ESP-NOW TX                |
└─────────────────────────────────────────────┘
                          │  (2.4GHz, peer-to-peer)
                          ▼
┌─────────────────────────────────────────────┐
│          GROUND STATION (ESP32 #2)          │
│                                             │
│                   ESP-NOW RX                │
│                        │                    │
│              Prepend 0xAA 0xBB header       │
│                        │                    │
│            Serial.write() — 18 raw bytes    │
└─────────────────────────┬───────────────────┘
                          │ USB Serial (115200 baud)
                          ▼
┌─────────────────────────────────────────────┐
│         GROUND CONTROL (Node.js Server)     │
│                                             │
│  serialport listener → buffer accumulator   │
│  → hunt for 0xAA 0xBB → slice 22-byte packet│
│  → WebSocket broadcast (ws://localhost:8080)│
└─────────────────────────┬───────────────────┘
                          │ WebSocket (binary arraybuffer)
                          ▼
┌─────────────────────────────────────────────┐
│          DASHBOARD (visualizer.html)        │
│                                             │
│  DataView → unpack 4x float32 quaternion    │
│  → Three.js Quaternion → F-22 model rotation│
└─────────────────────────────────────────────┘
```

**Data flow summary:**
1. MPU6050 sampled over I2C — raw 16-bit accelerometer and gyroscope registers
2. Accelerometer bias offsets subtracted (computed during startup calibration)
3. Madgwick filter fuses accel + gyro → unit quaternion `[W, X, Y, Z]`
4. Four floats packed into a 16-byte struct via `#pragma pack(push, 1)`
5. Struct broadcast via ESP-NOW to Ground Station MAC address
6. Ground Station prepends `0xAA 0xBB` and writes 18 bytes over USB serial
7. Node.js server parses the serial stream, extracts packets, broadcasts over WebSocket
8. Browser receives binary `arraybuffer`, unpacks with `DataView`, applies quaternion to 3D model

---

## Hardware

| Component | Role | Notes |
|-----------|------|-------|
| ESP32 Dev Board (×2) | Flight Node + Ground Station | Any 38-pin ESP32. Each has a unique MAC address used for ESP-NOW pairing. |
| MPU6050 | 6-DOF IMU (accel + gyro) | I2C address 0x68. Configured to ±4g / ±500 dps. |
| USB Cable (×1) | Ground Station → PC | Data + power. Ground Station is permanently tethered to host computer. |
| Breadboard + Jumper Wires | Prototyping | No soldering required for current revision. |

**Flight Node Wiring (ESP32 #1 ↔ MPU6050):**

| MPU6050 Pin | ESP32 Pin | Notes |
|-------------|-----------|-------|
| VCC | 5.0V | MPU6050 (3.3V to 5V) |
| GND | GND | |
| SDA | GPIO 21 | Hardware I2C |
| SCL | GPIO 22 | Hardware I2C |
| INT | GPIO 19 | Optional — interrupt-driven sampling |

**Ground Station (ESP32 #2):** No external components — connects to PC via USB only.

---

## Theory

### Sensor Fusion — Why Two Sensors?

The MPU6050 contains two independent sensors that measure complementary physical quantities, each with a fundamental flaw when used alone.

The **gyroscope** measures angular rate in degrees per second. Integrating this over time gives orientation — the result is smooth and fast-responding. But integration accumulates error. Any small bias in the sensor reading compounds with every sample, causing the estimated orientation to **drift** continuously away from reality. Left uncorrected, a gyro-only estimate becomes unusable within seconds.

The **accelerometer** measures the vector sum of all forces on the sensor, including gravity. When the sensor is stationary or at constant velocity, the accelerometer points directly at the ground — giving an absolute, drift-free tilt reference. But the moment the sensor accelerates, the reading is corrupted by linear acceleration and can no longer be trusted as a gravity reference.

Sensor fusion combines both: the gyroscope provides smooth, fast short-term tracking; the accelerometer provides a long-term absolute reference that corrects gyroscope drift.

---

### Accelerometer Calibration

Before the filter runs, a static calibration routine executes on startup. The ESP32 collects `N` samples of raw accelerometer data while the sensor sits at rest, computes the mean value on each axis, and stores the results as bias offsets. Every subsequent reading has these offsets subtracted before being passed to the Madgwick filter.

This corrects **zero-g offset error** — a fixed DC bias present in every MEMS accelerometer due to manufacturing tolerances, die stress, and PCB mounting. Without correction, the filter receives a skewed gravity vector and converges to a permanently tilted orientation estimate even when the sensor is perfectly flat.

Mean-based offset removal is the most practical entry-level IMU calibration technique and is a standard preprocessing step in embedded attitude estimation systems. It assumes the sensor is stationary during the startup window — a known limitation noted in future work.

---

### The Madgwick Filter

The Madgwick AHRS filter was chosen after evaluating two alternatives:

- **Complementary filter** — implemented first. A simple weighted average of gyro and accel outputs. Produced persistent drift regardless of parameter tuning. Abandoned.
- **Kalman filter** — considered next. Statistically optimal but computationally heavier and more complex to tune for a microcontroller. Deferred for future comparison.
- **Madgwick filter** — selected. Developed by Sebastian O.H. Madgwick (2010) specifically as a computationally efficient MEMS IMU fusion algorithm.

The filter works in three steps each cycle:

1. **Gyroscope propagation** — the current quaternion is updated by integrating the angular rate measurement, advancing the orientation estimate forward in time
2. **Gradient descent correction** — the accelerometer measurement is compared against the expected direction of gravity given the current orientation estimate. The orientation is nudged in the direction that minimizes that error
3. **Blending** — the magnitude of the correction is controlled by the **β parameter**, balancing gyroscope smoothness against accelerometer correction

The filter operates entirely in quaternion space, which avoids the gimbal lock problem inherent in Euler angle representations.

---

### Quaternions vs. Euler Angles

Euler angles (roll, pitch, yaw) are intuitive but suffer from **gimbal lock** — a singularity that occurs when two rotation axes align, causing the orientation representation to lose a degree of freedom. In practice this produces sudden, discontinuous jumps in the output. This was observed directly during development: the initial Euler-based visualization produced erratic behavior at certain orientations.

Quaternions represent a 3D rotation as four numbers **q = [w, x, y, z]**, where `w` is the scalar component (related to rotation angle) and `[x, y, z]` is the vector component (axis of rotation). They are free from gimbal lock, numerically stable under continuous integration, and directly supported in Three.js via the native `THREE.Quaternion` class.

Switching from Euler angle transmission to raw quaternion transmission eliminated the gimbal lock issue entirely and reduced the payload to the minimum possible representation of a 3D rotation — four floats.

---

### The β Parameter

β controls how aggressively the accelerometer corrects the gyroscope integration each cycle:

- **Higher β** → stronger accelerometer correction → less drift, but more susceptibility to noise and linear acceleration artifacts → more jitter
- **Lower β** → gyroscope dominates → smoother output, but orientation drifts over time
- A typical starting value is **β = 0.1**. Optimal tuning depends on the expected motion profile of the application.

For a flight vehicle experiencing sustained linear acceleration, β should be reduced or made adaptive — an area of active investigation for the next revision.

---

## Implementation

### Flight Node — Firmware

The Flight Node is the ESP32 attached to the MPU6050. Written using the **Arduino framework for ESP32** via Arduino IDE — the officially supported development environment for this hardware.

**File:** `flight_node/flight_node.ino`

**Startup sequence:**
1. Wake MPU6050 from sleep (`register 0x6B → 0x00`)
2. Configure accel range to ±4g (`register 0x1C`) and gyro range to ±500 dps (`register 0x1B`)
3. Run calibration loop — collect `N` samples, compute per-axis mean, store as `accelOffsetX/Y/Z`
4. Register Ground Station MAC address with ESP-NOW
5. Enter main loop

**Main loop:**
1. Read raw 16-bit accel and gyro registers over I2C — skip temperature bytes at `0x41–0x42`
2. Convert raw integers to physical units using sensitivity scale factors
3. Subtract calibration offsets from accelerometer readings
4. Pass calibrated accel + raw gyro to Madgwick filter → receive updated quaternion `[W, X, Y, Z]`
5. Pack quaternion into 16-byte struct via `#pragma pack(push, 1)`
6. Broadcast struct via `esp_now_send()` to Ground Station MAC address

**Packet structure:**
```cpp
#pragma pack(push, 1)
struct TelemetryPacket {
    float w, x, y, z;      // Quaternion components
};                          // Total: 16 bytes, zero padding
#pragma pack(pop)
```

> **Why `#pragma pack(push, 1)`?** By default the compiler inserts padding bytes between struct members to satisfy memory alignment requirements. A naive four-float struct could be 16–24 bytes depending on the target architecture. `pack(1)` forces one-byte alignment, producing a perfectly packed 16-byte block. The `DataView` on the JavaScript side reads from exact byte offsets — both sides must agree on the layout, and with `pack(1)` they always will.

**Key configurable parameters:**
```cpp
uint8_t groundStationMAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}; // Update to your Ground Station MAC
float beta = 0.1f;           // Madgwick gain
int calibrationSamples = 500; // Samples averaged on startup — sensor must be stationary
int sampleRateHz = 100;
```

---

### Ground Station — Firmware

The Ground Station ESP32 is a pure hardware bridge — no sensor, no computation. It receives the ESP-NOW radio packet and forwards it over USB serial.

**File:** `ground_station/ground_station.ino`

**Main logic:**
1. Register ESP-NOW receive callback
2. On packet received: copy 16-byte struct into local buffer
3. Write sync header: `Serial.write(0xAA); Serial.write(0xBB);`
4. Write payload: `Serial.write((uint8_t*)&packet, sizeof(packet));`
5. Total USB output: **18 bytes per update**

> **Why a sync header?** USB serial is a continuous byte stream with no packet boundaries. If the Node.js server starts reading mid-packet, every subsequent float is misaligned and the quaternion values are garbage. The `0xAA 0xBB` header acts as a known beacon — the parser discards bytes until it finds this sequence, then reads exactly 16 bytes of payload. This is the same framing strategy used in real embedded communication protocols (MAVLink, SBUS, CRSF).

---

### Ground Control — Node.js Server

The Node.js server bridges the hardware world (USB serial) and the browser world (WebSocket). Browsers are sandboxed — they cannot access COM ports directly. The server holds OS-level permissions to read the serial port and re-serves the data over a local WebSocket.

**File:** `server/server.js`

**Dependencies:**
```bash
npm install serialport ws
```

**Logic:**
1. Open COM port at 115200 baud using `serialport`
2. Accumulate incoming bytes in a rolling buffer
3. Scan buffer for `0xAA 0xBB` sync header
4. On header found: extract next 16 bytes as one packet
5. Broadcast raw binary packet over WebSocket to all connected clients

**Configuration:**
```js
const PORT = 'COM3';      // Windows — update to your Ground Station port
                          // Linux/Mac: '/dev/ttyUSB0' or '/dev/tty.usbserial-XXXX'
const BAUD = 115200;
const WS_PORT = 8080;
```

---

### Dashboard — Frontend Visualizer

The visualizer is a single standalone HTML file. No build tools, no framework, no install required — open directly in any browser.

**File:** `visualizer.html`

**What it does:**
1. Connects to the Node.js WebSocket at `ws://localhost:8080`
2. Requests binary data: `ws.binaryType = 'arraybuffer'`
3. Unpacks four `float32` values from the binary buffer using `DataView`:
```js
const view = new DataView(event.data);
const w = view.getFloat32(0, true);   // little-endian
const x = view.getFloat32(4, true);
const y = view.getFloat32(8, true);
const z = view.getFloat32(12, true);
```
4. Applies quaternion to the F-22 model:
```js
model.quaternion.set(x, y, z, w);    // Three.js uses [x, y, z, w] order
```

> **On binary vs. JSON:** The initial implementation transmitted quaternion data as a JSON string — e.g., `{"w":0.99,"x":0.01,"y":0.02,"z":0.00}`. This produced packets up to 85 bytes for 16 bytes of actual data, added CPU overhead for string formatting on the ESP32, and introduced parsing complexity on the receive side. Switching to raw binary reduced packet size by ~75%, eliminated formatting overhead entirely, and guaranteed exact 32-bit float precision with no rounding from decimal string conversion.

---

## Schematic

**Flight Node:**


*Ground Station has no external components — ESP32 to PC via USB only.*

---

## Getting Started

### Requirements

**Hardware:**
- 2× ESP32 development boards (38-pin)
- 1× MPU6050 breakout board
- Breadboard + jumper wires
- 2× USB cables

**Software:**
- [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board package
- [Node.js](https://nodejs.org/) v16 or later
- Chrome or any modern browser

**Arduino Libraries** (built-in with ESP32 core — no separate install needed):
- `Wire.h`, `WiFi.h`, `esp_now.h`

---

### Step 1 — Install ESP32 Board Package

In Arduino IDE: File → Preferences → Additional Board Manager URLs, add:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Then: Tools → Board → Board Manager → search "esp32" → Install

### Step 2 — Get the Ground Station MAC Address

Flash `utils/get_mac.ino` to **ESP32 #2**. Open Serial Monitor at 115200 baud. Copy the printed MAC address.

### Step 3 — Flash the Flight Node

1. Open `flight_node/flight_node.ino`
2. Paste the Ground Station MAC into `groundStationMAC[]`
3. Tools → Board → **ESP32 Dev Module** → Upload to **ESP32 #1**

### Step 4 — Flash the Ground Station

1. Open `ground_station/ground_station.ino`
2. No configuration needed
3. Upload to **ESP32 #2**, leave plugged into PC via USB

### Step 5 — Start the Node.js Server

```bash
cd server
npm install
```
Edit `server.js` — set `PORT` to your Ground Station's COM port. Then:
```bash
node server.js
```
Expected output: `Serial port open. WebSocket server listening on ws://localhost:8080`

### Step 6 — Launch the Visualizer

1. Hold the Flight Node **still** for ~5 seconds after powering on — calibration is running
2. Open `visualizer.html` in Chrome
3. The F-22 model will begin mirroring the physical sensor orientation in real time

---

## Engineering Log — Design Decisions & Dead Ends

This section documents the actual development process — what was tried, what failed, and why the final architecture looks the way it does.

### v1 — Raw Accelerometer over HTTP

First working version: read raw accelerometer data, serve as JSON over HTTP, poll from the browser using `fetch()`. Readings were immediately wrong — constant noise and a persistent tilt offset that made the visualization unusable.

**Root cause:** Zero-g offset bias. Every MEMS accelerometer has a manufacturing-level DC bias. Without correction, the sensor reports a non-zero acceleration even when perfectly stationary, which the visualizer interprets as a real tilt.

**Fix:** Mean-sample calibration on startup — collect N samples at rest, compute per-axis mean, subtract from all future readings.

---

### v2 — Complementary Filter

Added a complementary filter to fuse gyroscope and accelerometer. Still transmitting Euler angles over HTTP.

**Result:** Drift was reduced but not eliminated. The complementary filter's fixed-weight blending does not fully account for how sensor errors evolve over time. Tuning helped but could not resolve the fundamental limitation.

**Decision:** Research more rigorous fusion algorithms.

---

### v3 — Madgwick Filter + Gimbal Lock Discovery

Implemented the Madgwick AHRS filter after reviewing the original 2010 paper. The improvement over the complementary filter was immediate — stable orientation estimate, minimal drift.

New problem: **gimbal lock** in the visualization. At certain orientations, the 3D model would spin erratically.

**Root cause:** The firmware was outputting quaternions, but the visualizer was converting them to Euler angles before applying the rotation. The conversion itself introduced the singularity.

**Fix:** Transmit quaternions directly and apply them to Three.js's native `Quaternion` object — no Euler conversion anywhere in the pipeline.

**Additional fix:** Roll and pitch axes were swapped in the output due to the physical mounting orientation of the sensor on the breadboard. Corrected by swapping axis assignments in firmware.

---

### v4 — HTTP Bottleneck & JSON Overhead Identified

With Madgwick stable, update rate became the bottleneck. Browser `fetch()` over HTTP caps out at **2–4 Hz** due to TCP connection overhead on every request. JSON formatting added up to 85 bytes per packet for 16 bytes of data, and the string serialization consumed measurable CPU cycles on the ESP32.

**Options evaluated:**
- WebSocket over campus Wi-Fi → blocked by eduroam institutional firewall
- HTTP long-polling → still TCP-bound, marginal improvement
- ESP-NOW → peer-to-peer radio, no router, no IP stack overhead, works anywhere

---

### v5 — Current: Binary ESP-NOW Telemetry Pipeline

ESP-NOW was selected as the air-gap protocol. It operates on 2.4GHz but uses a connectionless framing with no SSID, no router, and no TCP stack — fundamentally lower overhead than Wi-Fi.

Data format redesigned from JSON to `#pragma pack(push, 1)` binary struct — 20 bytes flat, zero serialization overhead, exact float precision. A second ESP32 added as a dedicated Ground Station. Node.js bridges serial to WebSocket. Browser visualizer connects to WebSocket persistently.

The result is a low-latency telemetry link that also happens to mirror the architecture of real aerospace telemetry systems: a flight node, a radio link, a dedicated hardware receiver, a ground control software layer, and a visualization frontend.

---

## Known Issues & Future Work

### Current Issues
- **Calibration requires stationary startup** — if the sensor is moving when powered on, the bias offset computation is corrupted. A motion-detection gate during calibration would make this robust.
- **No yaw reference** — without a magnetometer, yaw drifts freely. The Madgwick filter corrects roll and pitch using gravity but has no absolute heading reference.
- **β not yet optimally tuned** — current value is a reasonable starting point, not a characterized optimum.

### Planned Improvements
- [ ] Gyro bias calibration — apply mean-offset approach to gyroscope at startup
- [ ] Dynamic β — reduce accelerometer correction weight during detected high-acceleration events
- [ ] Add magnetometer (HMC5883L or upgrade to MPU9250) for full 9-DOF AHRS with yaw reference
- [ ] Multi-position accelerometer calibration for scale factor correction (beyond bias-only removal)
- [ ] Implement Kalman filter for direct performance comparison against Madgwick
- [ ] Packet loss detection and logging using heartbeat counter
- [ ] Move to interrupt-driven IMU sampling for more consistent timing
- [ ] Add HUD overlay to visualizer — live quaternion readout, packet rate, heartbeat counter

---

## File Structure

```
/
├── flight_node/
│   └── flight_node.ino         # IMU read, calibration, Madgwick filter, ESP-NOW TX
├── ground_station/
│   └── ground_station.ino      # ESP-NOW RX, binary serial TX to PC
├── server/
│   ├── server.js               # Node.js serial → WebSocket bridge
│   └── package.json
├── visualizer.html             # Browser dashboard — WebSocket client, Three.js F-22 renderer
├── utils/
│   └── get_mac.ino             # Prints ESP32 MAC address to Serial Monitor
└── docs/
    ├── demo.gif
    └── schematic_flight_node.png
```

---

## References

- Madgwick, S.O.H. (2010). *An efficient orientation filter for inertial and inertial/magnetic sensor arrays.* University of Bristol. [PDF](https://x-io.co.uk/res/doc/madgwick_internal_report.pdf)
- [MPU-6000/MPU-6050 Product Specification Rev 3.4](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6000/MPU-6050 Register Map and Descriptions Rev 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [Espressif ESP-NOW Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [ESP32 Arduino Core Documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [Three.js Quaternion Documentation](https://threejs.org/docs/#api/en/math/Quaternion)
- Diebel, J. (2006). *Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors.* Stanford University.
- [Node.js serialport library](https://serialport.io/docs/)

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

*Built as a freshman EE project. The sensor module and telemetry architecture developed here will serve as the foundation for future AIAA rocketry and aerospace projects at UNLV.*
