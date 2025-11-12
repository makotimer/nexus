# makotimer/nexus — The Embedded Bridge

Part of the **MakoTimer Network**: a distributed system for family task management, scheduling, and real-time updates.  
`nexus` is the **real-time embedded gateway** — the critical link between the cloud-connected `cortex` and the offline `portal` and `slate`.

Built in **Rust on Zephyr RTOS** for the **nRF7002-DK**, it provides **Wi-Fi connectivity to `cortex`**, **BLE GATT bridging to `portal`**, **sensor integration**, and **secure data routing** — all in a low-power, reliable package.

---

## High-Level Overview

The MakoTimer Network is a multi-repo, multi-platform system:

- `cortex` (Python) → schedules tasks, sends emails, publishes via MQTT  
- `nexus` (**this repo**) → **Wi-Fi + BLE bridge**, sensor hub, command relay  
- `portal` (ESP32-P4 + LVGL) → 10.1" touch display (BLE-only)  
- `slate` (Flutter) → mobile app (BLE control + Find My Phone)  

`cortex` has internet access for email, weather, and external services.  
`nexus` connects to `cortex` via **Wi-Fi/MQTT** and to `portal` via **BLE GATT** — ensuring `portal` and `slate` remain lightweight and offline-capable.

---

## Technologies Used

| Layer | Tech |
|------|------|
| **Hardware** | nRF7002-DK (Cortex-M33, Wi-Fi 6 + BLE 5.2) |
| **RTOS** | Zephyr (real-time, low-power, multi-threaded) |
| **Language** | Rust (`no_std`, Embassy async runtime) |
| **Wi-Fi** | `embassy-net` + `cyw43` (MQTT over TCP) |
| **BLE** | `embassy-ble` (GATT Central + Peripheral) |
| **Serialization** | CBOR (`serde_cbor`, compact, schema-light) |
| **Sensors** | BME680 (temp/humidity), PIR (motion), PN532 (NFC) |
| **Build/Flash** | `cargo embed`, `probe-rs`, J-Link |
| **CI/CD** | GitHub Actions (build → firmware artifact) |
| **Shared Assets** | Git submodule to `makotimer/shared` |

---

## What `nexus` Does

| Function | Description |
|--------|-----------|
| **Wi-Fi STA + MQTT Client** | Joins home network, subscribes to `makotimer/to_nexus` |
| **BLE Central Bridge** | Connects to `portal`, notifies via `MAKO_DATA` characteristic |
| **Sensor Fusion** | BME680 (indoor env), PIR (motion), PN532 (NFC check-in) |
| **Command Relay** | Forwards `MAKO_CMD` from `portal` → MQTT → `cortex` |
| **Health Heartbeat** | Publishes `makotimer/heartbeat` every 5 min |
| **Offline Cache** | Stores last valid CBOR for `portal` fallback |
| **Find-My-Phone Beacon** | Advertises special BLE packet on request |
| **Secure & Efficient** | Encrypted GATT, 512B MTU, zero-copy CBOR, auto-reconnect |

**Future-Proof**: OTA DFU, tinyML for edge decisions.
