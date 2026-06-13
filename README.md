# SmartVehicle ECU — Adaptive Smart Cabin & Vehicle Health Monitoring System

**Visteon C++ Hackathon — Adaptive AUTOSAR Style ECU Simulation**  
**Group C-13** | Piyush Bhatt · Ashutosh Bajpayee · Smriti Chaterjee · Ragini Kumari

---

## Overview

A multithreaded, modern **C++17** simulation of a vehicle cockpit domain controller. The system monitors six sensor streams concurrently, raises and clears alerts in real time, logs all events to `logs/vehicle_log.txt`, and displays a live dashboard — all running in **four concurrent threads**.

Ten automotive-grade bonus features extend the core with OBD-II diagnostics, CAN bus simulation, OTA update management, crash-safe recording, and more.

---

## Folder Structure

```
SmartVehicle_WithFeatures_v13/
├── include/
│   ├── Sensor.hpp                  – Base Sensor + 6 derived sensor classes
│   ├── Alert.hpp                   – Alert (enum class, operator<<) + AlertManager
│   ├── AlertEvaluator.hpp          – Alert threshold evaluation logic
│   ├── Dashboard.hpp               – Dashboard + VehicleStatistics (operator<<)
│   ├── Logger.hpp                  – EventLogger (RAII, templates, lambda search)
│   ├── Statistics.hpp              – Statistical tracking utilities
│   ├── Performance.hpp             – Performance monitoring base
│   ├── PerformanceGraphStats.hpp   – Terminal bar-graph renderer (60-sample ring buffer)
│   ├── DriverProfile.hpp           – Named driver profiles (ECO / SPORT / LIMP)
│   ├── CanBus.hpp                  – CAN frame encoder / 500 kbps simulator
│   ├── DtcSimulator.hpp            – OBD-II DTC lifecycle (PENDING → CONFIRMED)
│   ├── EcuHealthMonitor.hpp        – Per-sensor fault-rate & overall ECU health
│   ├── OtaUpdateSimulator.hpp      – 8-state OTA lifecycle with rollback
│   ├── CrashSafeRecorder.hpp       – 64-slot ring buffer with CRC-32 NvM persistence
│   └── ServiceOrientedComm.hpp     – SOME/IP-inspired service-oriented communication
│
├── src/
│   ├── main.cpp                    – ThreadManager, 4 threads, interactive menu, entry point
│   ├── Sensor.cpp                  – All sensor implementations
│   ├── Alert.cpp                   – Alert & AlertManager implementations
│   ├── AlertEvaluator.cpp          – Alert condition evaluation
│   ├── Dashboard.cpp               – Dashboard & VehicleStatistics implementations
│   ├── Logger.cpp                  – EventLogger implementation
│   ├── Statistics.cpp              – Statistics implementation
│   ├── Performance.cpp             – Performance implementation
│   ├── PerformanceGraphStats.cpp   – Graph rendering implementation
│   ├── DriverProfile.cpp           – Driver profile CRUD + persistence
│   ├── CanBus.cpp                  – CAN frame encode/decode + error injection
│   ├── DtcSimulator.cpp            – DTC state machine + freeze-frame capture
│   ├── EcuHealthMonitor.cpp        – Health scoring + uptime tracking
│   ├── OtaUpdateSimulator.cpp      – OTA state machine + SHA-256/RSA-4096 verify
│   ├── CrashSafeRecorder.cpp       – Event classification + atomic NvM write
│   ├── ServiceOrientedComm.cpp     – SOME/IP SD exchange + event notification
│   └── testMain.cpp                – Unit test runner
│
├── data/
│   ├── config.txt                  – Runtime configuration reference
│   ├── driver_profiles.txt         – Persisted driver profiles (auto-generated)
│   ├── input.json                  – Sensor test-case array (JSON test runner)
│   ├── expected.json               – Expected alert outcomes for test cases
│   ├── output.txt                  – Test result output (auto-generated)
│   ├── status.txt                  – System status snapshot
│   └── test_cases.json             – Extended test scenarios
│
├── logs/
│   ├── vehicle_log.txt             – Main event log (auto-generated at runtime)
│   ├── edr_events.txt              – EDR / crash-safe event records
│   ├── edr_snapshot.bin            – Binary NvM snapshot (CRC-32 protected)
│   ├── json_test_log.txt           – JSON test run results
│   ├── manual_test_log.txt         – Manual test session log
│   ├── performance_report.txt      – Performance graph stats export
│   └── performance_report.txt      – Performance statistics export
│
├── build/                          – Compiled object files (auto-generated)
├── Makefile
├── README.md
└── SmartVehicle                    – Compiled binary (after make)
```

---

## Build & Run

```bash
# Prerequisites: g++ (C++17), GNU Make, pthreads — Ubuntu 22.04 recommended

make              # compile all sources
make run          # compile + launch interactive dashboard
make clean        # remove build/ and binary
```

**Minimum requirements:**

| Component         | Requirement              |
|-------------------|--------------------------|
| OS                | Ubuntu 20.04 LTS or later|
| Compiler          | GCC/G++ 9.0+ (11+ recommended) |
| C++ Standard      | C++17                    |
| Build tool        | GNU Make (latest)        |
| Threading         | POSIX pthreads           |
| CPU               | Dual-core, x86_64        |
| RAM               | 4 GB minimum             |

---

## Startup Sequence

| Phase      | What Happens                                                               |
|------------|----------------------------------------------------------------------------|
| **Phase 1**| Sensor objects created · Dashboard & Alert Manager initialised · Logger opened |
| **Phase 2**| Four worker threads spawned: Sensor · Monitoring · Dashboard · Logger       |
| **Phase 3**| Continuous loop: update sensors → evaluate → alert → display → log → repeat |
| **Shutdown**| Threads join · files closed · buffers flushed · stats saved · memory freed |

---

## Interactive Menu

Once the system starts, press **Enter** at any time to pause the live dashboard and access the main menu:

```
============================================================
         SmartVehicle ECU — Main Menu
============================================================
  [1]  View live sensor dashboard
  [2]  View active alerts
  [3]  Run JSON test suite          (data/input.json)
  [4]  Driver Profile Manager
  [5]  DTC Diagnostic Report        (OBD-II)
  [6]  ECU Health Monitor
  [7]  Performance Graph Statistics
  [8]  OTA Update Simulator
  [9]  Crash-Safe Event Recorder
  [10] CAN Bus Monitor
  [11] Service-Oriented Comm        (SOME/IP)
  [0]  Exit
============================================================
```

### Menu Options in Detail

**[1] Live Sensor Dashboard**  
Displays real-time sensor values (engine temp, battery voltage, speed, tire pressure, door status, seatbelt status) alongside the active alert list and session statistics. Refreshes every 3 seconds.

**[2] Active Alerts**  
Shows all currently active alerts with severity, timestamp, and description. Supports filtering by severity (CRITICAL / WARNING / INFO).

**[3] JSON Test Suite**  
Reads `data/input.json` (an array of sensor tick objects), feeds each tick through the full alert pipeline, compares results against `data/expected.json`, and writes a pass/fail report to `data/output.txt`. Useful for regression testing alert conditions.

**[4] Driver Profile Manager**  
CRUD interface for named driver profiles stored in `data/driver_profiles.txt`:
- Create / edit / delete profiles
- Built-in profiles: `Default`, `EcoDriver`, `SportDriver`
- Each profile stores: driving mode (ECO / NORMAL / SPORT / LIMP), speed threshold, temperature threshold, alert toggles
- Active profile adjusts alert thresholds at runtime

**[5] DTC Diagnostic Report (OBD-II)**  
Simulates six OBD-II Diagnostic Trouble Codes:

| DTC Code | Description                  |
|----------|------------------------------|
| P0217    | Engine Coolant Over-Temperature |
| P0562    | System Voltage Low           |
| P0082    | Intake Air Control Circuit Low |
| C0035    | Left Front Wheel Speed Sensor |
| B1001    | ECU Internal Fault           |
| B2101    | Seat Position Module Fault   |

Options: inject fault → 3-cycle debounce promotes PENDING → CONFIRMED → freeze-frame snapshot captured. Clear by DTC code or clear all. Full MIL (Malfunction Indicator Lamp) report.

**[6] ECU Health Monitor**  
Shows per-sensor fault rates, overall ECU health score, session uptime, total alert count, and alert rate per minute.

| Health State | Threshold          |
|--------------|--------------------|
| HEALTHY      | Fault rate ≥ 70%   |
| DEGRADED     | Fault rate ≥ 40%   |
| CRITICAL     | Fault rate < 40%   |

**[7] Performance Graph Statistics**  
Renders terminal bar graphs for speed and engine temperature from a 60-sample ring buffer. Colour-coded green/yellow/red. Displays min, average, and max below each graph. Exportable to `logs/performance_report.txt`.

**[8] OTA Update Simulator**  
Manages firmware updates for 5 ECU modules through an 8-state lifecycle:

```
IDLE → CHECKING → DOWNLOADING → VERIFYING → READY → INSTALLING → REBOOTING → SUCCESS
                                                                            ↘ FAILED (rollback)
```

Features: chunk-by-chunk animated download progress, SHA-256 + RSA-4096 signature verification, WP.29 R156 speed=0 safety gate (refuses update while vehicle is moving), 5% simulated flash-fault rollback.

**[9] Crash-Safe Event Recorder (EDR)**  
64-slot ring buffer recording crash and fault events. Auto-classifies events as NORMAL / WARNING / FAULT / CRASH based on:
- Deceleration > 25 km/h per cycle
- Temperature spike > 10°C per cycle
- Voltage drop > 1 V per cycle
- Critical active alerts

Each record has CRC-32/ISO-HDLC protection. Persisted via atomic write-then-rename to `logs/edr_snapshot.bin` and `logs/edr_events.txt`. Options: view history, export snapshot, clear log.

**[10] CAN Bus Monitor**  
Simulates a 500 kbps CAN-HS bus with six frames:

| Frame ID | Signal                        | Scaling           |
|----------|-------------------------------|-------------------|
| 0x100    | Engine Temperature            | 0.5 °C / LSB      |
| 0x200    | Battery Voltage               | 0.01 V / LSB      |
| 0x300    | Vehicle Speed                 | 0.1 km/h / LSB    |
| 0x400    | Tire Pressure (FL/FR/RL/RR)   | 0.25 PSI / LSB    |
| 0x500    | Door & Seatbelt Status        | Bitfield           |
| 0x600    | Alert Flags                   | Bitfield           |

Features: 1% bit-flip error injection, CRC-8/MAXIM frame validation, RX/TX logging, error counter.

**[11] Service-Oriented Communication (SOME/IP)**  
Simulates 5 SOME/IP-inspired services with full SD (Service Discovery) exchange:

```
SD_OFFER → SD_FIND → SD_SUBSCRIBE → NOTIFICATION (on value delta)
```

Services: EngineDataService, BatteryService, SpeedService, TireService, AlertService.  
All messages logged with session IDs, timestamps, and SOME/IP return codes.

---

## Alert Conditions

| Condition                          | Alert Code                  | Severity   |
|------------------------------------|-----------------------------|------------|
| Engine temperature > 110°C         | CRITICAL_ENGINE_OVERHEAT    | CRITICAL   |
| Engine temperature > 95°C          | HIGH_ENGINE_TEMP            | WARNING    |
| Battery voltage < 10 V             | LOW_BATTERY_WARNING         | CRITICAL   |
| Tire pressure < 25 PSI             | LOW_TIRE_PRESSURE           | WARNING    |
| Speed > 120 km/h                   | OVERSPEED_WARNING           | CRITICAL   |
| Door OPEN while speed > 10 km/h    | DOOR_OPEN_WARNING           | WARNING    |
| Seatbelt UNLOCKED while moving     | SEATBELT_WARNING            | WARNING    |

---

## Thread Architecture

| Thread              | Shared Resources              | Mutex Used                   | Interval |
|---------------------|-------------------------------|------------------------------|----------|
| **Sensor Thread**   | Sensor value objects          | `sensorMutex`                | 800 ms   |
| **Monitoring Thread**| Sensor values, alert list    | `sensorMutex`, `alertMutex`  | 900 ms   |
| **Dashboard Thread**| Sensor values, active alerts  | `sensorMutex`, `alertMutex`  | 3 s      |
| **Logger Thread**   | Event history, log file       | `logMutex`                   | 5 s      |

All threads share data through `std::shared_ptr` and protect every critical section with `std::mutex` + `std::lock_guard` (RAII).

---

## C++ Concepts Demonstrated

| Concept                  | Where Used                                                      |
|--------------------------|-----------------------------------------------------------------|
| Classes & Objects        | Sensor, Alert, Dashboard, AlertManager, EventLogger, all bonus  |
| Constructors/Destructors | All classes; RAII in EventLogger                                |
| RAII                     | EventLogger opens file in ctor, closes in dtor                  |
| Inheritance              | `Sensor` (base) → 6 derived sensor classes                      |
| Virtual Functions        | `update()` and `display()` are pure virtual                     |
| STL Containers           | `vector`, `map`, `set` in AlertManager, Logger, DTC, profiles   |
| Templates                | `EventLogger::log<T>()` generic log method                      |
| Smart Pointers           | `shared_ptr` throughout; `make_shared` factory pattern          |
| Exception Handling       | `try/catch` in `main()`; logger throws on file failure          |
| Threads & Mutex          | 4 `std::thread`s; `std::mutex` + `lock_guard` everywhere        |
| Lambdas                  | `filterHistory()`, `searchHistory()`, `searchEntries()`         |
| Operator Overloading     | `operator<<` for `Alert` and `VehicleStatistics`                |
| Static Members           | `Sensor::totalSensors_`, `Alert::totalAlertCount_`              |
| Copy/Move Semantics      | Alert has explicit copy/move constructors                       |
| `enum class`             | `AlertSeverity`, `DoorStatus`, `SeatbeltStatus`                 |
| Atomic Operations        | Sequence counters in `CrashSafeRecorder`                        |
| Ring Buffer              | `PerformanceGraphStats`, `CrashSafeRecorder`                    |
| State Machine            | OTA lifecycle, DTC debounce                                     |

---

## Bonus Features Summary

| #  | Feature                       | Source File                   | Complexity | Category    |
|----|-------------------------------|-------------------------------|------------|-------------|
| 01 | Driver Profile Management     | `DriverProfile.cpp`           | Medium     | Diagnostics |
| 02 | JSON/XML Config Loading       | `main.cpp` — Option 3         | High       | Diagnostics |
| 03 | CAN Message Simulation        | `CanBus.cpp`                  | High       | Diagnostics |
| 04 | Adaptive Alert Prioritization | `AdaptiveAlertPrioritizer.cpp`| Medium     | Diagnostics |
| 05 | DTC Simulation (OBD-II)       | `DtcSimulator.cpp`            | High       | Monitoring  |
| 06 | ECU Health Monitor            | `EcuHealthMonitor.cpp`        | Medium     | Monitoring  |
| 07 | Performance Graph Statistics  | `PerformanceGraphStats.cpp`   | Medium     | Monitoring  |
| 08 | OTA Update Simulation         | `OtaUpdateSimulator.cpp`      | High       | Safety/OTA  |
| 09 | Crash-Safe Event Recording    | `CrashSafeRecorder.cpp`       | High       | Safety/OTA  |
| 10 | Service-Oriented Comm (SOME/IP)| `ServiceOrientedComm.cpp`    | High       | Safety/OTA  |

---

## Real Automotive Mapping

| System Component        | Automotive Equivalent                          |
|-------------------------|------------------------------------------------|
| Sensor Framework        | MCAL / Sensor Abstraction Layer                |
| Alert Manager           | Vehicle Health Manager (VHM)                   |
| Dashboard               | Instrument Cluster / HMI                       |
| Event Logger            | Diagnostic Event Manager (DEM)                 |
| DTC Simulator           | OBD-II / UDS Fault Memory                      |
| CAN Bus Simulator       | CAN-HS Physical + Data Link Layer              |
| OTA Update Simulator    | Software Update Manager (SUM) — WP.29 R156     |
| Crash-Safe Recorder     | Event Data Recorder (EDR) / NvM Manager        |
| SOME/IP Comm            | Adaptive AUTOSAR ara::com / SOME/IP SD         |
| Threads                 | Adaptive AUTOSAR Execution Contexts            |
| Mutex + lock_guard      | Inter-process / inter-task synchronisation     |
| Runtime Alerts          | Safety Monitoring / FMEA responses             |

---

## Troubleshooting

| Error                       | Cause                      | Fix                              |
|-----------------------------|----------------------------|----------------------------------|
| `g++: command not found`    | Compiler not installed     | `sudo apt install g++`           |
| `make: command not found`   | Make not installed         | `sudo apt install make`          |
| `undefined reference: pthread` | Thread lib not linked  | Add `-pthread` to `CXXFLAGS`     |
| Log file not created        | Missing `logs/` directory  | `mkdir -p logs`                  |
| JSON test fails to parse    | Malformed `input.json`     | Validate against `expected.json` |