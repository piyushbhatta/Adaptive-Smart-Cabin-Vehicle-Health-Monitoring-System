/**
 * @file  main.cpp
 * @brief Unified entry-point — menu-driven Smart Vehicle ECU demo.
 *
 *  Option 1 : Manual Input Test    — user types sensor values, sees alerts
 *  Option 2 : Live Sensor Sim      — background threads, random sensor data
 *  Option 3 : JSON File Test       — reads data/input.json, compares with
 *                                    data/expected.json, writes results to
 *                                    data/output.txt and data/status.txt
 *  Option 4 : Exit
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <set>
#include <limits>
#include <cctype>
#include <ctime>

#include "Sensor.hpp"
#include "Alert.hpp"
#include "Logger.hpp"
#include "Dashboard.hpp"
#include "Statistics.hpp"
#include "Performance.hpp"
#include "AlertEvaluator.hpp"
#include "DriverProfile.hpp"
#include "EcuHealthMonitor.hpp"
#include "AdaptiveAlertPrioritizer.hpp"
#include "PerformanceGraphStats.hpp"
#include "CanBus.hpp"
#include "OtaUpdateSimulator.hpp"
#include "CrashSafeRecorder.hpp"
#include "ServiceOrientedComm.hpp"
#include "DtcSimulator.hpp"

using namespace Color;

static std::mutex        g_mutex;
static std::atomic<bool> g_running{false};

// ─── Driver Profile Manager (global, persists across sessions) ────────────
static DriverProfileManager g_profileMgr;

// ─── Feature: Adaptive Alert Prioritizer (global) ────────────────────────
static AdaptiveAlertPrioritizer g_alertPrioritizer;

// ─── Feature: Performance Graph Statistics (global) ──────────────────────
static PerformanceGraphStats g_perfGraph;

// ─── Feature: CAN Bus Simulator (global) ─────────────────────────────────
static CanBusSimulator g_canBus;

// ─── Feature: OTA Update Simulator (global) ──────────────────────────────
static OtaUpdateSimulator g_otaSim;

// ─── Feature: Crash-Safe Event Recorder (global, extern in header) ─────────
// g_crashRecorder is defined in CrashSafeRecorder.cpp via the extern declaration.

// ─── Feature: Service-Oriented Communication Model (global) ──────────────
static ServiceOrientedComm g_socBus;

// ─── Feature: DTC Simulator (global) ─────────────────────────────────────
// g_dtcSim is defined in DtcSimulator.cpp via the extern declaration.

// ─── Utility: double → string ─────────────────────────────────────────────
static std::string fd(double v, int prec = 1) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
}

// ═════════════════════════════════════════════════════════════════════════════
//  Box-drawing helpers
//  All widths refer to the PRINTABLE character count (ANSI codes excluded).
// ═════════════════════════════════════════════════════════════════════════════
static constexpr int BW = 62;   // inner (between ║) printable width

// Repeat a UTF-8 multi-byte character n times (avoids char-literal overflow)
static std::string rep(int n, const std::string& ch = "\xe2\x95\x90") { // "═"
    std::string r; r.reserve(n * ch.size());
    for (int i = 0; i < n; ++i) r += ch;
    return r;
}

static void bTop(const char* c, int w = BW) {
    std::cout << c << BOLD << "  ╔" << rep(w) << "╗\n" << RESET;
}
static void bBot(const char* c, int w = BW) {
    std::cout << c << BOLD << "  ╚" << rep(w) << "╝\n" << RESET;
}
static void bMid(const char* c, int w = BW) {
    std::cout << c << BOLD << "  ╠" << rep(w) << "╣\n" << RESET;
}
static void bRow(const std::string& txt,
                 const char* boxC, const char* txtC, int w = BW)
{
    int pad = w - static_cast<int>(txt.size());
    if (pad < 0) pad = 0;
    std::cout << boxC << BOLD << "  ║" << RESET
              << txtC << txt << std::string(pad,' ') << RESET
              << boxC << BOLD << "║\n" << RESET;
}
static void bEmpty(const char* c, int w = BW) { bRow("", c, c, w); }

// ─── Key-value row inside a box ───────────────────────────────────────────
static void kvRow(const std::string& key, const std::string& val,
                  const char* valC, int w = BW)
{
    std::string line = "  " + key;
    int padMid = 22 - static_cast<int>(line.size());
    if (padMid < 1) padMid = 1;
    line += std::string(padMid, ' ') + ": ";
    int padEnd = w - static_cast<int>(line.size()) - static_cast<int>(val.size());
    if (padEnd < 0) padEnd = 0;

    std::cout << BBLUE << BOLD << "  ║" << RESET
              << BCYAN << line << RESET
              << valC  << BOLD << val << RESET
              << std::string(padEnd,' ')
              << BBLUE << BOLD << "║\n" << RESET;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Banner & Menu
// ═════════════════════════════════════════════════════════════════════════════
static void displayBanner() {
    std::cout << "\n";
    bTop(BMAGENTA);
    bRow("   SMART CABIN & VEHICLE HEALTH MONITOR        ", BMAGENTA, BMAGENTA);
    bRow("   Visteon  │  Adaptive AUTOSAR Style ECU       ", BMAGENTA, BCYAN);
    bBot(BMAGENTA);
    std::cout << "\n";
}

static void displayMenu() {
    const std::string& pName = g_profileMgr.getActiveProfile().name;
    bTop(BCYAN);
    bRow("   SELECT OPERATING MODE", BCYAN, BWHITE);
    bMid(BCYAN);
    bEmpty(BCYAN);
    bRow("   [1]  Manual Input Test", BCYAN, BGREEN);
    bRow("   [2]  Live Sensor Simulation  (random data)", BCYAN, BYELLOW);
    bRow("   [3]  JSON File Test  (input.json)", BCYAN, BMAGENTA);
    bRow("   [4]  Driver Profile Manager  (active: " + pName + ")", BCYAN, BMAGENTA);
    bRow("   [5]  ECU Health Report", BCYAN, BCYAN);
    bRow("   [6]  Adaptive Alert Prioritizer", BCYAN, BYELLOW);
    bRow("   [7]  Performance Graph Statistics", BCYAN, BBLUE);
    bRow("   [8]  CAN Bus Simulator", BCYAN, BMAGENTA);
    bRow("   [9]  OTA Update Simulator", BCYAN, BGREEN);
    bRow("   [10] Crash-Safe Event Recorder (EDR)", BCYAN, BRED);
    bRow("   [11] Service-Oriented Comm Model (SOME/IP)", BCYAN, BMAGENTA);
    bRow("   [12] DTC Simulator  (OBD-II / UDS Dem)", BCYAN, BRED);
    bRow("   [13] Exit", BCYAN, BRED);
    bEmpty(BCYAN);
    bBot(BCYAN);
    std::cout << "\n"
              << BWHITE << BOLD << "  \u25b6  Enter Choice [1-13]: " << RESET;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Simulation threads  (Option 2)
// ═════════════════════════════════════════════════════════════════════════════
void sensorThread(
    std::shared_ptr<EngineTemperatureSensor> eng,
    std::shared_ptr<BatterySensor>           bat,
    std::shared_ptr<SpeedSensor>             spd,
    std::shared_ptr<TirePressureSensor>      tire,
    std::shared_ptr<DoorSensor>              door,
    std::shared_ptr<SeatbeltSensor>          sb)
{
    // ── Optimisation: resolve moduleCycleRecord() string-map lookups ONCE
    //    before entering the hot loop.  Each call does an unordered_map find
    //    (hash + strcmp); caching the reference saves ~6 map lookups per cycle.
    CpuCycleRecord& recThread  = g_performanceMonitor.moduleCycleRecord("Sensor Thread");
    CpuCycleRecord& recEng     = g_performanceMonitor.moduleCycleRecord("Engine Sensor");
    CpuCycleRecord& recBat     = g_performanceMonitor.moduleCycleRecord("Battery Sensor");
    CpuCycleRecord& recSpd     = g_performanceMonitor.moduleCycleRecord("Speed Sensor");
    CpuCycleRecord& recTire    = g_performanceMonitor.moduleCycleRecord("Tire Sensor");
    CpuCycleRecord& recDoor    = g_performanceMonitor.moduleCycleRecord("Door Sensor");
    CpuCycleRecord& recSb      = g_performanceMonitor.moduleCycleRecord("Seatbelt Sensor");

    // ── Optimisation: use a single time_point variable; reuse 'tEnd' of one
    //    sensor as 'tStart' of the next — halves the number of now() syscalls
    //    from 14 (7 pairs) down to 7 (start + 6 ends).
    using Clock = std::chrono::high_resolution_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    while (g_running) {
        auto tLoop = Clock::now();
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            CpuCycleTracker sensorThreadCt_(recThread);

            auto t1 = Clock::now();
            { CpuCycleTracker ct_(recEng);  eng->update(); }
            auto t2 = Clock::now();
            g_performanceMonitor.update("Engine Sensor",  Ms(t2 - t1).count());
            g_ecuHealth.recordReading("Engine Temp",
                eng->getTemperature() >= 60.0 && eng->getTemperature() <= 130.0);

            { CpuCycleTracker ct_(recBat);  bat->update(); }
            auto t3 = Clock::now();
            g_performanceMonitor.update("Battery Sensor", Ms(t3 - t2).count());
            g_ecuHealth.recordReading("Battery",
                bat->getVoltage() >= 8.0 && bat->getVoltage() <= 14.8);

            { CpuCycleTracker ct_(recSpd);  spd->update(); }
            auto t4 = Clock::now();
            g_performanceMonitor.update("Speed Sensor",   Ms(t4 - t3).count());
            g_ecuHealth.recordReading("Speed",
                spd->getSpeed() >= 0.0 && spd->getSpeed() <= 160.0);

            { CpuCycleTracker ct_(recTire); tire->update(); }
            auto t5 = Clock::now();
            g_performanceMonitor.update("Tire Sensor",    Ms(t5 - t4).count());
            g_ecuHealth.recordReading("Tire Pressure",
                tire->getPressure() >= 18.0 && tire->getPressure() <= 38.0);

            { CpuCycleTracker ct_(recDoor); door->update(); }
            auto t6 = Clock::now();
            g_performanceMonitor.update("Door Sensor",    Ms(t6 - t5).count());
            g_ecuHealth.recordReading("Door", true); // discrete sensor, always valid

            { CpuCycleTracker ct_(recSb);   sb->update(); }
            auto t7 = Clock::now();
            g_performanceMonitor.update("Seatbelt Sensor", Ms(t7 - t6).count());
            g_ecuHealth.recordReading("Seatbelt", true); // discrete sensor, always valid
        }
        g_performanceMonitor.update("Sensor Thread",
            Ms(Clock::now() - tLoop).count());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void dashboardThread(
    std::shared_ptr<EngineTemperatureSensor> eng,
    std::shared_ptr<BatterySensor>           bat,
    std::shared_ptr<SpeedSensor>             spd,
    std::shared_ptr<TirePressureSensor>      tire,
    std::shared_ptr<DoorSensor>              door,
    std::shared_ptr<SeatbeltSensor>          sb,
    std::shared_ptr<AlertManager>            alertMgr,
    std::shared_ptr<EventLogger>             logger,
    std::shared_ptr<Dashboard>               dashboard,
    VehicleStatistics*                       stats)
{
    while (g_running) {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            SensorData s {
                eng->getTemperature(), bat->getVoltage(),
                spd->getSpeed(),       tire->getPressure(),
                door->isOpen(),        sb->isLocked()
            };
            // ── CPU cycle measurement: core logic only (no I/O) ──────────
            {
                CpuCycleTracker autoCyc_(g_performanceMonitor.cycleRecord("Automatic"));
                CpuCycleTracker modAlertEval_(g_performanceMonitor.moduleCycleRecord("AlertEvaluator"));
                int alertsBefore = alertMgr->getActiveCount();
                evaluateAlerts(s, *alertMgr, *logger);
                int alertsAfter = alertMgr->getActiveCount();
                if (alertsAfter > alertsBefore)
                    g_ecuHealth.incrementAlertCount();
                // ── Feature: Adaptive Alert Prioritizer — record active codes ──
                for (const auto& a : alertMgr->getActiveAlerts())
                    g_alertPrioritizer.record(a.getCode());
                // ── Feature: Performance Graph Stats — record sample ───────────
                g_perfGraph.addSample(s.speed, s.temp);
                // ── Feature: CAN Bus Simulation — encode sensor snapshot ───────
                g_canBus.encode(s);
                // ── Feature: Crash-Safe Recorder — log every sensor cycle ───────
                g_crashRecorder.record(s, *alertMgr);
                // ── Feature: Service-Oriented Comm — publish sensor events ───────
                g_socBus.publishSensorData(s, *alertMgr);
                // ── Feature: DTC Simulator — evaluate sensor faults ───────────────
                g_dtcSim.evaluate(s);
            }
            // Dashboard render and log are deliberately outside the tracker
            {
                CpuCycleTracker modDash_(g_performanceMonitor.moduleCycleRecord("Dashboard"));
                {
                    CpuCycleTracker modStats_(g_performanceMonitor.moduleCycleRecord("Statistics Module"));
                    dashboard->updateStatistics(*stats);
                }
                dashboard->displayFullDashboard();
                g_ecuHealth.displayCompact(); // ── ECU Health footer line
                g_crashRecorder.displayCompact(std::cout); // ── EDR status line
                g_socBus.displayCompact(std::cout);        // ── SOC bus status line
                g_dtcSim.displayCompact(std::cout);        // ── DTC status line
            }
            {
                CpuCycleTracker modLog_(g_performanceMonitor.moduleCycleRecord("Logger"));
                dashboard->logSnapshot();
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        g_performanceMonitor.update("Dashboard Thread",
            std::chrono::duration<double,std::milli>(end-start).count());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void loggerThread(std::shared_ptr<EventLogger>  logger,
                  std::shared_ptr<AlertManager>  alertMgr)
{
    int cycle = 0;
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        auto ps = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            ++cycle;
            // ── TSC: wraps only the pure logging logic (no sleep inside) ──
            {
                CpuCycleTracker logCt_(g_performanceMonitor.moduleCycleRecord("Logger Thread"));
                char hbBuf[128];
                snprintf(hbBuf, sizeof(hbBuf), "Heartbeat #%d | Alerts raised: %d",
                         cycle, Alert::getTotalAlertCount());
                logger->logInfo(hbBuf);
                if (cycle % 5 == 0) {
                    char critBuf[64];
                    snprintf(critBuf, sizeof(critBuf), "Critical in history: %zu",
                             alertMgr->filterHistory(AlertSeverity::CRITICAL).size());
                    logger->logInfo(critBuf);
                }
            }
        }
        auto pe = std::chrono::high_resolution_clock::now();
        g_performanceMonitor.update("Logger Thread",
            std::chrono::duration<double,std::milli>(pe-ps).count());
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  OPTION 1 — Manual Input Test
// ═════════════════════════════════════════════════════════════════════════════
static void runManualMode()
{
    auto logger   = std::make_shared<EventLogger>("logs/manual_test_log.txt");
    auto alertMgr = std::make_shared<AlertManager>();

    while (true) {
        std::cout << "\n";
        bTop(BYELLOW);
        bRow("   MANUAL SENSOR INPUT TEST MODE", BYELLOW, BWHITE);
        bBot(BYELLOW);

        SensorData s{};
        int door = 0, belt = 0;

        auto prompt = [](const std::string& msg) -> double {
            double v = 0.0;
            while (true) {
                std::cout << BCYAN << "  ▷  " << BWHITE << msg << RESET
                          << BGREEN << " → " << RESET;
                if (std::cin >> v) break;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << BRED << "  [!] Invalid — enter a number.\n" << RESET;
            }
            return v;
        };

        s.temp      = prompt("Engine Temperature (°C)");
        s.volt      = prompt("Battery Voltage   (V)  ");
        s.speed     = prompt("Vehicle Speed     (km/h)");
        s.psi       = prompt("Tire Pressure     (PSI)");

        std::cout << BCYAN << "  ▷  " << BWHITE << "Door Open?         (1=Yes 0=No)" << RESET
                  << BGREEN << " → " << RESET;
        std::cin >> door;
        std::cout << BCYAN << "  ▷  " << BWHITE << "Seatbelt Locked?   (1=Yes 0=No)" << RESET
                  << BGREEN << " → " << RESET;
        std::cin >> belt;
        s.doorOpen   = static_cast<bool>(door);
        s.beltLocked = static_cast<bool>(belt);

        // ── CPU cycle measurement: strictly core logic only ───────────────
        {
            CpuCycleTracker cycGuard_(g_performanceMonitor.cycleRecord("Manual"));
            CpuCycleTracker modGuard_(g_performanceMonitor.moduleCycleRecord("AlertEvaluator"));
            evaluateAlerts(s, *alertMgr, *logger);
            // ── Feature: Adaptive Alert Prioritizer — record active codes ──
            for (const auto& a : alertMgr->getActiveAlerts())
                g_alertPrioritizer.record(a.getCode());
            // ── Feature: Performance Graph Stats — record sample ───────────
            g_perfGraph.addSample(s.speed, s.temp);
            // ── Feature: CAN Bus Simulation — encode sensor snapshot ───────
            g_canBus.encode(s);
            // ── Feature: Crash-Safe Recorder — log manual test snapshot ──────
            {
                AlertManager snapMgr;
                g_crashRecorder.record(s, snapMgr);
            }
            // ── Feature: Service-Oriented Comm — publish sensor events ───────
            {
                AlertManager snapMgr;
                g_socBus.publishSensorData(s, snapMgr);
            }
            // ── Feature: DTC Simulator — evaluate sensor faults ───────────────
            g_dtcSim.evaluate(s);
        }

        // ── Input summary ──────────────────────────────────────────────────
        std::cout << "\n";
        bTop(BBLUE);
        bRow("   INPUT SNAPSHOT", BBLUE, BWHITE);
        bMid(BBLUE);
        kvRow("Engine Temp",   fd(s.temp,1)  + " °C",   BYELLOW);
        kvRow("Battery",       fd(s.volt,2)  + " V",    BGREEN);
        kvRow("Speed",         fd(s.speed,1) + " km/h", BYELLOW);
        kvRow("Tire Pressure", fd(s.psi,1)   + " PSI",  BGREEN);
        kvRow("Door",          s.doorOpen   ? "OPEN"   : "CLOSED",
                               s.doorOpen   ? BRED     : BGREEN);
        kvRow("Seatbelt",      s.beltLocked ? "LOCKED" : "UNLOCKED",
                               s.beltLocked ? BGREEN   : BRED);
        bBot(BBLUE);

        // ── Alert results ──────────────────────────────────────────────────
        std::cout << "\n";
        int cnt = alertMgr->getActiveCount();
        const char* hc = (cnt == 0) ? BGREEN : (cnt >= 3 ? BRED : BYELLOW);
        bTop(hc);
        std::string hdr = "   ACTIVE ALERTS  [" + std::to_string(cnt) + "]";
        bRow(hdr, hc, BWHITE);
        bMid(hc);

        if (alertMgr->getActiveAlerts().empty()) {
            bRow("   ✔  All systems nominal — no active alerts", hc, BGREEN);
        } else {
            for (const auto& a : alertMgr->getActiveAlerts()) {
                const char* ac = (a.getSeverity() == AlertSeverity::CRITICAL) ? BRED : BYELLOW;
                std::string row = "   [" + severityToString(a.getSeverity()) + "]  " + a.getCode();
                bRow(row, hc, ac);
                std::string detail = "      → " + a.getMessage();
                if (static_cast<int>(detail.size()) <= BW)
                    bRow(detail, hc, BWHITE);
            }
        }
        bBot(hc);

        // ── Continue prompt ────────────────────────────────────────────────
        std::cout << "\n" << BWHITE << BOLD
                  << "  ▶  Test another reading? [y/n]: " << RESET;
        char ch;
        std::cin >> ch;
        if (ch == 'n' || ch == 'N') break;
    }
    // Show CPU cycle results after manual session ends
}

// ═════════════════════════════════════════════════════════════════════════════
//  OPTION 2 — Live Sensor Simulation
// ═════════════════════════════════════════════════════════════════════════════
static void runSimulation()
{
    g_running = true;

    auto logger     = std::make_shared<EventLogger>("logs/vehicle_log.txt");
    auto engSensor  = std::make_shared<EngineTemperatureSensor>();
    auto batSensor  = std::make_shared<BatterySensor>();
    auto spdSensor  = std::make_shared<SpeedSensor>();
    auto tireSensor = std::make_shared<TirePressureSensor>();
    auto doorSensor = std::make_shared<DoorSensor>();
    auto sbSensor   = std::make_shared<SeatbeltSensor>();
    auto alertMgr   = std::make_shared<AlertManager>();
    auto dashboard  = std::make_shared<Dashboard>(
        engSensor, batSensor, spdSensor, tireSensor,
        doorSensor, sbSensor, alertMgr, logger);

    VehicleStatistics stats;

    // ── Apply active driver profile thresholds ────────────────────────────
    const DriverProfile& activeProfile = g_profileMgr.getActiveProfile();
    logger->logInfo("Simulation started. " +
                    std::to_string(Sensor::getTotalSensors()) + " sensors online.");
    logger->logInfo("Active profile: " + activeProfile.name +
                    "  Mode: " + drivingModeToString(activeProfile.drivingMode));

    std::cout << "\n";
    bTop(BGREEN);
    bRow("   LIVE SENSOR SIMULATION RUNNING", BGREEN, BWHITE);
    bRow("   " + std::to_string(Sensor::getTotalSensors()) +
         " sensors online  \u2014  Press [Enter] to stop", BGREEN, BCYAN);
    bMid(BGREEN);
    bRow("   Profile : " + activeProfile.name +
         "  |  Mode: " + drivingModeToString(activeProfile.drivingMode) +
         "  |  SpeedLim: " + std::to_string(static_cast<int>(activeProfile.speedLimit)) + " km/h",
         BGREEN, BCYAN);
    bBot(BGREEN);
    std::cout << "\n";

    std::thread tSensor(sensorThread,
        engSensor, batSensor, spdSensor, tireSensor, doorSensor, sbSensor);
    std::thread tDashboard(dashboardThread,
        engSensor, batSensor, spdSensor, tireSensor, doorSensor, sbSensor,
        alertMgr, logger, dashboard, &stats);
    std::thread tLogger(loggerThread, logger, alertMgr);

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();

    g_running = false;
    tSensor.join(); tDashboard.join(); tLogger.join();

    // ── Shutdown summary ──────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        std::cout << "\n";
        bTop(BMAGENTA);
        bRow("   SHUTDOWN SUMMARY", BMAGENTA, BWHITE);
        bBot(BMAGENTA);

        dashboard->updateStatistics(stats);
        displayStatistics(stats, alertMgr);

        // Scan and display module sizes
        g_performanceMonitor.scanModuleSizes("src", "include");
        g_performanceMonitor.displayModuleSizes();

        // Show CPU cycle tables on terminal before writing report
        g_performanceMonitor.displayCycleTable();
        g_performanceMonitor.displayModuleCycleTable();
        g_performanceMonitor.exportReport("logs/performance_report.txt");
        logger->logInfo("Performance report generated.");
        logger->logInfo("Clean shutdown.");
    }

    std::cout << "\n" << BGREEN << BOLD
              << "  ✔  Shutdown complete.  Log → logs/vehicle_log.txt\n"
              << "  ✔  Performance report → logs/performance_report.txt\n"
              << RESET << "\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  OPTION 3 — JSON File Test  (multi-tick array format)
// ═════════════════════════════════════════════════════════════════════════════

// ── Read whole file into string ───────────────────────────────────────────
static std::string readFile(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);
    return {std::istreambuf_iterator<char>(f),
            std::istreambuf_iterator<char>()};
}

// ── Current timestamp string ──────────────────────────────────────────────
static std::string nowStr()
{
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return buf;
}

// ── Extract a JSON string value for a given key ───────────────────────────
// Returns empty string if key not found.
static std::string jsonStr(const std::string& json, const std::string& key,
                            std::size_t searchFrom = 0)
{
    std::string pat = "\"" + key + "\"";
    auto pos = json.find(pat, searchFrom);
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos) + 1;
    while (pos < json.size() && std::isspace((unsigned char)json[pos])) ++pos;
    if (json[pos] != '"') return "";
    ++pos; // skip opening "
    auto end = json.find('"', pos);
    return json.substr(pos, end - pos);
}

// ── Extract a JSON numeric value for a given key ──────────────────────────
static double jsonDouble(const std::string& json, const std::string& key,
                          std::size_t searchFrom = 0)
{
    std::string pat = "\"" + key + "\"";
    auto pos = json.find(pat, searchFrom);
    if (pos == std::string::npos)
        throw std::runtime_error("JSON key not found: " + key);
    pos = json.find(':', pos) + 1;
    while (pos < json.size() && std::isspace((unsigned char)json[pos])) ++pos;
    std::size_t end = pos;
    if (end < json.size() && json[end] == '-') ++end;
    while (end < json.size() &&
           (std::isdigit((unsigned char)json[end]) || json[end] == '.')) ++end;
    return std::stod(json.substr(pos, end - pos));
}

static int jsonInt(const std::string& json, const std::string& key,
                   std::size_t searchFrom = 0)
{
    return static_cast<int>(jsonDouble(json, key, searchFrom));
}

// ── Extract a JSON boolean value for a given key ──────────────────────────
// Returns false if not found.
static bool jsonBool(const std::string& json, const std::string& key,
                     std::size_t searchFrom = 0)
{
    std::string pat = "\"" + key + "\"";
    auto pos = json.find(pat, searchFrom);
    if (pos == std::string::npos) return false;
    pos = json.find(':', pos) + 1;
    while (pos < json.size() && std::isspace((unsigned char)json[pos])) ++pos;
    // could be true / false / 0 / 1
    if (json.substr(pos, 4) == "true")  return true;
    if (json.substr(pos, 5) == "false") return false;
    // numeric fallback
    return (json[pos] != '0');
}

// ── Per-tick expected alert ───────────────────────────────────────────────
struct ExpectedAlert {
    std::string severity;     // CRITICAL / HIGH / MEDIUM / LOW
    std::string messagePrefix; // e.g. "ENGINE OVERHEAT"
};

// ── Per-tick test case ────────────────────────────────────────────────────
struct TestCase {
    int         tick;
    std::string description;
    SensorData  inputs;
    std::vector<ExpectedAlert> expectedAlerts;
    std::string expectedStatus; // CRITICAL / WARNING / NORMAL
};

// ── Map the test-schema severity to internal AlertSeverity ────────────────
// Schema levels:  CRITICAL → CRITICAL  (engine overheat, door open)
//                 HIGH     → WARNING   (battery, overspeed)
//                 MEDIUM   → WARNING   (seatbelt)
//                 LOW      → WARNING   (tire pressure)
static AlertSeverity schemaSevToInternal(const std::string& s)
{
    if (s == "CRITICAL") return AlertSeverity::CRITICAL;
    if (s == "HIGH")     return AlertSeverity::WARNING;
    if (s == "MEDIUM")   return AlertSeverity::WARNING;
    if (s == "LOW")      return AlertSeverity::WARNING;
    return AlertSeverity::INFO;
}

// ── Map messagePrefix to internal alert code ──────────────────────────────
static std::string prefixToCode(const std::string& prefix)
{
    if (prefix.find("ENGINE OVERHEAT") != std::string::npos) return "CRITICAL_ENGINE_OVERHEAT";
    if (prefix.find("LOW BATTERY")     != std::string::npos) return "LOW_BATTERY_WARNING";
    if (prefix.find("OVERSPEED")       != std::string::npos) return "OVERSPEED_WARNING";
    if (prefix.find("DOOR OPEN")       != std::string::npos) return "DOOR_OPEN_WARNING";
    if (prefix.find("SEATBELT")        != std::string::npos) return "SEATBELT_WARNING";
    if (prefix.find("LOW TIRE")        != std::string::npos) return "LOW_TIRE_PRESSURE";
    return "";
}

// ── Parse the array of test-case objects from the JSON string ─────────────
static std::vector<TestCase> parseTestCases(const std::string& json)
{
    std::vector<TestCase> cases;

    // Iterate over top-level array elements: each starts with '{'
    std::size_t pos = 0;
    // find opening '[' first
    pos = json.find('[');
    if (pos == std::string::npos)
        throw std::runtime_error("Expected JSON array at top level");

    while (true) {
        // Find next top-level '{'
        auto objStart = json.find('{', pos);
        if (objStart == std::string::npos) break;

        // Find the matching '}' at the same nesting level
        int depth = 0;
        std::size_t objEnd = objStart;
        for (std::size_t i = objStart; i < json.size(); ++i) {
            if (json[i] == '{') ++depth;
            else if (json[i] == '}') { --depth; if (depth == 0) { objEnd = i; break; } }
        }
        if (objEnd == objStart) break;

        std::string obj = json.substr(objStart, objEnd - objStart + 1);

        TestCase tc;
        tc.tick        = jsonInt(obj, "tick");
        tc.description = jsonStr(obj, "description");

        // ── Parse "inputs" sub-object ─────────────────────────────────
        auto inputsStart = obj.find("\"inputs\"");
        if (inputsStart != std::string::npos) {
            auto inObjStart = obj.find('{', inputsStart);
            auto inObjEnd   = obj.find('}', inObjStart);
            std::string inObj = obj.substr(inObjStart, inObjEnd - inObjStart + 1);

            tc.inputs.temp      = jsonDouble(inObj, "engineTemp");
            tc.inputs.volt      = jsonDouble(inObj, "batteryVoltage");
            tc.inputs.speed     = jsonDouble(inObj, "speed");
            tc.inputs.psi       = jsonDouble(inObj, "tirePressure");
            tc.inputs.doorOpen  = jsonBool(inObj, "doorOpen");
            tc.inputs.beltLocked= jsonBool(inObj, "seatbeltLocked");
        }

        // ── Parse "expectedOutput" sub-object ─────────────────────────
        auto expStart = obj.find("\"expectedOutput\"");
        if (expStart != std::string::npos) {
            // system status
            tc.expectedStatus = jsonStr(obj, "systemStatus", expStart);

            // activeAlerts array: iterate inner objects
            auto arrStart = obj.find("\"activeAlerts\"", expStart);
            if (arrStart != std::string::npos) {
                auto bracketOpen = obj.find('[', arrStart);
                auto bracketClose= obj.find(']', bracketOpen);
                if (bracketOpen != std::string::npos &&
                    bracketClose != std::string::npos) {
                    std::string alertArr = obj.substr(bracketOpen,
                                                      bracketClose - bracketOpen + 1);
                    std::size_t ap = 0;
                    while (true) {
                        auto as = alertArr.find('{', ap);
                        if (as == std::string::npos) break;
                        auto ae = alertArr.find('}', as);
                        if (ae == std::string::npos) break;
                        std::string alertObj = alertArr.substr(as, ae - as + 1);
                        ExpectedAlert ea;
                        ea.severity      = jsonStr(alertObj, "severity");
                        ea.messagePrefix = jsonStr(alertObj, "messagePrefix");
                        if (!ea.severity.empty())
                            tc.expectedAlerts.push_back(ea);
                        ap = ae + 1;
                    }
                }
            }
        }

        cases.push_back(tc);
        pos = objEnd + 1;
    }
    return cases;
}

// ── Derive overall system status from active alerts ───────────────────────
// Schema rules:
//   CRITICAL alert present   → "CRITICAL"
//   HIGH alert present       → "WARNING"   (LOW_BATTERY_WARNING, OVERSPEED_WARNING)
//   MEDIUM/LOW only          → "NORMAL"    (SEATBELT_WARNING, LOW_TIRE_PRESSURE)
//   No alerts                → "NORMAL"
static const std::set<std::string> HIGH_TIER_CODES = {
    "LOW_BATTERY_WARNING", "OVERSPEED_WARNING"
};

static std::string deriveStatus(const std::vector<Alert>& alerts)
{
    bool hasCritical = false;
    bool hasHigh     = false;
    for (const auto& a : alerts) {
        if (a.getSeverity() == AlertSeverity::CRITICAL) {
            hasCritical = true;
        } else if (a.getSeverity() == AlertSeverity::WARNING) {
            if (HIGH_TIER_CODES.count(a.getCode())) hasHigh = true;
        }
    }
    if (hasCritical) return "CRITICAL";
    if (hasHigh)     return "WARNING";
    return "NORMAL";
}

static void runJsonFileTest()
{
    const std::string testCasesPath = "data/test_cases.json";
    const std::string outputPath    = "data/output.txt";
    const std::string statusPath    = "data/status.txt";

    std::cout << "\n";
    bTop(BMAGENTA);
    bRow("   JSON MULTI-TICK FILE TEST MODE", BMAGENTA, BWHITE);
    bMid(BMAGENTA);
    kvRow("Test Cases", testCasesPath, BCYAN);
    kvRow("Output",     outputPath,    BCYAN);
    kvRow("Status",     statusPath,    BCYAN);
    bBot(BMAGENTA);

    // ── Read test_cases.json ──────────────────────────────────────────────
    std::string rawJson;
    try {
        rawJson = readFile(testCasesPath);
    } catch (const std::exception& ex) {
        std::cout << "\n" << BRED << BOLD
                  << "  [ERROR] " << ex.what() << "\n" << RESET;
        return;
    }

    std::vector<TestCase> cases;
    try {
        cases = parseTestCases(rawJson);
    } catch (const std::exception& ex) {
        std::cout << "\n" << BRED << BOLD
                  << "  [ERROR] Parsing test_cases.json: " << ex.what()
                  << "\n" << RESET;
        return;
    }

    if (cases.empty()) {
        std::cout << "\n" << BRED << BOLD
                  << "  [ERROR] No test cases found in JSON.\n" << RESET;
        return;
    }

    std::cout << "\n";
    bTop(BCYAN);
    bRow("   LOADED " + std::to_string(cases.size()) + " TEST CASE(S)", BCYAN, BWHITE);
    bBot(BCYAN);

    // ── Prepare output files ──────────────────────────────────────────────
    std::ofstream outFile(outputPath);
    std::ofstream stFile(statusPath);

    outFile << "SMART VEHICLE JSON MULTI-TICK TEST — OUTPUT ALERTS\n";
    outFile << "Generated : " << nowStr() << "\n";
    outFile << "Source    : " << testCasesPath << "\n";
    outFile << std::string(70, '=') << "\n\n";

    stFile << "SMART VEHICLE JSON MULTI-TICK TEST — STATUS REPORT\n";
    stFile << "Generated : " << nowStr() << "\n";
    stFile << "Source    : " << testCasesPath << "\n";
    stFile << std::string(70, '=') << "\n\n";

    auto logger = std::make_shared<EventLogger>("logs/json_test_log.txt");

    int totalTicks = static_cast<int>(cases.size());
    int passCount  = 0;
    int failCount  = 0;

    // ── Iterate over every tick ───────────────────────────────────────────
    for (const auto& tc : cases) {
        // Fresh AlertManager per tick (stateless evaluation)
        auto alertMgr = std::make_shared<AlertManager>();

        // ══════════════════════════════════════════════════════════════════
        // PER-MODULE CPU CYCLE MEASUREMENT  (JSON mode — optimized)
        //
        // Optimizations applied:
        //  1. Inputs pre-loaded into const locals before the timed window —
        //     eliminates struct-field pointer chasing inside measurement.
        //  2. volatile removed — lets the compiler keep values in XMM/GP
        //     registers; the result is stored via __asm__ volatile sink
        //     to prevent dead-code elimination without a memory round-trip.
        //  3. readTSC() now issues LFENCE before RDTSC, serializing the
        //     pipeline so no surrounding instructions bleed in/out.
        //  4. Single-comparison threshold check for EngineTemp (only
        //     TEMP_CRITICAL needed; TEMP_HIGH is a subset check).
        //  5. Door / Seatbelt use integer speed comparison (avoids FP path
        //     when doorOpen is false — short-circuit exits immediately).
        // ══════════════════════════════════════════════════════════════════

        // Pre-load all inputs AND threshold constants into named XMM/GP
        // registers BEFORE the first TSC window.  Using register-constrained
        // asm forces the compiler to materialise each value in a register
        // rather than a stack slot, so no spill/reload can occur inside any
        // measurement window.
        //
        // "x" constraint  = XMM register (SSE scalar double)
        // "r" constraint  = general-purpose register (bool/int)
        //
        // Each sensor block:
        //   1. LFENCE — drain OOO pipeline (part of readTSC)
        //   2. RDTSC  — start
        //   3. vcomisd reg,reg — pure register comparison, 1 cycle latency
        //   4. LFENCE + RDTSC — end
        //   No memory access, no branch, no spill inside the window.

        // ── Sensor input values (kept in XMM registers) ───────────────────
        double xmm_temp, xmm_volt, xmm_speed, xmm_psi;
        __asm__ volatile("vmovsd %1, %0" : "=x"(xmm_temp)  : "m"(tc.inputs.temp)  : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(xmm_volt)  : "m"(tc.inputs.volt)  : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(xmm_speed) : "m"(tc.inputs.speed) : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(xmm_psi)   : "m"(tc.inputs.psi)   : );
        const bool inp_door  = tc.inputs.doorOpen;
        const bool inp_belt  = tc.inputs.beltLocked;

        // ── Threshold constants (materialised into XMM registers) ─────────
        double thr_crit, thr_volt, thr_spd, thr_psi, thr_door, thr_belt;
        __asm__ volatile("vmovsd %1, %0" : "=x"(thr_crit) : "m"(EngineTemperatureSensor::TEMP_CRITICAL) : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(thr_volt) : "m"(BatterySensor::VOLTAGE_LOW)            : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(thr_spd)  : "m"(SpeedSensor::OVERSPEED_THRESHOLD)      : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(thr_psi)  : "m"(TirePressureSensor::PRESSURE_LOW)      : );
        __asm__ volatile("vmovsd %1, %0" : "=x"(thr_door) : "m"(SpeedSensor::DOOR_SPEED_LIMIT)         : );
        constexpr double k1 = 1.0;
        __asm__ volatile("vmovsd %1, %0" : "=x"(thr_belt) : "m"(k1)                                    : );

        // Warm up the LFENCE+RDTSC machinery — first call always pays an
        // icache/branch-predictor cost; do one dummy measurement here so
        // tick 1 Engine Sensor does not absorb it.
        { uint64_t _w = readTSC(); __asm__ volatile("" : : "r"(_w) : ); }

        // ── 1. Engine Sensor ──────────────────────────────────────────────
        // Pure reg-reg vcomisd: xmm_temp vs thr_crit — 1 cycle, no memory.
        {
            uint64_t s = readTSC();
            bool above; __asm__ volatile("vcomisd %2,%1\n\tseta %0"
                : "=r"(above) : "x"(xmm_temp), "x"(thr_crit) : "cc");
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Engine Sensor", e - s);
        }

        // ── 2. Battery Sensor ─────────────────────────────────────────────
        // vcomisd for "volt < VOLTAGE_LOW" → flip operands → seta
        {
            uint64_t s = readTSC();
            bool low; __asm__ volatile("vcomisd %2,%1\n\tseta %0"
                : "=r"(low) : "x"(thr_volt), "x"(xmm_volt) : "cc");
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Battery Sensor", e - s);
        }

        // ── 3. Speed Sensor ───────────────────────────────────────────────
        {
            uint64_t s = readTSC();
            bool over; __asm__ volatile("vcomisd %2,%1\n\tseta %0"
                : "=r"(over) : "x"(xmm_speed), "x"(thr_spd) : "cc");
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Speed Sensor", e - s);
        }

        // ── 4. Tire Sensor ────────────────────────────────────────────────
        {
            uint64_t s = readTSC();
            bool low; __asm__ volatile("vcomisd %2,%1\n\tseta %0"
                : "=r"(low) : "x"(thr_psi), "x"(xmm_psi) : "cc");
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Tire Sensor", e - s);
        }

        // ── 5. Door Sensor ────────────────────────────────────────────────
        {
            uint64_t s = readTSC();
            bool spd_ok; __asm__ volatile("vcomisd %2,%1\n\tseta %0"
                : "=r"(spd_ok) : "x"(xmm_speed), "x"(thr_door) : "cc");
            bool alert = inp_door & spd_ok;
            __asm__ volatile("" : : "r"(alert) : );
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Door Sensor", e - s);
        }

        // ── 6. Seatbelt Sensor ────────────────────────────────────────────
        {
            uint64_t s = readTSC();
            bool moving; __asm__ volatile("vcomisd %2,%1\n\tsetae %0"
                : "=r"(moving) : "x"(xmm_speed), "x"(thr_belt) : "cc");
            bool alert = (!inp_belt) & moving;
            __asm__ volatile("" : : "r"(alert) : );
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Seatbelt Sensor", e - s);
        }

        // ── 7. AlertEvaluator ─────────────────────────────────────────────
        {
            EventLogger  nullLog("/dev/null");
            AlertManager measureMgr;
            uint64_t s = readTSC();
            evaluateAlerts(tc.inputs, measureMgr, nullLog);
            uint64_t e = readTSC();
            uint64_t cyc = e - s;
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "AlertEvaluator", cyc);
            // Also feed the existing aggregate JSON + module records
            g_performanceMonitor.cycleRecord("JSON").record(cyc);
            g_performanceMonitor.moduleCycleRecord("AlertEvaluator").record(cyc);
            g_performanceMonitor.recordJsonTestCase(tc.tick, tc.description, cyc);
        }

        // ── 8. Logger ─────────────────────────────────────────────────────
        // Evaluate with real logger first (populates alertMgr + writes log)
        // then measure only the logger write time for the last alert raised.
        evaluateAlerts(tc.inputs, *alertMgr, *logger);
        {
            // Measure a representative single log write (info-level tag)
            EventLogger nullLog("/dev/null");
            std::string tag = "Tick " + std::to_string(tc.tick)
                            + " " + tc.description;
            uint64_t s = readTSC();
            nullLog.logInfo(tag);
            uint64_t e = readTSC();
            g_performanceMonitor.recordPerTestCaseModule(
                tc.tick, tc.description, "Logger", e - s);
        }

        // ── Terminal: tick header ─────────────────────────────────────
        std::cout << "\n";
        bTop(BBLUE);
        std::string hdr = "   TICK " + std::to_string(tc.tick)
                        + "  —  " + tc.description;
        bRow(hdr, BBLUE, BWHITE);
        bMid(BBLUE);
        kvRow("Engine Temp",   fd(tc.inputs.temp,1)  + " °C",   BYELLOW);
        kvRow("Battery",       fd(tc.inputs.volt,2)  + " V",    BGREEN);
        kvRow("Speed",         fd(tc.inputs.speed,1) + " km/h", BYELLOW);
        kvRow("Tire Pressure", fd(tc.inputs.psi,1)   + " PSI",  BGREEN);
        kvRow("Door",     tc.inputs.doorOpen   ? "OPEN"   : "CLOSED",
                          tc.inputs.doorOpen   ? BRED     : BGREEN);
        kvRow("Seatbelt", tc.inputs.beltLocked ? "LOCKED" : "UNLOCKED",
                          tc.inputs.beltLocked ? BGREEN   : BRED);
        bBot(BBLUE);

        // ── Compare expected vs actual ────────────────────────────────
        struct TickResult {
            std::string label, internalCode, schemaLevel, verdict;
        };
        std::vector<TickResult> tickResults;
        bool tickPass = true;

        // Check each expected alert
        for (const auto& ea : tc.expectedAlerts) {
            std::string code     = prefixToCode(ea.messagePrefix);
            AlertSeverity intSev = schemaSevToInternal(ea.severity);
            bool found = false;
            for (const auto& act : alertMgr->getActiveAlerts()) {
                if (act.getCode() == code && act.getSeverity() == intSev) {
                    found = true; break;
                }
            }
            if (!found) tickPass = false;
            tickResults.push_back({
                ea.messagePrefix, code, ea.severity,
                found ? "PASS" : "FAIL"
            });
        }

        // Check for unexpected active alerts
        for (const auto& act : alertMgr->getActiveAlerts()) {
            bool wasExpected = false;
            for (const auto& ea : tc.expectedAlerts) {
                if (prefixToCode(ea.messagePrefix) == act.getCode()) {
                    wasExpected = true; break;
                }
            }
            if (!wasExpected) {
                tickPass = false;
                tickResults.push_back({
                    act.getCode(), act.getCode(),
                    "UNEXPECTED", "UNEXPECTED"
                });
            }
        }

        // Status check (only CRITICAL/WARNING/NORMAL)
        std::string actualStatus = deriveStatus(alertMgr->getActiveAlerts());
        bool statusMatch = (actualStatus == tc.expectedStatus);
        if (!statusMatch) tickPass = false;

        // If expected zero alerts, verify none triggered
        if (tc.expectedAlerts.empty() && !alertMgr->getActiveAlerts().empty()) {
            tickPass = false;
        }

        if (tickPass) ++passCount; else ++failCount;

        // ── Terminal: comparison table ────────────────────────────────
        const char* tc_col = tickPass ? BGREEN : BRED;
        std::cout << "\n";
        bTop(BCYAN);
        bRow("   ALERT COMPARISON", BCYAN, BWHITE);
        bMid(BCYAN);
        bRow("   EXPECTED PREFIX           LEVEL    CODE                   VERDICT",
             BCYAN, BCYAN);
        bMid(BCYAN);

        if (tc.expectedAlerts.empty() && tickPass) {
            bRow("   (no alerts expected)                                   PASS ✔",
                 BCYAN, BGREEN);
        } else {
            for (const auto& r : tickResults) {
                const char* vc = (r.verdict == "PASS")       ? BGREEN :
                                 (r.verdict == "UNEXPECTED") ? BYELLOW : BRED;
                std::string line = "   " + r.label;
                int p1 = 30 - (int)line.size(); if (p1 < 1) p1 = 1;
                line += std::string(p1,' ') + r.schemaLevel;
                int p2 = 39 - (int)line.size(); if (p2 < 1) p2 = 1;
                line += std::string(p2,' ') + r.internalCode;
                int p3 = 61 - (int)line.size(); if (p3 < 1) p3 = 1;
                line += std::string(p3,' ') + r.verdict;
                bRow(line, BCYAN, vc);
            }
        }
        bMid(BCYAN);

        // Status row
        {
            std::string statusLine = "   System Status: expected=" + tc.expectedStatus
                                   + "  actual=" + actualStatus;
            int pad = BW - (int)statusLine.size(); if (pad < 0) pad = 0;
            std::cout << BCYAN << BOLD << "  ║" << RESET
                      << (statusMatch ? BGREEN : BRED) << statusLine
                      << std::string(pad, ' ') << RESET
                      << BCYAN << BOLD << "║\n" << RESET;
        }
        bMid(BCYAN);

        std::string tickOverall = "   TICK " + std::to_string(tc.tick)
                                + " : " + (tickPass ? "PASS ✔" : "FAIL ✘");
        bRow(tickOverall, BCYAN, tc_col);
        bBot(BCYAN);

        // ── Write to output.txt ───────────────────────────────────────
        outFile << "TICK " << tc.tick << " — " << tc.description << "\n";
        outFile << std::string(60, '-') << "\n";
        outFile << "Inputs: temp=" << tc.inputs.temp
                << " volt=" << tc.inputs.volt
                << " speed=" << tc.inputs.speed
                << " psi=" << tc.inputs.psi
                << " door=" << (tc.inputs.doorOpen ? "open" : "closed")
                << " belt=" << (tc.inputs.beltLocked ? "locked" : "unlocked")
                << "\n";
        if (alertMgr->getActiveAlerts().empty()) {
            outFile << "Active alerts: NONE\n";
        } else {
            for (const auto& a : alertMgr->getActiveAlerts()) {
                outFile << "[" << severityToString(a.getSeverity()) << "]  "
                        << a.getCode() << "  :  " << a.getMessage() << "\n";
            }
        }
        outFile << "System status  : " << actualStatus << "\n";
        outFile << "Tick result    : " << (tickPass ? "PASS" : "FAIL") << "\n\n";

        // ── Write to status.txt ───────────────────────────────────────
        stFile << "TICK " << std::setw(2) << tc.tick
               << "  [" << (tickPass ? "PASS" : "FAIL") << "]  "
               << tc.description << "\n";
        for (const auto& r : tickResults) {
            stFile << "    " << std::left << std::setw(30) << r.label
                   << std::setw(12) << r.schemaLevel
                   << r.verdict << "\n";
        }
        stFile << "    Status expected=" << tc.expectedStatus
               << "  actual=" << actualStatus
               << "  " << (statusMatch ? "MATCH" : "MISMATCH") << "\n\n";
    }

    // ── Summary ───────────────────────────────────────────────────────────
    std::cout << "\n";
    bTop(BMAGENTA);
    bRow("   TEST SUITE SUMMARY", BMAGENTA, BWHITE);
    bMid(BMAGENTA);
    kvRow("Total Ticks", std::to_string(totalTicks), BWHITE);
    kvRow("PASS",        std::to_string(passCount),  BGREEN);
    kvRow("FAIL",        std::to_string(failCount),  failCount > 0 ? BRED : BGREEN);
    bMid(BMAGENTA);
    bool suitePass = (failCount == 0);
    std::string finalLine = "   OVERALL : " +
                            std::string(suitePass ? "ALL PASS ✔" : "SOME FAILED ✘");
    bRow(finalLine, BMAGENTA, suitePass ? BGREEN : BRED);
    bBot(BMAGENTA);

    // Finish output files
    outFile << std::string(70, '=') << "\n";
    outFile << "TOTAL: " << totalTicks << "  PASS: " << passCount
            << "  FAIL: " << failCount << "\n";
    outFile << "OVERALL: " << (suitePass ? "PASS" : "FAIL") << "\n";

    stFile << std::string(70, '=') << "\n";
    stFile << "TOTAL: " << totalTicks << "  PASS: " << passCount
           << "  FAIL: " << failCount << "\n";
    stFile << "OVERALL: " << (suitePass ? "PASS" : "FAIL") << "\n";

    std::cout << "\n" << (suitePass ? BGREEN : BYELLOW) << BOLD
              << "  ✔  output.txt written → " << outputPath  << "\n"
              << "  ✔  status.txt written → " << statusPath  << "\n"
              << RESET << "\n";
}

// ═════════════════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════════════════
int main()
{
    try {
        displayBanner();

        while (true) {
            displayMenu();

            int choice = 0;
            if (!(std::cin >> choice)) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
                std::cout << BRED << "\n  [!] Invalid input — enter 1-12.\n" << RESET;
                continue;
            }

            switch (choice) {
            case 1:
                runManualMode();
                // Export after RAII guard fires for all Manual iterations
                g_performanceMonitor.displayCycleTable();
                g_performanceMonitor.displayModuleCycleTable();
                g_performanceMonitor.scanModuleSizes("src", "include");
                g_performanceMonitor.exportReport("logs/performance_report.txt");
                std::cout << BGREEN << BOLD
                          << "  \u2714  Performance report \u2192 logs/performance_report.txt\n"
                          << RESET << "\n";
                break;
            case 2:
                runSimulation();
                break;
            case 3:
                runJsonFileTest();
                // Export cycle report AFTER function returns (so RAII guard has fired)
                g_performanceMonitor.displayCycleTable();
                g_performanceMonitor.displayModuleCycleTable();
                g_performanceMonitor.scanModuleSizes("src", "include");
                g_performanceMonitor.exportReport("logs/performance_report.txt");
                std::cout << BGREEN << BOLD
                          << "  \u2714  Performance report \u2192 logs/performance_report.txt\n"
                          << RESET << "\n";
                break;
            case 4:
                // ── Feature: Driver Profile Management ───────────────────
                runDriverProfileMenu(g_profileMgr);
                break;
            case 5:
                // ── Feature: ECU Health Report ────────────────────────────
                g_ecuHealth.display();
                std::cout << "\n" << BCYAN << BOLD
                          << "  Press [Enter] to continue...\n" << RESET;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cin.get();
                break;
            case 6:
                // ── Feature: Adaptive Alert Prioritization ────────────────
                {
                    // We need an AlertManager snapshot to display; use a
                    // shared one initialised from the most recent manual run.
                    // If no run has occurred yet, the prioritizer shows the
                    // empty state gracefully.
                    AlertManager snapMgr;
                    // Replay active codes recorded by the prioritizer so
                    // display() sees a representative alert set.
                    // (The global g_alertPrioritizer already holds hit counts.)
                    g_alertPrioritizer.display(std::cout, snapMgr);
                    std::cout << BCYAN << BOLD
                              << "  Press [Enter] to continue...\n" << RESET;
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cin.get();
                }
                break;
            case 7:
                // ── Feature: Performance Graph Statistics ─────────────────
                g_perfGraph.display(std::cout);
                std::cout << BCYAN << BOLD
                          << "  Press [Enter] to continue...\n" << RESET;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cin.get();
                break;
            case 8:
                // ── Feature: CAN Bus Simulator ────────────────────────────
                g_canBus.display(std::cout);
                std::cout << BCYAN << BOLD
                          << "  Press [Enter] to continue...\n" << RESET;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cin.get();
                break;
            case 9:
                // ── Feature: OTA Update Simulator ─────────────────────────
                {
                    // Pass last known speed (0 when not in simulation)
                    double liveSpeed = 0.0;
                    g_otaSim.runInteractiveMenu(std::cout, liveSpeed);
                }
                break;
            case 10:
                // ── Feature: Crash-Safe Event Recorder ───────────────────────
                g_crashRecorder.display(std::cout);
                g_crashRecorder.flushToNvm();
                std::cout << BCYAN << BOLD
                          << "  ✔  EDR snapshot flushed to disk.\n"
                          << "  Press [Enter] to continue...\n" << RESET;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cin.get();
                break;
            case 11:
                // ── Feature: Service-Oriented Communication Model ──────────────
                {
                    // Run SD on first visit, then demonstrate method calls
                    // and show the full interactive display.
                    if (!g_socBus.services().front().offering) {
                        g_socBus.runServiceDiscovery();
                        // Default subscriptions: Dashboard subscribes to all events
                        g_socBus.subscribe(SvcId::VEHICLE_SPEED,   EventId::SPEED_CHANGED,
                                           ServiceOrientedComm::CLIENT_DASHBOARD, "Dashboard");
                        g_socBus.subscribe(SvcId::ENGINE_TEMP,     EventId::TEMP_WARNING,
                                           ServiceOrientedComm::CLIENT_DASHBOARD, "Dashboard");
                        g_socBus.subscribe(SvcId::BATTERY,         EventId::LOW_BATTERY,
                                           ServiceOrientedComm::CLIENT_DASHBOARD, "Dashboard");
                        g_socBus.subscribe(SvcId::ALERT_BROADCAST, EventId::ALERT_FIRED,
                                           ServiceOrientedComm::CLIENT_DASHBOARD, "Dashboard");
                        g_socBus.subscribe(SvcId::ALERT_BROADCAST, EventId::ALERT_FIRED,
                                           ServiceOrientedComm::CLIENT_LOGGER,    "EventLogger");
                        g_socBus.subscribe(SvcId::ENGINE_TEMP,     EventId::TEMP_WARNING,
                                           ServiceOrientedComm::CLIENT_ECU_MGR,   "EcuManager");

                        // Simulate a few method calls on startup
                        g_socBus.callMethod(SvcId::VEHICLE_SPEED,   MethodId::GET_SPEED,
                                            ServiceOrientedComm::CLIENT_DASHBOARD);
                        g_socBus.callMethod(SvcId::ENGINE_TEMP,     MethodId::GET_ENGINE_TEMP,
                                            ServiceOrientedComm::CLIENT_DASHBOARD);
                        g_socBus.callMethod(SvcId::BATTERY,         MethodId::GET_VOLTAGE,
                                            ServiceOrientedComm::CLIENT_DASHBOARD);
                        g_socBus.callMethod(SvcId::ALERT_BROADCAST, MethodId::GET_ACTIVE_ALERTS,
                                            ServiceOrientedComm::CLIENT_LOGGER);
                        g_socBus.callMethod(SvcId::DIAGNOSTICS,     MethodId::RUN_BIST,
                                            ServiceOrientedComm::CLIENT_DIAG);
                        g_socBus.callMethod(SvcId::DIAGNOSTICS,     MethodId::GET_DTC_LIST,
                                            ServiceOrientedComm::CLIENT_DIAG);
                    }
                    g_socBus.display(std::cout);
                    std::cout << BCYAN << BOLD
                              << "  Press [Enter] to continue...\n" << RESET;
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cin.get();
                }
                break;
            case 12:
                // ── Feature: DTC Simulator ────────────────────────────────────
                g_dtcSim.runInteractiveMenu(std::cout);
                break;
            case 13:
                std::cout << "\n" << BMAGENTA << BOLD
                          << "  Goodbye — Safe driving!\n" << RESET << "\n";
                return 0;
            default:
                std::cout << BRED
                          << "\n  [!] Invalid choice — enter 1 to 13.\n"
                          << RESET;
            }
        }

    } catch (const std::exception& ex) {
        std::cerr << BRED << BOLD << "\n  [FATAL] " << ex.what() << "\n" << RESET;
        return 1;
    }
    return 0;
}
