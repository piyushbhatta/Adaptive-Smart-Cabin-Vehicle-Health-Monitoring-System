/**
 * @file    DtcSimulator.cpp
 * @brief   Implementation of DtcSimulator — OBD-II / UDS Diagnostic Trouble
 *          Code simulation for the Smart Vehicle ECU.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "DtcSimulator.hpp"
#include "AlertEvaluator.hpp"   // SensorData
#include "Sensor.hpp"           // Threshold constants
#include "Dashboard.hpp"        // Color namespace

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
//  Box-drawing constants (inner printable width = 62, same as rest of project)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int DBW = 62;

static std::string drep(int n, const std::string& ch = "\xe2\x95\x90") { // "═"
    std::string r; r.reserve(n * ch.size());
    for (int i = 0; i < n; ++i) r += ch;
    return r;
}

void DtcSimulator::dtcTop(std::ostream& os, const char* c) {
    os << c << BOLD << "  ╔" << drep(DBW) << "╗\n" << RESET;
}
void DtcSimulator::dtcBot(std::ostream& os, const char* c) {
    os << c << BOLD << "  ╚" << drep(DBW) << "╝\n" << RESET;
}
void DtcSimulator::dtcMid(std::ostream& os, const char* c) {
    os << c << BOLD << "  ╠" << drep(DBW) << "╣\n" << RESET;
}
void DtcSimulator::dtcRow(std::ostream& os, const std::string& txt,
                           const char* boxC, const char* txtC) {
    int pad = DBW - static_cast<int>(txt.size());
    if (pad < 0) pad = 0;
    os << boxC << BOLD << "  ║" << RESET
       << txtC << txt << std::string(pad, ' ') << RESET
       << boxC << BOLD << "║\n" << RESET;
}
void DtcSimulator::dtcEmpty(std::ostream& os, const char* c) {
    dtcRow(os, "", c, c);
}

// ─────────────────────────────────────────────────────────────────────────────
//  DtcRecord helpers
// ─────────────────────────────────────────────────────────────────────────────

char DtcRecord::categoryChar() const {
    switch (category) {
    case DtcCategory::POWERTRAIN: return 'P';
    case DtcCategory::CHASSIS:    return 'C';
    case DtcCategory::BODY:       return 'B';
    case DtcCategory::NETWORK:    return 'U';
    }
    return '?';
}

std::string DtcRecord::statusString() const {
    switch (status) {
    case DtcStatus::PENDING:   return "PENDING";
    case DtcStatus::CONFIRMED: return "CONFIRMED";
    case DtcStatus::CLEARED:   return "CLEARED";
    }
    return "UNKNOWN";
}

const char* DtcRecord::statusColour() const {
    switch (status) {
    case DtcStatus::PENDING:   return BYELLOW;
    case DtcStatus::CONFIRMED: return BRED;
    case DtcStatus::CLEARED:   return BGREEN;
    }
    return BWHITE;
}

std::string DtcRecord::categoryString() const {
    switch (category) {
    case DtcCategory::POWERTRAIN: return "Powertrain";
    case DtcCategory::CHASSIS:    return "Chassis";
    case DtcCategory::BODY:       return "Body";
    case DtcCategory::NETWORK:    return "Network";
    }
    return "Unknown";
}

std::string DtcRecord::lastSeenAgo() const {
    using namespace std::chrono;
    auto elapsed = steady_clock::now() - lastSeen;
    long secs = duration_cast<seconds>(elapsed).count();
    if (secs < 60) return std::to_string(secs) + "s ago";
    long mins = secs / 60;
    long rem  = secs % 60;
    std::ostringstream oss;
    oss << mins << "m " << rem << "s ago";
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Fault condition predicates
// ─────────────────────────────────────────────────────────────────────────────

bool DtcSimulator::faultEngineOverheat(const SensorData& s) {
    return s.temp >= EngineTemperatureSensor::TEMP_CRITICAL;
}
bool DtcSimulator::faultLowBattery(const SensorData& s) {
    return s.volt < BatterySensor::VOLTAGE_LOW;
}
bool DtcSimulator::faultOverspeed(const SensorData& s) {
    return s.speed > SpeedSensor::OVERSPEED_THRESHOLD;
}
bool DtcSimulator::faultLowTirePressure(const SensorData& s) {
    return s.psi < TirePressureSensor::PRESSURE_LOW;
}
bool DtcSimulator::faultDoorAjar(const SensorData& s) {
    return s.doorOpen && s.speed > SpeedSensor::DOOR_SPEED_LIMIT;
}
bool DtcSimulator::faultSeatbeltUnlocked(const SensorData& s) {
    return !s.beltLocked && s.speed >= 1.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  DtcSimulator — constructor
// ─────────────────────────────────────────────────────────────────────────────

DtcSimulator::DtcSimulator() {
    // Pre-populate debounce table with all known codes.
    for (const char* code : {
        "P0217", "P0562", "P0082", "C0035", "B1001", "B2101"
    }) {
        debounce_.push_back({ code, 0 });
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

DtcRecord* DtcSimulator::findOrCreate(const std::string& code,
                                       const std::string& description,
                                       DtcCategory        category)
{
    for (auto& rec : history_) {
        if (rec.code == code && rec.status != DtcStatus::CLEARED)
            return &rec;
    }
    // Create new entry
    DtcRecord r;
    r.code        = code;
    r.description = description;
    r.category    = category;
    r.status      = DtcStatus::PENDING;
    r.occurrences = 0;
    r.firstSeen   = std::chrono::steady_clock::now();
    r.lastSeen    = r.firstSeen;

    if (history_.size() >= MAX_HISTORY)
        history_.pop_front();
    history_.push_back(r);
    return &history_.back();
}

int DtcSimulator::incrementDebounce(const std::string& code) {
    for (auto& e : debounce_) {
        if (e.code == code) return ++e.count;
    }
    debounce_.push_back({ code, 1 });
    return 1;
}

void DtcSimulator::resetDebounce(const std::string& code) {
    for (auto& e : debounce_) {
        if (e.code == code) { e.count = 0; return; }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  evaluate() — main per-cycle update
// ─────────────────────────────────────────────────────────────────────────────

void DtcSimulator::evaluate(const SensorData& s) {

    // ── Fault descriptor table ─────────────────────────────────────────────
    struct FaultDef {
        std::string  code;
        std::string  description;
        DtcCategory  category;
        bool (*check)(const SensorData&);
    };

    static const FaultDef FAULTS[] = {
        { "P0217", "Engine Coolant Over Temperature Condition",
          DtcCategory::POWERTRAIN, faultEngineOverheat    },
        { "P0562", "System Voltage Low",
          DtcCategory::POWERTRAIN, faultLowBattery        },
        { "P0082", "Intake Air Temperature Sensor / Vehicle Overspeed",
          DtcCategory::POWERTRAIN, faultOverspeed         },
        { "C0035", "Left Front Wheel Speed Sensor / Tire Pressure Low",
          DtcCategory::CHASSIS,    faultLowTirePressure   },
        { "B1001", "Door Ajar While Vehicle Moving",
          DtcCategory::BODY,       faultDoorAjar          },
        { "B2101", "Seatbelt Not Fastened While Moving",
          DtcCategory::BODY,       faultSeatbeltUnlocked  },
    };

    auto now = std::chrono::steady_clock::now();

    for (const auto& fd : FAULTS) {
        bool active = fd.check(s);

        if (active) {
            int cnt = incrementDebounce(fd.code);
            DtcRecord* rec = findOrCreate(fd.code, fd.description, fd.category);
            rec->occurrences++;
            rec->lastSeen = now;

            if (cnt >= CONFIRM_THRESHOLD && rec->status == DtcStatus::PENDING) {
                // Promote to CONFIRMED and capture freeze-frame
                rec->status              = DtcStatus::CONFIRMED;
                rec->freezeFrame.engineTemp  = s.temp;
                rec->freezeFrame.batteryVolt = s.volt;
                rec->freezeFrame.speed       = s.speed;
                rec->freezeFrame.tirePsi     = s.psi;
                rec->freezeFrame.doorOpen    = s.doorOpen;
                rec->freezeFrame.beltLocked  = s.beltLocked;
            }
        } else {
            // Fault healed → reset debounce (DTC stays until explicitly cleared)
            resetDebounce(fd.code);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  clearAllDtcs() / clearDtc()
// ─────────────────────────────────────────────────────────────────────────────

void DtcSimulator::clearAllDtcs() {
    for (auto& rec : history_) {
        if (rec.status != DtcStatus::CLEARED)
            rec.status = DtcStatus::CLEARED;
    }
    for (auto& e : debounce_) e.count = 0;
}

void DtcSimulator::clearDtc(const std::string& code) {
    for (auto& rec : history_) {
        if (rec.code == code && rec.status != DtcStatus::CLEARED) {
            rec.status = DtcStatus::CLEARED;
        }
    }
    resetDebounce(code);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Query helpers
// ─────────────────────────────────────────────────────────────────────────────

std::vector<DtcRecord> DtcSimulator::getActiveDtcs() const {
    std::vector<DtcRecord> out;
    for (const auto& rec : history_) {
        if (rec.status != DtcStatus::CLEARED)
            out.push_back(rec);
    }
    return out;
}

int DtcSimulator::activeCount() const {
    int n = 0;
    for (const auto& rec : history_)
        if (rec.status != DtcStatus::CLEARED) ++n;
    return n;
}

bool DtcSimulator::milOn() const {
    for (const auto& rec : history_)
        if (rec.status == DtcStatus::CONFIRMED) return true;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  display() — full panel
// ─────────────────────────────────────────────────────────────────────────────

void DtcSimulator::display(std::ostream& os) const {

    os << "\n";

    // ── Header ───────────────────────────────────────────────────────────────
    const char* hdrC = milOn() ? BRED : BGREEN;
    dtcTop(os, hdrC);
    dtcRow(os, "   DIAGNOSTIC TROUBLE CODE (DTC) REPORT", hdrC, BWHITE);
    dtcRow(os, "   OBD-II / ISO 14229 UDS  —  Dem Simulation", hdrC, BCYAN);
    dtcMid(os, hdrC);

    // MIL status line
    std::string milLine = "   MIL Status : ";
    milLine += milOn() ? "ON  ⚠  (Malfunction Indicator Lamp illuminated)" :
                         "OFF  ✔  (No confirmed faults)";
    dtcRow(os, milLine, hdrC, milOn() ? BRED : BGREEN);

    auto active = getActiveDtcs();
    int pending   = 0, confirmed = 0;
    for (const auto& r : active) {
        if (r.status == DtcStatus::PENDING)   ++pending;
        if (r.status == DtcStatus::CONFIRMED) ++confirmed;
    }

    std::ostringstream summary;
    summary << "   Active DTCs : " << active.size()
            << "  (Confirmed: " << confirmed
            << "  Pending: " << pending << ")";
    dtcRow(os, summary.str(), hdrC, BWHITE);
    dtcBot(os, hdrC);

    // ── Active DTC table ─────────────────────────────────────────────────────
    if (active.empty()) {
        os << "\n";
        dtcTop(os, BGREEN);
        dtcRow(os, "   No active DTCs — all systems nominal", BGREEN, BGREEN);
        dtcBot(os, BGREEN);
    } else {
        os << "\n";
        dtcTop(os, BYELLOW);
        dtcRow(os, "   ACTIVE FAULT TABLE", BYELLOW, BWHITE);
        dtcMid(os, BYELLOW);
        dtcRow(os, "   CODE    CATEGORY     STATUS      OCC  LAST SEEN", BYELLOW, BCYAN);
        dtcMid(os, BYELLOW);

        for (const auto& rec : active) {
            const char* sc = rec.statusColour();
            std::ostringstream row;
            row << "   " << std::left << std::setw(8) << rec.code
                << std::setw(13) << rec.categoryString()
                << std::setw(12) << rec.statusString()
                << std::setw(5)  << rec.occurrences
                << rec.lastSeenAgo();
            dtcRow(os, row.str(), BYELLOW, sc);

            // Description sub-row
            std::string descRow = "      → " + rec.description;
            dtcRow(os, descRow, BYELLOW, BWHITE);
        }
        dtcBot(os, BYELLOW);

        // ── Freeze-frame data for confirmed DTCs ───────────────────────────
        bool hasConfirmed = false;
        for (const auto& rec : active)
            if (rec.status == DtcStatus::CONFIRMED) { hasConfirmed = true; break; }

        if (hasConfirmed) {
            os << "\n";
            dtcTop(os, BMAGENTA);
            dtcRow(os, "   FREEZE FRAME DATA  (Snapshot at fault confirmation)", BMAGENTA, BWHITE);

            for (const auto& rec : active) {
                if (rec.status != DtcStatus::CONFIRMED) continue;
                dtcMid(os, BMAGENTA);
                dtcRow(os, "   DTC: " + rec.code + "  —  " + rec.description,
                       BMAGENTA, BRED);
                dtcMid(os, BMAGENTA);

                auto& ff = rec.freezeFrame;
                auto fmtD = [](double v, int p = 1) {
                    std::ostringstream o;
                    o << std::fixed << std::setprecision(p) << v;
                    return o.str();
                };

                auto ffRow = [&](const std::string& k, const std::string& v) {
                    std::string line = "   " + k;
                    int pad = 22 - (int)line.size();
                    if (pad < 1) pad = 1;
                    line += std::string(pad, ' ') + ": " + v;
                    dtcRow(os, line, BMAGENTA, BWHITE);
                };

                ffRow("Engine Temp",   fmtD(ff.engineTemp)  + " °C");
                ffRow("Battery Volt",  fmtD(ff.batteryVolt, 2) + " V");
                ffRow("Speed",         fmtD(ff.speed)       + " km/h");
                ffRow("Tire Pressure", fmtD(ff.tirePsi)     + " PSI");
                ffRow("Door",          ff.doorOpen   ? "OPEN"   : "CLOSED");
                ffRow("Seatbelt",      ff.beltLocked ? "LOCKED" : "UNLOCKED");
            }
            dtcBot(os, BMAGENTA);
        }
    }

    // ── History (cleared DTCs, last 10) ──────────────────────────────────────
    std::vector<const DtcRecord*> cleared;
    for (auto it = history_.rbegin(); it != history_.rend(); ++it)
        if (it->status == DtcStatus::CLEARED) cleared.push_back(&(*it));
    if (cleared.size() > 10) cleared.resize(10);

    if (!cleared.empty()) {
        os << "\n";
        dtcTop(os, BCYAN);
        dtcRow(os, "   CLEARED DTC HISTORY  (last 10)", BCYAN, BWHITE);
        dtcMid(os, BCYAN);
        dtcRow(os, "   CODE    CATEGORY     OCC   DESCRIPTION", BCYAN, BCYAN);
        dtcMid(os, BCYAN);
        for (const auto* rec : cleared) {
            std::ostringstream row;
            row << "   " << std::left << std::setw(8) << rec->code
                << std::setw(13) << rec->categoryString()
                << std::setw(6)  << rec->occurrences
                << rec->description;
            std::string s = row.str();
            if ((int)s.size() > DBW) s = s.substr(0, DBW - 1);
            dtcRow(os, s, BCYAN, BGREEN);
        }
        dtcBot(os, BCYAN);
    }

    os << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  displayCompact() — one-line footer
// ─────────────────────────────────────────────────────────────────────────────

void DtcSimulator::displayCompact(std::ostream& os) const {
    int ac = activeCount();
    int cf = 0;
    for (const auto& r : history_)
        if (r.status == DtcStatus::CONFIRMED) ++cf;

    const char* c = milOn() ? BRED : (ac > 0 ? BYELLOW : BGREEN);
    os << c << BOLD << "  DTC: " << RESET
       << c << ac << " active (" << cf << " confirmed)"
       << (milOn() ? "  [MIL ON]" : "  [MIL OFF]")
       << RESET << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  runInteractiveMenu()
// ─────────────────────────────────────────────────────────────────────────────

void DtcSimulator::runInteractiveMenu(std::ostream& os) {

    while (true) {
        os << "\n";
        dtcTop(os, BMAGENTA);
        dtcRow(os, "   DTC SIMULATOR — INTERACTIVE MENU", BMAGENTA, BWHITE);
        dtcMid(os, BMAGENTA);
        dtcRow(os, "   [1]  View full DTC report", BMAGENTA, BGREEN);
        dtcRow(os, "   [2]  Clear ALL DTCs  (UDS 0x14)", BMAGENTA, BYELLOW);
        dtcRow(os, "   [3]  Clear specific DTC by code", BMAGENTA, BYELLOW);
        dtcRow(os, "   [4]  Inject fault (simulate fault condition)", BMAGENTA, BCYAN);
        dtcRow(os, "   [5]  Back to main menu", BMAGENTA, BRED);
        dtcEmpty(os, BMAGENTA);
        dtcBot(os, BMAGENTA);
        os << "\n" << BWHITE << BOLD << "  ▶  Enter choice [1-5]: " << RESET;

        int choice = 0;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        if (choice == 5) break;

        switch (choice) {
        case 1:
            display(os);
            break;

        case 2:
            clearAllDtcs();
            os << "\n" << BGREEN << BOLD
               << "  ✔  All DTCs cleared (UDS ClearDiagnosticInformation).\n"
               << RESET << "\n";
            break;

        case 3: {
            os << "\n" << BCYAN << BOLD
               << "  ▷  Enter DTC code to clear (e.g. P0217): " << RESET;
            std::string code;
            std::cin >> code;
            // Upper-case it
            for (auto& ch : code) ch = static_cast<char>(std::toupper(ch));
            clearDtc(code);
            os << BGREEN << BOLD
               << "  ✔  DTC " << code << " cleared (if it existed).\n"
               << RESET << "\n";
            break;
        }

        case 4: {
            os << "\n";
            dtcTop(os, BCYAN);
            dtcRow(os, "   FAULT INJECTION — choose a fault to simulate:", BCYAN, BWHITE);
            dtcMid(os, BCYAN);
            dtcRow(os, "   [1]  P0217 Engine Overheat      (temp = 115 °C)", BCYAN, BRED);
            dtcRow(os, "   [2]  P0562 Low Battery Voltage  (volt = 9.0 V)",  BCYAN, BYELLOW);
            dtcRow(os, "   [3]  P0082 Overspeed            (speed = 135 km/h)", BCYAN, BYELLOW);
            dtcRow(os, "   [4]  C0035 Low Tire Pressure    (psi = 20.0)",    BCYAN, BYELLOW);
            dtcRow(os, "   [5]  B1001 Door Ajar Moving     (door open @ 30 km/h)", BCYAN, BYELLOW);
            dtcRow(os, "   [6]  B2101 Seatbelt Unlatched   (no belt @ 50 km/h)", BCYAN, BYELLOW);
            dtcEmpty(os, BCYAN);
            dtcBot(os, BCYAN);
            os << "\n" << BWHITE << BOLD << "  ▶  Fault choice [1-6]: " << RESET;

            int fc = 0;
            if (!(std::cin >> fc)) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                break;
            }

            // Build a synthetic SensorData snapshot for the chosen fault
            SensorData inj{};
            inj.temp      = 85.0;  // nominal
            inj.volt      = 12.5;
            inj.speed     = 0.0;
            inj.psi       = 30.0;
            inj.doorOpen  = false;
            inj.beltLocked= true;

            switch (fc) {
            case 1: inj.temp     = 115.0; break;
            case 2: inj.volt     =   9.0; break;
            case 3: inj.speed    = 135.0; break;
            case 4: inj.psi      =  20.0; break;
            case 5: inj.doorOpen = true; inj.speed = 30.0; break;
            case 6: inj.beltLocked = false; inj.speed = 50.0; break;
            default: break;
            }

            // Inject enough cycles to confirm the DTC
            for (int i = 0; i < CONFIRM_THRESHOLD; ++i)
                evaluate(inj);

            os << "\n" << BGREEN << BOLD
               << "  ✔  Fault injected and confirmed (3 cycles simulated).\n"
               << RESET;
            display(os);
            break;
        }

        default:
            os << BRED << "\n  [!] Invalid choice.\n" << RESET;
            break;
        }

        os << "\n" << BCYAN << BOLD << "  Press [Enter] to continue...\n" << RESET;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Global instance
// ─────────────────────────────────────────────────────────────────────────────

DtcSimulator g_dtcSim;
