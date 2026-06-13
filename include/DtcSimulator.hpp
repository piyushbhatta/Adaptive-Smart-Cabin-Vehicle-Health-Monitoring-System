/**
 * @file    DtcSimulator.hpp
 * @brief   Diagnostic Trouble Code (DTC) simulation for the Smart Vehicle ECU.
 *
 * @details Models the OBD-II / ISO 14229 (UDS) DTC subsystem as found in
 *          production AUTOSAR Diagnostic Event Manager (Dem) components.
 *
 *  DTC categories simulated:
 *  ┌─────────────────┬──────────────────────────────────────────────────────┐
 *  │  Powertrain (P) │ Engine, fuel, ignition, emissions                   │
 *  │  Chassis   (C) │ Brakes, suspension, steering                         │
 *  │  Body      (B) │ Doors, windows, lights, seatbelt                     │
 *  │  Network   (U) │ CAN bus, SOME/IP communication faults                │
 *  └─────────────────┴──────────────────────────────────────────────────────┘
 *
 *  DTC lifecycle states (AUTOSAR Dem mapping):
 *  ┌──────────────────┬───────────────────────────────────────────────────────┐
 *  │  PENDING         │ Fault detected in current drive cycle; not yet        │
 *  │                  │ confirmed (within debounce window).                   │
 *  │  CONFIRMED       │ Fault confirmed across ≥ 1 drive cycles or debounce   │
 *  │                  │ threshold exceeded — MIL (warning lamp) illuminated.  │
 *  │  CLEARED         │ Cleared by scan-tool or automatic healed condition.   │
 *  └──────────────────┴───────────────────────────────────────────────────────┘
 *
 *  Freeze-frame data:
 *  Each confirmed DTC stores a snapshot of the sensor values at the time of
 *  first confirmation (analogous to ISO 14229 Freeze Frame Record).
 *
 *  Automotive mapping:
 *  - DtcSimulator   → AUTOSAR Dem (Diagnostic Event Manager)
 *  - DtcRecord      → DEM_EVENT_STATUS + FreezeFrame
 *  - clearAllDtcs() → UDS service 0x14 (ClearDiagnosticInformation)
 *  - reportDtcs()   → UDS service 0x19 (ReadDTCInformation)
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef DTC_SIMULATOR_HPP
#define DTC_SIMULATOR_HPP

#include <string>
#include <vector>
#include <deque>
#include <ostream>
#include <chrono>

// ─────────────────────────────────────────────────────────────────────────────
// Forward declaration
// ─────────────────────────────────────────────────────────────────────────────
struct SensorData;

// ─────────────────────────────────────────────────────────────────────────────
// DtcCategory
// ─────────────────────────────────────────────────────────────────────────────

enum class DtcCategory {
    POWERTRAIN, ///< P-codes: engine, fuel, ignition
    CHASSIS,    ///< C-codes: brakes, suspension
    BODY,       ///< B-codes: doors, windows, seatbelt
    NETWORK     ///< U-codes: CAN / SOME/IP bus faults
};

// ─────────────────────────────────────────────────────────────────────────────
// DtcStatus
// ─────────────────────────────────────────────────────────────────────────────

enum class DtcStatus {
    PENDING,    ///< Fault detected; awaiting confirmation
    CONFIRMED,  ///< MIL-on; confirmed across drive cycles
    CLEARED     ///< Cleared by scan-tool or healed
};

// ─────────────────────────────────────────────────────────────────────────────
// FreezeFrame  –  sensor snapshot captured at DTC confirmation time
// ─────────────────────────────────────────────────────────────────────────────

struct FreezeFrame {
    double engineTemp  {0.0};
    double batteryVolt {0.0};
    double speed       {0.0};
    double tirePsi     {0.0};
    bool   doorOpen    {false};
    bool   beltLocked  {true};
};

// ─────────────────────────────────────────────────────────────────────────────
// DtcRecord  –  one active/historical DTC entry
// ─────────────────────────────────────────────────────────────────────────────

struct DtcRecord {
    std::string  code;          ///< e.g. "P0217"
    std::string  description;   ///< Human-readable fault description
    DtcCategory  category;      ///< Powertrain / Chassis / Body / Network
    DtcStatus    status;        ///< PENDING / CONFIRMED / CLEARED
    int          occurrences;   ///< Times this fault has been detected
    FreezeFrame  freezeFrame;   ///< Sensor snapshot at first confirmation
    std::chrono::steady_clock::time_point firstSeen;   ///< First detection timestamp
    std::chrono::steady_clock::time_point lastSeen;    ///< Most recent detection timestamp

    /// Returns a short string for the category prefix letter.
    char categoryChar() const;

    /// Returns "PENDING", "CONFIRMED", or "CLEARED".
    std::string statusString() const;

    /// Returns ANSI colour code matching status.
    const char* statusColour() const;

    /// Returns category label string.
    std::string categoryString() const;

    /// Returns human-readable "MM:SS ago" elapsed string.
    std::string lastSeenAgo() const;
};

// ─────────────────────────────────────────────────────────────────────────────
// DtcSimulator
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  DtcSimulator
 * @brief  Simulates OBD-II / UDS DTC detection, confirmation, and clearing.
 *
 * @details Feed sensor data each cycle via evaluate().  The simulator maps
 *          sensor faults to standard OBD-II codes, manages a debounce counter
 *          (PENDING → CONFIRMED after N consecutive faults), stores freeze-
 *          frame snapshots, and provides a rich terminal display.
 *
 * Thread-safety: NOT internally synchronised — caller must hold g_mutex.
 */
class DtcSimulator {
public:
    // ── Debounce threshold ────────────────────────────────────────────────────
    /// Consecutive fault cycles required to promote PENDING → CONFIRMED.
    static constexpr int CONFIRM_THRESHOLD = 3;

    /// Maximum DTC history entries retained.
    static constexpr std::size_t MAX_HISTORY = 50;

    DtcSimulator();

    // ── Core API ──────────────────────────────────────────────────────────────

    /**
     * @brief  Evaluates sensor data and updates DTC states.
     *
     * @param  s  Current sensor snapshot.
     *
     * @details For each fault condition:
     *  1. If no DTC exists → creates a PENDING entry.
     *  2. If PENDING and debounce threshold reached → promotes to CONFIRMED
     *     and captures freeze-frame data.
     *  3. If condition clears while CONFIRMED → status stays CONFIRMED
     *     (requires explicit clearAllDtcs() call, like a real ECU).
     */
    void evaluate(const SensorData& s);

    /**
     * @brief  Clears all DTCs (simulates UDS 0x14 service).
     *
     * @details Marks all CONFIRMED and PENDING DTCs as CLEARED.
     *          Cleared DTCs remain in history for audit purposes.
     */
    void clearAllDtcs();

    /**
     * @brief  Clears a single DTC by code (simulates selective clear).
     * @param  code  DTC code string, e.g. "P0217".
     */
    void clearDtc(const std::string& code);

    // ── Query API ─────────────────────────────────────────────────────────────

    /**
     * @brief  Returns all active (PENDING or CONFIRMED) DTC records.
     */
    std::vector<DtcRecord> getActiveDtcs() const;

    /**
     * @brief  Returns the total count of active (non-CLEARED) DTCs.
     */
    int activeCount() const;

    /**
     * @brief  Returns true if any CONFIRMED DTC is present (MIL-on condition).
     */
    bool milOn() const;

    /**
     * @brief  Returns the full DTC history (including CLEARED entries).
     */
    const std::deque<DtcRecord>& history() const { return history_; }

    // ── Display ───────────────────────────────────────────────────────────────

    /**
     * @brief  Renders the full DTC report panel to the given stream.
     *
     * @details Shows:
     *  - MIL status and active DTC count.
     *  - Active DTC table with code, description, status, occurrences.
     *  - Freeze-frame data for each confirmed DTC.
     *  - History of cleared DTCs (last 10).
     */
    void display(std::ostream& os) const;

    /**
     * @brief  Renders a compact one-line DTC summary (for dashboard footer).
     *
     * @details Format: "DTC: 2 active (1 confirmed)  [MIL ON]"
     */
    void displayCompact(std::ostream& os) const;

    /**
     * @brief  Runs the interactive DTC menu (clear DTCs, inject faults, view).
     */
    void runInteractiveMenu(std::ostream& os);

private:
    std::deque<DtcRecord> history_;  ///< All DTCs ever seen (capped at MAX_HISTORY).

    // Debounce counters: maps DTC code → consecutive-fault tick count.
    // When a fault clears, counter resets to 0 (but DTC stays until cleared).
    struct DebounceEntry { std::string code; int count{0}; };
    std::vector<DebounceEntry> debounce_;

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Finds or creates a DtcRecord in history_ by code.
    DtcRecord* findOrCreate(const std::string& code,
                             const std::string& description,
                             DtcCategory category);

    /// Increments debounce counter; returns new count.
    int incrementDebounce(const std::string& code);

    /// Resets debounce counter for a code (fault healed).
    void resetDebounce(const std::string& code);

    // ── Fault condition checks (return true when fault is active) ─────────────
    static bool faultEngineOverheat   (const SensorData& s);
    static bool faultLowBattery       (const SensorData& s);
    static bool faultOverspeed        (const SensorData& s);
    static bool faultLowTirePressure  (const SensorData& s);
    static bool faultDoorAjar         (const SensorData& s);
    static bool faultSeatbeltUnlocked (const SensorData& s);

    // ── Box drawing helpers ───────────────────────────────────────────────────
    static void dtcTop(std::ostream& os, const char* c);
    static void dtcBot(std::ostream& os, const char* c);
    static void dtcMid(std::ostream& os, const char* c);
    static void dtcRow(std::ostream& os, const std::string& txt,
                       const char* boxC, const char* txtC);
    static void dtcEmpty(std::ostream& os, const char* c);
};

// ─────────────────────────────────────────────────────────────────────────────
// Global DTC simulator instance (defined in DtcSimulator.cpp)
// ─────────────────────────────────────────────────────────────────────────────
extern DtcSimulator g_dtcSim;

#endif // DTC_SIMULATOR_HPP
