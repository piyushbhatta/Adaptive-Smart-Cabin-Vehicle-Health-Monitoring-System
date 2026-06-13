/**
 * @file    OtaUpdateSimulator.hpp
 * @brief   Over-The-Air (OTA) firmware update simulation for the Smart Vehicle ECU.
 *
 * @details Models a realistic automotive OTA update lifecycle as defined by
 *          AUTOSAR Adaptive Platform (AP) and UNECE WP.29/R156 regulations:
 *
 *  Update lifecycle states:
 *  ┌───────────────┬──────────────────────────────────────────────────────┐
 *  │  IDLE         │ No update in progress.  Vehicle can drive normally.  │
 *  │  CHECKING     │ Polling the update server for available packages.    │
 *  │  DOWNLOADING  │ Fetching firmware package (chunked, with CRC check). │
 *  │  VERIFYING    │ Signature + hash validation of the received package. │
 *  │  READY        │ Package validated, awaiting install consent.         │
 *  │  INSTALLING   │ Flash write in progress — vehicle must be parked.    │
 *  │  REBOOTING    │ ECU reset to activate new firmware.                  │
 *  │  SUCCESS      │ Update applied; version bumped; rollback window open.│
 *  │  FAILED       │ Any stage error; rollback to previous version.       │
 *  └───────────────┴──────────────────────────────────────────────────────┘
 *
 *  Simulated ECU firmware modules (each independently versioned):
 *  ┌──────────────────────┬──────────┬──────────────────────────────────┐
 *  │  Module              │  ECU ID  │  Description                     │
 *  ├──────────────────────┼──────────┼──────────────────────────────────┤
 *  │  Body Control Module │  BCM     │  Doors, lights, HVAC             │
 *  │  Engine Control Unit │  ECU     │  Fuel, ignition, emissions       │
 *  │  Advanced Driver Sys │  ADAS    │  Lane-keep, collision avoidance  │
 *  │  Telematics Gateway  │  TCU     │  Connectivity, OTA channel itself│
 *  │  Instrument Cluster  │  IC      │  Dashboard, HMI                  │
 *  └──────────────────────┴──────────┴──────────────────────────────────┘
 *
 *  Safety constraints (WP.29 R156 compliance):
 *  - Install phase requires vehicle speed == 0 km/h.
 *  - Cryptographic signature verification is simulated with a SHA-256-style
 *    hash comparison using std::hash (deterministic in simulation).
 *  - Failed installs trigger automatic rollback to the last known-good version.
 *  - All events are timestamped and stored in a structured update history log.
 *
 *  Automotive mapping:
 *  - OtaPackage        → AP UCM (Update and Configuration Management) package
 *  - OtaUpdateSimulator → AUTOSAR UCM Master / OTA Client
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef OTA_UPDATE_SIMULATOR_HPP
#define OTA_UPDATE_SIMULATOR_HPP

#include <string>
#include <vector>
#include <deque>
#include <ostream>
#include <chrono>
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
//  OtaState  —  update lifecycle state machine
// ─────────────────────────────────────────────────────────────────────────────

enum class OtaState {
    IDLE,
    CHECKING,
    DOWNLOADING,
    VERIFYING,
    READY,
    INSTALLING,
    REBOOTING,
    SUCCESS,
    FAILED
};

/// Returns a human-readable label for an OtaState.
std::string otaStateToString(OtaState s);

/// Returns the ANSI colour code appropriate for the given state.
const char* otaStateColour(OtaState s);

// ─────────────────────────────────────────────────────────────────────────────
//  FirmwareVersion  —  semantic versioning (MAJOR.MINOR.PATCH)
// ─────────────────────────────────────────────────────────────────────────────

struct FirmwareVersion {
    int major{1};
    int minor{0};
    int patch{0};

    /// Returns "MAJOR.MINOR.PATCH" string.
    std::string toString() const;

    /// Bumps patch version (simulates a minor ECU update).
    FirmwareVersion nextPatch() const;

    /// Bumps minor version (simulates a feature update).
    FirmwareVersion nextMinor() const;

    bool operator==(const FirmwareVersion& o) const {
        return major == o.major && minor == o.minor && patch == o.patch;
    }
    bool operator!=(const FirmwareVersion& o) const { return !(*this == o); }
};

// ─────────────────────────────────────────────────────────────────────────────
//  EcuModule  —  one independently-versioned ECU firmware module
// ─────────────────────────────────────────────────────────────────────────────

struct EcuModule {
    std::string     id;              ///< Short identifier e.g. "BCM".
    std::string     name;            ///< Long name e.g. "Body Control Module".
    FirmwareVersion currentVersion;  ///< Installed version.
    FirmwareVersion previousVersion; ///< Last known-good version (for rollback).
    bool            updatePending{false}; ///< True when a newer package is queued.
};

// ─────────────────────────────────────────────────────────────────────────────
//  OtaPackage  —  a firmware update bundle for one ECU module
// ─────────────────────────────────────────────────────────────────────────────

struct OtaPackage {
    std::string     packageId;        ///< Unique package identifier string.
    std::string     targetModuleId;   ///< ECU module this package targets.
    FirmwareVersion newVersion;       ///< Version after install.
    std::size_t     totalChunks{0};   ///< Total download chunks.
    std::size_t     downloadedChunks{0}; ///< Chunks received so far.
    std::size_t     packageSizeKB{0}; ///< Simulated package size in kB.
    std::string     checksum;         ///< Expected SHA-256-style hex digest.
    std::string     signature;        ///< Simulated RSA-4096 signature token.
    std::string     releaseNotes;     ///< Human-readable change summary.

    /// Returns download progress as a percentage [0–100].
    double downloadPercent() const;
};

// ─────────────────────────────────────────────────────────────────────────────
//  OtaHistoryEntry  —  one event in the audit trail
// ─────────────────────────────────────────────────────────────────────────────

struct OtaHistoryEntry {
    std::string timestamp;   ///< Wall-clock timestamp string.
    std::string moduleId;    ///< Affected ECU module.
    OtaState    state;       ///< State the system transitioned into.
    std::string detail;      ///< Human-readable event detail.
};

// ─────────────────────────────────────────────────────────────────────────────
//  OtaStats  —  aggregate counters across all update sessions
// ─────────────────────────────────────────────────────────────────────────────

struct OtaStats {
    int  totalChecks{0};       ///< Number of server poll cycles.
    int  packagesFound{0};     ///< Packages discovered.
    int  successfulUpdates{0}; ///< Completed installs.
    int  failedUpdates{0};     ///< Aborted installs.
    int  rollbacks{0};         ///< Rollback events triggered.
    long totalDownloadKB{0};   ///< Cumulative download size in kB.
};

// ─────────────────────────────────────────────────────────────────────────────
//  OtaUpdateSimulator
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  OtaUpdateSimulator
 * @brief  Simulates the full AUTOSAR UCM-based OTA firmware update lifecycle
 *         for a fleet of ECU modules.
 *
 * @details
 *  Lifecycle (interactive):
 *    1. display() shows the current fleet status and available actions.
 *    2. checkForUpdates()  — polls server, queues OtaPackage objects.
 *    3. startDownload()    — chunks download with a progress bar.
 *    4. verifyPackage()    — checksum + signature validation.
 *    5. installUpdate()    — flash write (requires speed == 0).
 *    6. finalise() / rollback() — commits or reverts the version.
 *
 *  Fault injection:
 *    - 10% chance of a checksum mismatch during VERIFYING → triggers FAILED.
 *    - 5%  chance of a flash write error during INSTALLING → triggers FAILED.
 *    - FAILED always invokes rollback() automatically.
 *
 *  Thread-safety:
 *    All state is protected by otaMutex_.  The background download thread
 *    holds the mutex only while advancing downloadedChunks.
 */
class OtaUpdateSimulator {
public:
    // ── Constants ─────────────────────────────────────────────────────────────
    static constexpr int    CHUNK_SIZE_KB       = 64;   ///< Bytes per download chunk (sim).
    static constexpr int    TOTAL_CHUNKS        = 16;   ///< Chunks per package.
    static constexpr double CHECKSUM_FAIL_RATE  = 0.10; ///< 10% verify failure rate.
    static constexpr double FLASH_FAIL_RATE     = 0.05; ///< 5%  install failure rate.
    static constexpr int    HISTORY_CAPACITY    = 50;   ///< Max audit trail entries.

    /**
     * @brief  Constructs the simulator and initialises the default ECU module fleet.
     */
    OtaUpdateSimulator();

    /// Non-copyable — owns a mutex and thread.
    OtaUpdateSimulator(const OtaUpdateSimulator&)            = delete;
    OtaUpdateSimulator& operator=(const OtaUpdateSimulator&) = delete;

    // ── Core update-lifecycle API ─────────────────────────────────────────────

    /**
     * @brief  Polls the simulated update server.
     *         Transitions state: IDLE → CHECKING → (DOWNLOADING | IDLE).
     * @return Number of packages found.
     */
    int checkForUpdates();

    /**
     * @brief  Starts a background chunked download for the pending package.
     *         Transitions state: CHECKING → DOWNLOADING.
     *         Blocks until download completes (with progress updates to os).
     * @param  os  Output stream for progress bar.
     */
    void startDownload(std::ostream& os);

    /**
     * @brief  Validates checksum and signature of the downloaded package.
     *         Transitions state: DOWNLOADING → VERIFYING → (READY | FAILED).
     * @param  os  Output stream for verification log.
     * @return True if verification passed.
     */
    bool verifyPackage(std::ostream& os);

    /**
     * @brief  Writes the package to flash (simulated).
     *         Requires currentSpeed == 0.  Transitions: READY → INSTALLING
     *         → (REBOOTING → SUCCESS) | FAILED.
     * @param  currentSpeed  Live vehicle speed (km/h). Must be 0.
     * @param  os            Output stream for install log.
     * @return True if install succeeded.
     */
    bool installUpdate(double currentSpeed, std::ostream& os);

    /**
     * @brief  Rolls back the affected module to its previous known-good version.
     *         Called automatically on FAILED; also exposed for manual invocation.
     * @param  os  Output stream for rollback log.
     */
    void rollback(std::ostream& os);

    /**
     * @brief  Resets state machine to IDLE, discarding any in-progress package.
     *         Useful after SUCCESS or FAILED to begin the next cycle.
     */
    void resetToIdle();

    // ── Query API ─────────────────────────────────────────────────────────────

    /// Returns current state-machine state.
    OtaState currentState() const;

    /// Returns a snapshot of all ECU modules.
    std::vector<EcuModule> getModules() const;

    /// Returns the in-progress package (or empty optional-like struct).
    OtaPackage currentPackage() const;

    /// Returns a snapshot of aggregate OTA statistics.
    OtaStats getStats() const;

    /// Returns the last N audit-trail entries (most recent first).
    std::vector<OtaHistoryEntry> getHistory(std::size_t n = 20) const;

    // ── Display ───────────────────────────────────────────────────────────────

    /**
     * @brief  Renders the full interactive OTA management panel.
     *
     * @details Shows:
     *  - Current state badge and active package info.
     *  - ECU module fleet table (ID, name, version, status).
     *  - Aggregate OTA statistics.
     *  - Available action menu.
     *  - Audit trail (last 10 events).
     *
     * @param  os            Output stream (usually std::cout).
     * @param  currentSpeed  Live speed used for install-safety gating.
     */
    void display(std::ostream& os, double currentSpeed = 0.0) const;

    /**
     * @brief  Runs the full interactive OTA menu loop.
     * @param  os            Output stream.
     * @param  currentSpeed  Live vehicle speed for safety gating.
     */
    void runInteractiveMenu(std::ostream& os, double currentSpeed = 0.0);

private:
    // ── State ──────────────────────────────────────────────────────────────
    mutable std::mutex      otaMutex_;
    OtaState                state_{OtaState::IDLE};
    std::vector<EcuModule>  modules_;
    OtaPackage              activePackage_;
    bool                    hasActivePackage_{false};
    OtaStats                stats_;
    std::deque<OtaHistoryEntry> history_;

    // ── Private helpers ────────────────────────────────────────────────────

    /// Appends a timestamped entry to the audit trail.
    void logEvent(const std::string& moduleId, OtaState s, const std::string& detail);

    /// Returns a wall-clock timestamp string "YYYY-MM-DD HH:MM:SS".
    static std::string nowStr();

    /// Generates a deterministic pseudo-checksum for simulation.
    static std::string generateChecksum(const std::string& packageId, const FirmwareVersion& v);

    /// Generates a fake RSA signature token.
    static std::string generateSignature(const std::string& packageId);

    /// Generates a fake release-notes blurb for a module update.
    static std::string generateReleaseNotes(const std::string& moduleId,
                                            const FirmwareVersion& from,
                                            const FirmwareVersion& to);

    /// Finds a module by its ID string.  Returns nullptr if not found.
    EcuModule* findModule(const std::string& id);

    /// Renders a text progress bar string "[████░░░░]  75%".
    static std::string progressBar(double pct, int width = 30);
};

#endif // OTA_UPDATE_SIMULATOR_HPP
