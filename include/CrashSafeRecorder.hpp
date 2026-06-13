/**
 * @file    CrashSafeRecorder.hpp
 * @brief   Crash-safe event recorder for the Smart Vehicle ECU.
 *
 * @details Models the automotive EDR (Event Data Recorder) / DSSAD
 *          (Data Storage System for Automated Driving) as defined by:
 *          - ISO 15764  : Extended Data Link Security / EDR
 *          - UNECE R160 : EDR requirements for light motor vehicles
 *          - AUTOSAR    : NvM (Non-volatile Memory Manager) block semantics
 *
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │              CRASH-SAFE RECORDER — ARCHITECTURE                      │
 *  ├─────────────────────────────────────────────────────────────────────┤
 *  │  Two-layer persistence strategy:                                     │
 *  │                                                                      │
 *  │  Layer 1 — Ring buffer (RAM, lock-free write path)                   │
 *  │    • Circular array of RING_CAPACITY EventRecord slots.              │
 *  │    • Written every sensor cycle from the hot path.                   │
 *  │    • Never blocks; oldest entry silently overwritten on overflow.    │
 *  │    • Analogous to AUTOSAR NvM RAM mirror block.                      │
 *  │                                                                      │
 *  │  Layer 2 — Persistent EDR file (crash snapshot)                      │
 *  │    • flush() atomically commits the ring to "logs/edr_snapshot.bin". │
 *  │    • Uses write-then-rename (atomic on POSIX) to prevent corruption. │
 *  │    • CRC-32 footer validates every persisted block on reload.        │
 *  │    • Analogous to AUTOSAR NvM write-all / EEPROM emulation.          │
 *  │                                                                      │
 *  │  Crash detection heuristics (R160 §6.2 inspired):                    │
 *  │    • Sudden deceleration  > DECEL_THRESHOLD g-equivalent km/h/cycle  │
 *  │    • Engine temp spike    > TEMP_SPIKE_THRESHOLD °C in one cycle      │
 *  │    • Critical alert fired while speed > CRASH_SPEED_THRESHOLD km/h   │
 *  │    • Voltage drop         > VOLT_DROP_THRESHOLD V in one cycle        │
 *  └─────────────────────────────────────────────────────────────────────┘
 *
 *  Stored event fields (R160 §5 mandatory data set):
 *  ┌─────────────────────┬───────────────────────────────────────────────┐
 *  │  Field              │  Description                                  │
 *  ├─────────────────────┼───────────────────────────────────────────────┤
 *  │  timestamp          │  Wall-clock "YYYY-MM-DD HH:MM:SS"             │
 *  │  sequenceId         │  Monotonic 64-bit event counter               │
 *  │  eventType          │  NORMAL / WARNING / CRASH / FAULT / STARTUP   │
 *  │  speed              │  Vehicle speed (km/h)                         │
 *  │  engineTemp         │  Engine temperature (°C)                      │
 *  │  batteryVolt        │  Battery voltage (V)                          │
 *  │  tirePressure       │  Tire pressure (PSI)                          │
 *  │  doorOpen           │  Door state (bool)                            │
 *  │  beltLocked         │  Seatbelt state (bool)                        │
 *  │  activeAlertCodes   │  Comma-separated alert codes at event time    │
 *  │  deltaSpeed         │  Speed change vs previous cycle (km/h)        │
 *  │  deltaTemp          │  Temp change vs previous cycle (°C)           │
 *  │  triggerReason      │  Human-readable crash/warning trigger string  │
 *  └─────────────────────┴───────────────────────────────────────────────┘
 *
 *  Automotive mapping:
 *  - EventRecord        → EDR data element (R160 §5)
 *  - CrashSafeRecorder  → EDR + NvM Manager (AUTOSAR SWS_NvM)
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef CRASH_SAFE_RECORDER_HPP
#define CRASH_SAFE_RECORDER_HPP

#include "AlertEvaluator.hpp"   // SensorData
#include "Alert.hpp"            // AlertManager

#include <array>
#include <string>
#include <vector>
#include <ostream>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
//  EventType  —  classification of each recorded event
// ─────────────────────────────────────────────────────────────────────────────

enum class EventType : uint8_t {
    STARTUP  = 0,  ///< Recorder initialised / ECU power-on.
    NORMAL   = 1,  ///< Routine sensor snapshot (periodic recording).
    WARNING  = 2,  ///< Active WARNING-level alert at record time.
    CRASH    = 3,  ///< Crash heuristic triggered (see detection thresholds).
    FAULT    = 4,  ///< Hardware fault or sensor out-of-range.
    SHUTDOWN = 5,  ///< Clean ECU shutdown marker.
};

/// Returns a fixed-width display string for an EventType.
std::string eventTypeToString(EventType t);

/// Returns the ANSI colour appropriate for an EventType.
const char* eventTypeColour(EventType t);

// ─────────────────────────────────────────────────────────────────────────────
//  EventRecord  —  one stored EDR data element
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct EventRecord
 * @brief  One immutable entry in the EDR ring buffer.
 *
 * @details Sized to a round number of bytes so that the ring-buffer array
 *          does not require padding and the CRC calculation is deterministic.
 *          All numeric fields use fixed-width types for portability.
 */
struct EventRecord {
    // ── Identity ──────────────────────────────────────────────────────────
    uint64_t    sequenceId{0};          ///< Monotonic counter; 0 = empty slot.
    char        timestamp[20]{};        ///< "YYYY-MM-DD HH:MM:SS\0"
    EventType   eventType{EventType::NORMAL};

    // ── Sensor snapshot (R160 §5 mandatory data) ──────────────────────────
    float       speed{0.0f};            ///< km/h
    float       engineTemp{0.0f};       ///< °C
    float       batteryVolt{0.0f};      ///< V
    float       tirePressure{0.0f};     ///< PSI
    uint8_t     doorOpen{0};            ///< 1 = open
    uint8_t     beltLocked{0};          ///< 1 = locked

    // ── Delta values (change vs previous cycle) ───────────────────────────
    float       deltaSpeed{0.0f};       ///< km/h / cycle
    float       deltaTemp{0.0f};        ///< °C   / cycle

    // ── Alert state at record time ─────────────────────────────────────────
    char        alertCodes[128]{};      ///< Comma-separated active alert codes.

    // ── Trigger reason (crash / warning classification) ───────────────────
    char        triggerReason[64]{};    ///< Human-readable trigger string.

    // ── Integrity ─────────────────────────────────────────────────────────
    uint32_t    crc32{0};               ///< CRC-32/ISO-HDLC over all preceding fields.

    /// Returns true if this slot is populated (sequenceId != 0).
    bool isValid() const { return sequenceId != 0; }

    /// Renders a compact single-line summary.
    std::string toOneLine() const;
};

// ─────────────────────────────────────────────────────────────────────────────
//  RecorderStats  —  aggregate counters for the display panel
// ─────────────────────────────────────────────────────────────────────────────

struct RecorderStats {
    uint64_t totalRecorded{0};   ///< Events written to the ring since start.
    uint64_t crashEvents{0};     ///< CRASH-type events recorded.
    uint64_t warningEvents{0};   ///< WARNING-type events recorded.
    uint64_t faultEvents{0};     ///< FAULT-type events recorded.
    uint64_t flushCount{0};      ///< Number of successful NvM flushes.
    uint64_t overwriteCount{0};  ///< Ring-buffer overflow / overwrite count.
};

// ─────────────────────────────────────────────────────────────────────────────
//  CrashSafeRecorder
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  CrashSafeRecorder
 * @brief  Two-layer crash-safe EDR: RAM ring buffer + persistent NvM flush.
 *
 * @details
 *  Normal operation:
 *    record() is called every sensor cycle.  It evaluates crash heuristics,
 *    classifies the event, serialises the EventRecord into the ring buffer,
 *    and — on a CRASH or FAULT event — immediately triggers flushToNvm().
 *
 *  Crash-safe flush:
 *    flushToNvm() writes to a temp file then renames it over the canonical
 *    EDR file, guaranteeing the old snapshot is never left in a corrupt
 *    half-written state.  Each block is protected by a CRC-32 footer.
 *
 *  Replay on startup:
 *    loadFromNvm() reads the canonical EDR file back into the ring, verifying
 *    the CRC of every record and discarding corrupt entries.
 *
 *  Thread-safety:
 *    ring_ and stats_ are protected by mutex_.  record() and display() are
 *    both callable from any thread.
 */
class CrashSafeRecorder {
public:
    // ── Tunable constants ─────────────────────────────────────────────────────
    static constexpr std::size_t RING_CAPACITY         = 64;    ///< Max events in RAM ring.
    static constexpr double      DECEL_THRESHOLD        = 20.0;  ///< Speed drop (km/h/cycle) → CRASH.
    static constexpr double      TEMP_SPIKE_THRESHOLD   = 15.0;  ///< Temp rise  (°C/cycle)   → CRASH.
    static constexpr double      CRASH_SPEED_THRESHOLD  = 5.0;   ///< Min speed (km/h) for crash detect.
    static constexpr double      VOLT_DROP_THRESHOLD    = 1.5;   ///< Volt drop (V/cycle)      → FAULT.
    static constexpr int         PERIODIC_FLUSH_CYCLES  = 30;    ///< Flush every N normal records.
    static constexpr const char* EDR_FILE               = "logs/edr_snapshot.bin";
    static constexpr const char* EDR_TMP_FILE           = "logs/edr_snapshot.tmp";
    static constexpr const char* EDR_LOG_FILE           = "logs/edr_events.txt";

    /**
     * @brief  Constructs the recorder, writes a STARTUP event, and attempts
     *         to load any prior EDR snapshot from NvM.
     */
    CrashSafeRecorder();

    /**
     * @brief  Destructor — writes a SHUTDOWN event and performs a final NvM flush.
     */
    ~CrashSafeRecorder();

    // Non-copyable — owns a mutex and file handles.
    CrashSafeRecorder(const CrashSafeRecorder&)            = delete;
    CrashSafeRecorder& operator=(const CrashSafeRecorder&) = delete;

    // ── Core recording API ────────────────────────────────────────────────────

    /**
     * @brief  Records one sensor cycle.  The primary hot-path entry point.
     *
     * @details Steps performed:
     *  1. Compute delta values (speed change, temp change) vs last cycle.
     *  2. Run crash heuristics:
     *       - Sudden decel  > DECEL_THRESHOLD     → CRASH
     *       - Temp spike    > TEMP_SPIKE_THRESHOLD → CRASH
     *       - Voltage drop  > VOLT_DROP_THRESHOLD  → FAULT
     *       - Critical alert while moving          → CRASH
     *       - Any WARNING alert                    → WARNING
     *       - Otherwise                            → NORMAL
     *  3. Serialise into the next ring slot (overwriting oldest if full).
     *  4. On CRASH or FAULT: immediately call flushToNvm().
     *  5. On every PERIODIC_FLUSH_CYCLES normal records: call flushToNvm().
     *
     * @param  s        Live sensor snapshot for this cycle.
     * @param  alertMgr Alert manager for active alert state at record time.
     */
    void record(const SensorData& s, const AlertManager& alertMgr);

    /**
     * @brief  Manually inject a FAULT event (e.g. from an external fault handler).
     *
     * @param  reason  Human-readable reason string.
     * @param  s       Sensor state at fault time.
     */
    void recordFault(const std::string& reason, const SensorData& s);

    /**
     * @brief  Writes a SHUTDOWN marker then flushes to NvM.
     *         Called automatically by the destructor; also exposed for
     *         graceful-shutdown sequences.
     */
    void recordShutdown();

    // ── Persistence API ───────────────────────────────────────────────────────

    /**
     * @brief  Atomically persists the ring buffer to EDR_FILE.
     *
     * @details Writes to EDR_TMP_FILE first (guarantees old snapshot is intact
     *          if power is lost mid-write), then renames to EDR_FILE.
     *          Appends a human-readable mirror to EDR_LOG_FILE.
     *
     * @return true if the flush completed without error.
     */
    bool flushToNvm();

    /**
     * @brief  Loads a previously persisted snapshot from EDR_FILE into the ring.
     *
     * @details Validates the CRC-32 of each record; silently skips corrupt
     *          entries.  Valid records are inserted at the front of the ring
     *          so that the most recent live data occupies the newest slots.
     *
     * @return Number of valid records restored.
     */
    int loadFromNvm();

    // ── Query API ─────────────────────────────────────────────────────────────

    /**
     * @brief  Returns all populated ring-buffer entries, newest first.
     *         Uses std::copy_if + lambda (STL/lambda requirement).
     */
    std::vector<EventRecord> getEvents() const;

    /**
     * @brief  Returns only events matching a predicate.
     *         Uses std::copy_if with the supplied lambda.
     * @param  pred  Callable(const EventRecord&) → bool.
     */
    std::vector<EventRecord> filterEvents(
        const std::function<bool(const EventRecord&)>& pred) const;

    /**
     * @brief  Returns only CRASH-type events.
     */
    std::vector<EventRecord> getCrashEvents() const;

    /**
     * @brief  Returns aggregate statistics.
     */
    RecorderStats getStats() const;

    /**
     * @brief  Returns the sequence ID of the most recently recorded event.
     */
    uint64_t lastSequenceId() const;

    // ── Display ───────────────────────────────────────────────────────────────

    /**
     * @brief  Renders the full EDR management panel to the given stream.
     *
     * @details Panels shown:
     *  - Header: recorder config (ring capacity, thresholds, NvM file).
     *  - Statistics: total/crash/warning/fault counts and flush count.
     *  - Ring buffer: last N events in reverse-chronological order.
     *  - Crash event spotlight: detailed view of each CRASH entry.
     *  - Threshold legend.
     *
     * @param  os  Destination stream (usually std::cout).
     */
    void display(std::ostream& os) const;

    /**
     * @brief  Renders a compact single-line EDR summary for the dashboard footer.
     *
     * @details Format:
     *          "EDR: 42 events | 2 crashes | Last: 2025-06-01 10:23:45 [CRASH]"
     */
    void displayCompact(std::ostream& os) const;

    /**
     * @brief  Clears the ring buffer and resets statistics.
     *         Does NOT erase the NvM file (safety requirement: EDR data persists).
     */
    void clearRing();

private:
    // ── Ring buffer ────────────────────────────────────────────────────────
    using Ring = std::array<EventRecord, RING_CAPACITY>;

    mutable std::mutex  mutex_;
    Ring                ring_{};           ///< Fixed-size circular event store.
    std::size_t         head_{0};          ///< Next write slot index.
    std::size_t         count_{0};         ///< Populated slot count (≤ RING_CAPACITY).
    std::atomic<uint64_t> seqCounter_{0};  ///< Monotonic sequence ID generator.
    RecorderStats       stats_;

    // ── Crash heuristic state (previous-cycle values) ──────────────────────
    float               prevSpeed_{0.0f};
    float               prevTemp_{0.0f};
    float               prevVolt_{0.0f};
    bool                prevDataValid_{false};

    // ── Periodic flush counter ─────────────────────────────────────────────
    int                 normalCycleCount_{0};

    // ── Private helpers ────────────────────────────────────────────────────

    /**
     * @brief  Writes a fully-formed EventRecord into the ring.
     *         Must be called with mutex_ held.
     * @param  rec  Record to store (crc32 field is computed here).
     */
    void writeToRing(EventRecord& rec);

    /**
     * @brief  Builds an EventRecord from a sensor snapshot.
     *         Does NOT lock — caller must hold mutex_.
     */
    EventRecord buildRecord(const SensorData& s,
                            const AlertManager& alertMgr,
                            EventType type,
                            float deltaSpeed,
                            float deltaTemp,
                            const std::string& triggerReason);

    /**
     * @brief  Computes CRC-32/ISO-HDLC over the first @p len bytes of @p data.
     * @return 32-bit CRC value.
     */
    static uint32_t crc32(const uint8_t* data, std::size_t len);

    /**
     * @brief  Returns elapsed milliseconds since construction (for display).
     */
    uint64_t elapsedMs() const;

    /**
     * @brief  Returns wall-clock timestamp string "YYYY-MM-DD HH:MM:SS".
     */
    static std::string nowStr();

    /**
     * @brief  Fills @p buf (20 bytes) with the current timestamp.
     *         Zero-copies into EventRecord::timestamp without heap allocation.
     */
    static void fillTimestamp(char* buf);

    /**
     * @brief  Copies alert codes from the manager into a comma-separated buffer.
     * @param  alertMgr  Source of active alerts.
     * @param  buf       Destination buffer.
     * @param  bufLen    Size of destination buffer.
     */
    static void buildAlertCodeString(const AlertManager& alertMgr,
                                     char* buf, std::size_t bufLen);

    std::chrono::steady_clock::time_point startTime_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Global recorder instance (definition in CrashSafeRecorder.cpp)
// ─────────────────────────────────────────────────────────────────────────────
extern CrashSafeRecorder g_crashRecorder;

#endif // CRASH_SAFE_RECORDER_HPP
