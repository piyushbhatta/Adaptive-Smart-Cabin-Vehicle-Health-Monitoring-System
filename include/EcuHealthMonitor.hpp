/**
 * @file    EcuHealthMonitor.hpp
 * @brief   ECU Health Monitor for the Smart Vehicle ECU.
 *
 * @details Tracks per-sensor fault counts, system uptime, alert rate, and
 *          an overall ECU health score (0–100).  Renders a live health
 *          summary panel and can flag sensors that have exceeded their
 *          fault-rate threshold (analogous to a DTC P-code trigger in
 *          production AUTOSAR software).
 *
 *          Automotive mapping: Diagnostic Monitor / FiM (Function Inhibition
 *          Manager) layer in an AUTOSAR BSW stack.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef ECU_HEALTH_MONITOR_HPP
#define ECU_HEALTH_MONITOR_HPP

#include <string>
#include <vector>
#include <chrono>
#include <ostream>

// ─────────────────────────────────────────────────────────────────────────────
// SensorHealthRecord
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  SensorHealthRecord
 * @brief   Per-sensor health bookkeeping entry.
 *
 * @details Tracks how many times a sensor has reported a fault condition
 *          (value outside its normal operating range) and derives a simple
 *          health percentage from fault density.
 */
struct SensorHealthRecord {
    std::string sensorName;       ///< Sensor identifier string.
    int         totalReadings{0}; ///< Total update cycles recorded.
    int         faultReadings{0}; ///< Cycles where value was outside normal range.
    bool        currentlyFaulted{false}; ///< True if latest reading was a fault.

    /**
     * @brief  Returns the fault rate as a percentage (0.0–100.0).
     * @return faultReadings / totalReadings * 100, or 0 if no readings yet.
     */
    double faultRatePercent() const;

    /**
     * @brief  Returns a health score from 0–100 (100 = fully healthy).
     * @return 100 - faultRatePercent(), clamped to [0,100].
     */
    double healthScore() const;

    /**
     * @brief  Returns a status string: "OK", "DEGRADED", or "FAULT".
     */
    std::string statusString() const;
};

// ─────────────────────────────────────────────────────────────────────────────
// EcuHealthMonitor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  EcuHealthMonitor
 * @brief  Aggregates sensor health data and computes an overall ECU score.
 *
 * @details The monitor is updated once per sensor-thread cycle.  It exposes:
 *  - Per-sensor fault tracking via recordReading().
 *  - An overall ECU health score (weighted mean of per-sensor scores).
 *  - System uptime (wall-clock seconds since construction).
 *  - Alert rate (alerts per minute derived from supplied counters).
 *  - A formatted terminal display panel.
 *
 * Thread-safety: This class is NOT internally synchronised.  The caller
 * must hold the global mutex before calling any mutating method.
 *
 * Usage (sensor thread):
 * @code
 *   g_ecuHealth.recordReading("Engine Temp",  temp  >= 60 && temp  <= 130);
 *   g_ecuHealth.recordReading("Speed",        speed >= 0  && speed <= 160);
 * @endcode
 */
class EcuHealthMonitor {
public:
    /// Fault rate (%) above which a sensor is considered DEGRADED.
    static constexpr double DEGRADED_THRESHOLD = 15.0;
    /// Fault rate (%) above which a sensor is considered FAULT.
    static constexpr double FAULT_THRESHOLD    = 35.0;

    /**
     * @brief  Constructs the monitor, recording the start time.
     *
     * @param  sensorNames  Names of sensors to track (in display order).
     */
    explicit EcuHealthMonitor(const std::vector<std::string>& sensorNames);

    // ── Update API ────────────────────────────────────────────────────────────

    /**
     * @brief  Records one sensor reading cycle.
     *
     * @param  sensorName  Must match a name passed to the constructor.
     * @param  isNormal    true if the reading is within the normal range.
     */
    void recordReading(const std::string& sensorName, bool isNormal);

    /**
     * @brief  Increments the internal alert counter (call when a new alert fires).
     */
    void incrementAlertCount();

    // ── Query API ─────────────────────────────────────────────────────────────

    /**
     * @brief  Returns the overall ECU health score (0–100).
     *
     * @details Computed as the unweighted mean of all per-sensor health scores.
     *          A score below 70 is DEGRADED; below 40 is CRITICAL.
     */
    double overallHealthScore() const;

    /**
     * @brief  Returns system uptime in whole seconds.
     */
    long uptimeSeconds() const;

    /**
     * @brief  Returns a human-readable uptime string ("MM:SS").
     */
    std::string uptimeString() const;

    /**
     * @brief  Returns total alerts fired since construction.
     */
    int totalAlerts() const { return totalAlerts_; }

    /**
     * @brief  Returns the alert rate in alerts/minute.
     *
     * @details Computed as totalAlerts / (uptimeSeconds / 60.0).
     */
    double alertRatePerMinute() const;

    /**
     * @brief  Returns a status label for the overall ECU health.
     * @return "HEALTHY", "DEGRADED", or "CRITICAL".
     */
    std::string overallStatusString() const;

    /**
     * @brief  Returns const reference to a specific sensor's health record.
     * @param  sensorName  Name to look up.
     * @throws std::out_of_range if not found.
     */
    const SensorHealthRecord& getSensorRecord(const std::string& sensorName) const;

    /**
     * @brief  Returns all sensor health records.
     */
    const std::vector<SensorHealthRecord>& getAllRecords() const { return records_; }

    // ── Display ───────────────────────────────────────────────────────────────

    /**
     * @brief  Renders the full ECU health panel to stdout.
     *
     * @details Displays:
     *  - Overall health score bar (colour-coded).
     *  - Uptime, total alerts, and alert rate.
     *  - Per-sensor health table with fault rates and status.
     */
    void display() const;

    /**
     * @brief  Renders a compact single-line health summary (for dashboard footer).
     *
     * @details Format: "ECU Health: 94.2 %  [HEALTHY]  Up: 02:47  Alerts: 12"
     */
    void displayCompact() const;

private:
    std::vector<SensorHealthRecord>                    records_;     ///< Per-sensor records.
    std::chrono::steady_clock::time_point              startTime_;   ///< Construction time.
    int                                                totalAlerts_{0}; ///< Lifetime alert count.

    /// Returns a pointer to a mutable record by sensor name (nullptr if not found).
    SensorHealthRecord* findRecord(const std::string& name);
};

// ─────────────────────────────────────────────────────────────────────────────
// Global ECU health monitor instance (defined in EcuHealthMonitor.cpp)
// ─────────────────────────────────────────────────────────────────────────────
extern EcuHealthMonitor g_ecuHealth;

#endif // ECU_HEALTH_MONITOR_HPP
