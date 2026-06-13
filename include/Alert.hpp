/**
 * @file    Alert.hpp
 * @brief   Alert and AlertManager classes for the Smart Vehicle ECU.
 *
 * @details Provides:
 *          - AlertSeverity enum class (INFO / WARNING / CRITICAL).
 *          - Alert value type with operator<< for formatted terminal/file output.
 *          - AlertManager that maintains active + historical alert collections
 *            and exposes STL / lambda-based search and filter operations.
 *
 *          Automotive mapping:
 *          - Alert        → DTC (Diagnostic Trouble Code) event record.
 *          - AlertManager → Vehicle Health Manager / Fault Memory.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.1
 */

#pragma once
#ifndef ALERT_HPP
#define ALERT_HPP

#include <string>
#include <vector>
#include <map>
#include <set>
#include <ostream>
#include <chrono>
#include <ctime>

// ─────────────────────────────────────────────────────────────────────────────
// AlertSeverity
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @enum  AlertSeverity
 * @brief Ordered severity levels for vehicle alerts.
 *
 * @details The underlying integer values allow ordered comparison:
 *          INFO (0) < WARNING (1) < CRITICAL (2).
 *          Used in AlertManager::filterHistory() to retrieve all alerts
 *          at or above a chosen severity floor.
 */
enum class AlertSeverity {
    INFO     = 0, ///< Informational — no driver action required.
    WARNING  = 1, ///< Warning      — driver should take notice.
    CRITICAL = 2  ///< Critical     — immediate action required.
};

/**
 * @brief  Converts an AlertSeverity enum to a fixed-width display string.
 * @param  sev  Severity level to convert.
 * @return Padded 8-character string: "INFO    ", "WARNING ", or "CRITICAL".
 */
std::string severityToString(AlertSeverity sev);

// ─────────────────────────────────────────────────────────────────────────────
// Alert
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   Alert
 * @brief   Immutable value type representing a single vehicle alert event.
 *
 * @details Each Alert captures:
 *          - A severity level (AlertSeverity).
 *          - A unique string code (e.g. "LOW_TIRE_PRESSURE") used as a key
 *            in AlertManager to prevent duplicate active alerts.
 *          - A human-readable message with the current sensor value embedded.
 *          - A wall-clock timestamp (YYYY-MM-DD HH:MM:SS) at creation time.
 *
 *          Alerts are value types: copyable, movable, and comparable.
 *          operator< orders by descending severity then ascending code so
 *          that CRITICAL alerts sort first in containers.
 *
 * @note    The static totalAlertCount_ increments on every construction,
 *          providing a monotonically increasing event counter for diagnostics.
 */
class Alert {
public:
    /**
     * @brief  Constructs an Alert and captures the current timestamp.
     * @param  sev   Severity level of the alert.
     * @param  code  Unique string code identifying the alert condition.
     * @param  msg   Descriptive message with embedded sensor value.
     */
    Alert(AlertSeverity sev, const std::string& code, const std::string& msg);

    // Rule-of-five: Alert is a value type — all five are defaulted.
    Alert(const Alert&)            = default; ///< Copy constructor.
    Alert& operator=(const Alert&) = default; ///< Copy assignment.
    Alert(Alert&&)                 = default; ///< Move constructor.
    Alert& operator=(Alert&&)      = default; ///< Move assignment.
    ~Alert()                       = default; ///< Destructor.

    // ── Operator overloads ───────────────────────────────────────────────────

    /**
     * @brief  Stream insertion operator — formats alert for terminal/file output.
     *
     * @details Produces: "[TIMESTAMP] [SEVERITY] CODE                       : message"
     *          Example : "[2025-06-01 10:23:45] [WARNING ] LOW_TIRE_PRESSURE : 23.4 PSI"
     * @param  os  Output stream.
     * @param  a   Alert to format.
     * @return Reference to os (enables chaining).
     */
    friend std::ostream& operator<<(std::ostream& os, const Alert& a);

    /**
     * @brief  Less-than comparison: CRITICAL < WARNING < INFO, then by code.
     * @param  other  Alert to compare against.
     * @return true if this alert should sort before other.
     */
    bool operator<(const Alert& other) const;

    /**
     * @brief  Equality comparison by code string only (severity/message ignored).
     * @param  other  Alert to compare against.
     * @return true if both alerts have identical code strings.
     */
    bool operator==(const Alert& other) const;

    // ── Accessors ────────────────────────────────────────────────────────────

    AlertSeverity      getSeverity()  const { return severity_; }  ///< Returns severity level.
    const std::string& getCode()      const { return code_; }      ///< Returns unique alert code.
    const std::string& getMessage()   const { return message_; }   ///< Returns descriptive message.
    const std::string& getTimestamp() const { return timestamp_; } ///< Returns creation timestamp.

    // ── Static counter ───────────────────────────────────────────────────────

    /**
     * @brief  Returns the cumulative number of Alert objects ever constructed.
     * @return Total alert count (monotonically increasing).
     */
    static int getTotalAlertCount() { return totalAlertCount_; }

private:
    AlertSeverity severity_;        ///< Severity level.
    std::string   code_;            ///< Unique condition identifier.
    std::string   message_;         ///< Human-readable description with sensor value.
    std::string   timestamp_;       ///< Wall-clock timestamp at construction.
    static int    totalAlertCount_; ///< Process-wide Alert construction counter.

    /**
     * @brief  Generates the current wall-clock timestamp string.
     * @return String in "YYYY-MM-DD HH:MM:SS" format.
     */
    static std::string makeTimestamp();
};

// ─────────────────────────────────────────────────────────────────────────────
// AlertManager
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   AlertManager
 * @brief   Manages the lifecycle of vehicle alerts: raise, clear, search.
 *
 * @details Maintains two orthogonal collections:
 *          - activeAlerts_   : currently active (uncleared) alerts.
 *          - alertHistory_   : append-only historical log of every alert ever raised.
 *          - activeByCode_   : map from code → Alert for O(log N) lookup/dedup.
 *
 *          Additionally provides a pending-notification queue:
 *          - The monitoring thread pushes notification lines via raiseAlert().
 *          - The dashboard thread drains the queue atomically inside its own
 *            render block to prevent interleaved console output.
 *
 *          AlertManager is non-copyable (singleton-style ownership by main).
 *
 * @note    Automotive mapping: Fault Memory / DEM (Diagnostic Event Manager).
 */
class AlertManager {
public:
    AlertManager() = default; ///< Default constructor.

    AlertManager(const AlertManager&)            = delete; ///< Non-copyable.
    AlertManager& operator=(const AlertManager&) = delete; ///< Non-assignable.

    // ── Core alert lifecycle ─────────────────────────────────────────────────

    /**
     * @brief  Raises an alert if not already active.
     *
     * @details If the alert code is already in activeByCode_, the call is a
     *          no-op and returns false (duplicate suppression).
     *          On a new alert, it is added to both activeAlerts_ and
     *          alertHistory_, and a formatted notification line is pushed to
     *          pendingNotifications_ for the dashboard thread to display.
     *
     * @param  sev   Severity of the new alert.
     * @param  code  Unique string key for deduplication.
     * @param  msg   Descriptive message string.
     * @return true if a NEW alert was added; false if already active.
     */
    bool raiseAlert(AlertSeverity sev, const std::string& code, const std::string& msg);

    /**
     * @brief  Clears (deactivates) an alert by its unique code.
     *
     * @details Removes the alert from activeAlerts_ and activeByCode_.
     *          The alert remains in alertHistory_ for post-drive analysis.
     *          If the code is not currently active, this is a safe no-op.
     *
     * @param  code  Unique alert code to clear.
     */
    void clearAlert(const std::string& code);

    // ── Read access ──────────────────────────────────────────────────────────

    /// @brief Returns a const reference to the active alerts vector.
    const std::vector<Alert>& getActiveAlerts() const { return activeAlerts_; }

    /// @brief Returns a const reference to the full alert history vector.
    const std::vector<Alert>& getAlertHistory() const { return alertHistory_; }

    /// @brief Returns true if there is at least one active alert.
    bool hasActiveAlerts() const { return !activeAlerts_.empty(); }

    /// @brief Returns the number of currently active alerts.
    int  getActiveCount()  const { return static_cast<int>(activeAlerts_.size()); }

    /**
     * @brief  Returns true if the given alert code is currently active.
     * @param  code  Alert code to check.
     * @return true if active, false otherwise.
     */
    bool isActive(const std::string& code) const {
        return activeByCode_.find(code) != activeByCode_.end();
    }

    // ── Lambda-based search / filter (STL requirement) ───────────────────────

    /**
     * @brief  Filters alert history to those at or above a minimum severity.
     *
     * @details Uses std::copy_if with a lambda predicate.
     * @param  minSev  Minimum severity floor.
     * @return New vector containing only matching alerts (copies).
     */
    std::vector<Alert> filterHistory(AlertSeverity minSev) const;

    /**
     * @brief  Searches alert history for alerts whose code or message contains
     *         the given keyword (case-sensitive substring match).
     *
     * @details Uses std::copy_if with a lambda predicate.
     * @param  keyword  Substring to search for.
     * @return New vector containing only matching alerts (copies).
     */
    std::vector<Alert> searchHistory(const std::string& keyword) const;

    // ── Notification queue (dashboard/monitoring thread handoff) ─────────────

    /**
     * @brief  Pushes a pre-formatted notification line into the pending queue.
     * @param  line  ANSI-coloured string to display in the dashboard frame.
     */
    void pushNotification(const std::string& line);

    /**
     * @brief  Atomically drains and returns all pending notification lines.
     *
     * @details Called by the dashboard thread inside its render lock.  Swaps
     *          the internal vector with an empty one in O(1) and returns the
     *          drained contents.
     * @return Vector of pending notification strings (may be empty).
     */
    std::vector<std::string> drainNotifications();

    // ── Display helpers ──────────────────────────────────────────────────────

    /// @brief Prints all active alerts to stdout (plain, no box).
    void displayActiveAlerts() const;

    /// @brief Prints full alert history to stdout (plain, no box).
    void displayAlertHistory() const;

    /// @brief Clears active alerts and the active-by-code map (history preserved).
    void clearAllAlerts();

private:
    std::vector<Alert>           activeAlerts_;        ///< Currently active alerts.
    std::vector<Alert>           alertHistory_;        ///< Append-only historical log.
    std::map<std::string, Alert> activeByCode_;        ///< Fast O(log N) dedup lookup.
    std::vector<std::string>     pendingNotifications_; ///< Cross-thread notification queue.
};

#endif // ALERT_HPP
