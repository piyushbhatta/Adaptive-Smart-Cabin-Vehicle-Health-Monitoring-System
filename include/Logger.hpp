/**
 * @file    Logger.hpp
 * @brief   RAII file-based event logger for the Smart Vehicle ECU.
 *
 * @details EventLogger opens a log file in append mode at construction and
 *          flushes/closes it in the destructor, guaranteeing no log entries
 *          are lost even on unexpected termination paths.
 *
 *          All write operations are mutex-protected for safe concurrent access
 *          from the sensor, monitoring, and dashboard threads.
 *
 *          A templated log() method satisfies the hackathon template-usage
 *          requirement while keeping the interface flexible for future types.
 *
 *          Automotive mapping: DEM – Diagnostic Event Manager / NvM log.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.1
 */

#pragma once
#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <fstream>
#include <mutex>
#include <vector>
#include <functional>
#include <sstream>

class Alert;
enum class AlertSeverity;

// ─────────────────────────────────────────────────────────────────────────────
// EventLogger
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   EventLogger
 * @brief   Thread-safe, RAII file-based event logger.
 *
 * @details Responsibilities:
 *          - Opens the log file on construction (throws on failure — RAII).
 *          - Writes session start/end markers for replay analysis.
 *          - Provides severity-tagged log methods: logInfo, logWarning, logCritical.
 *          - Maintains an in-memory copy of all log entries for search/filter.
 *          - Exposes lambda-based search (std::copy_if) satisfying the hackathon
 *            STL + lambda requirement.
 *
 * @note    Console output is intentionally absent from EventLogger.
 *          All terminal rendering is the exclusive responsibility of the
 *          Dashboard thread to prevent interleaved output.
 *
 * @note    EventLogger is non-copyable; it is owned via std::shared_ptr in main.
 */
class EventLogger {
public:
    /**
     * @brief  Opens the log file in append mode and writes a session-start marker.
     *
     * @param  filepath  Path to the log file (e.g. "logs/vehicle_log.txt").
     * @throws std::runtime_error  if the file cannot be opened.
     */
    explicit EventLogger(const std::string& filepath);

    /**
     * @brief  Destructor — flushes, writes session-end marker, and closes file.
     *
     * @details Guarantees all buffered log data is persisted.  RAII resource
     *          management ensures the file handle is released even if an exception
     *          propagates through the call stack.
     */
    ~EventLogger();

    EventLogger(const EventLogger&)            = delete; ///< Non-copyable.
    EventLogger& operator=(const EventLogger&) = delete; ///< Non-assignable.

    // ── Templated log method (template usage requirement) ────────────────────

    /**
     * @brief  Generic templated log method — writes any streamable value to file.
     *
     * @details Acquires the mutex, formats "[TIMESTAMP] [TAG] value" using a
     *          std::ostringstream, then delegates to writeToFile().  No console
     *          output is produced.
     *
     * @tparam T    Any type that supports operator<< on std::ostream.
     * @param  tag  Descriptive tag string (e.g. "SPEED", "HEARTBEAT").
     * @param  value  Value to log — converted via operator<<.
     */
    template<typename T>
    void log(const std::string& tag, const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream oss;
        oss << "[" << currentTimestamp() << "] [" << tag << "] " << value;
        writeToFile(oss.str());
    }

    // ── Severity-tagged log methods ──────────────────────────────────────────

    /**
     * @brief  Logs an informational message (tag: INFO).
     * @param  msg  Message string to log.
     */
    void logInfo    (const std::string& msg);

    /**
     * @brief  Logs a warning message (tag: WARNING).
     * @param  msg  Message string to log.
     */
    void logWarning (const std::string& msg);

    /**
     * @brief  Logs a critical message (tag: CRITICAL).
     * @param  msg  Message string to log.
     */
    void logCritical(const std::string& msg);

    /**
     * @brief  Logs a fully-formed Alert object using its operator<< representation.
     * @param  alert  Alert to serialise and append to the log file.
     */
    void logAlert   (const Alert& alert);

    // ── Lambda-based search (STL + lambda requirement) ───────────────────────

    /**
     * @brief  Returns all log entries matching a custom predicate.
     *
     * @details Uses std::copy_if with the supplied lambda.  Thread-safe
     *          via the internal mutex.
     *
     * @param  predicate  Lambda / callable returning bool given a log-entry string.
     * @return Vector of matching log-entry strings (copies).
     */
    std::vector<std::string> searchEntries(
        const std::function<bool(const std::string&)>& predicate) const;

    /**
     * @brief  Returns all log entries that contain @p keyword as a substring.
     *
     * @details Delegates to searchEntries() with a substring-match lambda.
     * @param  keyword  Case-sensitive search string.
     * @return Vector of matching log-entry strings.
     */
    std::vector<std::string> searchByKeyword(const std::string& keyword) const;

    /**
     * @brief  Returns a copy of the entire in-memory log entry vector.
     * @return All log entries in chronological order.
     */
    std::vector<std::string> getAllEntries() const { return logEntries_; }

    /**
     * @brief  Returns true if the backing log file is currently open.
     * @return true if open, false if the file could not be opened or was closed.
     */
    bool isOpen() const { return logFile_.is_open(); }

private:
    std::ofstream            logFile_;    ///< RAII-managed output file stream.
    mutable std::mutex       mutex_;      ///< Guards all read/write operations.
    std::vector<std::string> logEntries_; ///< In-memory mirror for search/filter.
    std::string              filepath_;   ///< Stored path for diagnostics.

    // ── Cached timestamp — rebuilt at most once per second ────────────────
    mutable char        cachedTs_[20] = {};  ///< "YYYY-MM-DD HH:MM:SS\0" buffer.
    mutable std::time_t lastTsSec_    = 0;   ///< Second at which cache was last filled.

    /**
     * @brief  Returns the current wall-clock time as "YYYY-MM-DD HH:MM:SS".
     * @details Result is cached and refreshed only when the second changes,
     *          eliminating repeated system_clock + localtime + strftime calls.
     * @return Pointer to an internal 19-char buffer (valid until next call).
     */
    const char* fastTimestamp() const;

    /**
     * @brief  Returns the current wall-clock time as "YYYY-MM-DD HH:MM:SS".
     * @return Timestamp string.
     */
    static std::string currentTimestamp();

    /**
     * @brief  Writes @p entry to the log file and appends it to logEntries_.
     *
     * @details Caller must hold mutex_ before invoking this method.
     * @param  entry  Pre-formatted log line (no trailing newline needed).
     */
    void writeToFile(const std::string& entry);
};

#endif // LOGGER_HPP
