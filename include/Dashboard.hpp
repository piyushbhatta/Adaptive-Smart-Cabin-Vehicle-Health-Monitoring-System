/**
 * @file    Dashboard.hpp
 * @brief   Terminal dashboard renderer for the Smart Vehicle ECU.
 *
 * @details Dashboard owns shared references to all sensors, the AlertManager,
 *          and the EventLogger.  It provides the render loop called from the
 *          dashboard thread in main.cpp, producing a structured, ANSI-coloured
 *          display of live sensor readings and active alerts.
 *
 *          Vehicle statistics (max/min/avg values) have been separated into
 *          Statistics.hpp / Statistics.cpp so that the dashboard remains
 *          focused solely on live display output.
 *
 *          Automotive mapping: Instrument Cluster / HMI render layer.
 *
 * @see     Statistics.hpp  for VehicleStatistics and the statistics display.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.1
 */

#pragma once
#ifndef DASHBOARD_HPP
#define DASHBOARD_HPP

#include <string>
#include <memory>
#include <ostream>

class Sensor;
class EngineTemperatureSensor;
class BatterySensor;
class SpeedSensor;
class TirePressureSensor;
class DoorSensor;
class SeatbeltSensor;
class AlertManager;
class EventLogger;
struct VehicleStatistics;

// ─────────────────────────────────────────────────────────────────────────────
// Color  —  ANSI escape-code constants
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @namespace Color
 * @brief     ANSI terminal colour and style escape codes.
 *
 * @details   Use these constants to colour-code output:
 *            - Normal sensor values : BGREEN
 *            - Warning conditions   : BYELLOW
 *            - Critical conditions  : BRED
 *            - Section borders      : BBLUE
 *            - Sensor labels        : BCYAN
 *            - Values               : BWHITE
 *
 * @note      All sequences are standard VT100/ANSI and work on Linux terminals,
 *            macOS Terminal.app, and Windows Terminal (Win 10+).
 */
namespace Color {
    constexpr const char* RESET    = "\033[0m";
    constexpr const char* BOLD     = "\033[1m";

    // Standard text colours (soft pastel versions)
    constexpr const char* RED      = "\033[38;5;210m"; // Soft pastel red
    constexpr const char* GREEN    = "\033[38;5;114m"; // Soft pastel green
    constexpr const char* YELLOW   = "\033[38;5;222m"; // Soft amber
    constexpr const char* BLUE     = "\033[38;5;111m"; // Pastel blue
    constexpr const char* MAGENTA  = "\033[38;5;182m"; // Soft lavender
    constexpr const char* CYAN     = "\033[38;5;117m"; // Soft cyan
    constexpr const char* WHITE    = "\033[38;5;255m"; // Soft white

    // Bright (replaced with aesthetic muted variants)
    constexpr const char* BRED     = "\033[38;5;210m"; // Alert/Critical
    constexpr const char* BGREEN   = "\033[38;5;114m"; // Normal/Healthy
    constexpr const char* BYELLOW  = "\033[38;5;222m"; // Warning
    constexpr const char* BBLUE    = "\033[38;5;153m"; // Borders/Tables
    constexpr const char* BMAGENTA = "\033[38;5;183m"; // Headers
    constexpr const char* BCYAN    = "\033[38;5;152m"; // Sensor labels
    constexpr const char* BWHITE   = "\033[38;5;255m"; // Values
};

// ─────────────────────────────────────────────────────────────────────────────
// Dashboard
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   Dashboard
 * @brief   Renders live sensor readings and active alerts to the terminal.
 *
 * @details The dashboard thread calls displayFullDashboard() once per render
 *          cycle (every 3 seconds by default).  Each render:
 *          1. Prints a coloured title banner.
 *          2. Calls displayCurrentValues() — aligned sensor table.
 *          3. Calls displayActiveAlerts()  — active alert box.
 *          4. Drains and prints pending alert notifications.
 *
 *          VehicleStatistics display has been intentionally removed from the
 *          dashboard render.  Use Statistics.hpp / displayStatistics() for
 *          a dedicated statistics report at shutdown or on demand.
 *
 * @note    Dashboard is non-copyable.  All sensor and manager references are
 *          held as std::shared_ptr, consistent with the system's shared
 *          ownership model.
 */
class Dashboard {
public:
    /**
     * @brief  Constructs a Dashboard with shared references to all subsystems.
     *
     * @param  engSensor   Shared pointer to the engine temperature sensor.
     * @param  batSensor   Shared pointer to the battery voltage sensor.
     * @param  spdSensor   Shared pointer to the vehicle speed sensor.
     * @param  tireSensor  Shared pointer to the tyre pressure sensor.
     * @param  doorSensor  Shared pointer to the door status sensor.
     * @param  sbSensor    Shared pointer to the seatbelt sensor.
     * @param  alertMgr    Shared pointer to the alert manager.
     * @param  logger      Shared pointer to the event logger.
     */
    Dashboard(
        std::shared_ptr<EngineTemperatureSensor> engSensor,
        std::shared_ptr<BatterySensor>           batSensor,
        std::shared_ptr<SpeedSensor>             spdSensor,
        std::shared_ptr<TirePressureSensor>      tireSensor,
        std::shared_ptr<DoorSensor>              doorSensor,
        std::shared_ptr<SeatbeltSensor>          sbSensor,
        std::shared_ptr<AlertManager>            alertMgr,
        std::shared_ptr<EventLogger>             logger
    );

    ~Dashboard() = default; ///< Destructor — no owned resources.

    // ── Primary render methods ───────────────────────────────────────────────

    /**
     * @brief  Renders the sensor reading table to stdout.
     *
     * @details Iterates all six sensors via their virtual display() methods.
     *          Each row is coloured green (normal) or red (fault) based on
     *          threshold comparison.  Fixed-width columns ensure alignment.
     */
    void displayCurrentValues() const;

    /**
     * @brief  Renders the active-alerts box to stdout.
     *
     * @details Box header colour changes based on alert count:
     *          - 0 alerts : green  ("all systems normal").
     *          - 1–2      : yellow.
     *          - 3+       : red.
     *          Individual alert rows are coloured by severity.
     */
    void displayActiveAlerts() const;

    /**
     * @brief  Renders the full alert history box to stdout.
     *
     * @details Shown at shutdown in the summary display.  All past alerts
     *          are printed chronologically with timestamps.
     */
    void displayAlertHistory() const;

    /**
     * @brief  Executes one complete dashboard render cycle.
     *
     * @details Sequence:
     *          1. Title banner.
     *          2. displayCurrentValues().
     *          3. displayActiveAlerts().
     *          4. Drains and prints pending alert notifications.
     *
     * @note    Statistics are intentionally excluded — see Statistics.hpp.
     */
    void displayFullDashboard() const;

    // ── Statistics bridge ────────────────────────────────────────────────────

    /**
     * @brief  Updates the externally-managed VehicleStatistics struct.
     *
     * @details Called from the dashboard thread before each render so that
     *          statistics reflect the same coherent sensor snapshot that was
     *          used to evaluate alerts.
     *
     * @param  stats  Reference to the VehicleStatistics object to update.
     */
    void updateStatistics(VehicleStatistics& stats) const;

    // ── Logging ──────────────────────────────────────────────────────────────

    /**
     * @brief  Writes a single-line snapshot of all sensor values to the log file.
     *
     * @details Snapshot format:
     *          "Snapshot | Speed=XX km/h | Temp=XX C | Volt=XX V | PSI=XX |
     *           Door=OPEN/CLOSED | Belt=LOCKED/UNLOCKED | Alerts=N"
     */
    void logSnapshot() const;

private:
    std::shared_ptr<EngineTemperatureSensor> engSensor_;  ///< Engine temperature sensor.
    std::shared_ptr<BatterySensor>           batSensor_;  ///< Battery voltage sensor.
    std::shared_ptr<SpeedSensor>             spdSensor_;  ///< Vehicle speed sensor.
    std::shared_ptr<TirePressureSensor>      tireSensor_; ///< Tyre pressure sensor.
    std::shared_ptr<DoorSensor>              doorSensor_; ///< Door status sensor.
    std::shared_ptr<SeatbeltSensor>          sbSensor_;   ///< Seatbelt sensor.
    std::shared_ptr<AlertManager>            alertMgr_;   ///< Alert manager reference.
    std::shared_ptr<EventLogger>             logger_;     ///< Event logger reference.

    /**
     * @brief  Prints a double-line box separator row using the given ANSI colour.
     * @param  color  ANSI escape string (default: no colour).
     */
    static void printSeparator(const std::string& color = "");

    /**
     * @brief  Prints a double-line box header with centred title text.
     * @param  title  Title string to display inside the header.
     * @param  color  ANSI escape string (default: no colour).
     */
    static void printHeader(const std::string& title, const std::string& color = "");
};

#endif // DASHBOARD_HPP
