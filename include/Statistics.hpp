/**
 * @file    Statistics.hpp
 * @brief   Vehicle statistics tracking and display for the Smart Vehicle ECU.
 *
 * @details VehicleStatistics accumulates per-cycle speed and temperature data
 *          to produce running summary metrics (max, min, average, cycle count,
 *          alert tallies).
 *
 *          This module is deliberately separated from Dashboard.hpp so that
 *          the live dashboard render remains focused on real-time data.
 *          Statistics are displayed at shutdown or on explicit request via
 *          displayStatistics().
 *
 *          Automotive mapping: Vehicle Data Recorder / OBD-II statistics log.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.1
 */

#pragma once
#ifndef STATISTICS_HPP
#define STATISTICS_HPP

#include <ostream>
#include <memory>

class Dashboard;
class AlertManager;

// ─────────────────────────────────────────────────────────────────────────────
// VehicleStatistics
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  VehicleStatistics
 * @brief   Aggregated vehicle performance and health metrics.
 *
 * @details Fields are updated once per dashboard render cycle via
 *          Dashboard::updateStatistics().  The struct is owned by main()
 *          and passed by reference to avoid tight coupling between Dashboard
 *          and the statistics subsystem.
 *
 * @note    operator<< produces a formatted box suitable for terminal output.
 *          operator<<(os, stats) is called only at shutdown or on demand —
 *          never inside the live dashboard render loop.
 */
struct VehicleStatistics {
    double  maxSpeed          = 0.0;   ///< Maximum observed vehicle speed (km/h).
    double  minTemp           = 999.0; ///< Minimum observed engine temperature (°C).
    double  maxTemp           = -999.0;///< Maximum observed engine temperature (°C).
    double  avgSpeed          = 0.0;   ///< Running average vehicle speed (km/h).
    long    cycleCount        = 0;     ///< Total render cycles completed.
    int     totalAlertsRaised = 0;     ///< Cumulative alerts raised (all severities).
    int     criticalEvents    = 0;     ///< Number of CRITICAL alerts in history.

    /**
     * @brief  Stream insertion operator — renders a formatted statistics box.
     *
     * @details Produces a Unicode box-drawing table with labelled rows,
     *          coloured values (BCYAN for counts, BYELLOW for speed,
     *          BRED for critical events), and a title banner.
     *
     * @param  os  Output stream.
     * @param  vs  VehicleStatistics to format.
     * @return Reference to os (enables chaining).
     */
    friend std::ostream& operator<<(std::ostream& os, const VehicleStatistics& vs);

    /**
     * @brief  Updates statistics with values from one render cycle.
     *
     * @details Uses an incremental (Welford-style) running average for speed.
     *          Tracks min/max for temperature.  Increments cycleCount.
     *
     * @param  speed  Vehicle speed reading for this cycle (km/h).
     * @param  temp   Engine temperature reading for this cycle (°C).
     */
    void update(double speed, double temp);

    /**
     * @brief  Resets all statistics fields to their initial default values.
     *
     * @details Useful for driver-profile reset or limp-home mode transitions.
     */
    void reset();
};

// ─────────────────────────────────────────────────────────────────────────────
// Free function
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Prints a formatted statistics report to stdout.
 *
 * @details Renders the VehicleStatistics box followed by the complete alert
 *          history from alertMgr.  Called from main() at shutdown.
 *
 * @param  stats     Const reference to the accumulated VehicleStatistics.
 * @param  alertMgr  Shared pointer to the AlertManager (for history display).
 */
void displayStatistics(const VehicleStatistics& stats,
                       const std::shared_ptr<AlertManager>& alertMgr);

#endif // STATISTICS_HPP
