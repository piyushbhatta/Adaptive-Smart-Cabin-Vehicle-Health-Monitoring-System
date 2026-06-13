/**
 * @file    AdaptiveAlertPrioritizer.hpp
 * @brief   Adaptive alert prioritization for the Smart Vehicle ECU.
 *
 * @details Tracks per-alert-code hit counts and suppression scores to
 *          dynamically re-rank active alerts.  Alerts that fire very
 *          frequently (e.g. a noisy sensor) are downgraded in display
 *          priority so that rare, high-severity events surface first.
 *
 *          Priority score formula (per alert):
 *              score = (severity_weight * SEVERITY_FACTOR)
 *                    - (hitCount * FREQUENCY_PENALTY)
 *
 *          severity_weight : CRITICAL=3, WARNING=2, INFO=1
 *          SEVERITY_FACTOR : 100   (keeps severity dominant)
 *          FREQUENCY_PENALTY: 5   (each repeat reduces score by 5)
 *
 *          Automotive mapping: AUTOSAR DEM adaptive event significance.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef ADAPTIVE_ALERT_PRIORITIZER_HPP
#define ADAPTIVE_ALERT_PRIORITIZER_HPP

#include "Alert.hpp"
#include <map>
#include <vector>
#include <string>
#include <ostream>

// ─────────────────────────────────────────────────────────────────────────────
// PrioritizedAlert  — an Alert paired with its computed priority score
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct PrioritizedAlert
 * @brief  An Alert decorated with a dynamic priority score for display ranking.
 */
struct PrioritizedAlert {
    Alert  alert;          ///< The original alert.
    int    score;          ///< Computed adaptive priority score.
    int    hitCount;       ///< How many times this code has fired so far.

    /// @brief Descending score order (higher score = more urgent).
    bool operator<(const PrioritizedAlert& other) const {
        return score > other.score;   // descending
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// AdaptiveAlertPrioritizer
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  AdaptiveAlertPrioritizer
 * @brief  Re-ranks active alerts by adaptive priority score.
 *
 * @details Call record() each time any alert fires to accumulate hit counts.
 *          Call prioritize() to obtain a re-ranked snapshot of the alerts
 *          currently held by an AlertManager.
 */
class AdaptiveAlertPrioritizer {
public:
    // Tuning constants (public so tests can inspect them)
    static constexpr int SEVERITY_FACTOR   = 100; ///< Multiplier for base severity weight.
    static constexpr int FREQUENCY_PENALTY =   5; ///< Score reduction per repeat occurrence.

    AdaptiveAlertPrioritizer() = default;

    // Non-copyable — single owner manages the hit-count map.
    AdaptiveAlertPrioritizer(const AdaptiveAlertPrioritizer&)            = delete;
    AdaptiveAlertPrioritizer& operator=(const AdaptiveAlertPrioritizer&) = delete;

    // ── Core API ─────────────────────────────────────────────────────────────

    /**
     * @brief  Records one occurrence of an alert code.
     *
     * @details Must be called every time an alert is raised (new or duplicate).
     *          Increments the internal hit count for the given code so that
     *          prioritize() can apply the frequency penalty correctly.
     *
     * @param  code  The alert code that fired (e.g. "OVERSPEED_WARNING").
     */
    void record(const std::string& code);

    /**
     * @brief  Produces a priority-ranked snapshot of currently active alerts.
     *
     * @details Iterates the active alerts from alertMgr, computes a score
     *          for each using the hit-count map, and returns a vector sorted
     *          by descending score.
     *
     * @param  alertMgr  AlertManager whose active alerts are to be ranked.
     * @return Sorted vector of PrioritizedAlert (highest priority first).
     */
    std::vector<PrioritizedAlert>
    prioritize(const AlertManager& alertMgr) const;

    /**
     * @brief  Returns the cumulative hit count for a given alert code.
     * @param  code  Alert code to query.
     * @return Hit count (0 if never recorded).
     */
    int hitCount(const std::string& code) const;

    /**
     * @brief  Resets all accumulated hit counts (e.g. on driver-profile switch).
     */
    void reset();

    /**
     * @brief  Renders the prioritized alert table to the given output stream.
     *
     * @details Prints a formatted box listing each active alert with its
     *          priority rank, score, hit count, severity, and message.
     *          Uses ANSI colour codes matching the rest of the dashboard.
     *
     * @param  os        Destination stream (usually std::cout).
     * @param  alertMgr  AlertManager whose active alerts are to be displayed.
     */
    void display(std::ostream& os, const AlertManager& alertMgr) const;

private:
    std::map<std::string, int> hitCounts_; ///< code → cumulative occurrence count.

    /**
     * @brief  Converts a severity enum to its weight integer.
     * @param  sev  AlertSeverity to convert.
     * @return 3 for CRITICAL, 2 for WARNING, 1 for INFO.
     */
    static int severityWeight(AlertSeverity sev);

    /**
     * @brief  Computes the adaptive priority score for one alert.
     * @param  sev   Severity of the alert.
     * @param  hits  Cumulative hit count for that alert code.
     * @return Integer score (higher = more urgent).
     */
    static int computeScore(AlertSeverity sev, int hits);
};

#endif // ADAPTIVE_ALERT_PRIORITIZER_HPP
