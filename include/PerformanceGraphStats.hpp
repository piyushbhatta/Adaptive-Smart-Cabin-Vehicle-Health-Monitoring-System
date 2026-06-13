/**
 * @file    PerformanceGraphStats.hpp
 * @brief   Terminal bar-graph display of vehicle performance statistics.
 *
 * @details Renders per-cycle speed and temperature readings as ASCII/Unicode
 *          bar charts in the terminal, giving a visual history of the last
 *          N cycles.  Also overlays min/max/avg annotation lines.
 *
 *          Two graph types are provided:
 *          - Speed history graph   (km/h, max axis = SPEED_AXIS_MAX)
 *          - Temperature history graph (°C,  max axis = TEMP_AXIS_MAX)
 *
 *          The internal ring buffer holds up to MAX_SAMPLES entries; older
 *          samples are silently dropped when the buffer is full (ring semantics).
 *
 *          Automotive mapping: instrument-cluster trend graph / OBD-II live
 *          data visualisation.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef PERFORMANCE_GRAPH_STATS_HPP
#define PERFORMANCE_GRAPH_STATS_HPP

#include <vector>
#include <string>
#include <ostream>
#include <cstddef>

// ─────────────────────────────────────────────────────────────────────────────
// PerformanceGraphStats
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  PerformanceGraphStats
 * @brief  Accumulates speed and temperature samples; renders ASCII bar graphs.
 *
 * @details Thread-safety: not thread-safe.  The caller must ensure that
 *          addSample() and display() are not called concurrently.
 */
class PerformanceGraphStats {
public:
    // ── Tuneable constants ────────────────────────────────────────────────────
    static constexpr std::size_t MAX_SAMPLES    = 30;   ///< Ring-buffer capacity.
    static constexpr int         GRAPH_HEIGHT   = 8;    ///< Rows in each bar graph.
    static constexpr double      SPEED_AXIS_MAX = 160.0;///< Top of speed Y-axis (km/h).
    static constexpr double      TEMP_AXIS_MAX  = 130.0;///< Top of temp  Y-axis (°C).

    PerformanceGraphStats() = default;

    // Non-copyable — owns the sample ring buffer.
    PerformanceGraphStats(const PerformanceGraphStats&)            = delete;
    PerformanceGraphStats& operator=(const PerformanceGraphStats&) = delete;

    // ── Core API ──────────────────────────────────────────────────────────────

    /**
     * @brief  Records one speed + temperature sample.
     *
     * @details Appends to the ring buffer.  When MAX_SAMPLES is reached the
     *          oldest entry is overwritten (ring semantics).
     *
     * @param  speed  Vehicle speed in km/h.
     * @param  temp   Engine temperature in °C.
     */
    void addSample(double speed, double temp);

    /**
     * @brief  Returns the number of samples currently held.
     */
    std::size_t sampleCount() const { return speedSamples_.size(); }

    /**
     * @brief  Clears all accumulated samples.
     */
    void reset();

    /**
     * @brief  Renders both bar graphs plus a summary row to the given stream.
     *
     * @details Produces two stacked Unicode bar-chart panels: one for speed
     *          history and one for temperature history.  Each bar column
     *          represents one sample, scaled to GRAPH_HEIGHT rows.  A legend
     *          below each graph shows min / avg / max.
     *
     * @param  os  Destination stream (usually std::cout).
     */
    void display(std::ostream& os) const;

private:
    std::vector<double> speedSamples_; ///< Ring buffer of speed readings (km/h).
    std::vector<double> tempSamples_;  ///< Ring buffer of temperature readings (°C).

    /**
     * @brief  Renders one bar-graph panel.
     *
     * @param  os       Output stream.
     * @param  title    Panel title string (e.g. "SPEED HISTORY (km/h)").
     * @param  samples  Data series to plot.
     * @param  axisMax  Maximum value on the Y-axis.
     * @param  unit     Unit label (e.g. "km/h").
     * @param  barColor ANSI color code for the bar fill character.
     * @param  warnThreshold  Value above which bars are rendered in warning color.
     * @param  critThreshold  Value above which bars are rendered in critical color.
     */
    static void renderGraph(std::ostream&              os,
                            const std::string&         title,
                            const std::vector<double>& samples,
                            double                     axisMax,
                            const std::string&         unit,
                            const char*                barColor,
                            double                     warnThreshold,
                            double                     critThreshold);

    /**
     * @brief  Computes min, mean, and max over a sample vector.
     * @param  v    Input samples.
     * @param  mn   Output minimum.
     * @param  avg  Output mean.
     * @param  mx   Output maximum.
     */
    static void calcStats(const std::vector<double>& v,
                          double& mn, double& avg, double& mx);
};

#endif // PERFORMANCE_GRAPH_STATS_HPP
