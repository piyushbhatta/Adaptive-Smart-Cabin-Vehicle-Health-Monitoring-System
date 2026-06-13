/**
 * @file    PerformanceGraphStats.cpp
 * @brief   Implementation of PerformanceGraphStats.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "PerformanceGraphStats.hpp"
#include "Dashboard.hpp"    // Color namespace

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
// Threshold constants (matching Sensor.hpp values without pulling it in)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr double SPEED_WARN_KMH  = 100.0;  ///< Yellow bar above this speed.
static constexpr double SPEED_CRIT_KMH  = 120.0;  ///< Red bar above this speed.
static constexpr double TEMP_WARN_DEG   =  95.0;  ///< Yellow bar above this temp.
static constexpr double TEMP_CRIT_DEG   = 110.0;  ///< Red bar above this temp.

// ─────────────────────────────────────────────────────────────────────────────
// addSample / reset
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceGraphStats::addSample(double speed, double temp)
{
    // Ring-buffer: drop the oldest sample when full
    if (speedSamples_.size() >= MAX_SAMPLES) {
        speedSamples_.erase(speedSamples_.begin());
        tempSamples_.erase(tempSamples_.begin());
    }
    speedSamples_.push_back(speed);
    tempSamples_.push_back(temp);
}

void PerformanceGraphStats::reset()
{
    speedSamples_.clear();
    tempSamples_.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// calcStats
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceGraphStats::calcStats(const std::vector<double>& v,
                                      double& mn, double& avg, double& mx)
{
    if (v.empty()) { mn = avg = mx = 0.0; return; }
    mn  = *std::min_element(v.begin(), v.end());
    mx  = *std::max_element(v.begin(), v.end());
    avg = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// renderGraph  —  single panel renderer
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceGraphStats::renderGraph(std::ostream&              os,
                                        const std::string&         title,
                                        const std::vector<double>& samples,
                                        double                     axisMax,
                                        const std::string&         unit,
                                        const char*                barColor,
                                        double                     warnThreshold,
                                        double                     critThreshold)
{
    constexpr int BOX_W = 72;   // inner width of the enclosing box

    // ── Panel header ─────────────────────────────────────────────────────────
    os << BOLD << BBLUE
       << "  \u2554" << std::string(BOX_W, '=') << "\u2557\n"
       << "  \u2551" << std::left << std::setw(BOX_W)
       << ("  " + title) << "\u2551\n"
       << "  \u2560" << std::string(BOX_W, '-') << "\u2563\n"
       << RESET;

    if (samples.empty()) {
        std::string msg = "  No samples recorded yet.";
        int pad = BOX_W - static_cast<int>(msg.size());
        os << BWHITE << "  \u2551" << msg
           << std::string(pad < 0 ? 0 : pad, ' ') << "\u2551\n" << RESET;
        os << BOLD << BBLUE << "  \u255a" << std::string(BOX_W, '=') << "\u255d\n" << RESET;
        return;
    }

    // ── Build the bar grid ───────────────────────────────────────────────────
    // Each sample maps to a bar height in [0, GRAPH_HEIGHT].
    const int    H     = GRAPH_HEIGHT;
    const double scale = (axisMax > 0.0) ? static_cast<double>(H) / axisMax : 1.0;

    std::vector<int> heights;
    heights.reserve(samples.size());
    for (double v : samples) {
        int h = static_cast<int>(std::round(std::min(v, axisMax) * scale));
        heights.push_back(std::max(0, std::min(h, H)));
    }

    // ── Render rows top-to-bottom ─────────────────────────────────────────
    for (int row = H; row >= 1; --row) {
        // Y-axis label (every other row to avoid clutter)
        double rowVal = (static_cast<double>(row) / static_cast<double>(H)) * axisMax;
        std::ostringstream yLabel;
        if (row % 2 == 0 || row == H || row == 1)
            yLabel << std::fixed << std::setprecision(0) << std::right
                   << std::setw(5) << rowVal << " \u2502";
        else
            yLabel << "      \u2502";

        std::string yStr = yLabel.str();

        // Bar cells
        std::ostringstream cells;
        for (std::size_t i = 0; i < heights.size(); ++i) {
            double sampleVal = samples[i];
            if (heights[i] >= row) {
                // Choose colour based on value
                const char* col = barColor;
                if (sampleVal >= critThreshold)      col = BRED;
                else if (sampleVal >= warnThreshold) col = BYELLOW;
                cells << col << BOLD << "\u2588" << RESET;  // █
            } else {
                cells << " ";
            }
            cells << " ";  // column spacing
        }

        // Pad the inner content to BOX_W
        std::string cellStr = cells.str();
        // cellStr contains ANSI codes; we must count visible chars separately
        int visibleCells = static_cast<int>(heights.size()) * 2;
        int innerUsed    = static_cast<int>(yStr.size()) + visibleCells;
        int pad          = BOX_W - innerUsed;

        os << BOLD << BBLUE << "  \u2551" << RESET
           << BWHITE << yStr << RESET
           << cellStr
           << std::string(pad < 0 ? 0 : pad, ' ')
           << BOLD << BBLUE << "\u2551\n" << RESET;
    }

    // ── X-axis baseline ───────────────────────────────────────────────────
    std::string xAxis = "      \u2534" + std::string(static_cast<int>(heights.size()) * 2, '-');
    int xPad = BOX_W - static_cast<int>(xAxis.size());
    os << BOLD << BBLUE << "  \u2551" << RESET
       << BCYAN << xAxis << std::string(xPad < 0 ? 0 : xPad, ' ') << RESET
       << BOLD << BBLUE << "\u2551\n" << RESET;

    // ── Stats legend ──────────────────────────────────────────────────────
    double mn, avg, mx;
    calcStats(samples, mn, avg, mx);

    auto fmtVal = [&](double v) -> std::string {
        std::ostringstream o;
        o << std::fixed << std::setprecision(1) << v << " " << unit;
        return o.str();
    };

    std::ostringstream legend;
    legend << "  Min: " << fmtVal(mn)
           << "   Avg: " << fmtVal(avg)
           << "   Max: " << fmtVal(mx)
           << "   Samples: " << samples.size();
    std::string legStr = legend.str();
    int legPad = BOX_W - static_cast<int>(legStr.size());

    os << BOLD << BBLUE << "  \u2551" << RESET
       << BGREEN << BOLD << legStr << std::string(legPad < 0 ? 0 : legPad, ' ') << RESET
       << BOLD << BBLUE << "\u2551\n" << RESET;

    os << BOLD << BBLUE << "  \u255a" << std::string(BOX_W, '=') << "\u255d\n" << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// display  —  renders both graphs + summary
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceGraphStats::display(std::ostream& os) const
{
    os << "\n";

    renderGraph(os,
                "SPEED HISTORY  (km/h)",
                speedSamples_,
                SPEED_AXIS_MAX,
                "km/h",
                BGREEN,
                SPEED_WARN_KMH,
                SPEED_CRIT_KMH);

    os << "\n";

    renderGraph(os,
                "ENGINE TEMPERATURE HISTORY  (deg C)",
                tempSamples_,
                TEMP_AXIS_MAX,
                "C",
                BCYAN,
                TEMP_WARN_DEG,
                TEMP_CRIT_DEG);

    os << "\n";

    // ── Colour legend ────────────────────────────────────────────────────────
    constexpr int LW = 72;
    os << BOLD << BBLUE
       << "  \u2554" << std::string(LW, '-') << "\u2557\n"
       << "  \u2551" << std::left << std::setw(LW) << "  COLOUR LEGEND" << "\u2551\n"
       << "  \u2560" << std::string(LW, '-') << "\u2563\n"
       << RESET;

    auto legendRow = [&](const char* col, const std::string& label) {
        std::string txt = "  " + label;
        int pad = LW - static_cast<int>(txt.size());
        os << BOLD << BBLUE << "  \u2551" << RESET
           << col << BOLD << txt << std::string(pad < 0 ? 0 : pad, ' ') << RESET
           << BOLD << BBLUE << "\u2551\n" << RESET;
    };

    legendRow(BGREEN,  "\u2588  Normal    Speed < 100 km/h  |  Temp < 95 C");
    legendRow(BYELLOW, "\u2588  Warning   Speed 100-120 km/h  |  Temp 95-110 C");
    legendRow(BRED,    "\u2588  Critical  Speed > 120 km/h  |  Temp > 110 C");

    os << BOLD << BBLUE << "  \u255a" << std::string(LW, '-') << "\u255d\n" << RESET << "\n";
}
