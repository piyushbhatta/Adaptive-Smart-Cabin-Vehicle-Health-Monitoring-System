#include "Statistics.hpp"
#include "Alert.hpp"
#include "Sensor.hpp"
#include "Dashboard.hpp"   // Color namespace
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace Color;

static constexpr int STAT_W = 44; // inner box width for statistics

// ─────────────────────────────────────────────────────────────────────────────
// VehicleStatistics::update
// ─────────────────────────────────────────────────────────────────────────────
void VehicleStatistics::update(double speed, double temp) {
    ++cycleCount;
    if (speed > maxSpeed) maxSpeed = speed;
    if (temp  < minTemp)  minTemp  = temp;
    if (temp  > maxTemp)  maxTemp  = temp;
    // Welford incremental mean
    avgSpeed = avgSpeed + (speed - avgSpeed) / static_cast<double>(cycleCount);
}

void VehicleStatistics::reset() {
    maxSpeed          = 0.0;
    minTemp           = 999.0;
    maxTemp           = -999.0;
    avgSpeed          = 0.0;
    cycleCount        = 0;
    totalAlertsRaised = 0;
    criticalEvents    = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// VehicleStatistics::operator<<
// ─────────────────────────────────────────────────────────────────────────────
std::ostream& operator<<(std::ostream& os, const VehicleStatistics& vs) {
    auto fmt1 = [](double v, const std::string& unit) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(1) << v << " " << unit;
        return o.str();
    };

    // Helper lambda to print one statistics row
    auto row = [&](const std::string& label, const std::string& val, const char* valColor) {
        int labelW = 22;
        int valW   = STAT_W - labelW - 7; // 7 = "  | " + "  " borders
        int pad    = valW - static_cast<int>(val.size());
        os << BOLD << BCYAN << "  \u2551 " << RESET
           << BWHITE << BOLD << std::left << std::setw(labelW) << label << RESET
           << BCYAN << BOLD << " \u2502 " << RESET
           << valColor << BOLD << val << std::string(pad < 0 ? 0 : pad,' ') << RESET
           << BOLD << BCYAN << "\u2551\n" << RESET;
    };

    os << "\n" << BOLD << BCYAN
       << "  \u2554" << std::string(STAT_W,'=') << "\u2557\n"
       << "  \u2551" << std::setw(STAT_W) << std::left << "  VEHICLE STATISTICS" << "\u2551\n"
       << "  \u2560" << std::string(STAT_W,'=') << "\u2563\n"
       << RESET;

    row("Cycles Processed",  std::to_string(vs.cycleCount),        BGREEN);
    row("Max Speed",         fmt1(vs.maxSpeed,  "km/h"),            BYELLOW);
    row("Avg Speed",         fmt1(vs.avgSpeed,  "km/h"),            BYELLOW);
    row("Min Engine Temp",   fmt1(vs.minTemp,   "deg C"),           BCYAN);
    row("Max Engine Temp",   fmt1(vs.maxTemp,   "deg C"),           BRED);
    row("Alerts Raised",     std::to_string(vs.totalAlertsRaised), BYELLOW);
    row("Critical Events",   std::to_string(vs.criticalEvents),
        vs.criticalEvents > 0 ? BRED : BGREEN);
    row("Total Sensors",     std::to_string(Sensor::getTotalSensors()), BCYAN);

    os << BOLD << BCYAN
       << "  \u255a" << std::string(STAT_W,'=') << "\u255d\n"
       << RESET;
    return os;
}

// ─────────────────────────────────────────────────────────────────────────────
// displayStatistics  — full shutdown/on-demand report
// ─────────────────────────────────────────────────────────────────────────────
void displayStatistics(const VehicleStatistics& stats,
                       const std::shared_ptr<AlertManager>& alertMgr)
{
    std::cout << stats;

    // Alert history section
    constexpr int HW = 90;
    std::cout << BOLD << BMAGENTA
              << "  \u2554" << std::string(HW,'=') << "\u2557\n"
              << "  \u2551  ALERT HISTORY" << std::string(HW - 16,' ') << "\u2551\n"
              << "  \u2560" << std::string(HW,'=') << "\u2563\n"
              << RESET;

    if (alertMgr->getAlertHistory().empty()) {
        std::string msg = "  No alerts were raised this session.";
        int pad = HW - static_cast<int>(msg.size());
        std::cout << BWHITE << "  \u2551" << msg
                  << std::string(pad < 0 ? 0 : pad,' ')
                  << "\u2551\n" << RESET;
    } else {
    for (const auto& a : alertMgr->getAlertHistory()) {

        const char* ac =
            (a.getSeverity() == AlertSeverity::CRITICAL)
                ? BRED
                : BYELLOW;

        std::ostringstream line;
        line << a;
        std::string ls = line.str();

        // Available width inside box after "║  "
        constexpr int CONTENT_W = HW - 4;

        if (static_cast<int>(ls.size()) > CONTENT_W)
            ls = ls.substr(0, CONTENT_W - 3) + "...";

        int pad = CONTENT_W - static_cast<int>(ls.size());

        std::cout
            << ac
            << "  ║  "
            << ls
            << std::string(pad, ' ')
            << "║\n"
            << RESET;
    }
}
    std::cout << BOLD << BMAGENTA
              << "  \u255a" << std::string(HW,'=') << "\u255d\n"
              << RESET;
}