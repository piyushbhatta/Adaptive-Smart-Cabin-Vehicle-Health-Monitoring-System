#include "Dashboard.hpp"
#include "Statistics.hpp"
#include "Sensor.hpp"
#include "Alert.hpp"
#include "Logger.hpp"
#include "Performance.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
// Dashboard constructor
// ─────────────────────────────────────────────────────────────────────────────
Dashboard::Dashboard(
    std::shared_ptr<EngineTemperatureSensor> eng,
    std::shared_ptr<BatterySensor>           bat,
    std::shared_ptr<SpeedSensor>             spd,
    std::shared_ptr<TirePressureSensor>      tire,
    std::shared_ptr<DoorSensor>              door,
    std::shared_ptr<SeatbeltSensor>          sb,
    std::shared_ptr<AlertManager>            mgr,
    std::shared_ptr<EventLogger>             log)
    : engSensor_(std::move(eng)),   batSensor_(std::move(bat))
    , spdSensor_(std::move(spd)),   tireSensor_(std::move(tire))
    , doorSensor_(std::move(door)), sbSensor_(std::move(sb))
    , alertMgr_(std::move(mgr)),    logger_(std::move(log))
{}

// ─────────────────────────────────────────────────────────────────────────────
// Box-drawing helpers
// ─────────────────────────────────────────────────────────────────────────────

// Box inner width = 64 printable characters (between the ║ borders)
static constexpr int BOX_W = 90;

void Dashboard::printSeparator(const std::string& color) {
    // ╠══...══╣
    std::cout << color << BOLD
              << "  \u2560" << std::string(BOX_W, '\xE2') // placeholder
              << "\u2563\n" << RESET;
    // Use proper box chars directly
    std::cout.flush();
}

void Dashboard::printHeader(const std::string& title, const std::string& color) {
    int pad = BOX_W - 2 - static_cast<int>(title.size());
    if (pad < 0) pad = 0;
    std::cout << color << BOLD
              << "  \u2554" << std::string(BOX_W, '=') << "\u2557\n"
              << "  \u2551  " << title << std::string(pad, ' ') << "\u2551\n"
              << "  \u255a" << std::string(BOX_W, '=') << "\u255d\n"
              << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// displayCurrentValues — sensor table
// ─────────────────────────────────────────────────────────────────────────────

void Dashboard::displayCurrentValues() const
{
    constexpr int LABEL_W  = 30;
    constexpr int VALUE_W  = 15;
    constexpr int STATUS_W = 25;

    std::cout
        << BOLD << BBLUE
        << "  +" << std::string(LABEL_W, '-')
        << "+" << std::string(VALUE_W, '-')
        << "+" << std::string(STATUS_W, '-')
        << "+\n"

        << "  |"
        << BCYAN << std::left << std::setw(LABEL_W)
        << " SENSOR"
        << BBLUE << "|"
        << BCYAN << std::setw(VALUE_W)
        << " VALUE"
        << BBLUE << "|"
        << BCYAN << std::setw(STATUS_W)
        << " STATUS"
        << BBLUE << "|\n"

        << "  +" << std::string(LABEL_W, '-')
        << "+" << std::string(VALUE_W, '-')
        << "+" << std::string(STATUS_W, '-')
        << "+\n"
        << RESET;

    engSensor_->display();
    batSensor_->display();
    spdSensor_->display();
    tireSensor_->display();
    doorSensor_->display();
    sbSensor_->display();

    std::cout
        << BOLD << BBLUE
        << "  +" << std::string(LABEL_W, '-')
        << "+" << std::string(VALUE_W, '-')
        << "+" << std::string(STATUS_W, '-')
        << "+\n"
        << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// displayActiveAlerts
// ─────────────────────────────────────────────────────────────────────────────
void Dashboard::displayActiveAlerts() const {
    int count = alertMgr_->getActiveCount();
    const char* hdrColor = (count == 0) ? BGREEN : (count >= 3 ? BRED : BYELLOW);

    std::cout << BOLD << hdrColor
              << "  \u2554" << std::string(BOX_W,'=') << "\u2557\n";

    // Header line — count badge
    std::ostringstream hdr;
    hdr << "  ACTIVE ALERTS  [" << count << "]";
    int hpad = BOX_W - static_cast<int>(hdr.str().size()) - 2;
    if (hpad < 0) hpad = 0;
    std::cout << "  \u2551" << BOLD << hdrColor
              << hdr.str() << std::string(hpad,' ')
              << "\u2551\n"
              << "  \u2560" << std::string(BOX_W,'=') << "\u2563\n"
              << RESET;

    if (alertMgr_->getActiveAlerts().empty()) {
        std::string ok = "  [OK] All systems normal — no active alerts";
        int pad = BOX_W - static_cast<int>(ok.size());
        if (pad < 0) pad = 0;
        std::cout << BOLD << BGREEN
                  << "  \u2551" << ok << std::string(pad,' ') << "\u2551\n"
                  << RESET;
    } else {
        for (const auto& a : alertMgr_->getActiveAlerts()) {
            const char* ac = (a.getSeverity() == AlertSeverity::CRITICAL) ? BRED : BYELLOW;
            std::string ls =
    "[" + severityToString(a.getSeverity()) + "] "
    + a.getCode();
            // Strip raw ANSI bytes that may have leaked into Alert::operator<<
            // and keep printable length under BOX_W-4
            if (static_cast<int>(ls.size()) > BOX_W - 8)
                ls = ls.substr(0, BOX_W - 11) + "...";
            int pad = BOX_W - 4 - static_cast<int>(ls.size());
            if (pad < 0) pad = 0;
            std::cout << BOLD << ac
                      << "  \u2551  " << ls << std::string(pad,' ')
                      << "\u2551\n" << RESET;
        }
    }
    std::cout << BOLD << hdrColor
              << "  \u255a" << std::string(BOX_W,'=') << "\u255d\n"
              << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// displayAlertHistory
// ─────────────────────────────────────────────────────────────────────────────
void Dashboard::displayAlertHistory() const {
    std::cout << BOLD << BMAGENTA
              << "  \u2554" << std::string(BOX_W,'=') << "\u2557\n"
              << "  \u2551  ALERT HISTORY" << std::string(BOX_W-16,' ') << "\u2551\n"
              << "  \u2560" << std::string(BOX_W,'=') << "\u2563\n"
              << RESET;
    if (alertMgr_->getAlertHistory().empty()) {
        std::string msg = "  No alert history recorded.";
        int pad = BOX_W - static_cast<int>(msg.size());
        std::cout << BWHITE << "  \u2551" << msg
                  << std::string(pad < 0 ? 0 : pad,' ')
                  << "\u2551\n" << RESET;
    } else {
        for (const auto& a : alertMgr_->getAlertHistory()) {
            const char* ac = (a.getSeverity() == AlertSeverity::CRITICAL) ? BRED : BYELLOW;
            std::ostringstream line;
            line << a;
            std::string ls = line.str();
            if (static_cast<int>(ls.size()) > BOX_W - 8)
                ls = ls.substr(0, BOX_W - 7) + "...";
            int pad = BOX_W - 4 - static_cast<int>(ls.size());
            std::cout << ac
                      << "  \u2551  " << ls
                      << std::string(pad < 0 ? 0 : pad,' ')
                      << "\u2551\n" << RESET;
        }
    }
    std::cout << BOLD << BMAGENTA
              << "  \u255a" << std::string(BOX_W,'=') << "\u255d\n"
              << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// displayFullDashboard  — live render cycle
// NOTE: Statistics are intentionally excluded here (see Statistics.cpp)
// ─────────────────────────────────────────────────────────────────────────────
void Dashboard::displayFullDashboard() const {
    std::cout << "\n";
    // Title banner
    std::string t1 = "  SMART CABIN & VEHICLE HEALTH MONITORING SYSTEM";
    std::string t2 = "  Visteon  |  Adaptive AUTOSAR Style ECU";
    int p1 = BOX_W - static_cast<int>(t1.size());
    int p2 = BOX_W - static_cast<int>(t2.size());
    std::cout << BOLD << BMAGENTA
              << "  \u2554" << std::string(BOX_W,'=') << "\u2557\n"
              << "  \u2551" << t1 << std::string(p1 < 0 ? 0 : p1,' ') << "\u2551\n"
              << "  \u2551" << t2 << std::string(p2 < 0 ? 0 : p2,' ') << "\u2551\n"
              << "  \u255a" << std::string(BOX_W,'=') << "\u255d\n"
              << RESET;

    displayCurrentValues();
    displayActiveAlerts();

    // Drain and print new alert notifications in a separate highlight box
    auto notifications = alertMgr_->drainNotifications();
    if (!notifications.empty()) {
        std::cout << "\n" << BOLD << BRED
                  << "  \u250c" << std::string(BOX_W,'-') << "\u2510\n"
                  << "  \u2502  *** NEW ALERTS THIS CYCLE ***"
                  << std::string(BOX_W - 32,' ') << "\u2502\n"
                  << "  \u251c" << std::string(BOX_W,'-') << "\u2524\n"
                  << RESET;
        for (const auto& n : notifications) {
            // Remove leading spaces
            std::string clean = n;
            while (!clean.empty() && clean[0] == ' ') clean = clean.substr(1);
            int pad = BOX_W - 4 - static_cast<int>(clean.size());
            std::cout << BOLD << BRED << "  \u2502  " << BYELLOW << clean
                      << std::string(pad < 0 ? 0 : pad,' ')
                      << BRED << "\u2502\n" << RESET;
        }
        std::cout << BOLD << BRED
                  << "  \u2514" << std::string(BOX_W,'-') << "\u2518\n"
                  << RESET;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// updateStatistics  — delegate to VehicleStatistics
// ─────────────────────────────────────────────────────────────────────────────
void Dashboard::updateStatistics(
    VehicleStatistics& stats) const
{
    auto start =
        std::chrono::high_resolution_clock::now();

    stats.update(
        spdSensor_->getSpeed(),
        engSensor_->getTemperature());

    stats.totalAlertsRaised =
        Alert::getTotalAlertCount();

    stats.criticalEvents =
        static_cast<int>(
            alertMgr_->filterHistory(
                AlertSeverity::CRITICAL).size());

    auto end =
        std::chrono::high_resolution_clock::now();

    g_performanceMonitor.update(
        "Statistics Module",
        std::chrono::duration<double,std::milli>(
            end - start).count());
}

// ─────────────────────────────────────────────────────────────────────────────
// logSnapshot
// ─────────────────────────────────────────────────────────────────────────────
void Dashboard::logSnapshot() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "Snapshot"
        << " | Speed=" << spdSensor_->getSpeed()        << "km/h"
        << " | Temp="  << engSensor_->getTemperature()   << "C"
        << " | Volt="  << batSensor_->getVoltage()       << "V"
        << " | PSI="   << tireSensor_->getPressure()
        << " | Door="  << (doorSensor_->isOpen() ? "OPEN" : "CLOSED")
        << " | Belt="  << (sbSensor_->isLocked() ? "LOCKED" : "UNLOCKED")
        << " | Alerts=" << alertMgr_->getActiveCount();
    logger_->logInfo(oss.str());
}
