/**
 * @file  EcuHealthMonitor.cpp
 * @brief ECU Health Monitor implementation.
 */

#include "EcuHealthMonitor.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
// Global instance — initialised in main.cpp after sensor names are known
// Forward-declare here; definition at bottom of this file.
// ─────────────────────────────────────────────────────────────────────────────

// (definition is at the bottom of this file)

// ─────────────────────────────────────────────────────────────────────────────
// Box-drawing helpers (mirror main.cpp style)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int HBW = 62;

static std::string hrep(int n, const std::string& ch = "\xe2\x95\x90") {
    std::string r; r.reserve(n * ch.size());
    for (int i = 0; i < n; ++i) r += ch;
    return r;
}
static void hbTop(const char* c) {
    std::cout << c << BOLD << "  \xe2\x95\x94" << hrep(HBW) << "\xe2\x95\x97\n" << RESET;
}
static void hbBot(const char* c) {
    std::cout << c << BOLD << "  \xe2\x95\x9a" << hrep(HBW) << "\xe2\x95\x9d\n" << RESET;
}
static void hbMid(const char* c) {
    std::cout << c << BOLD << "  \xe2\x95\xa0" << hrep(HBW) << "\xe2\x95\xa3\n" << RESET;
}
static void hbRow(const std::string& txt, const char* boxC, const char* txtC) {
    int pad = HBW - static_cast<int>(txt.size());
    if (pad < 0) pad = 0;
    std::cout << boxC << BOLD << "  \xe2\x95\x91" << RESET
              << txtC << txt << std::string(pad, ' ') << RESET
              << boxC << BOLD << "\xe2\x95\x91\n" << RESET;
}
static void hbKv(const std::string& key, const std::string& val, const char* valC) {
    std::string line = "  " + key;
    int padMid = 22 - static_cast<int>(line.size());
    if (padMid < 1) padMid = 1;
    line += std::string(padMid, ' ') + ": ";
    int padEnd = HBW - static_cast<int>(line.size()) - static_cast<int>(val.size());
    if (padEnd < 0) padEnd = 0;
    std::cout << BBLUE << BOLD << "  \xe2\x95\x91" << RESET
              << BCYAN << line << RESET
              << valC  << BOLD << val << RESET
              << std::string(padEnd, ' ')
              << BBLUE << BOLD << "\xe2\x95\x91\n" << RESET;
}

// Helper: render a compact progress bar  "████████░░░░  83.0 %"
static std::string healthBar(double pct, int width = 20) {
    int filled = static_cast<int>(std::round(pct / 100.0 * width));
    filled = std::max(0, std::min(width, filled));
    std::string bar(static_cast<std::size_t>(filled),       '\xe2'); // placeholder char
    // Build bar from Unicode block characters
    std::string result;
    for (int i = 0; i < width; ++i)
        result += (i < filled) ? "\xe2\x96\x88" : "\xe2\x96\x91"; // █ vs ░
    std::ostringstream oss;
    oss << result << "  " << std::fixed << std::setprecision(1) << pct << " %";
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// SensorHealthRecord
// ─────────────────────────────────────────────────────────────────────────────

double SensorHealthRecord::faultRatePercent() const {
    if (totalReadings == 0) return 0.0;
    return 100.0 * static_cast<double>(faultReadings) /
                   static_cast<double>(totalReadings);
}

double SensorHealthRecord::healthScore() const {
    return std::max(0.0, 100.0 - faultRatePercent());
}

std::string SensorHealthRecord::statusString() const {
    double fr = faultRatePercent();
    if (fr >= EcuHealthMonitor::FAULT_THRESHOLD)    return "FAULT   ";
    if (fr >= EcuHealthMonitor::DEGRADED_THRESHOLD) return "DEGRADED";
    return "OK      ";
}

// ─────────────────────────────────────────────────────────────────────────────
// EcuHealthMonitor — construction
// ─────────────────────────────────────────────────────────────────────────────

EcuHealthMonitor::EcuHealthMonitor(const std::vector<std::string>& sensorNames)
    : startTime_(std::chrono::steady_clock::now())
{
    records_.reserve(sensorNames.size());
    for (const auto& n : sensorNames) {
        SensorHealthRecord r;
        r.sensorName = n;
        records_.push_back(r);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Update API
// ─────────────────────────────────────────────────────────────────────────────

SensorHealthRecord* EcuHealthMonitor::findRecord(const std::string& name) {
    for (auto& r : records_)
        if (r.sensorName == name) return &r;
    return nullptr;
}

void EcuHealthMonitor::recordReading(const std::string& sensorName, bool isNormal) {
    SensorHealthRecord* r = findRecord(sensorName);
    if (!r) return;
    ++r->totalReadings;
    if (!isNormal) {
        ++r->faultReadings;
        r->currentlyFaulted = true;
    } else {
        r->currentlyFaulted = false;
    }
}

void EcuHealthMonitor::incrementAlertCount() {
    ++totalAlerts_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Query API
// ─────────────────────────────────────────────────────────────────────────────

double EcuHealthMonitor::overallHealthScore() const {
    if (records_.empty()) return 100.0;
    double sum = 0.0;
    for (const auto& r : records_) sum += r.healthScore();
    return sum / static_cast<double>(records_.size());
}

long EcuHealthMonitor::uptimeSeconds() const {
    auto now     = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime_);
    return static_cast<long>(elapsed.count());
}

std::string EcuHealthMonitor::uptimeString() const {
    long secs  = uptimeSeconds();
    long mins  = secs / 60;
    long rem   = secs % 60;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << mins << ":"
        << std::setw(2) << std::setfill('0') << rem;
    return oss.str();
}

double EcuHealthMonitor::alertRatePerMinute() const {
    double mins = static_cast<double>(uptimeSeconds()) / 60.0;
    if (mins < 0.01) return 0.0;
    return static_cast<double>(totalAlerts_) / mins;
}

std::string EcuHealthMonitor::overallStatusString() const {
    double h = overallHealthScore();
    if (h >= 70.0) return "HEALTHY ";
    if (h >= 40.0) return "DEGRADED";
    return "CRITICAL";
}

const SensorHealthRecord& EcuHealthMonitor::getSensorRecord(const std::string& name) const {
    for (const auto& r : records_)
        if (r.sensorName == name) return r;
    throw std::out_of_range("EcuHealthMonitor: sensor not found: " + name);
}

// ─────────────────────────────────────────────────────────────────────────────
// Display — full panel
// ─────────────────────────────────────────────────────────────────────────────

void EcuHealthMonitor::display() const {
    double overall = overallHealthScore();
    const char* hColor = (overall >= 70.0) ? BGREEN :
                         (overall >= 40.0) ? BYELLOW : BRED;

    std::cout << "\n";
    hbTop(hColor);
    hbRow("   ECU HEALTH MONITOR", hColor, BWHITE);
    hbMid(hColor);

    // Overall score bar
    std::string barStr = healthBar(overall);
    hbKv("Overall Health", barStr, hColor);
    hbKv("Status",         overallStatusString(), hColor);
    hbKv("Uptime",         uptimeString() + " (MM:SS)", BCYAN);

    std::ostringstream alertRateStr;
    alertRateStr << std::fixed << std::setprecision(1) << alertRatePerMinute() << " /min";
    hbKv("Alerts Total",   std::to_string(totalAlerts_), BYELLOW);
    hbKv("Alert Rate",     alertRateStr.str(),            BYELLOW);

    hbMid(hColor);
    // Per-sensor table header
    hbRow("   Sensor                  Health     FaultRate  Status", hColor, BCYAN);
    hbMid(hColor);

    for (const auto& r : records_) {
        double  hs   = r.healthScore();
        double  fr   = r.faultRatePercent();
        const char* sc = (hs >= 85.0) ? BGREEN :
                         (hs >= 60.0) ? BYELLOW : BRED;

        // Format: name padded to 24, health% 6chars, faultRate% 7chars, status
        std::ostringstream row;
        row << "   ";
        std::string namePad = r.sensorName;
        namePad.resize(24, ' ');
        row << namePad;
        row << std::fixed << std::setprecision(1) << std::setw(6) << hs << "%  ";
        row << std::fixed << std::setprecision(1) << std::setw(6) << fr << "%   ";
        row << r.statusString();

        hbRow(row.str(), hColor, sc);
    }
    hbBot(hColor);
}

// ─────────────────────────────────────────────────────────────────────────────
// Display — compact footer line
// ─────────────────────────────────────────────────────────────────────────────

void EcuHealthMonitor::displayCompact() const {
    double overall = overallHealthScore();
    const char* hColor = (overall >= 70.0) ? BGREEN :
                         (overall >= 40.0) ? BYELLOW : BRED;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << overall;

    std::cout << hColor << BOLD
              << "  ╔══ ECU Health: " << oss.str() << " %"
              << "  [" << overallStatusString() << "]"
              << "  Up: " << uptimeString()
              << "  Alerts: " << totalAlerts_
              << " ══╗\n"
              << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// Global instance — sensor names set by main.cpp at startup
// ─────────────────────────────────────────────────────────────────────────────

EcuHealthMonitor g_ecuHealth({
    "Engine Temp",
    "Battery",
    "Speed",
    "Tire Pressure",
    "Door",
    "Seatbelt"
});
