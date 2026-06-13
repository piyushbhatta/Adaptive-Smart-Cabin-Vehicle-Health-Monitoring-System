/**
 * @file  DriverProfile.cpp
 * @brief Driver Profile Management implementation.
 */

#include "DriverProfile.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <limits>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
// Box-drawing helpers (mirrors main.cpp style)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int PBW = 62; // profile box inner width

static std::string prep(int n, const std::string& ch = "\xe2\x95\x90") {
    std::string r; r.reserve(n * ch.size());
    for (int i = 0; i < n; ++i) r += ch;
    return r;
}
static void pbTop(const char* c) {
    std::cout << c << BOLD << "  \xe2\x95\x94" << prep(PBW) << "\xe2\x95\x97\n" << RESET;
}
static void pbBot(const char* c) {
    std::cout << c << BOLD << "  \xe2\x95\x9a" << prep(PBW) << "\xe2\x95\x9d\n" << RESET;
}
static void pbMid(const char* c) {
    std::cout << c << BOLD << "  \xe2\x95\xa0" << prep(PBW) << "\xe2\x95\xa3\n" << RESET;
}
static void pbRow(const std::string& txt, const char* boxC, const char* txtC) {
    int pad = PBW - static_cast<int>(txt.size());
    if (pad < 0) pad = 0;
    std::cout << boxC << BOLD << "  \xe2\x95\x91" << RESET
              << txtC << txt << std::string(pad, ' ') << RESET
              << boxC << BOLD << "\xe2\x95\x91\n" << RESET;
}
static void pbKv(const std::string& key, const std::string& val, const char* valC) {
    std::string line = "  " + key;
    int padMid = 22 - static_cast<int>(line.size());
    if (padMid < 1) padMid = 1;
    line += std::string(padMid, ' ') + ": ";
    int padEnd = PBW - static_cast<int>(line.size()) - static_cast<int>(val.size());
    if (padEnd < 0) padEnd = 0;
    std::cout << BBLUE << BOLD << "  \xe2\x95\x91" << RESET
              << BCYAN << line << RESET
              << valC  << BOLD << val << RESET
              << std::string(padEnd, ' ')
              << BBLUE << BOLD << "\xe2\x95\x91\n" << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// DrivingMode helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string drivingModeToString(DrivingMode m) {
    switch (m) {
        case DrivingMode::ECO:    return "ECO";
        case DrivingMode::NORMAL: return "NORMAL";
        case DrivingMode::SPORT:  return "SPORT";
        case DrivingMode::LIMP:   return "LIMP";
    }
    return "NORMAL";
}

DrivingMode drivingModeFromString(const std::string& s) {
    if (s == "ECO")   return DrivingMode::ECO;
    if (s == "SPORT") return DrivingMode::SPORT;
    if (s == "LIMP")  return DrivingMode::LIMP;
    return DrivingMode::NORMAL;
}

// ─────────────────────────────────────────────────────────────────────────────
// DriverProfile — operator<<
// ─────────────────────────────────────────────────────────────────────────────

std::ostream& operator<<(std::ostream& os, const DriverProfile& p) {
    // Redirect cout for the box helpers
    // (simple approach: write directly since helpers use cout)
    pbTop(BMAGENTA);
    std::string title = "   DRIVER PROFILE — " + p.name;
    pbRow(title, BMAGENTA, BWHITE);
    pbMid(BMAGENTA);
    pbKv("Driving Mode",     drivingModeToString(p.drivingMode),
         p.drivingMode == DrivingMode::ECO    ? BGREEN  :
         p.drivingMode == DrivingMode::SPORT  ? BYELLOW :
         p.drivingMode == DrivingMode::LIMP   ? BRED    : BCYAN);
    pbKv("Speed Limit",      std::to_string(static_cast<int>(p.speedLimit))  + " km/h", BYELLOW);
    pbKv("Temp Warning",     std::to_string(static_cast<int>(p.tempWarning)) + " °C",   BYELLOW);
    pbKv("Temp Critical",    std::to_string(static_cast<int>(p.tempCritical))+ " °C",   BRED);
    pbKv("Alerts",           p.alertsEnabled ? "ENABLED" : "DISABLED",
                             p.alertsEnabled ? BGREEN : BRED);
    pbKv("Total Sessions",   std::to_string(p.totalSessions), BCYAN);
    pbBot(BMAGENTA);
    return os;
}

// ─────────────────────────────────────────────────────────────────────────────
// DriverProfileManager — construction
// ─────────────────────────────────────────────────────────────────────────────

DriverProfileManager::DriverProfileManager() {
    loadProfiles();
    if (profiles_.empty()) {
        buildDefaults();
        saveProfiles();
    }
}

void DriverProfileManager::buildDefaults() {
    DriverProfile def;
    def.name          = "Default";
    def.drivingMode   = DrivingMode::NORMAL;
    def.speedLimit    = 120.0;
    def.tempWarning   = 95.0;
    def.tempCritical  = 110.0;
    def.alertsEnabled = true;
    def.totalSessions = 0;
    profiles_.push_back(def);

    DriverProfile eco;
    eco.name          = "EcoDriver";
    eco.drivingMode   = DrivingMode::ECO;
    eco.speedLimit    = 90.0;
    eco.tempWarning   = 88.0;
    eco.tempCritical  = 100.0;
    eco.alertsEnabled = true;
    eco.totalSessions = 0;
    profiles_.push_back(eco);

    DriverProfile sport;
    sport.name          = "SportDriver";
    sport.drivingMode   = DrivingMode::SPORT;
    sport.speedLimit    = 150.0;
    sport.tempWarning   = 105.0;
    sport.tempCritical  = 120.0;
    sport.alertsEnabled = true;
    sport.totalSessions = 0;
    profiles_.push_back(sport);

    activeIndex_ = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence — save
// ─────────────────────────────────────────────────────────────────────────────

bool DriverProfileManager::saveProfiles() const {
    std::ofstream f(PROFILE_FILE);
    if (!f) return false;

    f << "# SmartVehicle Driver Profiles\n";
    f << "ACTIVE=" << profiles_[activeIndex_].name << "\n\n";

    for (const auto& p : profiles_) {
        f << "[PROFILE]\n";
        f << "NAME="           << p.name                              << "\n";
        f << "MODE="           << drivingModeToString(p.drivingMode)  << "\n";
        f << "SPEED_LIMIT="    << p.speedLimit                        << "\n";
        f << "TEMP_WARNING="   << p.tempWarning                       << "\n";
        f << "TEMP_CRITICAL="  << p.tempCritical                      << "\n";
        f << "ALERTS="         << (p.alertsEnabled ? "1" : "0")       << "\n";
        f << "SESSIONS="       << p.totalSessions                     << "\n";
        f << "\n";
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence — load
// ─────────────────────────────────────────────────────────────────────────────

void DriverProfileManager::loadProfiles() {
    profiles_.clear();
    activeIndex_ = 0;

    std::ifstream f(PROFILE_FILE);
    if (!f) return;

    std::string activeName;
    std::string line;
    DriverProfile current;
    bool inBlock = false;

    auto trim = [](std::string s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
        return s;
    };

    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        if (line.rfind("ACTIVE=", 0) == 0) {
            activeName = trim(line.substr(7));
            continue;
        }
        if (line == "[PROFILE]") {
            if (inBlock) profiles_.push_back(current);
            current  = DriverProfile{};
            inBlock  = true;
            continue;
        }
        if (!inBlock) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));

        if      (key == "NAME")          current.name          = val;
        else if (key == "MODE")          current.drivingMode   = drivingModeFromString(val);
        else if (key == "SPEED_LIMIT")   current.speedLimit    = std::stod(val);
        else if (key == "TEMP_WARNING")  current.tempWarning   = std::stod(val);
        else if (key == "TEMP_CRITICAL") current.tempCritical  = std::stod(val);
        else if (key == "ALERTS")        current.alertsEnabled = (val == "1");
        else if (key == "SESSIONS")      current.totalSessions = std::stoi(val);
    }
    if (inBlock) profiles_.push_back(current);

    // Restore active index
    if (!activeName.empty()) {
        for (std::size_t i = 0; i < profiles_.size(); ++i) {
            if (profiles_[i].name == activeName) {
                activeIndex_ = i;
                break;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CRUD
// ─────────────────────────────────────────────────────────────────────────────

std::size_t DriverProfileManager::findByName(const std::string& name) const {
    for (std::size_t i = 0; i < profiles_.size(); ++i)
        if (profiles_[i].name == name) return i;
    return std::string::npos;
}

bool DriverProfileManager::createProfile(const std::string& name,
                                          DrivingMode        mode,
                                          double             speedLimit,
                                          double             tempWarning,
                                          double             tempCritical,
                                          bool               alerts)
{
    if (findByName(name) != std::string::npos) return false;
    DriverProfile p;
    p.name          = name.substr(0, 20);
    p.drivingMode   = mode;
    p.speedLimit    = speedLimit;
    p.tempWarning   = tempWarning;
    p.tempCritical  = tempCritical;
    p.alertsEnabled = alerts;
    p.totalSessions = 0;
    profiles_.push_back(p);
    saveProfiles();
    return true;
}

bool DriverProfileManager::deleteProfile(const std::string& name) {
    std::size_t idx = findByName(name);
    if (idx == std::string::npos) return false;
    if (idx == activeIndex_)      return false; // can't delete active profile
    profiles_.erase(profiles_.begin() + static_cast<std::ptrdiff_t>(idx));
    if (activeIndex_ > idx) --activeIndex_;
    saveProfiles();
    return true;
}

bool DriverProfileManager::setActiveProfile(const std::string& name) {
    std::size_t idx = findByName(name);
    if (idx == std::string::npos) return false;
    activeIndex_ = idx;
    profiles_[activeIndex_].totalSessions++;
    saveProfiles();
    return true;
}

const DriverProfile& DriverProfileManager::getActiveProfile() const {
    return profiles_[activeIndex_];
}

// ─────────────────────────────────────────────────────────────────────────────
// Display
// ─────────────────────────────────────────────────────────────────────────────

void DriverProfileManager::displayAllProfiles() const {
    std::cout << "\n";
    pbTop(BCYAN);
    pbRow("   ALL DRIVER PROFILES  [" + std::to_string(profiles_.size()) + " stored]",
          BCYAN, BWHITE);
    pbMid(BCYAN);
    for (std::size_t i = 0; i < profiles_.size(); ++i) {
        const auto& p  = profiles_[i];
        bool active    = (i == activeIndex_);
        std::string tag = active ? "  ► [ACTIVE]  " : "     ";
        std::string row = tag + p.name
                        + "  |  " + drivingModeToString(p.drivingMode)
                        + "  |  Spd≤" + std::to_string(static_cast<int>(p.speedLimit)) + " km/h"
                        + "  |  Sess:" + std::to_string(p.totalSessions);
        pbRow(row, BCYAN, active ? BGREEN : BWHITE);
    }
    pbBot(BCYAN);
}

void DriverProfileManager::displayActiveProfile() const {
    std::cout << profiles_[activeIndex_]; // uses operator<<
}

// ─────────────────────────────────────────────────────────────────────────────
// Interactive sub-menu
// ─────────────────────────────────────────────────────────────────────────────

void runDriverProfileMenu(DriverProfileManager& mgr) {
    while (true) {
        std::cout << "\n";
        pbTop(BMAGENTA);
        pbRow("   DRIVER PROFILE MANAGER", BMAGENTA, BWHITE);
        pbMid(BMAGENTA);
        pbRow("   [1]  View all profiles",             BMAGENTA, BGREEN);
        pbRow("   [2]  View active profile",           BMAGENTA, BGREEN);
        pbRow("   [3]  Switch active profile",         BMAGENTA, BYELLOW);
        pbRow("   [4]  Create new profile",            BMAGENTA, BYELLOW);
        pbRow("   [5]  Delete a profile",              BMAGENTA, BRED);
        pbRow("   [6]  Return to main menu",           BMAGENTA, BCYAN);
        pbBot(BMAGENTA);
        std::cout << "\n" << BWHITE << BOLD << "  ▶  Enter Choice [1-6]: " << RESET;

        int choice = 0;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if (choice == 1) {
            mgr.displayAllProfiles();

        } else if (choice == 2) {
            mgr.displayActiveProfile();

        } else if (choice == 3) {
            mgr.displayAllProfiles();
            std::cout << "\n" << BWHITE << "  ▷  Profile name to activate: " << RESET;
            std::string name; std::getline(std::cin, name);
            if (mgr.setActiveProfile(name))
                std::cout << BGREEN << "  ✔  Active profile → " << name << "\n" << RESET;
            else
                std::cout << BRED   << "  ✗  Profile \"" << name << "\" not found.\n" << RESET;

        } else if (choice == 4) {
            std::cout << BCYAN << "\n  --- Create New Profile ---\n" << RESET;
            auto ask = [](const std::string& msg) -> std::string {
                std::cout << BWHITE << "  ▷  " << msg << ": " << RESET;
                std::string s; std::getline(std::cin, s); return s;
            };
            auto askD = [](const std::string& msg, double def) -> double {
                std::cout << BWHITE << "  ▷  " << msg << " [" << def << "]: " << RESET;
                std::string s; std::getline(std::cin, s);
                if (s.empty()) return def;
                try { return std::stod(s); } catch (...) { return def; }
            };

            std::string name = ask("Name (max 20 chars)");
            if (name.empty()) { std::cout << BRED << "  ✗  Name required.\n" << RESET; continue; }

            std::cout << BWHITE << "  ▷  Driving Mode [0=ECO 1=NORMAL 2=SPORT 3=LIMP]: " << RESET;
            int modeInt = 1;
            std::string ms; std::getline(std::cin, ms);
            if (!ms.empty()) try { modeInt = std::stoi(ms); } catch (...) {}
            DrivingMode mode = static_cast<DrivingMode>(
                std::max(0, std::min(3, modeInt)));

            double spd  = askD("Speed limit (km/h)",   120.0);
            double warn = askD("Temp warning (°C)",      95.0);
            double crit = askD("Temp critical (°C)",    110.0);

            std::cout << BWHITE << "  ▷  Enable alerts? [1=Yes 0=No]: " << RESET;
            std::string as; std::getline(std::cin, as);
            bool alerts = (as != "0");

            if (mgr.createProfile(name, mode, spd, warn, crit, alerts)) {
                std::cout << BGREEN << "  ✔  Profile \"" << name << "\" created.\n" << RESET;
            } else {
                std::cout << BRED << "  ✗  A profile with that name already exists.\n" << RESET;
            }

        } else if (choice == 5) {
            mgr.displayAllProfiles();
            std::cout << "\n" << BWHITE << "  ▷  Profile name to delete: " << RESET;
            std::string name; std::getline(std::cin, name);
            if (mgr.deleteProfile(name))
                std::cout << BGREEN << "  ✔  Profile \"" << name << "\" deleted.\n" << RESET;
            else
                std::cout << BRED   << "  ✗  Cannot delete — profile not found or currently active.\n" << RESET;

        } else if (choice == 6) {
            break;
        }
    }
}
