/**
 * @file    DriverProfile.hpp
 * @brief   Driver Profile Management for the Smart Vehicle ECU.
 *
 * @details Provides named driver profiles that store personalised alert
 *          thresholds (speed limit, temperature sensitivity) and driving
 *          mode.  Profiles persist to a simple text file so they survive
 *          across sessions.
 *
 *          Automotive mapping: Driver Identification / Personalisation ECU,
 *          similar to keyfob-linked driver memory in production vehicles.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef DRIVER_PROFILE_HPP
#define DRIVER_PROFILE_HPP

#include <string>
#include <vector>
#include <ostream>

// ─────────────────────────────────────────────────────────────────────────────
// DrivingMode
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @enum  DrivingMode
 * @brief Driving style preset that scales alert sensitivity.
 *
 * @details
 *  - ECO      : Conservative thresholds, emphasises fuel economy.
 *  - NORMAL   : Factory default OEM thresholds.
 *  - SPORT    : Relaxed speed/temp limits for performance driving.
 *  - LIMP     : Reduced limits for fault/limp-home scenarios.
 */
enum class DrivingMode {
    ECO    = 0,
    NORMAL = 1,
    SPORT  = 2,
    LIMP   = 3
};

/// Converts DrivingMode enum to a display string.
std::string drivingModeToString(DrivingMode m);

/// Converts a string back to DrivingMode (for file I/O).
DrivingMode drivingModeFromString(const std::string& s);

// ─────────────────────────────────────────────────────────────────────────────
// DriverProfile
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  DriverProfile
 * @brief   Immutable record of a single driver's preferences.
 *
 * @details Stores:
 *  - name            : Human-readable profile name (≤ 20 chars).
 *  - drivingMode     : DrivingMode enum (affects threshold scaling).
 *  - speedLimit      : Personal speed-warning threshold (km/h).
 *  - tempWarning     : Personal engine-temp warning threshold (°C).
 *  - tempCritical    : Personal engine-temp critical threshold (°C).
 *  - alertsEnabled   : Whether audible/visual alert notifications fire.
 *  - totalSessions   : Lifetime session count (incremented on load).
 *
 * @note  operator<< renders a formatted profile box for the terminal.
 */
struct DriverProfile {
    std::string name          = "Default";  ///< Display name.
    DrivingMode drivingMode   = DrivingMode::NORMAL; ///< Driving style.
    double      speedLimit    = 120.0;      ///< Personal over-speed threshold (km/h).
    double      tempWarning   = 95.0;       ///< Engine temp warning threshold (°C).
    double      tempCritical  = 110.0;      ///< Engine temp critical threshold (°C).
    bool        alertsEnabled = true;       ///< Whether alerts are active.
    int         totalSessions = 0;          ///< Lifetime session counter.

    /// Stream insertion — renders a labelled profile box.
    friend std::ostream& operator<<(std::ostream& os, const DriverProfile& p);
};

// ─────────────────────────────────────────────────────────────────────────────
// DriverProfileManager
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  DriverProfileManager
 * @brief  Manages a collection of DriverProfile objects with file persistence.
 *
 * @details On construction the manager loads profiles from a text file
 *          (data/driver_profiles.txt).  Profiles are stored one-per-block
 *          in a simple KEY=VALUE format.
 *
 *          The active profile determines the thresholds forwarded to
 *          AlertEvaluator at the start of each simulation session.
 *
 * Usage:
 * @code
 *   DriverProfileManager pm;
 *   pm.createProfile("Alice", DrivingMode::ECO, 100.0, 90.0, 105.0, true);
 *   pm.setActiveProfile("Alice");
 *   const DriverProfile& p = pm.getActiveProfile();
 * @endcode
 */
class DriverProfileManager {
public:
    /// Path to the on-disk profile store.
    static constexpr const char* PROFILE_FILE = "data/driver_profiles.txt";

    /**
     * @brief  Constructs the manager and loads profiles from disk.
     *
     * @details If the file does not exist, a built-in "Default" profile is
     *          created automatically so the system always has a valid active
     *          profile.
     */
    DriverProfileManager();

    // ── Profile CRUD ─────────────────────────────────────────────────────────

    /**
     * @brief  Creates and stores a new driver profile.
     *
     * @param  name         Profile name (must be unique, ≤ 20 chars).
     * @param  mode         Driving mode preset.
     * @param  speedLimit   Personal speed-alert threshold (km/h).
     * @param  tempWarning  Engine temp warning (°C).
     * @param  tempCritical Engine temp critical (°C).
     * @param  alerts       Whether alerts fire for this profile.
     * @return true if created; false if a profile with that name already exists.
     */
    bool createProfile(const std::string& name,
                       DrivingMode        mode,
                       double             speedLimit,
                       double             tempWarning,
                       double             tempCritical,
                       bool               alerts);

    /**
     * @brief  Deletes a named profile from the store.
     * @param  name  Profile to remove.
     * @return true if deleted; false if not found or if it was the active profile.
     */
    bool deleteProfile(const std::string& name);

    /**
     * @brief  Sets the active profile by name.
     * @param  name  Profile to activate.
     * @return true if activated; false if not found.
     */
    bool setActiveProfile(const std::string& name);

    // ── Accessors ─────────────────────────────────────────────────────────────

    /**
     * @brief  Returns a const reference to the currently active profile.
     * @return Active DriverProfile.
     */
    const DriverProfile& getActiveProfile() const;

    /**
     * @brief  Returns all stored profiles.
     * @return Const reference to the internal vector.
     */
    const std::vector<DriverProfile>& getAllProfiles() const { return profiles_; }

    /**
     * @brief  Returns the number of stored profiles.
     */
    std::size_t count() const { return profiles_.size(); }

    // ── Persistence ───────────────────────────────────────────────────────────

    /**
     * @brief  Saves all profiles to PROFILE_FILE.
     * @return true on success.
     */
    bool saveProfiles() const;

    /**
     * @brief  Reloads profiles from PROFILE_FILE (discards unsaved changes).
     */
    void loadProfiles();

    // ── Display ───────────────────────────────────────────────────────────────

    /**
     * @brief  Renders a formatted list of all profiles to stdout.
     */
    void displayAllProfiles() const;

    /**
     * @brief  Renders the active profile to stdout.
     */
    void displayActiveProfile() const;

private:
    std::vector<DriverProfile> profiles_;      ///< All loaded profiles.
    std::size_t                activeIndex_{0}; ///< Index of the active profile.

    /// Returns the index of a profile by name, or npos if not found.
    std::size_t findByName(const std::string& name) const;

    /// Builds default profiles when no file exists.
    void buildDefaults();
};

// ─────────────────────────────────────────────────────────────────────────────
// Interactive menu helper (called from main.cpp)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Runs the interactive Driver Profile Management sub-menu.
 *
 * @details Presents create / delete / switch / list operations and then
 *          returns.  The caller owns the DriverProfileManager and reads the
 *          active profile after this returns.
 *
 * @param  mgr  Reference to the application-level DriverProfileManager.
 */
void runDriverProfileMenu(DriverProfileManager& mgr);

#endif // DRIVER_PROFILE_HPP
