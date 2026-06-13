/**
 * @file    Sensor.hpp
 * @brief   Sensor abstraction layer for the Smart Vehicle ECU.
 *
 * @details Defines the abstract base class Sensor and all concrete derived
 *          sensor types used in the Smart Cabin & Vehicle Health Monitoring
 *          System.  The hierarchy demonstrates runtime polymorphism, RAII,
 *          static member tracking, and shared random-number generation.
 *
 *          Automotive mapping:
 *          - Sensor        → MCAL / Sensor Abstraction Layer interface
 *          - Derived types → Individual ECU hardware abstraction drivers
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.1
 */

#pragma once
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <string>
#include <random>
#include <chrono>

// ─────────────────────────────────────────────────────────────────────────────
// Sensor  —  Abstract Base Class
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   Sensor
 * @brief   Abstract interface that every physical/simulated sensor must satisfy.
 *
 * @details Provides:
 *          - A pure-virtual update/display contract (runtime polymorphism).
 *          - A shared Mersenne-Twister RNG engine (seeded once per process).
 *          - A static instance counter for diagnostic purposes.
 *          - RAII-friendly default construction and virtual destructor.
 *
 * @note    Constructor increments totalSensors_; destructor does NOT decrement
 *          it (sensors are owned by shared_ptr and destroyed at shutdown).
 */
class Sensor {
public:
    /**
     * @brief  Constructs a named sensor.
     * @param  name  Human-readable identifier used in log / display output.
     */
    explicit Sensor(const std::string& name) : sensorName_(name), valid_(true) {}

    /// Virtual destructor — ensures correct destruction through base pointers.
    virtual ~Sensor() = default;

    // ── Pure-virtual interface ───────────────────────────────────────────────

    /**
     * @brief  Advances the sensor simulation by one time step.
     *
     * @details Implementations must clamp values to physical limits and apply
     *          randomised delta updates using the shared rng() engine.
     */
    virtual void update() = 0;

    /**
     * @brief  Renders one formatted row in the terminal sensor table.
     *
     * @details The row is coloured green (normal) or red (fault condition).
     *          Each implementation must emit exactly one newline-terminated
     *          line that fits inside the dashboard's box border.
     */
    virtual void display() const = 0;

    // ── Accessors ────────────────────────────────────────────────────────────

    /**
     * @brief  Returns the sensor's human-readable name.
     * @return Sensor name string set at construction.
     */
    virtual std::string getName() const { return sensorName_; }

    /**
     * @brief  Returns whether the sensor is considered valid / operational.
     * @return true if valid_, false if a fault has been flagged.
     */
    bool isValid() const { return valid_; }

    // ── Static diagnostic ────────────────────────────────────────────────────

    /**
     * @brief  Returns the total number of Sensor-derived objects ever created.
     * @return Cumulative instantiation count (never decremented).
     */
    static int getTotalSensors() { return totalSensors_; }

protected:
    std::string sensorName_; ///< Human-readable sensor identifier.
    bool        valid_;       ///< Operational validity flag.
    static int  totalSensors_; ///< Cumulative sensor instance count.

    /**
     * @brief  Lazily-initialised, process-wide Mersenne-Twister RNG engine.
     *
     * @details Seeded from the monotonic steady clock at first call.  Shared
     *          across all sensor subclasses so construction order does not
     *          affect randomness quality.
     * @return Reference to the static engine.
     */
    static std::mt19937& rng() {
        static std::mt19937 engine(
            static_cast<unsigned>(
                std::chrono::steady_clock::now().time_since_epoch().count()
            )
        );
        return engine;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// EngineTemperatureSensor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   EngineTemperatureSensor
 * @brief   Simulates the engine coolant / block temperature sensor.
 *
 * @details Value range : 60 °C – 130 °C (clamped).
 *          Delta per cycle : ±5 °C (uniform random).
 *          Initial value   : 85 °C.
 *
 * @note    Thresholds:
 *          - TEMP_HIGH     (95 °C) → WARNING alert via AlertManager.
 *          - TEMP_CRITICAL (110 °C) → CRITICAL alert via AlertManager.
 */
class EngineTemperatureSensor : public Sensor {
public:
    static constexpr double TEMP_HIGH     = 95.0;  ///< Warning threshold (°C).
    static constexpr double TEMP_CRITICAL = 110.0; ///< Critical threshold (°C).

    /// @brief Default constructor — initialises temperature to 85 °C.
    EngineTemperatureSensor();

    void   update()        override; ///< Applies ±5 °C random delta, clamped [60,130].
    void   display() const override; ///< Prints coloured temperature row.

    /**
     * @brief  Returns the current simulated engine temperature.
     * @return Temperature in degrees Celsius.
     */
    double getTemperature() const { return temperature_; }

private:
    double temperature_; ///< Current simulated temperature (°C).
    /// @brief Pre-constructed distribution — avoids per-call heap allocation.
    std::uniform_real_distribution<double> dist_{-5.0, 5.0};
};

// ─────────────────────────────────────────────────────────────────────────────
// BatterySensor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   BatterySensor
 * @brief   Simulates the 12 V lead-acid / LiFePO4 vehicle battery voltage sensor.
 *
 * @details Value range : 8.0 V – 14.8 V (clamped).
 *          Delta per cycle : ±0.3 V (uniform random).
 *          Initial value   : 12.5 V.
 *
 * @note    Threshold:
 *          - VOLTAGE_LOW (10 V) → CRITICAL alert via AlertManager.
 */
class BatterySensor : public Sensor {
public:
    static constexpr double VOLTAGE_LOW = 10.0; ///< Critical low-voltage threshold (V).

    /// @brief Default constructor — initialises voltage to 12.5 V.
    BatterySensor();

    void   update()        override; ///< Applies ±0.3 V random delta, clamped [8.0,14.8].
    void   display() const override; ///< Prints coloured voltage row.

    /**
     * @brief  Returns the current simulated battery voltage.
     * @return Voltage in Volts.
     */
    double getVoltage() const { return voltage_; }

private:
    double voltage_; ///< Current simulated voltage (V).
    /// @brief Pre-constructed distribution — avoids per-call heap allocation.
    std::uniform_real_distribution<double> dist_{-0.3, 0.3};
};

// ─────────────────────────────────────────────────────────────────────────────
// SpeedSensor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   SpeedSensor
 * @brief   Simulates the vehicle speed sensor (VSS) / wheel encoder output.
 *
 * @details Value range : 0 km/h – 160 km/h (clamped).
 *          Delta per cycle : ±10 km/h (uniform random).
 *          Initial value   : 72 km/h.
 *
 * @note    Thresholds:
 *          - OVERSPEED_THRESHOLD (120 km/h) → CRITICAL alert.
 *          - DOOR_SPEED_LIMIT    (10  km/h) → used by door-open alert logic.
 */
class SpeedSensor : public Sensor {
public:
    static constexpr double OVERSPEED_THRESHOLD = 120.0; ///< Overspeed alert threshold (km/h).
    static constexpr double DOOR_SPEED_LIMIT    =  10.0; ///< Door-open-while-moving limit (km/h).

    /// @brief Default constructor — initialises speed to 72 km/h.
    SpeedSensor();

    void   update()        override; ///< Applies ±10 km/h random delta, clamped [0,160].
    void   display() const override; ///< Prints coloured speed row.

    /**
     * @brief  Returns the current simulated vehicle speed.
     * @return Speed in km/h.
     */
    double getSpeed() const { return speed_; }

private:
    double speed_; ///< Current simulated speed (km/h).
    /// @brief Pre-constructed distribution — avoids per-call heap allocation.
    std::uniform_real_distribution<double> dist_{-10.0, 10.0};
};

// ─────────────────────────────────────────────────────────────────────────────
// TirePressureSensor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   TirePressureSensor
 * @brief   Simulates a TPMS (Tyre Pressure Monitoring System) sensor.
 *
 * @details Models a single representative tyre.  Value range: 18–38 PSI.
 *          Delta per cycle: biased –1.5 to +0.5 PSI to simulate slow leaks.
 *          Initial value: 30 PSI.
 *
 * @note    Threshold:
 *          - PRESSURE_LOW (25 PSI) → WARNING alert via AlertManager.
 */
class TirePressureSensor : public Sensor {
public:
    static constexpr double PRESSURE_LOW = 25.0; ///< Low-pressure warning threshold (PSI).

    /// @brief Default constructor — initialises pressure to 30 PSI.
    TirePressureSensor();

    void   update()        override; ///< Applies biased random delta, clamped [18,38].
    void   display() const override; ///< Prints coloured pressure row.

    /**
     * @brief  Returns the current simulated tyre pressure.
     * @return Pressure in PSI.
     */
    double getPressure() const { return pressure_; }

private:
    double pressure_; ///< Current simulated tyre pressure (PSI).
    /// @brief Pre-constructed distribution — avoids per-call heap allocation.
    std::uniform_real_distribution<double> dist_{-1.5, 0.5};
};

// ─────────────────────────────────────────────────────────────────────────────
// DoorSensor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   DoorSensor
 * @brief   Simulates a door-ajar / door-open contact sensor.
 *
 * @details Toggles between OPEN and CLOSED with a 15 % probability per cycle,
 *          subject to a minimum hold of MIN_HOLD cycles to prevent rapid
 *          oscillation.  The first 3 cycles are skipped to let the system
 *          stabilise before generating events.
 *
 * @note    Used in conjunction with SpeedSensor to raise DOOR_OPEN_WARNING
 *          when the door is open above DOOR_SPEED_LIMIT km/h.
 */
class DoorSensor : public Sensor {
public:
    /**
     * @enum  DoorStatus
     * @brief Discrete state of the door contact sensor.
     */
    enum class DoorStatus {
        CLOSED, ///< Door is fully closed and latched.
        OPEN    ///< Door is ajar or fully open.
    };

    /// @brief Default constructor — initialises door to CLOSED.
    DoorSensor();

    void       update()        override; ///< Probabilistically toggles door state.
    void       display() const override; ///< Prints coloured door-status row.

    /**
     * @brief  Returns the current door status enum value.
     * @return DoorStatus::OPEN or DoorStatus::CLOSED.
     */
    DoorStatus getStatus() const { return status_; }

    /**
     * @brief  Convenience boolean accessor.
     * @return true if door is OPEN, false if CLOSED.
     */
    bool       isOpen()    const { return status_ == DoorStatus::OPEN; }

private:
    DoorStatus status_;                    ///< Current door state.
    int        cycleCount_    = 0;         ///< Total update cycles elapsed.
    int        holdCycles_    = 0;         ///< Cycles held in current state.
    static constexpr int MIN_HOLD = 5;     ///< Minimum hold cycles before toggle allowed.
    /// @brief Pre-constructed distribution — avoids per-call heap allocation.
    std::uniform_int_distribution<int> dist_{0, 19};
};

// ─────────────────────────────────────────────────────────────────────────────
// SeatbeltSensor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   SeatbeltSensor
 * @brief   Simulates the driver seatbelt buckle sensor.
 *
 * @details Toggles between LOCKED and UNLOCKED with a 15 % probability per
 *          cycle, subject to a minimum hold of MIN_HOLD cycles.
 *
 * @note    Used in conjunction with SpeedSensor to raise SEATBELT_WARNING
 *          when the belt is unlocked while vehicle speed > 5 km/h.
 */
class SeatbeltSensor : public Sensor {
public:
    /**
     * @enum  SeatbeltStatus
     * @brief Discrete state of the seatbelt buckle sensor.
     */
    enum class SeatbeltStatus {
        LOCKED,   ///< Seatbelt is fastened / buckled.
        UNLOCKED  ///< Seatbelt is unfastened.
    };

    /// @brief Default constructor — initialises belt to LOCKED.
    SeatbeltSensor();

    void           update()        override; ///< Probabilistically toggles belt state.
    void           display() const override; ///< Prints coloured seatbelt-status row.

    /**
     * @brief  Returns the current seatbelt status enum value.
     * @return SeatbeltStatus::LOCKED or SeatbeltStatus::UNLOCKED.
     */
    SeatbeltStatus getStatus() const { return status_; }

    /**
     * @brief  Convenience boolean accessor.
     * @return true if belt is LOCKED, false if UNLOCKED.
     */
    bool           isLocked()  const { return status_ == SeatbeltStatus::LOCKED; }

private:
    SeatbeltStatus status_;                 ///< Current seatbelt state.
    int            holdCycles_    = 0;      ///< Cycles held in current state.
    static constexpr int MIN_HOLD = 6;      ///< Minimum hold cycles before toggle allowed.
    /// @brief Pre-constructed distribution — avoids per-call heap allocation.
    std::uniform_int_distribution<int> dist_{0, 19};
};

#endif // SENSOR_HPP
