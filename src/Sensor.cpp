#include "Sensor.hpp"
#include "Dashboard.hpp"   // Color namespace
#include <iostream>
#include <iomanip>
#include <cstdio>

using namespace Color;

int Sensor::totalSensors_ = 0;

// ── Helper: format double with fixed decimal places ──────────────────────────
// Uses snprintf into a stack buffer — avoids heap allocation and
// ostringstream locale machinery that fires on every display() call.
static std::string fmtVal(double v, int prec = 1) {
    char buf[24];
    if (prec == 2)
        std::snprintf(buf, sizeof(buf), "%.2f", v);
    else
        std::snprintf(buf, sizeof(buf), "%.1f", v);
    return buf;   // single SSO construction; no locale overhead
}

// ── Table row printer ─────────────────────────────────────────────────────────
//
//  The sensor table uses a fixed box width of 64 characters.
//  Emoji glyphs occupy 2 terminal columns each, so we use plain-text
//  label padding to guarantee consistent column alignment regardless of
//  font/terminal emoji support.
//
//  Row format:
//   "  ║  <label:22>  <value+unit:12>  <tag>   ║"
//
static void printRow(const std::string& label,
                     const std::string& value,
                     const std::string& unit,
                     const std::string& tag,
                     const char* valueColor,
                     bool isAlert)
{
    constexpr int LABEL_W  = 30;
    constexpr int VALUE_W  = 15;
    constexpr int STATUS_W = 25;

    // Build "value [unit]" without a heap-allocated ostringstream
    std::string valStr = unit.empty() ? value : (value + " " + unit);

    std::cout
        << BOLD << BBLUE << "  ║" << RESET

        << BCYAN << std::left
        << std::setw(LABEL_W)
        << (" " + label)
        << RESET

        << BOLD << BBLUE << "│" << RESET

        << valueColor
        << std::right
        << std::setw(VALUE_W)
        << valStr
        << RESET

        << BOLD << BBLUE << "│" << RESET;

    if (isAlert)
    {
        std::cout << BRED;
    }
    else
    {
        std::cout << BGREEN;
    }

    std::cout
        << std::left
        << std::setw(STATUS_W)
        << (" " + tag)
        << RESET

        << BOLD << BBLUE
        << "║"
        << RESET
        << "\n";
}

// ─── EngineTemperatureSensor ──────────────────────────────────────────────────
EngineTemperatureSensor::EngineTemperatureSensor()
    : Sensor("EngineTemp"), temperature_(85.0) { ++totalSensors_; }

void EngineTemperatureSensor::update() {
    temperature_ += dist_(rng());
    if (temperature_ < 60.0)  temperature_ = 60.0;
    if (temperature_ > 130.0) temperature_ = 130.0;
}

void EngineTemperatureSensor::display() const {
    bool   isCritical = temperature_ > TEMP_CRITICAL;
    bool   isHigh     = temperature_ > TEMP_HIGH;
    bool   isAlert    = isCritical || isHigh;
    const char* vc    = isCritical ? BRED : (isHigh ? BYELLOW : BGREEN);
    std::string tag   = isCritical ? "[!! CRITICAL OVERHEAT]"
                      : (isHigh   ? "[HIGH TEMP]"
                                  : "[NORMAL]");
    printRow("[ENG] Engine Temp", fmtVal(temperature_), "deg C", tag, vc, isAlert);
}

// ─── BatterySensor ────────────────────────────────────────────────────────────
BatterySensor::BatterySensor()
    : Sensor("Battery"), voltage_(12.5) { ++totalSensors_; }

void BatterySensor::update() {
    voltage_ += dist_(rng());
    if (voltage_ < 8.0)  voltage_ = 8.0;
    if (voltage_ > 14.8) voltage_ = 14.8;
}

void BatterySensor::display() const {
    bool        isAlert = voltage_ < VOLTAGE_LOW;
    const char* vc      = isAlert ? BRED : BGREEN;
    std::string tag     = isAlert ? "[!! LOW BATTERY]" : "[NORMAL]";
    printRow("[BAT] Battery Volt", fmtVal(voltage_, 2), "V", tag, vc, isAlert);
}

// ─── SpeedSensor ──────────────────────────────────────────────────────────────
SpeedSensor::SpeedSensor()
    : Sensor("Speed"), speed_(72.0) { ++totalSensors_; }

void SpeedSensor::update() {
    speed_ += dist_(rng());
    if (speed_ < 0.0)   speed_ = 0.0;
    if (speed_ > 160.0) speed_ = 160.0;
}

void SpeedSensor::display() const {
    bool        isOver  = speed_ > OVERSPEED_THRESHOLD;
    bool        isHigh  = speed_ > 100.0;
    bool        isAlert = isOver;
    const char* vc      = isOver ? BRED : (isHigh ? BYELLOW : BGREEN);
    std::string tag     = isOver ? "[!! OVERSPEED]"
                        : (isHigh ? "[HIGH SPEED]"
                                  : "[NORMAL]");
    printRow("[SPD] Vehicle Speed", fmtVal(speed_), "km/h", tag, vc, isAlert);
}

// ─── TirePressureSensor ───────────────────────────────────────────────────────
TirePressureSensor::TirePressureSensor()
    : Sensor("TirePressure"), pressure_(30.0) { ++totalSensors_; }

void TirePressureSensor::update() {
    pressure_ += dist_(rng());
    if (pressure_ < 18.0) pressure_ = 18.0;
    if (pressure_ > 38.0) pressure_ = 38.0;
}

void TirePressureSensor::display() const {
    bool        isAlert = pressure_ < PRESSURE_LOW;
    const char* vc      = isAlert ? BRED : BGREEN;
    std::string tag     = isAlert ? "[!! LOW PRESSURE]" : "[NORMAL]";
    printRow("[TYR] Tire Pressure", fmtVal(pressure_), "PSI", tag, vc, isAlert);
}

// ─── DoorSensor ───────────────────────────────────────────────────────────────
DoorSensor::DoorSensor()
    : Sensor("Door"), status_(DoorStatus::CLOSED), cycleCount_(0), holdCycles_(0)
{ ++totalSensors_; }

void DoorSensor::update() {
    ++cycleCount_;
    if (cycleCount_ <= 3) return;
    ++holdCycles_;
    if (holdCycles_ < MIN_HOLD) return;
    if (dist_(rng()) < 3) {
        status_     = (status_ == DoorStatus::CLOSED) ? DoorStatus::OPEN : DoorStatus::CLOSED;
        holdCycles_ = 0;
    }
}

void DoorSensor::display() const {
    bool        open    = (status_ == DoorStatus::OPEN);
    const char* vc      = open ? BRED : BGREEN;
    std::string val     = open ? "OPEN  " : "CLOSED";
    std::string tag     = open ? "[!! DOOR OPEN]" : "[SECURE]";
    printRow("[DOR] Door Status", val, "", tag, vc, open);
}

// ─── SeatbeltSensor ───────────────────────────────────────────────────────────
SeatbeltSensor::SeatbeltSensor()
    : Sensor("Seatbelt"), status_(SeatbeltStatus::LOCKED), holdCycles_(0)
{ ++totalSensors_; }

void SeatbeltSensor::update() {
    ++holdCycles_;
    if (holdCycles_ < MIN_HOLD) return;
    if (dist_(rng()) < 3) {
        status_     = (status_ == SeatbeltStatus::LOCKED) ? SeatbeltStatus::UNLOCKED : SeatbeltStatus::LOCKED;
        holdCycles_ = 0;
    }
}

void SeatbeltSensor::display() const {
    bool        locked  = (status_ == SeatbeltStatus::LOCKED);
    const char* vc      = locked ? BGREEN : BRED;
    std::string val     = locked ? "LOCKED  " : "UNLOCKED";
    std::string tag     = locked ? "[SECURE]" : "[!! BELT OPEN]";
    printRow("[SBT] Seatbelt", val, "", tag, vc, !locked);
}
