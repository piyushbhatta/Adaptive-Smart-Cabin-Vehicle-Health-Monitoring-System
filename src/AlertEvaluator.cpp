#include "AlertEvaluator.hpp"
#include "Sensor.hpp"

#include <sstream>
#include <iomanip>

static std::string fd(double v, int prec = 1)
{
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
}

void evaluateAlerts(
    const SensorData& s,
    AlertManager& mgr,
    EventLogger& log)
{

    if (s.temp > EngineTemperatureSensor::TEMP_CRITICAL)
    {
        bool n = mgr.raiseAlert(
            AlertSeverity::CRITICAL,
            "CRITICAL_ENGINE_OVERHEAT",
            "Engine temp " + fd(s.temp) + "C > 110C");

        if (n)
            log.logCritical("Engine overheat: " + fd(s.temp) + "C");

        mgr.clearAlert("HIGH_ENGINE_TEMP");
    }
    else
    {
        mgr.clearAlert("CRITICAL_ENGINE_OVERHEAT");

        if (s.temp > EngineTemperatureSensor::TEMP_HIGH &&
            s.temp < EngineTemperatureSensor::TEMP_CRITICAL)
        {
            bool n = mgr.raiseAlert(
                AlertSeverity::WARNING,
                "HIGH_ENGINE_TEMP",
                "Engine temp " + fd(s.temp) + "C > 95C");

            if (n)
                log.logWarning("High engine temp: " + fd(s.temp) + "C");
        }
        else
        {
            mgr.clearAlert("HIGH_ENGINE_TEMP");
        }
    }

    if (s.volt < BatterySensor::VOLTAGE_LOW)
    {
        bool n = mgr.raiseAlert(
            AlertSeverity::WARNING,
            "LOW_BATTERY_WARNING",
            "Battery " + fd(s.volt, 2) + "V < 10.0V");

        if (n)
            log.logWarning("Low battery: " + fd(s.volt, 2) + "V");
    }
    else
    {
        mgr.clearAlert("LOW_BATTERY_WARNING");
    }


    if (s.psi < TirePressureSensor::PRESSURE_LOW)
    {
        bool n = mgr.raiseAlert(
            AlertSeverity::WARNING,
            "LOW_TIRE_PRESSURE",
            "Tire pressure " + fd(s.psi) + " PSI < 25 PSI");

        if (n)
            log.logWarning("Low tire pressure: " + fd(s.psi) + " PSI");
    }
    else
    {
        mgr.clearAlert("LOW_TIRE_PRESSURE");
    }

    if (s.speed > SpeedSensor::OVERSPEED_THRESHOLD)
    {
        bool n = mgr.raiseAlert(
            AlertSeverity::WARNING,
            "OVERSPEED_WARNING",
            "Speed " + fd(s.speed) + " km/h > 120 km/h");

        if (n)
            log.logWarning("Overspeed: " + fd(s.speed) + " km/h");
    }
    else
    {
        mgr.clearAlert("OVERSPEED_WARNING");
    }

    if (s.doorOpen && s.speed > SpeedSensor::DOOR_SPEED_LIMIT)
    {
        bool n = mgr.raiseAlert(
            AlertSeverity::CRITICAL,
            "DOOR_OPEN_WARNING",
            "Door OPEN at " + fd(s.speed) + " km/h");

        if (n)
            log.logCritical("Door open while moving");
    }
    else
    {
        mgr.clearAlert("DOOR_OPEN_WARNING");
    }
    
    if (!s.beltLocked && s.speed >= 1.0)
    {
        bool n = mgr.raiseAlert(
            AlertSeverity::WARNING,
            "SEATBELT_WARNING",
            "Seatbelt UNLOCKED at " + fd(s.speed) + " km/h");

        if (n)
            log.logWarning("Seatbelt unlocked while moving");
    }
    else
    {
        mgr.clearAlert("SEATBELT_WARNING");
    }
}
