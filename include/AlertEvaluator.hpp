#pragma once

#include "Alert.hpp"
#include "Logger.hpp"

struct SensorData
{
    double temp;
    double volt;
    double speed;
    double psi;

    bool doorOpen;
    bool beltLocked;
};

void evaluateAlerts(
    const SensorData& s,
    AlertManager& mgr,
    EventLogger& log);