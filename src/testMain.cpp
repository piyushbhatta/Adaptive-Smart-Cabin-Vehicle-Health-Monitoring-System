#include <iostream>
#include <memory>

#include "Alert.hpp"
#include "Logger.hpp"
#include "AlertEvaluator.hpp"

int main()
{
    auto logger =
        std::make_shared<EventLogger>(
            "logs/manual_test_log.txt");

    auto alertMgr =
        std::make_shared<AlertManager>();

    while (true)
    {
        SensorData s;

        int door;
        int belt;

        std::cout
            << "\n========================================\n"
            << "      MANUAL SENSOR TEST MODE\n"
            << "========================================\n";

        std::cout
            << "Engine Temperature (C): ";
        std::cin >> s.temp;

        std::cout
            << "Battery Voltage (V): ";
        std::cin >> s.volt;

        std::cout
            << "Vehicle Speed (km/h): ";
        std::cin >> s.speed;

        std::cout
            << "Tire Pressure (PSI): ";
        std::cin >> s.psi;

        std::cout
            << "Door Open ? (1=yes 0=no): ";
        std::cin >> door;

        std::cout
            << "Seatbelt Locked ? (1=yes 0=no): ";
        std::cin >> belt;

        s.doorOpen = door;
        s.beltLocked = belt;

        evaluateAlerts(
            s,
            *alertMgr,
            *logger);

        std::cout
            << "\n========================================\n"
            << "ACTIVE ALERTS\n"
            << "========================================\n";

        alertMgr->displayActiveAlerts();

        std::cout
            << "\nContinue Testing ? (y/n): ";

        char ch;
        std::cin >> ch;

        if (ch == 'n' ||
            ch == 'N')
        {
            break;
        }
    }

    return 0;
}