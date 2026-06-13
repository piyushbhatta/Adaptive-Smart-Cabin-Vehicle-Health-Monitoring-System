/**
 * @file    AdaptiveAlertPrioritizer.cpp
 * @brief   Implementation of AdaptiveAlertPrioritizer.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "AdaptiveAlertPrioritizer.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace Color;

int AdaptiveAlertPrioritizer::severityWeight(AlertSeverity sev)
{
    switch (sev) {
        case AlertSeverity::CRITICAL: return 3;
        case AlertSeverity::WARNING:  return 2;
        case AlertSeverity::INFO:     return 1;
        default:                      return 1;
    }
}

int AdaptiveAlertPrioritizer::computeScore(AlertSeverity sev, int hits)
{
    int base    = severityWeight(sev) * SEVERITY_FACTOR;
    int penalty = hits * FREQUENCY_PENALTY;
    return std::max(1, base - penalty);
}


void AdaptiveAlertPrioritizer::record(const std::string& code)
{
    ++hitCounts_[code];   
}

std::vector<PrioritizedAlert>
AdaptiveAlertPrioritizer::prioritize(const AlertManager& alertMgr) const
{
    const auto& active = alertMgr.getActiveAlerts();

    std::vector<PrioritizedAlert> ranked;
    ranked.reserve(active.size());

    for (const Alert& a : active) {
        auto it   = hitCounts_.find(a.getCode());
        int  hits = (it != hitCounts_.end()) ? it->second : 0;
        int  score = computeScore(a.getSeverity(), hits);
        ranked.push_back({ a, score, hits });
    }

    std::sort(ranked.begin(), ranked.end());
    return ranked;
}

int AdaptiveAlertPrioritizer::hitCount(const std::string& code) const
{
    auto it = hitCounts_.find(code);
    return (it != hitCounts_.end()) ? it->second : 0;
}

void AdaptiveAlertPrioritizer::reset()
{
    hitCounts_.clear();
}


void AdaptiveAlertPrioritizer::display(std::ostream& os,
                                       const AlertManager& alertMgr) const
{
    constexpr int W = 72;   

    auto line = [&](char fill) {
        os << BOLD << BYELLOW << "  \u2551" << std::string(W, fill) << "\u2551\n" << RESET;
    };

    os << "\n" << BOLD << BYELLOW
       << "  \u2554" << std::string(W, '=') << "\u2557\n"
       << "  \u2551" << std::left << std::setw(W)
       << "  ADAPTIVE ALERT PRIORITIZER" << "\u2551\n"
       << "  \u2560" << std::string(W, '=') << "\u2563\n"
       << RESET;

    auto ranked = prioritize(alertMgr);

    if (ranked.empty()) {
        std::string msg = "  No active alerts — all systems nominal.";
        int pad = W - static_cast<int>(msg.size());
        os << BGREEN << BOLD
           << "  \u2551" << msg << std::string(pad < 0 ? 0 : pad, ' ') << "\u2551\n"
           << RESET;
    } else {
      
        std::ostringstream hdr;
        hdr << "  "
            << std::left  << std::setw(4)  << "Rank"
            << std::right << std::setw(6)  << "Score"
            << std::right << std::setw(6)  << "Hits"
            << "  "
            << std::left  << std::setw(10) << "Severity"
            << std::left  << std::setw(28) << "Code"
            << "Message";
        std::string hdrStr = hdr.str();
        int hdrPad = W - static_cast<int>(hdrStr.size());
        os << BOLD << BCYAN
           << "  \u2551" << hdrStr << std::string(hdrPad < 0 ? 0 : hdrPad, ' ') << "\u2551\n"
           << RESET;
        line('-');

   
        int rank = 1;
        for (const auto& pa : ranked) {
            const char* sevColor =
                (pa.alert.getSeverity() == AlertSeverity::CRITICAL) ? BRED :
                (pa.alert.getSeverity() == AlertSeverity::WARNING)  ? BYELLOW : BGREEN;

            std::string sevStr = severityToString(pa.alert.getSeverity());
            
            sevStr.erase(sevStr.find_last_not_of(' ') + 1);

          
            std::string msg = pa.alert.getMessage();
            constexpr int MSG_MAX = 18;
            if (static_cast<int>(msg.size()) > MSG_MAX)
                msg = msg.substr(0, MSG_MAX - 2) + "..";

            std::ostringstream row;
            row << "  "
                << std::left  << std::setw(4)  << ("#" + std::to_string(rank))
                << std::right << std::setw(6)  << pa.score
                << std::right << std::setw(6)  << pa.hitCount
                << "  "
                << std::left  << std::setw(10) << sevStr
                << std::left  << std::setw(28) << pa.alert.getCode()
                << msg;

            std::string rowStr = row.str();
            int pad = W - static_cast<int>(rowStr.size());

            os << BOLD << BYELLOW << "  \u2551" << RESET
               << sevColor << BOLD << rowStr
               << std::string(pad < 0 ? 0 : pad, ' ')
               << RESET
               << BOLD << BYELLOW << "\u2551\n" << RESET;
            ++rank;
        }
    }

   
    os << BOLD << BYELLOW
       << "  \u255a" << std::string(W, '=') << "\u255d\n"
       << RESET << "\n";
}
