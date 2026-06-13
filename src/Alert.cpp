#include "Alert.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <ctime>

int Alert::totalAlertCount_ = 0;


std::string severityToString(AlertSeverity sev) {
    switch (sev) {
        case AlertSeverity::INFO:     return "INFO    ";
        case AlertSeverity::WARNING:  return "WARNING ";
        case AlertSeverity::CRITICAL: return "CRITICAL";
        default:                      return "UNKNOWN ";
    }
}

std::string Alert::makeTimestamp() {
    auto now    = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return std::string(buf);
}

Alert::Alert(AlertSeverity sev, const std::string& code, const std::string& msg)
    : severity_(sev), code_(code), message_(msg), timestamp_(makeTimestamp())
{ ++totalAlertCount_; }

std::ostream& operator<<(std::ostream& os, const Alert& a) {
    os << "[" << a.timestamp_ << "] "
       << "[" << severityToString(a.severity_) << "] "
       << std::left << std::setw(32) << a.code_
       << " : " << a.message_;
    return os;
}

bool Alert::operator<(const Alert& other) const {
    if (severity_ != other.severity_)
        return static_cast<int>(severity_) > static_cast<int>(other.severity_);
    return code_ < other.code_;
}

bool Alert::operator==(const Alert& other) const {
    return code_ == other.code_;
}


bool AlertManager::raiseAlert(AlertSeverity sev, const std::string& code,
                              const std::string& msg)
{
    if (activeByCode_.find(code) != activeByCode_.end())
        return false;   // already active – silent, no duplicate

    Alert alert(sev, code, msg);
    activeByCode_.emplace(code, alert);
    activeAlerts_.push_back(alert);
    alertHistory_.push_back(alert);

    std::ostringstream oss;
    oss << "ALERT: " << code;
    pendingNotifications_.push_back(oss.str());

    return true;   
}

void AlertManager::clearAlert(const std::string& code) {
    auto it = activeByCode_.find(code);
    if (it == activeByCode_.end()) return;

    activeAlerts_.erase(
        std::remove_if(activeAlerts_.begin(), activeAlerts_.end(),
            [&code](const Alert& a){ return a.getCode() == code; }),
        activeAlerts_.end()
    );
    activeByCode_.erase(it);
}

void AlertManager::pushNotification(const std::string& line) {
    pendingNotifications_.push_back(line);
}

std::vector<std::string> AlertManager::drainNotifications() {
    std::vector<std::string> out;
    out.swap(pendingNotifications_);
    return out;
}

std::vector<Alert> AlertManager::filterHistory(AlertSeverity minSev) const {
    std::vector<Alert> result;
    std::copy_if(alertHistory_.begin(), alertHistory_.end(),
                 std::back_inserter(result),
                 [minSev](const Alert& a){
                     return static_cast<int>(a.getSeverity()) >=
                            static_cast<int>(minSev);
                 });
    return result;
}

std::vector<Alert> AlertManager::searchHistory(const std::string& keyword) const {
    std::vector<Alert> result;
    std::copy_if(alertHistory_.begin(), alertHistory_.end(),
                 std::back_inserter(result),
                 [&keyword](const Alert& a){
                     return a.getMessage().find(keyword) != std::string::npos ||
                            a.getCode().find(keyword)    != std::string::npos;
                 });
    return result;
}

void AlertManager::displayActiveAlerts() const {
    if (activeAlerts_.empty()) {
        std::cout << "  No active alerts.\n";
        return;
    }
    for (const auto& a : activeAlerts_)
        std::cout << "  " << a << "\n";
}

void AlertManager::displayAlertHistory() const {
    if (alertHistory_.empty()) {
        std::cout << "  No alert history.\n";
        return;
    }
    for (const auto& a : alertHistory_)
        std::cout << "  " << a << "\n";
}

void AlertManager::clearAllAlerts() {
    activeAlerts_.clear();
    activeByCode_.clear();
}
