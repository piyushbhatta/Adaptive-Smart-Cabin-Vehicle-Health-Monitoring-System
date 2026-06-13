#include "Logger.hpp"
#include "Alert.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <sstream>

EventLogger::EventLogger(const std::string& filepath) : filepath_(filepath) {
    logFile_.open(filepath, std::ios::app);
    if (!logFile_.is_open())
        throw std::runtime_error("EventLogger: cannot open: " + filepath);
    std::string hdr = "\n======== SESSION START: " + currentTimestamp() + " ========";
    writeToFile(hdr);
}

EventLogger::~EventLogger() {
    if (logFile_.is_open()) {
        writeToFile("======== SESSION END:   " + currentTimestamp() + " ========\n");
        logFile_.flush();
        logFile_.close();
    }
}

std::string EventLogger::currentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return std::string(buf);
}

// ── fastTimestamp: cached per-second to avoid repeated syscalls ──────────────
const char* EventLogger::fastTimestamp() const {
    std::time_t now = std::time(nullptr);
    if (now != lastTsSec_) {
        std::strftime(cachedTs_, sizeof(cachedTs_), "%Y-%m-%d %H:%M:%S",
                      std::localtime(&now));
        lastTsSec_ = now;
    }
    return cachedTs_;
}

void EventLogger::writeToFile(const std::string& entry) {
    if (logFile_.is_open()) {
        logFile_ << entry << "\n";
        logFile_.flush();
    }
    logEntries_.push_back(entry);
}

void EventLogger::logInfo(const std::string& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    char buf[256];
    snprintf(buf, sizeof(buf), "[%s] [INFO    ] %s", fastTimestamp(), msg.c_str());
    writeToFile(buf);
}

void EventLogger::logWarning(const std::string& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    char buf[256];
    snprintf(buf, sizeof(buf), "[%s] [ALERT] %s", fastTimestamp(), msg.c_str());
    writeToFile(buf);
}

void EventLogger::logCritical(const std::string& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    char buf[256];
    snprintf(buf, sizeof(buf), "[%s] [CRITICAL] %s", fastTimestamp(), msg.c_str());
    writeToFile(buf);
}

void EventLogger::logAlert(const Alert& alert) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream oss;
    oss << alert;
    char buf[256];
    snprintf(buf, sizeof(buf), "[%s] [ALERT   ] %s", fastTimestamp(), oss.str().c_str());
    writeToFile(buf);
}

std::vector<std::string> EventLogger::searchEntries(
    const std::function<bool(const std::string&)>& predicate) const
{
    std::vector<std::string> result;
    std::copy_if(logEntries_.begin(), logEntries_.end(),
                 std::back_inserter(result), predicate);
    return result;
}

std::vector<std::string> EventLogger::searchByKeyword(const std::string& keyword) const {
    return searchEntries([&keyword](const std::string& e){
        return e.find(keyword) != std::string::npos;
    });
}
