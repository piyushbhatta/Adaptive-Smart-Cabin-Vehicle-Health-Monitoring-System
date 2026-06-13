/**
 * @file    CanBus.cpp
 * @brief   Implementation of CanFrame, CanBusSimulator.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "CanBus.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <random>
#include <chrono>
#include <thread>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
//  Thread-local RNG for error injection (avoids mutex contention on rng)
// ─────────────────────────────────────────────────────────────────────────────
static std::mt19937& rng() {
    thread_local std::mt19937 gen{std::random_device{}()};
    return gen;
}
static double randDouble() {
    thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng());
}

// ─────────────────────────────────────────────────────────────────────────────
//  CanFrame helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string CanFrame::toHexString() const
{
    std::ostringstream os;
    os << "ID:0x" << std::uppercase << std::hex << std::setw(3) << std::setfill('0')
       << arbId
       << " [" << std::dec << static_cast<int>(dlc) << "] ";
    for (uint8_t i = 0; i < dlc && i < 8; ++i)
        os << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(data[i]) << " ";
    os << "| CRC:0x" << std::uppercase << std::hex
       << std::setw(2) << std::setfill('0') << static_cast<int>(crc8);
    if (errorFrame) os << " [ERR]";
    return os.str();
}

std::string CanFrame::decodeToPhysical() const
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(1);

    auto u16 = [this](int byteOffset) -> uint16_t {
        return (static_cast<uint16_t>(data[byteOffset]) << 8) |
                static_cast<uint16_t>(data[byteOffset + 1]);
    };

    switch (arbId) {
    case CanId::ENGINE_TEMP: {
        double v = static_cast<double>(u16(0)) * 0.5 - 40.0;
        os << "Engine Temp:    " << v << " °C";
        break;
    }
    case CanId::BATTERY_VOLT: {
        double v = static_cast<double>(u16(0)) * 0.01;
        os << "Battery Volt:   " << v << " V";
        break;
    }
    case CanId::VEHICLE_SPEED: {
        double v = static_cast<double>(u16(0)) * 0.1;
        os << "Vehicle Speed:  " << v << " km/h";
        break;
    }
    case CanId::TIRE_PRESSURE: {
        double v = static_cast<double>(u16(0)) * 0.1;
        os << "Tire Pressure:  " << v << " PSI";
        break;
    }
    case CanId::DOOR_BELT: {
        bool doorOpen   = (data[0] & 0x01) != 0;
        bool beltLocked = (data[0] & 0x02) != 0;
        os << "Door/Belt:      Door=" << (doorOpen   ? "OPEN" : "CLOSED")
           << " Belt=" << (beltLocked ? "LOCKED" : "UNLOCKED");
        break;
    }
    case CanId::ALERT_FLAGS: {
        os << "Alert Flags:    0x" << std::uppercase << std::hex
           << static_cast<int>(data[0]);
        break;
    }
    default:
        os << "Unknown ID:0x" << std::uppercase << std::hex
           << std::setw(3) << std::setfill('0') << arbId;
        break;
    }
    return os.str();
}

std::ostream& operator<<(std::ostream& os, const CanFrame& f)
{
    os << "[+" << std::setw(6) << f.timestampMs << "ms] "
       << f.toHexString()
       << "  →  " << f.decodeToPhysical();
    return os;
}

// ─────────────────────────────────────────────────────────────────────────────
//  CanBusSimulator — private statics
// ─────────────────────────────────────────────────────────────────────────────

uint8_t CanBusSimulator::crc8(const uint8_t* d, uint8_t len)
{
    // CRC-8/MAXIM polynomial: 0x31, init=0x00, no reflect (simplified)
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= d[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
    return crc;
}

void CanBusSimulator::maybeInjectError(CanFrame& frame)
{
    if (randDouble() < ERROR_RATE) {
        // Flip a random bit in a random data byte
        std::uniform_int_distribution<int> byteIdx(0, frame.dlc > 0 ? frame.dlc-1 : 0);
        std::uniform_int_distribution<int> bitIdx(0, 7);
        frame.data[byteIdx(rng())] ^= static_cast<uint8_t>(1 << bitIdx(rng()));
        frame.errorFrame = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CanBusSimulator — construction / destruction
// ─────────────────────────────────────────────────────────────────────────────

CanBusSimulator::CanBusSimulator()
    : startTime_(std::chrono::steady_clock::now())
{
    running_ = true;
    txThread_ = std::thread(&CanBusSimulator::transmitLoop, this);
}

CanBusSimulator::~CanBusSimulator()
{
    running_ = false;
    if (txThread_.joinable())
        txThread_.join();
}

// ─────────────────────────────────────────────────────────────────────────────
//  CanBusSimulator — private helpers
// ─────────────────────────────────────────────────────────────────────────────

uint64_t CanBusSimulator::elapsedMs() const
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime_).count());
}

CanFrame CanBusSimulator::buildFrame(uint16_t id, const uint8_t* bytes, uint8_t dlc) const
{
    CanFrame f;
    f.arbId       = id & 0x7FFu;   // mask to 11-bit
    f.dlc         = dlc;
    std::memcpy(f.data.data(), bytes, dlc);
    f.crc8        = crc8(bytes, dlc);
    f.timestampMs = elapsedMs();
    f.errorFrame  = false;
    return f;
}

void CanBusSimulator::updateBusLoad(int frameBits)
{
    // Bus load = frameBits / (BUS_SPEED_KBPS * 1000 bits/s) expressed as %
    // We approximate per-frame contribution and blend into a running average.
    // One frame contributes frameBits bits out of 1 second of bus capacity.
    double contribution = (static_cast<double>(frameBits) /
                           static_cast<double>(BUS_SPEED_KBPS * 1000)) * 100.0;
    // Exponential moving average (alpha = 0.1)
    stats_.busLoadPct = 0.9 * stats_.busLoadPct + 0.1 * contribution;
}

// ─────────────────────────────────────────────────────────────────────────────
//  transmitLoop  —  background thread
// ─────────────────────────────────────────────────────────────────────────────

void CanBusSimulator::transmitLoop()
{
    // CAN frame on-wire bit count ≈ 44 + 8*dlc  (header + stuffing overhead ~20%)
    // At 500 kbps, one 8-byte frame ≈ (44+64)*1.2 / 500000 s ≈ 0.26 ms
    // We sleep a proportional amount after each frame to simulate bus speed.

    while (running_) {
        CanFrame frame;
        bool hasFrame = false;
        {
            std::lock_guard<std::mutex> lk(busMutex_);
            if (!txQueue_.empty()) {
                frame    = txQueue_.front();
                txQueue_.pop_front();
                hasFrame = true;
            }
        }

        if (hasFrame) {
            // Simulate error injection
            maybeInjectError(frame);

            {
                std::lock_guard<std::mutex> lk(busMutex_);

                ++stats_.txFrames;
                ++stats_.rxFrames;   // loopback — every TX is also an RX here

                if (frame.errorFrame) {
                    ++stats_.errorFrames;
                    if (errorLog_.size() >= 20) errorLog_.erase(errorLog_.begin());
                    errorLog_.push_back(frame);
                }

                if (rxLog_.size() >= RX_LOG_CAPACITY)
                    rxLog_.erase(rxLog_.begin());
                rxLog_.push_back(frame);

                // On-wire bits: 44 (fixed overhead) + 8*dlc
                int frameBits = 44 + 8 * static_cast<int>(frame.dlc);
                updateBusLoad(frameBits);
            }

            // Simulate transmission delay: bits/speed in microseconds
            int frameBits = 44 + 8 * static_cast<int>(frame.dlc);
            int delayUs   = (frameBits * 1000) / BUS_SPEED_KBPS;   // µs
            std::this_thread::sleep_for(std::chrono::microseconds(delayUs));
        } else {
            // No frames — idle sleep
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  encode  —  sensor snapshot → 6 CAN frames → TX queue
// ─────────────────────────────────────────────────────────────────────────────

void CanBusSimulator::encode(const SensorData& s)
{
    // Build all 6 frames before acquiring the lock
    std::array<CanFrame, 6> frames;

    // ── 0x100  Engine Temperature ─────────────────────────────────────────
    {
        uint16_t raw = static_cast<uint16_t>((s.temp + 40.0) / 0.5);
        uint8_t  b[2] = { static_cast<uint8_t>(raw >> 8),
                          static_cast<uint8_t>(raw & 0xFF) };
        frames[0] = buildFrame(CanId::ENGINE_TEMP, b, 2);
    }

    // ── 0x200  Battery Voltage ────────────────────────────────────────────
    {
        uint16_t raw = static_cast<uint16_t>(s.volt / 0.01);
        uint8_t  b[2] = { static_cast<uint8_t>(raw >> 8),
                          static_cast<uint8_t>(raw & 0xFF) };
        frames[1] = buildFrame(CanId::BATTERY_VOLT, b, 2);
    }

    // ── 0x300  Vehicle Speed ──────────────────────────────────────────────
    {
        uint16_t raw = static_cast<uint16_t>(s.speed / 0.1);
        uint8_t  b[2] = { static_cast<uint8_t>(raw >> 8),
                          static_cast<uint8_t>(raw & 0xFF) };
        frames[2] = buildFrame(CanId::VEHICLE_SPEED, b, 2);
    }

    // ── 0x400  Tire Pressure ──────────────────────────────────────────────
    {
        uint16_t raw = static_cast<uint16_t>(s.psi / 0.1);
        uint8_t  b[2] = { static_cast<uint8_t>(raw >> 8),
                          static_cast<uint8_t>(raw & 0xFF) };
        frames[3] = buildFrame(CanId::TIRE_PRESSURE, b, 2);
    }

    // ── 0x500  Door + Seatbelt bitmask ────────────────────────────────────
    {
        uint8_t mask = 0x00;
        if (s.doorOpen)    mask |= 0x01;
        if (s.beltLocked)  mask |= 0x02;
        frames[4] = buildFrame(CanId::DOOR_BELT, &mask, 1);
    }

    // ── 0x600  Alert Flags (reserved / future) ───────────────────────────
    {
        uint8_t flags = 0x00;
        frames[5] = buildFrame(CanId::ALERT_FLAGS, &flags, 1);
    }

    // ── Enqueue ───────────────────────────────────────────────────────────
    std::lock_guard<std::mutex> lk(busMutex_);
    for (auto& f : frames) {
        if (txQueue_.size() < TX_QUEUE_CAPACITY) {
            txQueue_.push_back(f);
        } else {
            ++stats_.txDropped;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  search / filter
// ─────────────────────────────────────────────────────────────────────────────

std::vector<CanFrame> CanBusSimulator::searchRxLog(
    const std::function<bool(const CanFrame&)>& pred) const
{
    std::lock_guard<std::mutex> lk(busMutex_);
    std::vector<CanFrame> result;
    std::copy_if(rxLog_.begin(), rxLog_.end(),
                 std::back_inserter(result), pred);
    return result;
}

std::vector<CanFrame> CanBusSimulator::filterById(uint16_t arbId) const
{
    return searchRxLog([arbId](const CanFrame& f){
        return f.arbId == arbId;
    });
}

CanBusStats CanBusSimulator::getStats() const
{
    std::lock_guard<std::mutex> lk(busMutex_);
    return stats_;
}

std::size_t CanBusSimulator::txQueueDepth() const
{
    std::lock_guard<std::mutex> lk(busMutex_);
    return txQueue_.size();
}

void CanBusSimulator::reset()
{
    std::lock_guard<std::mutex> lk(busMutex_);
    txQueue_.clear();
    rxLog_.clear();
    errorLog_.clear();
    stats_ = CanBusStats{};
}

// ─────────────────────────────────────────────────────────────────────────────
//  display
// ─────────────────────────────────────────────────────────────────────────────

void CanBusSimulator::display(std::ostream& os) const
{
    constexpr int W = 78;   // inner box width

    auto hdr = [&](const std::string& title, const char* col) {
        os << "\n" << BOLD << col
           << "  \u2554" << std::string(W, '=') << "\u2557\n"
           << "  \u2551" << std::left << std::setw(W) << ("  " + title) << "\u2551\n"
           << "  \u2560" << std::string(W, '=') << "\u2563\n"
           << RESET;
    };
    auto foot = [&](const char* col) {
        os << BOLD << col << "  \u255a" << std::string(W, '=') << "\u255d\n" << RESET;
    };
    auto row = [&](const char* col, const std::string& label, const std::string& val) {
        std::string line = "  " + label + ": " + val;
        int pad = W - static_cast<int>(line.size());
        os << BOLD << col << "  \u2551" << RESET
           << col << line << std::string(pad < 0 ? 0 : pad, ' ') << RESET
           << BOLD << col << "\u2551\n" << RESET;
    };
    auto textRow = [&](const char* col, const std::string& text) {
        std::string line = "  " + text;
        int pad = W - static_cast<int>(line.size());
        os << BOLD << col << "  \u2551" << RESET
           << col << line << std::string(pad < 0 ? 0 : pad, ' ') << RESET
           << BOLD << col << "\u2551\n" << RESET;
    };
    auto divider = [&](const char* col) {
        os << BOLD << col << "  \u2560" << std::string(W, '-') << "\u2563\n" << RESET;
    };

    // ── Bus Config ────────────────────────────────────────────────────────
    hdr("CAN BUS SIMULATOR  |  ISO 11898  |  500 kbps CAN-HS", BBLUE);

    CanBusStats st = getStats();
    std::size_t qd = txQueueDepth();

    row(BWHITE, "Bus Speed",        std::to_string(BUS_SPEED_KBPS) + " kbps");
    row(BWHITE, "TX Queue Capacity",std::to_string(TX_QUEUE_CAPACITY) + " frames");
    row(BWHITE, "RX Log Capacity",  std::to_string(RX_LOG_CAPACITY) + " frames");
    row(BWHITE, "Error Inject Rate",std::to_string(static_cast<int>(ERROR_RATE*100)) + "%  (1 in 100 frames)");

    // ── Live Stats ────────────────────────────────────────────────────────
    divider(BBLUE);
    textRow(BCYAN, "LIVE BUS STATISTICS");
    divider(BBLUE);

    auto fmt1 = [](double v) {
        std::ostringstream o; o << std::fixed << std::setprecision(1) << v; return o.str();
    };

    row(BGREEN,  "TX Frames",      std::to_string(st.txFrames));
    row(BGREEN,  "RX Frames",      std::to_string(st.rxFrames));
    row(st.errorFrames > 0 ? BRED : BGREEN,
                 "Error Frames",   std::to_string(st.errorFrames));
    row(st.txDropped > 0 ? BYELLOW : BGREEN,
                 "TX Dropped",     std::to_string(st.txDropped));
    row(BYELLOW, "TX Queue Depth", std::to_string(qd));
    row(BCYAN,   "Bus Load",       fmt1(st.busLoadPct) + " %  (est.)");

    // ── RX Log — last 10 frames ───────────────────────────────────────────
    divider(BBLUE);
    textRow(BCYAN, "RX LOG  (last 10 frames, newest first)");
    divider(BBLUE);

    std::vector<CanFrame> snapshot;
    {
        std::lock_guard<std::mutex> lk(busMutex_);
        snapshot = rxLog_;
    }

    if (snapshot.empty()) {
        textRow(BWHITE, "No frames received yet — run Option 1 or 2 first.");
    } else {
        // Show last 10 in reverse (newest first)
        int start = static_cast<int>(snapshot.size()) - 1;
        int end   = std::max(start - 9, 0);
        for (int i = start; i >= end; --i) {
            const CanFrame& f = snapshot[static_cast<std::size_t>(i)];
            const char* col = f.errorFrame ? BRED : BGREEN;
            std::string line = f.toHexString() + "  →  " + f.decodeToPhysical();
            // Prefix with timestamp
            std::ostringstream ts;
            ts << "[+" << std::setw(6) << f.timestampMs << "ms] ";
            line = ts.str() + line;
            if (static_cast<int>(line.size()) > W - 2)
                line = line.substr(0, W - 5) + "...";
            textRow(col, line);
        }
    }

    // ── Error Log — last 5 ────────────────────────────────────────────────
    divider(BBLUE);
    textRow(BRED, "ERROR FRAME LOG  (last 5)");
    divider(BBLUE);

    std::vector<CanFrame> errSnap;
    {
        std::lock_guard<std::mutex> lk(busMutex_);
        errSnap = errorLog_;
    }

    if (errSnap.empty()) {
        textRow(BGREEN, "No error frames recorded.");
    } else {
        int start = static_cast<int>(errSnap.size()) - 1;
        int end   = std::max(start - 4, 0);
        for (int i = start; i >= end; --i) {
            const CanFrame& f = errSnap[static_cast<std::size_t>(i)];
            std::ostringstream ts;
            ts << "[+" << std::setw(6) << f.timestampMs << "ms] ERR  "
               << f.toHexString();
            std::string line = ts.str();
            if (static_cast<int>(line.size()) > W - 2)
                line = line.substr(0, W - 5) + "...";
            textRow(BRED, line);
        }
    }

    // ── CAN ID Map legend ─────────────────────────────────────────────────
    divider(BBLUE);
    textRow(BCYAN, "ASSIGNED CAN IDs");
    divider(BBLUE);
    textRow(BWHITE, "0x100  Engine Temperature  (2 bytes, 0.5 deg/LSB, offset -40)");
    textRow(BWHITE, "0x200  Battery Voltage     (2 bytes, 0.01V/LSB)");
    textRow(BWHITE, "0x300  Vehicle Speed       (2 bytes, 0.1km/h/LSB)");
    textRow(BWHITE, "0x400  Tire Pressure       (2 bytes, 0.1PSI/LSB)");
    textRow(BWHITE, "0x500  Door + Seatbelt     (1 byte bitmask: bit0=Door, bit1=Belt)");
    textRow(BWHITE, "0x600  Alert Flags         (1 byte, reserved)");

    foot(BBLUE);
    os << "\n";
}
