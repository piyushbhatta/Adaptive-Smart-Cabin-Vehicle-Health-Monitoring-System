/**
 * @file    CrashSafeRecorder.cpp
 * @brief   Implementation of CrashSafeRecorder — crash-safe EDR for the
 *          Smart Vehicle ECU.
 *
 * @details Two-layer persistence:
 *   Layer 1: Fixed RAM ring buffer (lock-guarded, zero-allocation write path).
 *   Layer 2: Atomic file flush using write-then-rename (POSIX rename(2) is
 *            guaranteed atomic on any POSIX-compliant filesystem).
 *
 *   CRC-32/ISO-HDLC (polynomial 0xEDB88320, reflected) is computed over the
 *   payload bytes of every EventRecord before the crc32 field itself, giving
 *   reliable detection of single-bit and burst errors in the persisted file.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "CrashSafeRecorder.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <ctime>
#include <cstdio>       // rename(), std::remove
#include <chrono>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
//  Global instance
// ─────────────────────────────────────────────────────────────────────────────
CrashSafeRecorder g_crashRecorder;

// ─────────────────────────────────────────────────────────────────────────────
//  EventType helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string eventTypeToString(EventType t)
{
    switch (t) {
    case EventType::STARTUP:  return "STARTUP ";
    case EventType::NORMAL:   return "NORMAL  ";
    case EventType::WARNING:  return "WARNING ";
    case EventType::CRASH:    return "CRASH   ";
    case EventType::FAULT:    return "FAULT   ";
    case EventType::SHUTDOWN: return "SHUTDOWN";
    }
    return "UNKNOWN ";
}

const char* eventTypeColour(EventType t)
{
    switch (t) {
    case EventType::STARTUP:  return BCYAN;
    case EventType::NORMAL:   return BGREEN;
    case EventType::WARNING:  return BYELLOW;
    case EventType::CRASH:    return BRED;
    case EventType::FAULT:    return BMAGENTA;
    case EventType::SHUTDOWN: return BCYAN;
    }
    return BWHITE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  EventRecord::toOneLine
// ─────────────────────────────────────────────────────────────────────────────

std::string EventRecord::toOneLine() const
{
    std::ostringstream os;
    os << "[" << timestamp << "]"
       << " #" << std::setw(5) << std::setfill('0') << sequenceId
       << " " << eventTypeToString(eventType)
       << " spd=" << std::fixed << std::setprecision(1) << speed  << "km/h"
       << " tmp=" << engineTemp << "°C"
       << " vlt=" << batteryVolt << "V"
       << " psi=" << tirePressure;
    if (triggerReason[0] != '\0')
        os << " [" << triggerReason << "]";
    return os.str();
}

// ─────────────────────────────────────────────────────────────────────────────
//  CRC-32 / ISO-HDLC  (reflected, poly 0xEDB88320)
// ─────────────────────────────────────────────────────────────────────────────

uint32_t CrashSafeRecorder::crc32(const uint8_t* data, std::size_t len)
{
    // Pre-computed lookup table (generated from poly 0xEDB88320)
    static const uint32_t table[256] = {
        0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,
        0xE963A535,0x9E6495A3,0x0EDB8832,0x79DCB8A4,0xE0D5E91B,0x97D2D988,
        0x09B64C2B,0x7EB17CBF,0xE7B82D09,0x90BF1D9F,0x1DB71064,0x6AB020F2,
        0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
        0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,
        0xFA0F3D63,0x8D080DF5,0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,
        0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,0x35B5A8FA,0x42B2986C,
        0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
        0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F927,0x56B3C9B1,
        0xCFBA9899,0xB8BDA80F,0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,
        0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,0x76DC4190,0x01DB7106,
        0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
        0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6B4BBA,0x086D3D2D,
        0x91646C97,0xE6635C01,0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,
        0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,0x65B0D9C6,0x12B7E950,
        0x8BBEB8EA,0xFCB9887C,0x62DD1FDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
        0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,
        0xA4D1C46D,0xD3D6F4FB,0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,
        0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,0x5005713C,0x270241AA,
        0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
        0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,
        0xB7BD5C3B,0xC0BA6CAD,0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,
        0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,0xE3630B12,0x94643B84,
        0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
        0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,
        0x196C3671,0x6E6B06E7,0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,
        0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,0xD6D6A3E8,0xA1D1937E,
        0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
        0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA8670955,
        0x316658EF,0x466768B9,0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D,
        0x72072F65,0x05005713,0x95BF4A82,0xE2B87A14,0x7BB12BAE,0x0CB61B38,
        0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,0x86D3D2D4,0xF1D4E242,
        0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,
        0x88085AE6,0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,
        0x616BFFD3,0x166CCF45,0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,
        0xA7672661,0xD06016F7,0x4969474D,0x3E6E77DB,0xAED16A4A,0xD9D65ADC,
        0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,0x47B2CF7F,0x30B5FFE9,
        0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,0xCDD70693,
        0x54DE5729,0x23D967BF,0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,
        0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D
    };
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i)
        crc = (crc >> 8) ^ table[(crc ^ data[i]) & 0xFF];
    return crc ^ 0xFFFFFFFFu;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Static helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string CrashSafeRecorder::nowStr()
{
    std::time_t t  = std::time(nullptr);
    std::tm    tm  = *std::localtime(&t);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return std::string(buf);
}

void CrashSafeRecorder::fillTimestamp(char* buf)
{
    std::time_t t  = std::time(nullptr);
    std::tm    tm  = *std::localtime(&t);
    std::strftime(buf, 20, "%Y-%m-%d %H:%M:%S", &tm);
}

void CrashSafeRecorder::buildAlertCodeString(const AlertManager& alertMgr,
                                              char* buf, std::size_t bufLen)
{
    buf[0] = '\0';
    const auto& alerts = alertMgr.getActiveAlerts();
    std::size_t pos    = 0;
    for (std::size_t i = 0; i < alerts.size(); ++i) {
        const std::string& code = alerts[i].getCode();
        if (pos + code.size() + 2 >= bufLen) break;
        if (i > 0) { buf[pos++] = ','; buf[pos++] = ' '; }
        std::memcpy(buf + pos, code.c_str(), code.size());
        pos += code.size();
        buf[pos] = '\0';
    }
}

uint64_t CrashSafeRecorder::elapsedMs() const
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime_).count());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Construction / destruction
// ─────────────────────────────────────────────────────────────────────────────

CrashSafeRecorder::CrashSafeRecorder()
    : startTime_(std::chrono::steady_clock::now())
{
    // Attempt to replay any previously persisted EDR snapshot
    loadFromNvm();

    // Write a STARTUP sentinel record
    SensorData empty{};
    AlertManager emptyMgr;
    EventRecord startup = buildRecord(empty, emptyMgr,
                                      EventType::STARTUP, 0.0f, 0.0f,
                                      "ECU power-on / recorder init");
    {
        std::lock_guard<std::mutex> lk(mutex_);
        writeToRing(startup);
    }
}

CrashSafeRecorder::~CrashSafeRecorder()
{
    recordShutdown();
}

// ─────────────────────────────────────────────────────────────────────────────
//  writeToRing  (must be called with mutex_ held)
// ─────────────────────────────────────────────────────────────────────────────

void CrashSafeRecorder::writeToRing(EventRecord& rec)
{
    // Compute CRC over all fields except the crc32 field itself
    constexpr std::size_t CRC_PAYLOAD_LEN =
        offsetof(EventRecord, crc32);
    rec.crc32 = crc32(reinterpret_cast<const uint8_t*>(&rec), CRC_PAYLOAD_LEN);

    // Track overwrite
    if (count_ >= RING_CAPACITY)
        ++stats_.overwriteCount;
    else
        ++count_;

    ring_[head_] = rec;
    head_        = (head_ + 1) % RING_CAPACITY;

    ++stats_.totalRecorded;
    switch (rec.eventType) {
    case EventType::CRASH:    ++stats_.crashEvents;   break;
    case EventType::WARNING:  ++stats_.warningEvents; break;
    case EventType::FAULT:    ++stats_.faultEvents;   break;
    default: break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  buildRecord  (must be called with mutex_ held)
// ─────────────────────────────────────────────────────────────────────────────

EventRecord CrashSafeRecorder::buildRecord(const SensorData& s,
                                            const AlertManager& alertMgr,
                                            EventType type,
                                            float deltaSpeed,
                                            float deltaTemp,
                                            const std::string& triggerReason)
{
    EventRecord rec{};
    rec.sequenceId  = ++seqCounter_;
    fillTimestamp(rec.timestamp);
    rec.eventType   = type;
    rec.speed       = static_cast<float>(s.speed);
    rec.engineTemp  = static_cast<float>(s.temp);
    rec.batteryVolt = static_cast<float>(s.volt);
    rec.tirePressure= static_cast<float>(s.psi);
    rec.doorOpen    = s.doorOpen  ? 1u : 0u;
    rec.beltLocked  = s.beltLocked? 1u : 0u;
    rec.deltaSpeed  = deltaSpeed;
    rec.deltaTemp   = deltaTemp;

    buildAlertCodeString(alertMgr, rec.alertCodes, sizeof(rec.alertCodes));

    // Copy trigger reason (truncate safely)
    std::size_t rlen = std::min(triggerReason.size(),
                                sizeof(rec.triggerReason) - 1u);
    std::memcpy(rec.triggerReason, triggerReason.c_str(), rlen);
    rec.triggerReason[rlen] = '\0';

    return rec;
}

// ─────────────────────────────────────────────────────────────────────────────
//  record  —  hot-path entry point
// ─────────────────────────────────────────────────────────────────────────────

void CrashSafeRecorder::record(const SensorData& s, const AlertManager& alertMgr)
{
    bool needFlush = false;
    std::unique_lock<std::mutex> lk(mutex_);

    const float curSpeed = static_cast<float>(s.speed);
    const float curTemp  = static_cast<float>(s.temp);
    const float curVolt  = static_cast<float>(s.volt);

    float dSpeed = 0.0f, dTemp = 0.0f;
    if (prevDataValid_) {
        dSpeed = curSpeed - prevSpeed_;   // negative = deceleration
        dTemp  = curTemp  - prevTemp_;
    }
    const float absDecel    = -dSpeed;         // positive value for deceleration
    const float absVoltDrop = prevVolt_ - curVolt; // positive = drop

    // ── Crash / fault / warning heuristics ───────────────────────────────
    EventType   type    = EventType::NORMAL;
    std::string trigger = "";

    if (prevDataValid_) {
        // 1. Sudden deceleration (R160 §6.2 trigger A)
        if (absDecel > static_cast<float>(DECEL_THRESHOLD) &&
            curSpeed > static_cast<float>(CRASH_SPEED_THRESHOLD))
        {
            type    = EventType::CRASH;
            trigger = "Sudden decel " + std::to_string(static_cast<int>(absDecel)) + " km/h/cyc";
        }
        // 2. Temperature spike (thermal runaway proxy)
        else if (dTemp > static_cast<float>(TEMP_SPIKE_THRESHOLD))
        {
            type    = EventType::CRASH;
            trigger = "Temp spike +" + std::to_string(static_cast<int>(dTemp)) + " degC/cyc";
        }
        // 3. Voltage drop (power loss / short circuit)
        else if (absVoltDrop > static_cast<float>(VOLT_DROP_THRESHOLD))
        {
            type    = EventType::FAULT;
            trigger = "Volt drop -" + std::to_string(static_cast<int>(absVoltDrop * 10) / 10.0) + "V/cyc";
        }
    }

    // 4. Critical alert while moving
    if (type == EventType::NORMAL) {
        for (const auto& a : alertMgr.getActiveAlerts()) {
            if (a.getSeverity() == AlertSeverity::CRITICAL &&
                curSpeed > static_cast<float>(CRASH_SPEED_THRESHOLD))
            {
                type    = EventType::CRASH;
                trigger = "Critical alert: " + a.getCode();
                break;
            }
        }
    }

    // 5. Any WARNING alert
    if (type == EventType::NORMAL && alertMgr.hasActiveAlerts()) {
        type    = EventType::WARNING;
        trigger = "Active alert(s) present";
    }

    // ── Build and store ───────────────────────────────────────────────────
    EventRecord rec = buildRecord(s, alertMgr, type, dSpeed, dTemp, trigger);
    writeToRing(rec);

    prevSpeed_     = curSpeed;
    prevTemp_      = curTemp;
    prevVolt_      = curVolt;
    prevDataValid_ = true;

    // ── Persistence policy ────────────────────────────────────────────────
    needFlush = (type == EventType::CRASH || type == EventType::FAULT);
    if (!needFlush) {
        ++normalCycleCount_;
        if (normalCycleCount_ >= PERIODIC_FLUSH_CYCLES) {
            needFlush = true;
            normalCycleCount_ = 0;
        }
    } else {
        normalCycleCount_ = 0;
    }
    // Release lock before flushToNvm() (which acquires its own lock).
    lk.unlock();

    if (needFlush) {
        flushToNvm();   // safe: called without mutex held
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  recordFault
// ─────────────────────────────────────────────────────────────────────────────

void CrashSafeRecorder::recordFault(const std::string& reason, const SensorData& s)
{
    {
        std::lock_guard<std::mutex> lk(mutex_);
        AlertManager emptyMgr;
        EventRecord rec = buildRecord(s, emptyMgr, EventType::FAULT,
                                      0.0f, 0.0f, reason);
        writeToRing(rec);
    }
    flushToNvm();
}

// ─────────────────────────────────────────────────────────────────────────────
//  recordShutdown
// ─────────────────────────────────────────────────────────────────────────────

void CrashSafeRecorder::recordShutdown()
{
    {
        std::lock_guard<std::mutex> lk(mutex_);
        SensorData empty{};
        AlertManager emptyMgr;
        EventRecord rec = buildRecord(empty, emptyMgr, EventType::SHUTDOWN,
                                      0.0f, 0.0f, "ECU clean shutdown");
        writeToRing(rec);
    }
    flushToNvm();
}

// ─────────────────────────────────────────────────────────────────────────────
//  flushToNvm  —  atomic write-then-rename persistence
// ─────────────────────────────────────────────────────────────────────────────

bool CrashSafeRecorder::flushToNvm()
{
    // Take ring snapshot under lock
    Ring   snapRing;
    std::size_t snapHead, snapCount;
    uint64_t snapSeq;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        snapRing  = ring_;
        snapHead  = head_;
        snapCount = count_;
        snapSeq   = seqCounter_.load();
    }

    // ── Write to temp file ────────────────────────────────────────────────
    std::ofstream tmp(EDR_TMP_FILE, std::ios::binary | std::ios::trunc);
    if (!tmp.is_open()) return false;

    // File header: magic + version + count
    const char magic[8] = "EDRV1.0";
    tmp.write(magic, 8);
    uint64_t cnt64 = static_cast<uint64_t>(snapCount);
    tmp.write(reinterpret_cast<const char*>(&cnt64), sizeof(cnt64));

    // Ordered from oldest to newest
    // Oldest slot = (head - count) mod RING_CAPACITY
    for (std::size_t i = 0; i < snapCount; ++i) {
        std::size_t idx =
            (snapHead + RING_CAPACITY - snapCount + i) % RING_CAPACITY;
        const EventRecord& rec = snapRing[idx];
        if (!rec.isValid()) continue;
        tmp.write(reinterpret_cast<const char*>(&rec), sizeof(EventRecord));
    }
    tmp.flush();
    tmp.close();

    // ── Atomic rename ─────────────────────────────────────────────────────
    if (std::rename(EDR_TMP_FILE, EDR_FILE) != 0) {
        std::remove(EDR_TMP_FILE);
        return false;
    }

    // ── Append human-readable mirror to edr_events.txt ───────────────────
    {
        std::ofstream log(EDR_LOG_FILE, std::ios::app);
        if (log.is_open()) {
            log << "\n=== EDR FLUSH @ " << nowStr()
                << "  seq=" << snapSeq
                << "  records=" << snapCount << " ===\n";
            for (std::size_t i = 0; i < snapCount; ++i) {
                std::size_t idx =
                    (snapHead + RING_CAPACITY - snapCount + i) % RING_CAPACITY;
                const EventRecord& rec = snapRing[idx];
                if (rec.isValid())
                    log << rec.toOneLine() << "\n";
            }
        }
    }

    {
        std::lock_guard<std::mutex> lk(mutex_);
        ++stats_.flushCount;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  loadFromNvm  —  parse and CRC-validate the persisted EDR file
// ─────────────────────────────────────────────────────────────────────────────

int CrashSafeRecorder::loadFromNvm()
{
    std::ifstream f(EDR_FILE, std::ios::binary);
    if (!f.is_open()) return 0;

    char magic[8] = {};
    f.read(magic, 8);
    if (std::string(magic, 7) != "EDRV1.0") return 0;

    uint64_t storedCount = 0;
    f.read(reinterpret_cast<char*>(&storedCount), sizeof(storedCount));

    int restored = 0;
    constexpr std::size_t CRC_PAYLOAD_LEN = offsetof(EventRecord, crc32);

    for (uint64_t i = 0; i < storedCount && i < RING_CAPACITY; ++i) {
        EventRecord rec{};
        f.read(reinterpret_cast<char*>(&rec), sizeof(EventRecord));
        if (!f) break;
        if (!rec.isValid()) continue;

        // Validate CRC
        uint32_t computed = crc32(reinterpret_cast<const uint8_t*>(&rec),
                                  CRC_PAYLOAD_LEN);
        if (computed != rec.crc32) continue;   // Corrupt record — skip

        // Insert into ring (oldest first → they'll be the first overwritten
        // when new live data arrives, which is the correct FIFO behaviour)
        ring_[head_] = rec;
        head_        = (head_ + 1) % RING_CAPACITY;
        if (count_ < RING_CAPACITY) ++count_;

        // Keep seqCounter_ above the highest restored ID
        if (rec.sequenceId >= seqCounter_.load())
            seqCounter_.store(rec.sequenceId + 1);

        ++restored;
    }
    return restored;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Query API
// ─────────────────────────────────────────────────────────────────────────────

std::vector<EventRecord> CrashSafeRecorder::getEvents() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<EventRecord> result;
    result.reserve(count_);

    // Collect in order newest → oldest
    for (std::size_t i = 0; i < count_; ++i) {
        std::size_t idx =
            (head_ + RING_CAPACITY - 1 - i) % RING_CAPACITY;
        if (ring_[idx].isValid())
            result.push_back(ring_[idx]);
    }
    return result;
}

std::vector<EventRecord> CrashSafeRecorder::filterEvents(
    const std::function<bool(const EventRecord&)>& pred) const
{
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<EventRecord> all;
    all.reserve(count_);
    for (std::size_t i = 0; i < count_; ++i) {
        std::size_t idx = (head_ + RING_CAPACITY - 1 - i) % RING_CAPACITY;
        if (ring_[idx].isValid()) all.push_back(ring_[idx]);
    }
    std::vector<EventRecord> result;
    std::copy_if(all.begin(), all.end(), std::back_inserter(result), pred);
    return result;
}

std::vector<EventRecord> CrashSafeRecorder::getCrashEvents() const
{
    return filterEvents([](const EventRecord& r){
        return r.eventType == EventType::CRASH;
    });
}

RecorderStats CrashSafeRecorder::getStats() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return stats_;
}

uint64_t CrashSafeRecorder::lastSequenceId() const
{
    return seqCounter_.load();
}

void CrashSafeRecorder::clearRing()
{
    std::lock_guard<std::mutex> lk(mutex_);
    ring_   = Ring{};
    head_   = 0;
    count_  = 0;
    stats_  = RecorderStats{};
    prevDataValid_ = false;
    normalCycleCount_ = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  displayCompact
// ─────────────────────────────────────────────────────────────────────────────

void CrashSafeRecorder::displayCompact(std::ostream& os) const
{
    RecorderStats st;
    EventRecord   last{};
    {
        std::lock_guard<std::mutex> lk(mutex_);
        st = stats_;
        if (count_ > 0) {
            std::size_t idx = (head_ + RING_CAPACITY - 1) % RING_CAPACITY;
            last = ring_[idx];
        }
    }

    const char* crashCol = (st.crashEvents > 0) ? BRED : BGREEN;
    os << BCYAN << BOLD << "  EDR" << RESET
       << BWHITE << "  Events: " << RESET << BGREEN << st.totalRecorded << RESET
       << BWHITE << "  Crashes: " << RESET << crashCol << st.crashEvents << RESET
       << BWHITE << "  Flushes: " << RESET << BCYAN << st.flushCount << RESET;
    if (last.isValid()) {
        os << BWHITE << "  Last: " << RESET
           << eventTypeColour(last.eventType)
           << "[" << eventTypeToString(last.eventType) << "]" << RESET
           << BCYAN << " @ " << last.timestamp << RESET;
    }
    os << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  display  —  full EDR management panel
// ─────────────────────────────────────────────────────────────────────────────

void CrashSafeRecorder::display(std::ostream& os) const
{
    constexpr int W = 78;

    // ── Box helpers ───────────────────────────────────────────────────────
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
    auto textRow = [&](const char* col, const std::string& text) {
        std::string line = "  " + text;
        int pad = W - static_cast<int>(line.size());
        os << BOLD << col << "  \u2551" << RESET
           << col << line
           << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ') << RESET
           << BOLD << col << "\u2551\n" << RESET;
    };
    auto kvRow = [&](const char* bCol, const std::string& key,
                     const std::string& val, const char* vCol) {
        std::string line = "  " + key;
        int padMid = 30 - static_cast<int>(line.size());
        if (padMid < 1) padMid = 1;
        line += std::string(static_cast<std::size_t>(padMid), ' ') + ": ";
        int padEnd = W - static_cast<int>(line.size()) - static_cast<int>(val.size());
        os << BOLD << bCol << "  \u2551" << RESET
           << BCYAN << line << RESET
           << vCol << BOLD << val << RESET
           << std::string(padEnd < 0 ? 0 : static_cast<std::size_t>(padEnd), ' ')
           << BOLD << bCol << "\u2551\n" << RESET;
    };
    auto divider = [&](const char* col) {
        os << BOLD << col << "  \u2560" << std::string(W, '-') << "\u2563\n" << RESET;
    };

    // ── Snapshot ──────────────────────────────────────────────────────────
    RecorderStats snapStats;
    std::vector<EventRecord> events;
    std::size_t snapCount;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        snapStats  = stats_;
        snapCount  = count_;
        events.reserve(count_);
        for (std::size_t i = 0; i < count_; ++i) {
            std::size_t idx = (head_ + RING_CAPACITY - 1 - i) % RING_CAPACITY;
            if (ring_[idx].isValid())
                events.push_back(ring_[idx]);
        }
    }

    // ── Header ────────────────────────────────────────────────────────────
    const char* titleCol = (snapStats.crashEvents > 0) ? BRED : BCYAN;
    hdr("CRASH-SAFE EVENT RECORDER  |  EDR / AUTOSAR NvM  |  ISO R160", titleCol);

    // ── Configuration ─────────────────────────────────────────────────────
    kvRow(titleCol, "Ring Capacity",
          std::to_string(RING_CAPACITY) + " events", BWHITE);
    kvRow(titleCol, "Populated Slots",
          std::to_string(snapCount) + " / " + std::to_string(RING_CAPACITY), BWHITE);
    kvRow(titleCol, "NvM File", std::string(EDR_FILE), BWHITE);
    kvRow(titleCol, "Log Mirror", std::string(EDR_LOG_FILE), BWHITE);

    // ── Thresholds ────────────────────────────────────────────────────────
    divider(titleCol);
    textRow(BCYAN, "CRASH DETECTION THRESHOLDS");
    divider(titleCol);
    {
        std::ostringstream d, t, v, s;
        d << DECEL_THRESHOLD   << " km/h/cycle";
        t << TEMP_SPIKE_THRESHOLD << " degC/cycle";
        v << VOLT_DROP_THRESHOLD  << " V/cycle";
        s << CRASH_SPEED_THRESHOLD << " km/h (min moving speed)";
        kvRow(titleCol, "Sudden deceleration",  d.str(), BYELLOW);
        kvRow(titleCol, "Temperature spike",    t.str(), BYELLOW);
        kvRow(titleCol, "Voltage drop",         v.str(), BYELLOW);
        kvRow(titleCol, "Speed gate",           s.str(), BWHITE);
        kvRow(titleCol, "Periodic NvM flush",
              "Every " + std::to_string(PERIODIC_FLUSH_CYCLES) + " normal cycles", BWHITE);
    }

    // ── Statistics ────────────────────────────────────────────────────────
    divider(titleCol);
    textRow(BCYAN, "RECORDER STATISTICS");
    divider(titleCol);
    kvRow(titleCol, "Total Events Recorded",
          std::to_string(snapStats.totalRecorded), BWHITE);
    kvRow(titleCol, "CRASH Events",
          std::to_string(snapStats.crashEvents),
          snapStats.crashEvents > 0 ? BRED : BGREEN);
    kvRow(titleCol, "WARNING Events",
          std::to_string(snapStats.warningEvents),
          snapStats.warningEvents > 0 ? BYELLOW : BGREEN);
    kvRow(titleCol, "FAULT Events",
          std::to_string(snapStats.faultEvents),
          snapStats.faultEvents > 0 ? BMAGENTA : BGREEN);
    kvRow(titleCol, "Ring Overwrites",
          std::to_string(snapStats.overwriteCount),
          snapStats.overwriteCount > 0 ? BYELLOW : BGREEN);
    kvRow(titleCol, "NvM Flush Count",
          std::to_string(snapStats.flushCount), BCYAN);
    kvRow(titleCol, "Last Sequence ID",
          std::to_string(seqCounter_.load()), BWHITE);

    // ── Ring buffer — last 20 events ──────────────────────────────────────
    divider(titleCol);
    textRow(BCYAN, "EVENT RING  (newest first, last 20)");
    divider(titleCol);

    // Table column header
    {
        std::string hline = "  #      Timestamp            Type     "
                            "Spd    Tmp    Vlt    Trigger";
        textRow(BCYAN, hline);
    }
    divider(titleCol);

    if (events.empty()) {
        textRow(BWHITE, "  No events recorded yet.");
    } else {
        std::size_t shown = std::min(events.size(), static_cast<std::size_t>(20));
        for (std::size_t i = 0; i < shown; ++i) {
            const EventRecord& r = events[i];
            const char* rCol = eventTypeColour(r.eventType);

            std::ostringstream row;
            row << "  "
                << std::right << std::setw(5) << std::setfill('0') << r.sequenceId
                << std::setfill(' ')
                << "  " << r.timestamp
                << "  " << eventTypeToString(r.eventType)
                << "  " << std::fixed << std::setprecision(1)
                        << std::setw(5) << r.speed    << "km/h"
                << "  " << std::setw(5) << r.engineTemp << "C"
                << "  " << std::setw(4) << r.batteryVolt << "V";

            if (r.triggerReason[0] != '\0') {
                std::string trg = r.triggerReason;
                if (trg.size() > 22) trg = trg.substr(0, 19) + "...";
                row << "  " << trg;
            }

            std::string rowStr = row.str();
            int pad = W - static_cast<int>(rowStr.size());
            os << BOLD << titleCol << "  \u2551" << RESET
               << rCol << rowStr
               << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ') << RESET
               << BOLD << titleCol << "\u2551\n" << RESET;
        }
    }

    // ── Crash event spotlight ─────────────────────────────────────────────
    std::vector<EventRecord> crashes;
    std::copy_if(events.begin(), events.end(),
                 std::back_inserter(crashes),
                 [](const EventRecord& r){ return r.eventType == EventType::CRASH; });

    if (!crashes.empty()) {
        divider(titleCol);
        textRow(BRED, "CRASH EVENT SPOTLIGHT");
        divider(titleCol);

        std::size_t cShown = std::min(crashes.size(), static_cast<std::size_t>(5));
        for (std::size_t i = 0; i < cShown; ++i) {
            const EventRecord& r = crashes[i];

            textRow(BRED, "  ─── CRASH #" + std::to_string(r.sequenceId) +
                          " @ " + std::string(r.timestamp) + " ───");

            kvRow(BRED, "  Trigger",     std::string(r.triggerReason),   BRED);
            kvRow(BRED, "  Speed",
                  std::to_string(static_cast<int>(r.speed)) + " km/h"
                  + "  (Δ " + (r.deltaSpeed >= 0 ? "+" : "")
                  + std::to_string(static_cast<int>(r.deltaSpeed)) + " km/h/cyc)",
                  BYELLOW);
            kvRow(BRED, "  Engine Temp",
                  std::to_string(static_cast<int>(r.engineTemp)) + " °C"
                  + "  (Δ " + (r.deltaTemp >= 0 ? "+" : "")
                  + std::to_string(static_cast<int>(r.deltaTemp)) + " °C/cyc)",
                  BYELLOW);
            kvRow(BRED, "  Battery",
                  std::to_string(r.batteryVolt) + " V", BWHITE);
            kvRow(BRED, "  Door",
                  r.doorOpen ? "OPEN" : "CLOSED",
                  r.doorOpen ? BRED : BGREEN);
            kvRow(BRED, "  Seatbelt",
                  r.beltLocked ? "LOCKED" : "UNLOCKED",
                  r.beltLocked ? BGREEN : BRED);
            if (r.alertCodes[0] != '\0')
                kvRow(BRED, "  Active Alerts", std::string(r.alertCodes), BRED);

            if (i + 1 < cShown) divider(titleCol);
        }
    }

    foot(titleCol);
    os << "\n";
}
