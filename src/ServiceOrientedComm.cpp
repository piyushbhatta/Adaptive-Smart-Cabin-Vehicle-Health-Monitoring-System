/**
 * @file    ServiceOrientedComm.cpp
 * @brief   Implementation of the Service-Oriented Communication model for the
 *          Smart Vehicle ECU.
 *
 * @details SOME/IP-inspired in-process service bus simulation.
 *          All inter-service communication is serialised through the shared
 *          mutex so this class is safe to call from multiple threads.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "ServiceOrientedComm.hpp"
#include "Dashboard.hpp"    // Color namespace

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <chrono>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
//  SomeIpMsgType helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string msgTypeToString(SomeIpMsgType t)
{
    switch (t) {
    case SomeIpMsgType::REQUEST:           return "REQUEST          ";
    case SomeIpMsgType::REQUEST_NO_RETURN: return "REQUEST_NO_RETURN";
    case SomeIpMsgType::NOTIFICATION:      return "NOTIFICATION     ";
    case SomeIpMsgType::RESPONSE:          return "RESPONSE         ";
    case SomeIpMsgType::ERROR:             return "ERROR            ";
    case SomeIpMsgType::SD_OFFER:          return "SD_OFFER         ";
    case SomeIpMsgType::SD_FIND:           return "SD_FIND          ";
    case SomeIpMsgType::SD_SUBSCRIBE:      return "SD_SUBSCRIBE     ";
    }
    return "UNKNOWN          ";
}

const char* msgTypeColour(SomeIpMsgType t)
{
    switch (t) {
    case SomeIpMsgType::REQUEST:           return BCYAN;
    case SomeIpMsgType::REQUEST_NO_RETURN: return BCYAN;
    case SomeIpMsgType::NOTIFICATION:      return BYELLOW;
    case SomeIpMsgType::RESPONSE:          return BGREEN;
    case SomeIpMsgType::ERROR:             return BRED;
    case SomeIpMsgType::SD_OFFER:          return BMAGENTA;
    case SomeIpMsgType::SD_FIND:           return BBLUE;
    case SomeIpMsgType::SD_SUBSCRIBE:      return BMAGENTA;
    }
    return BWHITE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SomeIpReturnCode helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string returnCodeToString(SomeIpReturnCode rc)
{
    switch (rc) {
    case SomeIpReturnCode::E_OK:                 return "E_OK";
    case SomeIpReturnCode::E_NOT_OK:             return "E_NOT_OK";
    case SomeIpReturnCode::E_UNKNOWN_SERVICE:    return "E_UNKNOWN_SERVICE";
    case SomeIpReturnCode::E_UNKNOWN_METHOD:     return "E_UNKNOWN_METHOD";
    case SomeIpReturnCode::E_NOT_READY:          return "E_NOT_READY";
    case SomeIpReturnCode::E_WRONG_PROTOCOL_VER: return "E_WRONG_PROTOCOL_VER";
    }
    return "E_UNKNOWN";
}

// ─────────────────────────────────────────────────────────────────────────────
//  SomeIpMessage::toOneLine
// ─────────────────────────────────────────────────────────────────────────────

std::string SomeIpMessage::toOneLine() const
{
    std::ostringstream os;
    os << "[" << timestamp << "]"
       << " SvcID=0x" << std::hex << std::uppercase
                      << std::setw(4) << std::setfill('0') << header.serviceId
       << " MthID=0x" << std::setw(4) << std::setfill('0') << header.methodId
       << std::dec
       << " Ses=" << std::setw(4) << std::setfill('0') << header.sessionId
       << " " << msgTypeToString(header.msgType).substr(0,17)
       << "  " << serviceName << "::" << methodName;
    if (!payload.empty())
        os << "  [" << payload << "]";
    return os.str();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor — register built-in services
// ─────────────────────────────────────────────────────────────────────────────

ServiceOrientedComm::ServiceOrientedComm()
{
    // ── VehicleSpeedService ───────────────────────────────────────────────────
    {
        ServiceDescriptor sd;
        sd.serviceId   = SvcId::VEHICLE_SPEED;
        sd.name        = "VehicleSpeedService";
        sd.description = "Provides real-time vehicle speed readings";
        sd.methods[MethodId::GET_SPEED]        = "GetSpeed";
        sd.events [EventId::SPEED_CHANGED]     = "SpeedChanged";
        services_.push_back(std::move(sd));
    }

    // ── EngineTemperatureService ──────────────────────────────────────────────
    {
        ServiceDescriptor sd;
        sd.serviceId   = SvcId::ENGINE_TEMP;
        sd.name        = "EngineTemperatureService";
        sd.description = "Provides engine thermal monitoring";
        sd.methods[MethodId::GET_ENGINE_TEMP]  = "GetEngineTemp";
        sd.events [EventId::TEMP_WARNING]      = "TempWarning";
        services_.push_back(std::move(sd));
    }

    // ── BatteryService ────────────────────────────────────────────────────────
    {
        ServiceDescriptor sd;
        sd.serviceId   = SvcId::BATTERY;
        sd.name        = "BatteryService";
        sd.description = "Provides battery voltage monitoring";
        sd.methods[MethodId::GET_VOLTAGE]      = "GetVoltage";
        sd.events [EventId::LOW_BATTERY]       = "LowBattery";
        services_.push_back(std::move(sd));
    }

    // ── AlertBroadcastService ─────────────────────────────────────────────────
    {
        ServiceDescriptor sd;
        sd.serviceId   = SvcId::ALERT_BROADCAST;
        sd.name        = "AlertBroadcastService";
        sd.description = "Broadcasts active ECU alerts to all subscribers";
        sd.methods[MethodId::GET_ACTIVE_ALERTS]= "GetActiveAlerts";
        sd.events [EventId::ALERT_FIRED]       = "AlertFired";
        services_.push_back(std::move(sd));
    }

    // ── DiagnosticsService ────────────────────────────────────────────────────
    {
        ServiceDescriptor sd;
        sd.serviceId   = SvcId::DIAGNOSTICS;
        sd.name        = "DiagnosticsService";
        sd.description = "Remote diagnostics — DTC management and BIST";
        sd.methods[MethodId::GET_DTC_LIST] = "GetDtcList";
        sd.methods[MethodId::CLEAR_DTCS]   = "ClearDtcs";
        sd.methods[MethodId::RUN_BIST]     = "RunBist";
        services_.push_back(std::move(sd));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string ServiceOrientedComm::nowString()
{
    using Clock = std::chrono::system_clock;
    std::time_t t = Clock::to_time_t(Clock::now());
    char buf[20];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&t));
    return std::string(buf);
}

SomeIpHeader ServiceOrientedComm::makeHeader(uint16_t svc, uint16_t method,
                                              uint16_t client,
                                              SomeIpMsgType type,
                                              SomeIpReturnCode rc) const
{
    SomeIpHeader h;
    h.serviceId  = svc;
    h.methodId   = method;
    h.clientId   = client;
    h.sessionId  = sessionCounter_++;
    h.msgType    = type;
    h.returnCode = rc;
    return h;
}

const ServiceDescriptor* ServiceOrientedComm::findService(uint16_t id) const
{
    for (const auto& s : services_)
        if (s.serviceId == id) return &s;
    return nullptr;
}

ServiceDescriptor* ServiceOrientedComm::findServiceMut(uint16_t id)
{
    for (auto& s : services_)
        if (s.serviceId == id) return &s;
    return nullptr;
}

std::string ServiceOrientedComm::resolveMethod(const ServiceDescriptor& svc,
                                                uint16_t mid)
{
    auto it = svc.methods.find(mid);
    return (it != svc.methods.end()) ? it->second : "UnknownMethod";
}

std::string ServiceOrientedComm::resolveEvent(const ServiceDescriptor& svc,
                                               uint16_t eid)
{
    auto it = svc.events.find(eid);
    return (it != svc.events.end()) ? it->second : "UnknownEvent";
}

void ServiceOrientedComm::appendLog(SomeIpMessage msg)
{
    if (log_.size() >= MAX_LOG_ENTRIES)
        log_.pop_front();
    log_.push_back(std::move(msg));
    ++totalMessages_;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Service Discovery
// ─────────────────────────────────────────────────────────────────────────────

void ServiceOrientedComm::runServiceDiscovery()
{
    std::lock_guard<std::mutex> lk(mutex_);

    for (auto& svc : services_) {
        SomeIpMessage msg;
        msg.header      = makeHeader(svc.serviceId, 0xFFFF,
                                     0x0000,  // server itself
                                     SomeIpMsgType::SD_OFFER);
        msg.timestamp   = nowString();
        msg.serviceName = svc.name;
        msg.methodName  = "OfferService";
        msg.payload     = "instance=0x" +
                           [&]() {
                               std::ostringstream ss;
                               ss << std::hex << std::uppercase
                                  << std::setw(4) << std::setfill('0')
                                  << svc.instanceId;
                               return ss.str();
                           }();
        appendLog(std::move(msg));
        svc.offering = true;
    }

    // Simulate clients sending FindService for the services they need
    struct ClientFind { uint16_t client; uint16_t svc; std::string name; };
    const ClientFind finds[] = {
        { CLIENT_DASHBOARD, SvcId::VEHICLE_SPEED,   "Dashboard"   },
        { CLIENT_DASHBOARD, SvcId::ENGINE_TEMP,     "Dashboard"   },
        { CLIENT_DASHBOARD, SvcId::BATTERY,         "Dashboard"   },
        { CLIENT_DASHBOARD, SvcId::ALERT_BROADCAST, "Dashboard"   },
        { CLIENT_DIAG,      SvcId::DIAGNOSTICS,     "DiagTool"    },
        { CLIENT_LOGGER,    SvcId::ALERT_BROADCAST, "EventLogger" },
        { CLIENT_ECU_MGR,   SvcId::ENGINE_TEMP,     "EcuManager"  },
        { CLIENT_ECU_MGR,   SvcId::BATTERY,         "EcuManager"  },
    };

    for (const auto& f : finds) {
        const ServiceDescriptor* svc = findService(f.svc);
        if (!svc) continue;
        SomeIpMessage msg;
        msg.header      = makeHeader(f.svc, 0xFFFF, f.client,
                                     SomeIpMsgType::SD_FIND);
        msg.timestamp   = nowString();
        msg.serviceName = svc->name;
        msg.methodName  = "FindService";
        msg.payload     = "client=" + f.name;
        appendLog(std::move(msg));
    }

    sdComplete_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  subscribe
// ─────────────────────────────────────────────────────────────────────────────

bool ServiceOrientedComm::subscribe(uint16_t serviceId, uint16_t eventId,
                                     uint16_t clientId,
                                     const std::string& clientName)
{
    std::lock_guard<std::mutex> lk(mutex_);

    const ServiceDescriptor* svc = findService(serviceId);
    if (!svc) return false;
    auto eit = svc->events.find(eventId);
    if (eit == svc->events.end()) return false;

    // Avoid duplicate subscriptions
    for (const auto& s : subs_)
        if (s.serviceId == serviceId && s.eventId == eventId &&
            s.subscriberClientId == clientId) return true;

    EventSubscription sub;
    sub.serviceId          = serviceId;
    sub.eventId            = eventId;
    sub.subscriberClientId = clientId;
    sub.subscriberName     = clientName;
    sub.subscribedAt       = std::chrono::steady_clock::now();
    subs_.push_back(sub);

    SomeIpMessage msg;
    msg.header      = makeHeader(serviceId, eventId, clientId,
                                 SomeIpMsgType::SD_SUBSCRIBE);
    msg.timestamp   = nowString();
    msg.serviceName = svc->name;
    msg.methodName  = resolveEvent(*svc, eventId);
    msg.payload     = "subscriber=" + clientName;
    appendLog(std::move(msg));

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  callMethod
// ─────────────────────────────────────────────────────────────────────────────

SomeIpReturnCode ServiceOrientedComm::callMethod(uint16_t serviceId,
                                                   uint16_t methodId,
                                                   uint16_t clientId,
                                                   const std::string& payload)
{
    std::lock_guard<std::mutex> lk(mutex_);
    ++totalMethodCalls_;

    const ServiceDescriptor* svc = findService(serviceId);
    if (!svc) {
        // Log an ERROR response
        SomeIpMessage msg;
        msg.header      = makeHeader(serviceId, methodId, clientId,
                                     SomeIpMsgType::ERROR,
                                     SomeIpReturnCode::E_UNKNOWN_SERVICE);
        msg.timestamp   = nowString();
        msg.serviceName = "UNKNOWN";
        msg.methodName  = "UNKNOWN";
        msg.payload     = "ERROR: service 0x" + [&](){
            std::ostringstream ss; ss << std::hex << serviceId; return ss.str(); }()
                          + " not found";
        appendLog(std::move(msg));
        return SomeIpReturnCode::E_UNKNOWN_SERVICE;
    }

    if (!svc->offering)
        return SomeIpReturnCode::E_NOT_READY;

    bool methodFound = svc->methods.count(methodId) > 0;
    if (!methodFound)
        return SomeIpReturnCode::E_UNKNOWN_METHOD;

    // ── REQUEST ───────────────────────────────────────────────────────────────
    {
        SomeIpMessage req;
        req.header      = makeHeader(serviceId, methodId, clientId,
                                     SomeIpMsgType::REQUEST);
        req.timestamp   = nowString();
        req.serviceName = svc->name;
        req.methodName  = resolveMethod(*svc, methodId);
        req.payload     = payload.empty() ? "(no args)" : payload;
        appendLog(std::move(req));
    }

    // ── RESPONSE  (simulated; real result would be in payload) ───────────────
    {
        std::string respPayload;
        // Simulate response bodies per service/method
        if (serviceId == SvcId::VEHICLE_SPEED && methodId == MethodId::GET_SPEED)
            respPayload = "speed=<live>";
        else if (serviceId == SvcId::ENGINE_TEMP && methodId == MethodId::GET_ENGINE_TEMP)
            respPayload = "engineTemp=<live>";
        else if (serviceId == SvcId::BATTERY && methodId == MethodId::GET_VOLTAGE)
            respPayload = "voltage=<live>";
        else if (serviceId == SvcId::ALERT_BROADCAST && methodId == MethodId::GET_ACTIVE_ALERTS)
            respPayload = "alerts=<live>";
        else if (serviceId == SvcId::DIAGNOSTICS && methodId == MethodId::GET_DTC_LIST)
            respPayload = "dtcs=[P0301,U0100]";
        else if (serviceId == SvcId::DIAGNOSTICS && methodId == MethodId::CLEAR_DTCS)
            respPayload = "dtcsCleared=OK";
        else if (serviceId == SvcId::DIAGNOSTICS && methodId == MethodId::RUN_BIST)
            respPayload = "bist=PASS sensors=6/6";
        else
            respPayload = "OK";

        SomeIpMessage resp;
        resp.header      = makeHeader(serviceId, methodId, clientId,
                                      SomeIpMsgType::RESPONSE,
                                      SomeIpReturnCode::E_OK);
        resp.timestamp   = nowString();
        resp.serviceName = svc->name;
        resp.methodName  = resolveMethod(*svc, methodId);
        resp.payload     = respPayload;
        appendLog(std::move(resp));
    }

    return SomeIpReturnCode::E_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  publishSensorData  —  driven by the simulation hot path
// ─────────────────────────────────────────────────────────────────────────────

void ServiceOrientedComm::publishSensorData(const SensorData& data,
                                              const AlertManager& mgr)
{
    std::lock_guard<std::mutex> lk(mutex_);

    auto notify = [&](uint16_t serviceId, uint16_t eventId,
                      const std::string& payload)
    {
        const ServiceDescriptor* svc = findService(serviceId);
        if (!svc || !svc->offering) return;

        // Deliver to each matching subscriber
        for (auto& sub : subs_) {
            if (sub.serviceId != serviceId || sub.eventId != eventId) continue;
            SomeIpMessage msg;
            msg.header      = makeHeader(serviceId, eventId,
                                         sub.subscriberClientId,
                                         SomeIpMsgType::NOTIFICATION);
            msg.timestamp   = nowString();
            msg.serviceName = svc->name;
            msg.methodName  = resolveEvent(*svc, eventId);
            msg.payload     = payload + " -> " + sub.subscriberName;
            appendLog(std::move(msg));
            ++sub.deliveryCount;
            ++totalNotifications_;
        }
    };

    // ── SpeedChanged: notify on any speed change > 2 km/h ───────────────────
    if (lastSpeed_ < 0.0 || std::abs(data.speed - lastSpeed_) > 2.0) {
        std::ostringstream ss;
        ss << "speed=" << std::fixed << std::setprecision(1) << data.speed << "km/h";
        notify(SvcId::VEHICLE_SPEED, EventId::SPEED_CHANGED, ss.str());
        lastSpeed_ = data.speed;
    }

    // ── TempWarning: notify on any temp change > 3 °C ────────────────────────
    if (lastTemp_ < 0.0 || std::abs(data.temp - lastTemp_) > 3.0) {
        std::ostringstream ss;
        ss << "engTemp=" << std::fixed << std::setprecision(1) << data.temp << "C";
        notify(SvcId::ENGINE_TEMP, EventId::TEMP_WARNING, ss.str());
        lastTemp_ = data.temp;
    }

    // ── LowBattery: notify on any voltage change > 0.2 V ────────────────────
    if (lastVolt_ < 0.0 || std::abs(data.volt - lastVolt_) > 0.2) {
        std::ostringstream ss;
        ss << "volt=" << std::fixed << std::setprecision(2) << data.volt << "V";
        notify(SvcId::BATTERY, EventId::LOW_BATTERY, ss.str());
        lastVolt_ = data.volt;
    }

    // ── AlertFired: notify when active-alert set changes ─────────────────────
    const auto& active = mgr.getActiveAlerts();
    uint32_t alertBits = 0;
    for (const auto& a : active) {
        // Simple hash: fold alert code into bitmask for change detection
        alertBits ^= static_cast<uint32_t>(std::hash<std::string>{}(a.getCode()))
                     & 0xFFFFFFFF;
    }
    if (alertBits != lastAlertSet_) {
        std::ostringstream ss;
        ss << "activeAlerts=" << active.size();
        notify(SvcId::ALERT_BROADCAST, EventId::ALERT_FIRED, ss.str());
        lastAlertSet_ = alertBits;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  display  —  rich interactive SOC dashboard
// ─────────────────────────────────────────────────────────────────────────────

void ServiceOrientedComm::display(std::ostream& os) const
{
    std::lock_guard<std::mutex> lk(mutex_);

    constexpr int W = 70;
    auto rep = [](int n, const std::string& ch = "\xe2\x95\x90") {  // ═
        std::string r; for (int i=0;i<n;++i) r+=ch; return r;
    };
    auto bTop = [&](const char* c) {
        os << c << BOLD << "  ╔" << rep(W) << "╗\n" << RESET;
    };
    auto bBot = [&](const char* c) {
        os << c << BOLD << "  ╚" << rep(W) << "╝\n" << RESET;
    };
    auto bMid = [&](const char* c) {
        os << c << BOLD << "  ╠" << rep(W) << "╣\n" << RESET;
    };
    auto bRow = [&](const std::string& txt, const char* bc, const char* tc) {
        int pad = W - static_cast<int>(txt.size());
        if (pad < 0) pad = 0;
        os << bc << BOLD << "  ║" << RESET
           << tc << txt << std::string(pad,' ') << RESET
           << bc << BOLD << "║\n" << RESET;
    };
    auto bEmpty = [&](const char* c) { bRow("", c, c); };

    os << "\n";
    bTop(BMAGENTA);
    bRow("   SERVICE-ORIENTED COMMUNICATION MODEL (SOME/IP)", BMAGENTA, BMAGENTA);
    bRow("   AUTOSAR Adaptive Platform — ara::com Simulation", BMAGENTA, BCYAN);
    bBot(BMAGENTA);
    os << "\n";

    // ── Summary stats ─────────────────────────────────────────────────────────
    bTop(BBLUE);
    bRow("   BUS STATISTICS", BBLUE, BWHITE);
    bMid(BBLUE);
    {
        auto kv = [&](const std::string& k, const std::string& v,
                      const char* vc) {
            std::string line = "  " + k;
            int pad = 28 - static_cast<int>(line.size());
            if (pad < 1) pad = 1;
            line += std::string(pad,' ') + ": ";
            int padE = W - static_cast<int>(line.size()) - static_cast<int>(v.size());
            if (padE < 0) padE = 0;
            os << BBLUE << BOLD << "  ║" << RESET
               << BCYAN << line << RESET
               << vc << BOLD << v << RESET
               << std::string(padE,' ')
               << BBLUE << BOLD << "║\n" << RESET;
        };
        kv("Total Messages",       std::to_string(totalMessages_.load()),      BWHITE);
        kv("Method Calls",         std::to_string(totalMethodCalls_.load()),   BGREEN);
        kv("Event Notifications",  std::to_string(totalNotifications_.load()), BYELLOW);
        kv("Registered Services",  std::to_string(services_.size()),           BWHITE);
        kv("Active Subscriptions", std::to_string(subs_.size()),               BWHITE);
        kv("SD Complete",          sdComplete_ ? "YES" : "NO (call runSD())",  sdComplete_ ? BGREEN : BYELLOW);
    }
    bBot(BBLUE);
    os << "\n";

    // ── Service Registry ──────────────────────────────────────────────────────
    bTop(BCYAN);
    bRow("   SERVICE REGISTRY", BCYAN, BWHITE);
    bMid(BCYAN);
    bEmpty(BCYAN);

    for (const auto& svc : services_) {
        std::ostringstream line;
        line << "  [0x" << std::hex << std::uppercase
             << std::setw(4) << std::setfill('0') << svc.serviceId << "]  "
             << std::dec
             << std::left << std::setw(28) << svc.name
             << (svc.offering ? " ● OFFERING" : " ○ IDLE    ");
        bRow(line.str(), BCYAN, svc.offering ? BGREEN : BYELLOW);

        // Methods
        for (const auto& m : svc.methods) {
            std::ostringstream ml;
            ml << "       METHOD  0x" << std::hex << std::uppercase
               << std::setw(4) << std::setfill('0') << m.first << std::dec
               << "  " << m.second;
            bRow(ml.str(), BCYAN, BWHITE);
        }
        // Events
        for (const auto& e : svc.events) {
            std::ostringstream el;
            el << "       EVENT   0x" << std::hex << std::uppercase
               << std::setw(4) << std::setfill('0') << e.first << std::dec
               << "  " << e.second;
            bRow(el.str(), BCYAN, BYELLOW);
        }
        bEmpty(BCYAN);
    }
    bBot(BCYAN);
    os << "\n";

    // ── Subscription Table ────────────────────────────────────────────────────
    bTop(BGREEN);
    bRow("   ACTIVE EVENT SUBSCRIPTIONS", BGREEN, BWHITE);
    bMid(BGREEN);
    if (subs_.empty()) {
        bRow("   (no subscriptions — call subscribe() or run SD)", BGREEN, BYELLOW);
    } else {
        for (const auto& sub : subs_) {
            const ServiceDescriptor* svc = findService(sub.serviceId);
            std::string evtName = svc ? resolveEvent(*svc, sub.eventId) : "?";
            std::string svcName = svc ? svc->name : "?";

            std::ostringstream line;
            line << "  " << std::left << std::setw(22) << sub.subscriberName
                 << " ← " << std::setw(24) << svcName
                 << " :: " << std::setw(14) << evtName
                 << " (delivered: " << sub.deliveryCount << ")";
            bRow(line.str(), BGREEN, BWHITE);
        }
    }
    bBot(BGREEN);
    os << "\n";

    // ── Message Bus Log ───────────────────────────────────────────────────────
    bTop(BBLUE);
    bRow("   MESSAGE BUS LOG  (last 20 messages)", BBLUE, BWHITE);
    bMid(BBLUE);
    bEmpty(BBLUE);

    // Show last 20 entries
    size_t startIdx = log_.size() > 20 ? log_.size() - 20 : 0;
    for (size_t i = startIdx; i < log_.size(); ++i) {
        const auto& msg = log_[i];
        std::string line = "  " + msg.toOneLine();
        // Trim to width
        if (static_cast<int>(line.size()) > W)
            line = line.substr(0, W - 3) + "...";
        const char* tc = msgTypeColour(msg.header.msgType);
        bRow(line, BBLUE, tc);
    }
    bEmpty(BBLUE);
    bBot(BBLUE);
    os << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  displayCompact
// ─────────────────────────────────────────────────────────────────────────────

void ServiceOrientedComm::displayCompact(std::ostream& os) const
{
    std::lock_guard<std::mutex> lk(mutex_);
    os << BMAGENTA << BOLD << "  [SOC] "  << RESET
       << BCYAN << "Msgs=" << BWHITE << totalMessages_.load()
       << BCYAN << "  Notif=" << BYELLOW << totalNotifications_.load()
       << BCYAN << "  Calls=" << BGREEN  << totalMethodCalls_.load()
       << BCYAN << "  Subs=" << BWHITE   << subs_.size()
       << "  SD=" << (sdComplete_ ? BGREEN : BRED)
                  << (sdComplete_ ? "OK" : "--")
       << RESET << "\n";
}
