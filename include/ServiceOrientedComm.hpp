/**
 * @file    ServiceOrientedComm.hpp
 * @brief   Service-Oriented Communication (SOC) model for the Smart Vehicle ECU.
 *
 * @details Models an automotive Service-Oriented Architecture (SOA) communication
 *          layer inspired by AUTOSAR Adaptive Platform and SOME/IP (Scalable
 *          service-Oriented MiddlewarE over IP, PRS_SOMEIP) specifications:
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │           SERVICE-ORIENTED COMMUNICATION — ARCHITECTURE                 │
 *  ├─────────────────────────────────────────────────────────────────────────┤
 *  │  Service Layer                                                           │
 *  │    Each "service" exposes a named interface (e.g. SpeedService,         │
 *  │    AlertService) with a unique 16-bit Service ID and an instance ID.    │
 *  │    Services register themselves with the ServiceRegistry on startup.    │
 *  │                                                                          │
 *  │  Method Calls (SOME/IP Method invocation)                               │
 *  │    Clients send Request messages; services reply with Response.         │
 *  │    Each call has a Client ID + Session ID for correlation.              │
 *  │    Fire-and-forget (no-reply) methods are also supported.               │
 *  │                                                                          │
 *  │  Event / Publish-Subscribe (SOME/IP EventGroup)                         │
 *  │    Services publish events whenever sensor data changes.                │
 *  │    Clients subscribe to EventGroups; the broker delivers notifications. │
 *  │                                                                          │
 *  │  Service Discovery (SOME/IP SD)                                         │
 *  │    On startup, services broadcast OfferService SD messages.             │
 *  │    Clients send FindService; matched by service ID + instance ID.       │
 *  │                                                                          │
 *  │  Message Bus (in-process simulation)                                    │
 *  │    An in-process message queue simulates the network transport.         │
 *  │    Messages carry a SOME/IP header: Service ID, Method ID, Length,     │
 *  │    Client ID, Session ID, Protocol Version, Interface Version, Type.   │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *  Registered Services (this simulation):
 *  ┌──────────────────────────┬──────────┬──────────────────────────────────┐
 *  │  Service Name            │  Svc ID  │  Methods / Events                │
 *  ├──────────────────────────┼──────────┼──────────────────────────────────┤
 *  │  VehicleSpeedService     │  0x0101  │  GetSpeed, SpeedChanged (event)  │
 *  │  EngineTemperatureService│  0x0102  │  GetEngineTemp, TempWarning(evt) │
 *  │  BatteryService          │  0x0103  │  GetVoltage, LowBattery (event)  │
 *  │  AlertBroadcastService   │  0x0201  │  GetActiveAlerts, AlertFired(evt)│
 *  │  DiagnosticsService      │  0x0301  │  GetDtcList, ClearDtcs, RunBist  │
 *  └──────────────────────────┴──────────┴──────────────────────────────────┘
 *
 *  SOME/IP Message Types:
 *  ┌──────────────────┬───────────────────────────────────────────────────┐
 *  │  REQUEST (0x00)  │  Method call with expected response               │
 *  │  RESPONSE (0x80) │  Reply to a REQUEST                               │
 *  │  NOTIFICATION(1) │  Event notification to subscribers                │
 *  │  REQUEST_NO_RETURN(0x01) │ Fire-and-forget method call               │
 *  └──────────────────┴───────────────────────────────────────────────────┘
 *
 *  Automotive mapping:
 *  - ServiceRegistry    → AUTOSAR ara::com Service Discovery
 *  - SomeIpMessage      → AUTOSAR SOME/IP PDU header
 *  - ServiceProxy       → ara::com Proxy (client-side handle)
 *  - ServiceSkeleton    → ara::com Skeleton (service-side handle)
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef SERVICE_ORIENTED_COMM_HPP
#define SERVICE_ORIENTED_COMM_HPP

#include "AlertEvaluator.hpp"   // SensorData
#include "Alert.hpp"            // AlertManager

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <unordered_map>
#include <functional>
#include <ostream>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

// ─────────────────────────────────────────────────────────────────────────────
//  SOME/IP Message Type enumeration
// ─────────────────────────────────────────────────────────────────────────────

enum class SomeIpMsgType : uint8_t {
    REQUEST            = 0x00,  ///< Method request (expects response).
    REQUEST_NO_RETURN  = 0x01,  ///< Fire-and-forget method call.
    NOTIFICATION       = 0x02,  ///< Event notification to subscribers.
    RESPONSE           = 0x80,  ///< Successful response to REQUEST.
    ERROR              = 0x81,  ///< Error response to REQUEST.
    SD_OFFER           = 0xF0,  ///< Service Discovery — OfferService.
    SD_FIND            = 0xF1,  ///< Service Discovery — FindService.
    SD_SUBSCRIBE       = 0xF2,  ///< Service Discovery — Subscribe EventGroup.
};

/// Human-readable label for SomeIpMsgType.
std::string msgTypeToString(SomeIpMsgType t);

/// ANSI colour for SomeIpMsgType (for terminal display).
const char* msgTypeColour(SomeIpMsgType t);

// ─────────────────────────────────────────────────────────────────────────────
//  Return Codes
// ─────────────────────────────────────────────────────────────────────────────

enum class SomeIpReturnCode : uint8_t {
    E_OK                = 0x00,  ///< No error.
    E_NOT_OK            = 0x01,  ///< Unspecified error.
    E_UNKNOWN_SERVICE   = 0x02,  ///< Service ID not found.
    E_UNKNOWN_METHOD    = 0x03,  ///< Method ID not found on service.
    E_NOT_READY         = 0x04,  ///< Service exists but not yet offering.
    E_WRONG_PROTOCOL_VER= 0x08,  ///< Protocol version mismatch.
};

/// Human-readable label for SomeIpReturnCode.
std::string returnCodeToString(SomeIpReturnCode rc);

// ─────────────────────────────────────────────────────────────────────────────
//  SomeIpHeader  —  simulated SOME/IP 16-byte fixed header
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct SomeIpHeader
 * @brief  SOME/IP wire-format header (PRS_SOMEIP §4.1.1).
 *
 * @note   In this simulation all fields are stored in host byte-order.
 *         A real implementation would serialise to big-endian (network order).
 */
struct SomeIpHeader {
    uint16_t serviceId      {0x0000}; ///< Identifies the service interface.
    uint16_t methodId       {0x0000}; ///< Method or Event ID within the service.
    uint32_t length         {0};      ///< Payload length in bytes (after header).
    uint16_t clientId       {0x0000}; ///< Identifies the calling client.
    uint16_t sessionId      {0x0000}; ///< Unique per-client request counter.
    uint8_t  protocolVersion{0x01};   ///< SOME/IP protocol version (always 1).
    uint8_t  interfaceVersion{0x01};  ///< Service interface major version.
    SomeIpMsgType  msgType  {SomeIpMsgType::REQUEST};
    SomeIpReturnCode returnCode{SomeIpReturnCode::E_OK};
};

// ─────────────────────────────────────────────────────────────────────────────
//  SomeIpMessage  —  one complete bus message (header + payload string)
// ─────────────────────────────────────────────────────────────────────────────

struct SomeIpMessage {
    SomeIpHeader header;
    std::string  payload;          ///< Human-readable serialised payload.
    std::string  timestamp;        ///< Wall-clock time of creation.
    std::string  serviceName;      ///< Resolved service name (for display).
    std::string  methodName;       ///< Resolved method/event name (for display).

    /// Returns a single-line summary for the message bus log.
    std::string toOneLine() const;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Service Descriptor  —  static metadata for one registered service
// ─────────────────────────────────────────────────────────────────────────────

struct ServiceDescriptor {
    uint16_t    serviceId;
    uint16_t    instanceId  {0x0001};
    std::string name;
    std::string description;
    bool        offering    {false};  ///< True when SD OfferService has fired.

    /// Method/Event sub-entries: method ID → name.
    std::map<uint16_t, std::string> methods;
    std::map<uint16_t, std::string> events;
};

// ─────────────────────────────────────────────────────────────────────────────
//  EventSubscription  —  tracks one subscriber for one event group
// ─────────────────────────────────────────────────────────────────────────────

struct EventSubscription {
    uint16_t serviceId;
    uint16_t eventId;
    uint16_t subscriberClientId;
    std::string subscriberName;
    std::chrono::steady_clock::time_point subscribedAt;
    uint32_t deliveryCount {0};  ///< How many event notifications delivered.
};

// ─────────────────────────────────────────────────────────────────────────────
//  ServiceOrientedComm  —  main class
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class ServiceOrientedComm
 * @brief Models an in-process SOME/IP-inspired service-oriented middleware bus.
 *
 * @details Provides:
 *   - A service registry (register, discover, offer/find SD messages).
 *   - A publish-subscribe event bus with subscription tracking.
 *   - A method-call dispatcher with request/response logging.
 *   - A message bus log (circular, capped at MAX_LOG_ENTRIES).
 *   - Live sensor integration: publishSensorData() drives event notifications.
 *   - An interactive terminal display via display().
 */
class ServiceOrientedComm {
public:
    // ── Bus capacity constants ────────────────────────────────────────────────
    static constexpr size_t   MAX_LOG_ENTRIES  = 200;  ///< Message bus ring size.
    static constexpr uint16_t CLIENT_DASHBOARD = 0xC001; ///< Dashboard client ID.
    static constexpr uint16_t CLIENT_DIAG      = 0xC002; ///< Diagnostics client ID.
    static constexpr uint16_t CLIENT_LOGGER    = 0xC003; ///< Logger client ID.
    static constexpr uint16_t CLIENT_ECU_MGR   = 0xC004; ///< ECU manager client ID.

    /// Constructs the bus and registers all built-in vehicle services.
    ServiceOrientedComm();
    ~ServiceOrientedComm() = default;

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /**
     * @brief  Runs Service Discovery: broadcasts OfferService for each service.
     * @details Generates one SD_OFFER message per registered service and
     *          marks each as offering = true.
     */
    void runServiceDiscovery();

    /**
     * @brief  Subscribes a client to an event on a given service.
     * @param  serviceId   Target service.
     * @param  eventId     Event ID within that service.
     * @param  clientId    Subscribing client ID.
     * @param  clientName  Human-readable client label.
     * @return true on success; false if service/event not found.
     */
    bool subscribe(uint16_t serviceId, uint16_t eventId,
                   uint16_t clientId, const std::string& clientName);

    /**
     * @brief  Dispatches a method call to a service; appends REQUEST + RESPONSE.
     * @param  serviceId  Target service ID.
     * @param  methodId   Method to call.
     * @param  clientId   Calling client.
     * @param  payload    Serialised arguments (human-readable for simulation).
     * @return Return code of the method call.
     */
    SomeIpReturnCode callMethod(uint16_t serviceId, uint16_t methodId,
                                uint16_t clientId,
                                const std::string& payload = "");

    /**
     * @brief  Publishes live sensor data, triggering relevant event notifications.
     * @details Compares each sensor value against its last published value;
     *          generates NOTIFICATION messages for changed events (SpeedChanged,
     *          TempWarning, LowBattery, AlertFired) and records all published
     *          values.
     * @param  data  Latest sensor snapshot.
     * @param  mgr   Alert manager for active-alert queries.
     */
    void publishSensorData(const SensorData& data, const AlertManager& mgr);

    // ── Queries ───────────────────────────────────────────────────────────────

    /// Returns the full message bus log (newest last).
    const std::deque<SomeIpMessage>& messageLog() const { return log_; }

    /// Returns the service registry.
    const std::vector<ServiceDescriptor>& services() const { return services_; }

    /// Returns all active subscriptions.
    const std::vector<EventSubscription>& subscriptions() const { return subs_; }

    /// Returns total messages published since construction.
    uint64_t totalMessages() const { return totalMessages_; }

    /// Returns total event notifications delivered.
    uint64_t totalNotifications() const { return totalNotifications_; }

    /// Returns total method calls dispatched.
    uint64_t totalMethodCalls() const { return totalMethodCalls_; }

    // ── Display ───────────────────────────────────────────────────────────────

    /**
     * @brief  Renders an interactive SOC dashboard to the given stream.
     * @details Shows the service registry, subscription table, SD state,
     *          and a scrolling message bus log.
     * @param  os   Output stream (typically std::cout).
     */
    void display(std::ostream& os) const;

    /**
     * @brief  Renders a compact one-line status (for live sim integration).
     * @param  os   Output stream.
     */
    void displayCompact(std::ostream& os) const;

private:
    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Appends a message to log_, dropping oldest when at capacity.
    void appendLog(SomeIpMessage msg);

    /// Generates a SOME/IP header with auto-incremented session ID.
    SomeIpHeader makeHeader(uint16_t svc, uint16_t method,
                             uint16_t client, SomeIpMsgType type,
                             SomeIpReturnCode rc = SomeIpReturnCode::E_OK) const;

    /// Looks up service by ID; returns nullptr if not found.
    const ServiceDescriptor* findService(uint16_t serviceId) const;
          ServiceDescriptor* findServiceMut(uint16_t serviceId);

    /// Resolves method name from a service descriptor.
    static std::string resolveMethod(const ServiceDescriptor& svc, uint16_t mid);

    /// Resolves event name from a service descriptor.
    static std::string resolveEvent(const ServiceDescriptor& svc, uint16_t eid);

    /// Returns current wall-clock timestamp string.
    static std::string nowString();

    // ── State ─────────────────────────────────────────────────────────────────
    std::vector<ServiceDescriptor> services_;   ///< Registered service catalogue.
    std::vector<EventSubscription> subs_;       ///< Active subscriptions.
    std::deque<SomeIpMessage>      log_;        ///< Message bus ring log.

    mutable std::mutex             mutex_;
    mutable uint16_t               sessionCounter_{0x0001}; ///< Session ID counter.

    std::atomic<uint64_t> totalMessages_     {0};
    std::atomic<uint64_t> totalNotifications_{0};
    std::atomic<uint64_t> totalMethodCalls_  {0};

    // ── Last-published values for change-detection ────────────────────────────
    double   lastSpeed_   {-1.0};
    double   lastTemp_    {-1.0};
    double   lastVolt_    {-1.0};
    uint32_t lastAlertSet_{0};     ///< Bitmask of active alert codes.

    bool sdComplete_ {false};      ///< True after runServiceDiscovery().
};

// ─────────────────────────────────────────────────────────────────────────────
//  Well-known Service IDs
// ─────────────────────────────────────────────────────────────────────────────
namespace SvcId {
    constexpr uint16_t VEHICLE_SPEED  = 0x0101;
    constexpr uint16_t ENGINE_TEMP    = 0x0102;
    constexpr uint16_t BATTERY        = 0x0103;
    constexpr uint16_t ALERT_BROADCAST= 0x0201;
    constexpr uint16_t DIAGNOSTICS    = 0x0301;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Well-known Method IDs (shared across services where applicable)
// ─────────────────────────────────────────────────────────────────────────────
namespace MethodId {
    // VehicleSpeedService
    constexpr uint16_t GET_SPEED        = 0x0001;
    // EngineTemperatureService
    constexpr uint16_t GET_ENGINE_TEMP  = 0x0001;
    // BatteryService
    constexpr uint16_t GET_VOLTAGE      = 0x0001;
    // AlertBroadcastService
    constexpr uint16_t GET_ACTIVE_ALERTS= 0x0001;
    // DiagnosticsService
    constexpr uint16_t GET_DTC_LIST     = 0x0001;
    constexpr uint16_t CLEAR_DTCS       = 0x0002;
    constexpr uint16_t RUN_BIST         = 0x0003;  ///< Built-In Self Test.
}

// ─────────────────────────────────────────────────────────────────────────────
//  Well-known Event IDs
// ─────────────────────────────────────────────────────────────────────────────
namespace EventId {
    constexpr uint16_t SPEED_CHANGED  = 0x8001;  ///< Speed crossed threshold.
    constexpr uint16_t TEMP_WARNING   = 0x8002;  ///< Engine temp crossed limit.
    constexpr uint16_t LOW_BATTERY    = 0x8003;  ///< Voltage dropped below limit.
    constexpr uint16_t ALERT_FIRED    = 0x8004;  ///< New alert active.
}

#endif // SERVICE_ORIENTED_COMM_HPP
