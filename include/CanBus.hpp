/**
 * @file    CanBus.hpp
 * @brief   CAN (Controller Area Network) bus simulation for the Smart Vehicle ECU.
 *
 * @details Models a realistic automotive CAN bus at the frame level:
 *
 *  Frame structure (ISO 11898 standard 11-bit IDs):
 *  ┌──────────────┬─────┬──────────────────────┬──────┐
 *  │ Arbitration  │ DLC │  Data (0–8 bytes)     │  CRC │
 *  │  ID (11-bit) │ (4) │                       │ (sim)│
 *  └──────────────┴─────┴──────────────────────┴──────┘
 *
 *  Assigned CAN IDs (automotive style):
 *  ┌────────────┬──────────────────────────────────────────┐
 *  │  0x100     │ Engine Temperature  (2 bytes, 0.5°C/LSB) │
 *  │  0x200     │ Battery Voltage     (2 bytes, 0.01V/LSB) │
 *  │  0x300     │ Vehicle Speed       (2 bytes, 0.1km/h/LSB│
 *  │  0x400     │ Tire Pressure       (2 bytes, 0.1PSI/LSB)│
 *  │  0x500     │ Door + Seatbelt     (1 byte,  bitmask)   │
 *  │  0x600     │ Alert flags         (1 byte,  bitmask)   │
 *  └────────────┴──────────────────────────────────────────┘
 *
 *  The CanBusSimulator:
 *  - Encodes live SensorData into CAN frames.
 *  - Maintains a TX ring-buffer (capacity TX_QUEUE_CAPACITY frames).
 *  - Simulates a background "bus transmit" thread at BUS_SPEED_KBPS.
 *  - Keeps a full RX log for post-drive analysis and filtering.
 *  - Detects and counts bus-off / bit-error events (injected at 1% probability).
 *  - Decodes frames back to human-readable values for the diagnostic display.
 *
 *  Automotive mapping:
 *  - CanFrame         → AUTOSAR ComStack PDU / MCAL CAN frame
 *  - CanBusSimulator  → CAN Interface / CanIf + CAN Driver layer
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#pragma once
#ifndef CANBUS_HPP
#define CANBUS_HPP

#include "AlertEvaluator.hpp"   // SensorData

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <ostream>
#include <chrono>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
//  CAN ID assignments
// ─────────────────────────────────────────────────────────────────────────────
namespace CanId {
    constexpr uint16_t ENGINE_TEMP   = 0x100; ///< Engine temperature frame.
    constexpr uint16_t BATTERY_VOLT  = 0x200; ///< Battery voltage frame.
    constexpr uint16_t VEHICLE_SPEED = 0x300; ///< Vehicle speed frame.
    constexpr uint16_t TIRE_PRESSURE = 0x400; ///< Tire pressure frame.
    constexpr uint16_t DOOR_BELT     = 0x500; ///< Door + seatbelt bitmask frame.
    constexpr uint16_t ALERT_FLAGS   = 0x600; ///< Alert active-flag frame.
}

// ─────────────────────────────────────────────────────────────────────────────
//  CanFrame  —  ISO 11898 standard-format CAN frame (11-bit ID)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct CanFrame
 * @brief  One CAN bus frame: arbitration ID, DLC, up to 8 data bytes, and
 *         a simulated CRC-8 checksum.
 *
 * @details All fields use fixed-width integer types matching the on-wire
 *          representation.  The timestamp field records simulation cycle time
 *          (milliseconds since CanBusSimulator construction) for replay.
 */
struct CanFrame {
    uint16_t              arbId{0};       ///< 11-bit arbitration ID.
    uint8_t               dlc{0};         ///< Data Length Code (0–8 bytes).
    std::array<uint8_t,8> data{};         ///< Payload bytes (only dlc bytes are valid).
    uint8_t               crc8{0};        ///< Simulated CRC-8 over data bytes.
    uint64_t              timestampMs{0}; ///< Milliseconds since bus start.
    bool                  errorFrame{false}; ///< True if this is a simulated bus-error.

    /**
     * @brief  Returns a compact hex string: "ID:0x100 [02] AA BB | CRC:0xA3"
     */
    std::string toHexString() const;

    /**
     * @brief  Decodes the frame payload to a human-readable physical value string.
     *
     * @details Applies the scaling/offset rules defined for each CanId.
     *          Returns e.g. "Engine Temp: 97.5 °C".
     *          Unknown IDs return "Unknown ID:0xXXX".
     */
    std::string decodeToPhysical() const;

    /**
     * @brief  Stream insertion — one-line formatted frame for log output.
     */
    friend std::ostream& operator<<(std::ostream& os, const CanFrame& f);
};

// ─────────────────────────────────────────────────────────────────────────────
//  CanBusStats  —  accumulated bus-health counters
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct CanBusStats
 * @brief  Counters updated by CanBusSimulator for diagnostic display.
 */
struct CanBusStats {
    uint64_t txFrames{0};    ///< Total frames transmitted.
    uint64_t rxFrames{0};    ///< Total frames received (echoed into RX log).
    uint64_t errorFrames{0}; ///< Frames that triggered a simulated bit-error.
    uint64_t txDropped{0};   ///< Frames dropped because TX queue was full.
    double   busLoadPct{0.0};///< Estimated bus load percentage (running avg).
};

// ─────────────────────────────────────────────────────────────────────────────
//  CanBusSimulator
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class  CanBusSimulator
 * @brief  Encodes sensor data into CAN frames, simulates a TX queue +
 *         background transmit thread, and keeps an RX log for analysis.
 *
 * @details
 *  Lifecycle:
 *    1. Construct — spawns the background transmit thread.
 *    2. Call encode() each sensor cycle to push 6 frames to the TX queue.
 *    3. The transmit thread drains the queue at BUS_SPEED_KBPS.
 *    4. Call display() or searchRxLog() for diagnostics.
 *    5. Destructor joins the transmit thread cleanly.
 *
 *  Thread-safety:
 *    txQueue_ and rxLog_ are protected by busMutex_.
 *    encode() and display() acquire the mutex.
 *    The transmit thread holds the mutex only while moving one frame.
 */
class CanBusSimulator {
public:
    // ── Bus configuration constants ───────────────────────────────────────────
    static constexpr int      BUS_SPEED_KBPS      = 500;     ///< Simulated 500 kbps CAN-HS.
    static constexpr std::size_t TX_QUEUE_CAPACITY = 64;     ///< Max TX queue depth.
    static constexpr std::size_t RX_LOG_CAPACITY   = 200;    ///< Max RX log entries kept.
    static constexpr double   ERROR_RATE            = 0.01;  ///< 1% frame error probability.

    /**
     * @brief  Constructs the simulator and starts the background TX thread.
     */
    CanBusSimulator();

    /**
     * @brief  Stops the TX thread and joins it cleanly.
     */
    ~CanBusSimulator();

    // Non-copyable — owns a thread and mutexes.
    CanBusSimulator(const CanBusSimulator&)            = delete;
    CanBusSimulator& operator=(const CanBusSimulator&) = delete;

    // ── Core API ──────────────────────────────────────────────────────────────

    /**
     * @brief  Encodes one complete sensor snapshot into 6 CAN frames and
     *         enqueues them in the TX queue.
     *
     * @details Encoding rules:
     *  - ENGINE_TEMP   : uint16 = (temp  + 40.0) / 0.5   (offset -40°C, 0.5°C/LSB)
     *  - BATTERY_VOLT  : uint16 = volt  / 0.01            (0.01V/LSB)
     *  - VEHICLE_SPEED : uint16 = speed / 0.1             (0.1km/h/LSB)
     *  - TIRE_PRESSURE : uint16 = psi   / 0.1             (0.1PSI/LSB)
     *  - DOOR_BELT     : uint8  = [bit0=doorOpen, bit1=beltLocked]
     *  - ALERT_FLAGS   : uint8  = reserved (0x00 unless overridden)
     *
     * @param  s  SensorData snapshot to encode.
     */
    void encode(const SensorData& s);

    /**
     * @brief  Searches the RX log for frames matching a predicate.
     *
     * @details Uses std::copy_if with the supplied lambda — satisfies the
     *          STL + lambda hackathon requirement.
     *
     * @param  pred  Callable(const CanFrame&) → bool.
     * @return Vector of matching frames (copies).
     */
    std::vector<CanFrame> searchRxLog(
        const std::function<bool(const CanFrame&)>& pred) const;

    /**
     * @brief  Returns all RX log entries for a given arbitration ID.
     * @param  arbId  CAN arbitration ID to filter on.
     * @return Vector of matching frames.
     */
    std::vector<CanFrame> filterById(uint16_t arbId) const;

    /**
     * @brief  Returns a snapshot of the current bus statistics.
     */
    CanBusStats getStats() const;

    /**
     * @brief  Returns the number of frames currently waiting in the TX queue.
     */
    std::size_t txQueueDepth() const;

    /**
     * @brief  Renders a full diagnostic display panel to the given stream.
     *
     * @details Shows:
     *  - Bus configuration banner (speed, queue capacity, error rate).
     *  - Live bus statistics table.
     *  - Last 10 RX log entries decoded to physical values.
     *  - Error frame log (last 5).
     *
     * @param  os  Destination stream (usually std::cout).
     */
    void display(std::ostream& os) const;

    /**
     * @brief  Resets statistics counters and clears the RX log.
     */
    void reset();

private:
    // ── State ──────────────────────────────────────────────────────────────
    mutable std::mutex     busMutex_;         ///< Guards txQueue_, rxLog_, stats_.
    std::deque<CanFrame>   txQueue_;          ///< Pending frames awaiting TX.
    std::vector<CanFrame>  rxLog_;            ///< Received (transmitted) frame log.
    std::vector<CanFrame>  errorLog_;         ///< Error frames for diagnostic display.
    CanBusStats            stats_;            ///< Accumulated bus counters.

    std::thread            txThread_;         ///< Background transmit thread.
    std::atomic<bool>      running_{false};   ///< Transmit-thread run flag.

    std::chrono::steady_clock::time_point startTime_; ///< Bus start timestamp.

    // ── Private helpers ────────────────────────────────────────────────────

    /**
     * @brief  Background transmit loop — drains txQueue_ at BUS_SPEED_KBPS.
     */
    void transmitLoop();

    /**
     * @brief  Builds a single CAN frame with proper CRC and timestamp.
     * @param  id    Arbitration ID.
     * @param  bytes Pointer to payload bytes.
     * @param  dlc   Payload length (1–8).
     * @return Populated CanFrame.
     */
    CanFrame buildFrame(uint16_t id, const uint8_t* bytes, uint8_t dlc) const;

    /**
     * @brief  Computes CRC-8/MAXIM over the data bytes.
     * @param  data  Pointer to byte array.
     * @param  len   Number of bytes.
     * @return 8-bit CRC value.
     */
    static uint8_t crc8(const uint8_t* data, uint8_t len);

    /**
     * @brief  Returns elapsed milliseconds since bus start.
     */
    uint64_t elapsedMs() const;

    /**
     * @brief  Injects a random bit-error into a frame with ERROR_RATE probability.
     * @param  frame  Frame to potentially corrupt.
     */
    static void maybeInjectError(CanFrame& frame);

    /**
     * @brief  Updates the running bus-load estimate.
     * @param  frameBits  Number of bits in the just-transmitted frame.
     */
    void updateBusLoad(int frameBits);
};

#endif // CANBUS_HPP
