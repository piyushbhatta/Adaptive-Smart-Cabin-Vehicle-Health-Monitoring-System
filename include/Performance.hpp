/**
 * @file    Performance.hpp
 * @brief   Runtime timing, CPU cycle profiling, and module-size reporter.
 *
 * @details Provides two complementary measurement systems:
 *
 *  1. Wall-clock timing (PerformanceMonitor::update)
 *     Records min/max/avg execution time in milliseconds for each named
 *     module.  Used by the Live Simulation threads.
 *
 *  2. CPU cycle counting (CpuCycleTracker)
 *     Uses the processor Time-Stamp Counter (TSC via __builtin_ia32_rdtsc
 *     where available, otherwise std::clock as fallback) to measure raw
 *     CPU cycles consumed by each discrete test invocation.
 *
 *     Three tracker instances are maintained — one per interactive mode:
 *       - "Manual"    : Option 1  — each complete manual test iteration
 *       - "Automatic" : Option 2  — each dashboard render cycle
 *       - "JSON"      : Option 3  — each JSON parse + evaluate call
 *
 *     Per-tracker metrics stored:
 *       - cycleMin   : smallest single-call cycle count
 *       - cycleMax   : largest  single-call cycle count
 *       - cycleAvg   : running mean (Welford algorithm)
 *       - cycleCount : total samples recorded
 *       - cycleTotal : cumulative sum of all recorded cycles
 *
 *  exportReport() writes BOTH sections — timing metrics + CPU cycle
 *  analysis — to logs/performance_report.txt.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 2.0
 */

#pragma once
#ifndef PERFORMANCE_HPP
#define PERFORMANCE_HPP

#include <string>
#include <map>
#include <vector>
#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <memory>

// ─────────────────────────────────────────────────────────────────────────────
// TSC helper — reads the hardware Time-Stamp Counter
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Reads the CPU Time-Stamp Counter (TSC) with LFENCE serialization.
 *
 * @details On x86/x86-64, issues an LFENCE before RDTSC to drain the
 *          out-of-order execution pipeline, preventing earlier instructions
 *          from leaking into the measured window.  This gives tight,
 *          reproducible cycle counts for micro-benchmarking.
 *
 *          Falls back to std::clock() on non-x86 architectures.
 *
 * @return 64-bit serialized cycle count.
 */
inline uint64_t readTSC()
{
#if defined(__i386__) || defined(__x86_64__)
    __builtin_ia32_lfence();          // serialize: drain OOO pipeline
    return __builtin_ia32_rdtsc();
#else
    return static_cast<uint64_t>(std::clock());
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// CpuCycleRecord  —  accumulated stats for one named test mode
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  CpuCycleRecord
 * @brief   Per-mode CPU cycle statistics — min, max, avg, total, count.
 *
 * @details One instance exists for each of the three interactive modes:
 *          Manual, Automatic, and JSON.  All values are in raw TSC ticks.
 */
struct CpuCycleRecord {
    std::string modeName;                         ///< "Manual" / "Automatic" / "JSON"
    uint64_t    cycleMin   = UINT64_MAX;          ///< Minimum single-call cycles.
    uint64_t    cycleMax   = 0;                   ///< Maximum single-call cycles.
    double      cycleAvg   = 0.0;                 ///< Running mean cycles (Welford).
    uint64_t    cycleTotal = 0;                   ///< Sum of all recorded cycles.
    uint64_t    cycleCount = 0;                   ///< Total samples recorded.

    /**
     * @brief  Records one CPU cycle measurement and updates all statistics.
     * @param  cycles  Raw TSC delta for this invocation.
     */
    void record(uint64_t cycles);

    /**
     * @brief  Returns true if at least one sample has been recorded.
     */
    bool hasData() const { return cycleCount > 0; }
};

// ─────────────────────────────────────────────────────────────────────────────
// CpuCycleTracker  —  RAII scope guard for automatic cycle measurement
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   CpuCycleTracker
 * @brief   RAII scope guard: captures TSC at construction, records delta on
 *          destruction into a CpuCycleRecord.
 *
 * @details Usage:
 * @code
 *   {
 *       CpuCycleTracker guard(g_perfMon.cycleRecord("Manual"));
 *       // ... one iteration of manual input processing ...
 *   }  // ← cycles recorded here automatically
 * @endcode
 */
class CpuCycleTracker {
public:
    /**
     * @brief  Captures the start TSC.
     * @param  record  Reference to the CpuCycleRecord to update on scope exit.
     */
    explicit CpuCycleTracker(CpuCycleRecord& record);

    /**
     * @brief  Calculates elapsed cycles and records them.
     */
    ~CpuCycleTracker();

    CpuCycleTracker(const CpuCycleTracker&)            = delete;
    CpuCycleTracker& operator=(const CpuCycleTracker&) = delete;

private:
    CpuCycleRecord& record_;  ///< Target record.
    uint64_t        start_;   ///< TSC value at construction.
};

// ─────────────────────────────────────────────────────────────────────────────
// ModuleMetrics  —  wall-clock timing per module (unchanged from v1)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  ModuleMetrics
 * @brief   Wall-clock execution timing for one named software module.
 *
 * @details Times are in milliseconds.  Updated by PerformanceMonitor::update().
 */
struct ModuleMetrics {
    double             minMs   = DBL_MAX; ///< Minimum execution time (ms).
    double             maxMs   = 0.0;     ///< Maximum execution time (ms).
    double             totalMs = 0.0;     ///< Cumulative execution time (ms).
    unsigned long long count   = 0;       ///< Sample count.
};

// ─────────────────────────────────────────────────────────────────────────────
// ModuleSize  —  source-file byte sizes per module
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  ModuleSize
 * @brief   Source-file byte counts for one module (.cpp + .hpp).
 */
struct ModuleSize {
    std::size_t srcBytes = 0;           ///< .cpp file size in bytes.
    std::size_t incBytes = 0;           ///< .hpp file size in bytes.
    std::size_t total()  const { return srcBytes + incBytes; } ///< Combined size.
};

// ─────────────────────────────────────────────────────────────────────────────
// PerTestCaseModuleResult  —  per-module per-test-case cycle (JSON mode only)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  PerTestCaseModuleResult
 * @brief   CPU cycle measurement for one module within one JSON test-case tick.
 *
 * @details Recorded for each of the 8 tracked modules per tick:
 *          Engine Sensor, Battery Sensor, Speed Sensor, Tire Sensor,
 *          Door Sensor, Seatbelt Sensor, AlertEvaluator, Logger.
 */
struct PerTestCaseModuleResult {
    int         tick        = 0;   ///< Tick number (1-based).
    std::string description;       ///< Human-readable test-case description.
    std::string moduleName;        ///< Module name (e.g. "Engine Sensor").
    uint64_t    cycles      = 0;   ///< Raw TSC ticks for this module this tick.
};

// ─────────────────────────────────────────────────────────────────────────────
// JsonTestCaseResult  —  per-test-case CPU cycle result (JSON mode only)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @struct  JsonTestCaseResult
 * @brief   CPU cycle measurement for a single JSON test-case invocation.
 *
 * @details Stores the tick number, human-readable description, and the raw
 *          TSC delta recorded during evaluateAlerts() for that test case.
 */
struct JsonTestCaseResult {
    int         tick        = 0;   ///< Tick number from test_cases.json.
    std::string description;       ///< Human-readable description.
    uint64_t    cycles      = 0;   ///< Raw TSC ticks for this invocation.
};

// ─────────────────────────────────────────────────────────────────────────────
// PerformanceMonitor  —  central registry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class   PerformanceMonitor
 * @brief   Singleton-style global performance registry.
 *
 * @details Responsibilities:
 *          - Wall-clock timing for simulation modules (update / displayModuleSizes).
 *          - CPU cycle tracking for the three interactive modes.
 *          - Source-file size scanning (scanModuleSizes).
 *          - exportReport() — writes everything to performance_report.txt.
 *
 * @note    Accessed via the global g_performanceMonitor instance.
 */
class PerformanceMonitor {
public:
    PerformanceMonitor();

    // ── Wall-clock timing ─────────────────────────────────────────────────────

    /**
     * @brief  Records one wall-clock timing sample for a named module.
     * @param  module   Module identifier string.
     * @param  cycleMs  Elapsed milliseconds for this cycle.
     */
    void update(const std::string& module, double cycleMs);

    // ── CPU cycle tracking ────────────────────────────────────────────────────

    /**
     * @brief  Returns a reference to the CpuCycleRecord for the given mode.
     *
     * @param  mode  One of: "Manual", "Automatic", "JSON".
     * @return Reference to the mode's CpuCycleRecord.
     */
    CpuCycleRecord& cycleRecord(const std::string& mode);

    /**
     * @brief  Returns a const reference to the CpuCycleRecord for the given mode.
     * @param  mode  One of: "Manual", "Automatic", "JSON".
     */
    const CpuCycleRecord& cycleRecord(const std::string& mode) const;

    /**
     * @brief  Returns all three CpuCycleRecord objects in insertion order.
     * @return Vector of const references (Manual, Automatic, JSON).
     */
    std::vector<const CpuCycleRecord*> allCycleRecords() const;

    // ── Per-module CPU cycle tracking ─────────────────────────────────────

    /**
     * @brief  Returns a reference to the CpuCycleRecord for a named module.
     *
     * @details Creates the record on first access with modeName set to
     *          @p module.  Used with CpuCycleTracker to benchmark
     *          individual modules (Alert, AlertEvaluator, Dashboard, Logger …).
     *
     * @param  module  Module name string (e.g. "AlertEvaluator", "Dashboard").
     * @return Reference to the module's CpuCycleRecord.
     */
    CpuCycleRecord& moduleCycleRecord(const std::string& module);

    /**
     * @brief  Returns all per-module CpuCycleRecord objects in insertion order.
     * @return Vector of const pointers.
     */
    std::vector<const CpuCycleRecord*> allModuleCycleRecords() const;

    /// @brief Prints the per-module CPU cycle table to stdout with ANSI colours.
    void displayModuleCycleTable() const;

    // ── Per-test-case JSON cycle tracking ─────────────────────────────────────

    /**
     * @brief  Records the CPU cycles for one JSON test-case invocation.
     *
     * @details Called once per tick inside runJsonFileTest(), after the
     *          CpuCycleTracker scope exits.  The measurement is stored in
     *          jsonTestCaseResults_ and written to performance_report.txt by
     *          exportReport().
     *
     * @param  tick         Tick number (1-based, from test_cases.json).
     * @param  description  Human-readable test-case description.
     * @param  cycles       Raw TSC delta measured during evaluateAlerts().
     */
    void recordJsonTestCase(int tick,
                            const std::string& description,
                            uint64_t cycles);

    /**
     * @brief  Records CPU cycles for one module within one JSON test-case tick.
     *
     * @details Called once per module per tick inside runJsonFileTest().
     *          Six sensor modules are recorded: Engine Sensor, Battery Sensor,
     *          Speed Sensor, Tire Sensor, Door Sensor, Seatbelt Sensor.
     *          Written to performance_report.txt Section 2d by exportReport().
     *
     * @param  tick        Tick number (1-based).
     * @param  desc        Human-readable test-case description.
     * @param  moduleName  Module name string.
     * @param  cycles      Raw TSC delta for this module this tick.
     */
    void recordPerTestCaseModule(int tick,
                                 const std::string& desc,
                                 const std::string& moduleName,
                                 uint64_t cycles);

    // ── Module size scanning ──────────────────────────────────────────────────

    /**
     * @brief  Scans source directories and populates per-module file sizes.
     * @param  srcDir  Path to the src/ directory.
     * @param  incDir  Path to the include/ directory.
     */
    void scanModuleSizes(const std::string& srcDir = "src",
                         const std::string& incDir = "include");

    // ── Display helpers ───────────────────────────────────────────────────────

    /// @brief Prints the module-size table to stdout with ANSI colours.
    void displayModuleSizes() const;

    /// @brief Prints the CPU cycle analysis table to stdout with ANSI colours.
    void displayCycleTable() const;

    // ── Export ────────────────────────────────────────────────────────────────

    /**
     * @brief  Writes the full performance report to @p filename.
     *
     * @details Sections written:
     *          1. TIMING METRICS          (wall-clock per module)
     *          2. CPU CYCLE ANALYSIS      (TSC per mode: Manual/Auto/JSON)
     *          3. MODULE SIZE REPORT      (source-file sizes)
     *
     * @param  filename  Output file path (e.g. "logs/performance_report.txt").
     */
    void exportReport(const std::string& filename) const;

private:
    std::map<std::string, ModuleMetrics> metrics_;  ///< Wall-clock timing map.
    std::map<std::string, ModuleSize>    sizes_;    ///< File-size map.

    // CPU cycle records — fixed set, insertion-ordered via vector
    std::map<std::string, CpuCycleRecord>  cycleMap_;        ///< Keyed by mode name.
    std::vector<std::string>               cycleOrder_;      ///< Preserves Manual/Auto/JSON order.

    // Per-module CPU cycle records — dynamically registered on first access
    std::map<std::string, CpuCycleRecord>  moduleCycleMap_;  ///< Keyed by module name.
    std::vector<std::string>               moduleCycleOrder_;///< Insertion order for display.

    // Per-test-case JSON cycle results (JSON mode only, stored in tick order)
    std::vector<JsonTestCaseResult>        jsonTestCaseResults_; ///< One entry per JSON tick.

    // Per-module per-test-case cycle results (JSON mode only)
    std::vector<PerTestCaseModuleResult>   perTestModuleResults_; ///< One entry per module per tick.
};

/// @brief Process-wide PerformanceMonitor instance (defined in Performance.cpp).
extern PerformanceMonitor g_performanceMonitor;

#endif // PERFORMANCE_HPP
