#include "Performance.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <map>
#include <array>
#include <numeric>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <cstring>
#include <stdexcept>
#include <ctime>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Repeat a UTF-8 box-drawing char (═ by default) n times
static std::string repW(int n, const std::string& ch = "\xe2\x95\x90") {
    std::string r; r.reserve(n * ch.size());
    for (int i = 0; i < n; ++i) r += ch;
    return r;
}

// Format uint64 with thousands separator  e.g. 1234567 → "1,234,567"
static std::string fmtCycles(uint64_t v) {
    std::string s = std::to_string(v);
    int insertPos = static_cast<int>(s.size()) - 3;
    while (insertPos > 0) { s.insert(insertPos, ","); insertPos -= 3; }
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// CpuCycleRecord
// ─────────────────────────────────────────────────────────────────────────────

void CpuCycleRecord::record(uint64_t cycles) {
    ++cycleCount;
    cycleTotal += cycles;
    if (cycles < cycleMin) cycleMin = cycles;
    if (cycles > cycleMax) cycleMax = cycles;
    // Welford incremental mean
    cycleAvg = cycleAvg + (static_cast<double>(cycles) - cycleAvg)
                        / static_cast<double>(cycleCount);
}

// ─────────────────────────────────────────────────────────────────────────────
// CpuCycleTracker
// ─────────────────────────────────────────────────────────────────────────────

CpuCycleTracker::CpuCycleTracker(CpuCycleRecord& record)
    : record_(record), start_(readTSC()) {}

CpuCycleTracker::~CpuCycleTracker() {
    uint64_t elapsed = readTSC() - start_;
    record_.record(elapsed);
}

// ─────────────────────────────────────────────────────────────────────────────
// PerformanceMonitor — constructor: register the three fixed modes
// ─────────────────────────────────────────────────────────────────────────────

PerformanceMonitor::PerformanceMonitor() {
    for (const std::string name : {"Manual", "Automatic", "JSON"}) {
        CpuCycleRecord r;
        r.modeName  = name;
        cycleMap_[name]  = r;
        cycleOrder_.push_back(name);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wall-clock timing
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceMonitor::update(const std::string& module, double cycleMs) {
    auto& m = metrics_[module];
    if (cycleMs < m.minMs) m.minMs = cycleMs;
    if (cycleMs > m.maxMs) m.maxMs = cycleMs;
    m.totalMs += cycleMs;
    ++m.count;
}

// ─────────────────────────────────────────────────────────────────────────────
// CPU cycle record accessors
// ─────────────────────────────────────────────────────────────────────────────

CpuCycleRecord& PerformanceMonitor::cycleRecord(const std::string& mode) {
    auto it = cycleMap_.find(mode);
    if (it == cycleMap_.end())
        throw std::out_of_range("Unknown cycle-track mode: " + mode);
    return it->second;
}

const CpuCycleRecord& PerformanceMonitor::cycleRecord(const std::string& mode) const {
    auto it = cycleMap_.find(mode);
    if (it == cycleMap_.end())
        throw std::out_of_range("Unknown cycle-track mode: " + mode);
    return it->second;
}

std::vector<const CpuCycleRecord*> PerformanceMonitor::allCycleRecords() const {
    std::vector<const CpuCycleRecord*> out;
    for (const auto& name : cycleOrder_) {
        auto it = cycleMap_.find(name);
        if (it != cycleMap_.end()) out.push_back(&it->second);
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-module CPU cycle record accessors
// ─────────────────────────────────────────────────────────────────────────────

CpuCycleRecord& PerformanceMonitor::moduleCycleRecord(const std::string& module) {
    auto it = moduleCycleMap_.find(module);
    if (it == moduleCycleMap_.end()) {
        CpuCycleRecord r;
        r.modeName = module;
        moduleCycleMap_[module] = r;
        moduleCycleOrder_.push_back(module);
        return moduleCycleMap_[module];
    }
    return it->second;
}

std::vector<const CpuCycleRecord*> PerformanceMonitor::allModuleCycleRecords() const {
    std::vector<const CpuCycleRecord*> out;
    for (const auto& name : moduleCycleOrder_) {
        auto it = moduleCycleMap_.find(name);
        if (it != moduleCycleMap_.end()) out.push_back(&it->second);
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// recordJsonTestCase  — store one per-test-case cycle measurement (JSON only)
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceMonitor::recordJsonTestCase(int tick,
                                            const std::string& description,
                                            uint64_t cycles) {
    JsonTestCaseResult r;
    r.tick        = tick;
    r.description = description;
    r.cycles      = cycles;
    jsonTestCaseResults_.push_back(r);
}

// ─────────────────────────────────────────────────────────────────────────────
// recordPerTestCaseModule  — store one module's cycle for one JSON tick
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceMonitor::recordPerTestCaseModule(int tick,
                                                  const std::string& desc,
                                                  const std::string& moduleName,
                                                  uint64_t cycles) {
    PerTestCaseModuleResult r;
    r.tick        = tick;
    r.description = desc;
    r.moduleName  = moduleName;
    r.cycles      = cycles;
    perTestModuleResults_.push_back(r);
}

// ─────────────────────────────────────────────────────────────────────────────
// Module size scanning
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceMonitor::scanModuleSizes(const std::string& srcDir,
                                          const std::string& incDir) {
    sizes_.clear();
    DIR* dir = opendir(srcDir.c_str());
    if (!dir) return;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string fname = entry->d_name;
        if (fname.size() <= 4) continue;
        if (fname.substr(fname.size()-4) != ".cpp") continue;

        std::string stem    = fname.substr(0, fname.size()-4);
        std::string srcPath = srcDir + "/" + fname;
        std::string incPath = incDir + "/" + stem + ".hpp";

        struct stat st{};
        if (stat(srcPath.c_str(), &st) == 0)
            sizes_[stem].srcBytes = static_cast<std::size_t>(st.st_size);
        if (stat(incPath.c_str(), &st) == 0)
            sizes_[stem].incBytes = static_cast<std::size_t>(st.st_size);
    }
    closedir(dir);
}

// ─────────────────────────────────────────────────────────────────────────────
// displayModuleSizes  — (unchanged from v1, still needed for terminal output)
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceMonitor::displayModuleSizes() const {
    if (sizes_.empty()) {
        std::cout << BYELLOW
                  << "  (No module-size data — run scanModuleSizes() first)\n"
                  << RESET;
        return;
    }

    constexpr int W = 90;
    std::cout << "\n" << BOLD << BBLUE
              << "  ╔" << repW(W) << "╗\n"
              << "  ║  MODULE SIZE REPORT" << std::string(W-20,' ') << "║\n"
              << "  ╠" << repW(W) << "╣\n" << RESET;

    std::cout << BOLD << BBLUE << "  ║" << RESET
              << BCYAN << BOLD
              << std::left  << std::setw(22) << "  MODULE"
              << std::right << std::setw(14) << "SRC (.cpp)"
              << std::right << std::setw(14) << "INC (.hpp)"
              << std::right << std::setw(14) << "TOTAL"
              << std::left  << std::setw(W - 64) << ""
              << RESET
              << BOLD << BBLUE << "║\n"
              << "  ╠" << repW(W) << "╣\n" << RESET;

    auto fmt = [](std::size_t b) -> std::string {
        std::ostringstream o;
        if (b >= 1024) o << std::fixed << std::setprecision(1) << b/1024.0 << " KB";
        else           o << b << " B";
        return o.str();
    };

    std::size_t grandTotal = 0;
    for (const auto& [name, sz] : sizes_) {
        grandTotal += sz.total();
        const char* vc = (sz.total() > 20000) ? BRED :
                         (sz.total() > 10000) ? BYELLOW : BGREEN;
        int pad = W - 2 - 22 - 14 - 14 - 14;
        if (pad < 0) pad = 0;
        std::cout << BOLD << BBLUE << "  ║" << RESET
                  << BCYAN  << std::left  << std::setw(22) << ("  " + name)
                  << vc     << std::right << std::setw(14) << fmt(sz.srcBytes)
                  << vc     << std::right << std::setw(14) << fmt(sz.incBytes)
                  << BWHITE << BOLD       << std::right << std::setw(14) << fmt(sz.total())
                  << std::string(pad,' ')
                  << RESET << BOLD << BBLUE << "║\n" << RESET;
    }

    std::cout << BOLD << BBLUE
              << "  ╠" << repW(W) << "╣\n"
              << "  ║" << RESET
              << BWHITE << BOLD
              << std::left  << std::setw(50) << "  GRAND TOTAL"
              << std::right << std::setw(14) << fmt(grandTotal)
              << std::string(W - 64,' ')
              << RESET << BOLD << BBLUE << "║\n"
              << "  ╚" << repW(W) << "╝\n" << RESET;
}

// ─────────────────────────────────────────────────────────────────────────────
// displayCycleTable  — terminal output for CPU cycle analysis
// ─────────────────────────────────────────────────────────────────────────────
//
// Column layout (printable widths):
//   MODE(14) | COUNT(8) | MIN CYCLES(18) | MAX CYCLES(18) | AVG CYCLES(18) | TOTAL CYCLES(22)
//
static constexpr int CC_MODE  = 14;
static constexpr int CC_CNT   =  8;
static constexpr int CC_MIN   = 18;
static constexpr int CC_MAX   = 18;
static constexpr int CC_AVG   = 18;

static void cycSep(const char* L, const char* M, const char* R, char F) {
    auto seg = [&](int w) { std::cout << std::string(w+2, F); };
    std::cout << BOLD << BCYAN << "  " << L;
    seg(CC_MODE); std::cout << M;
    seg(CC_CNT);  std::cout << M;
    seg(CC_MIN);  std::cout << M;
    seg(CC_MAX);  std::cout << M;
    seg(CC_AVG);  std::cout << R << "\n" << RESET;
}

void PerformanceMonitor::displayCycleTable() const {
    // Title
    std::cout << "\n" << BOLD << BMAGENTA;
    int TW = CC_MODE + CC_CNT + CC_MIN + CC_MAX + CC_AVG + 5*3 + 2;
    std::string title = "  CPU CYCLE ANALYSIS  (TSC ticks per test-case invocation)";
    int tpad = TW - 4 - static_cast<int>(title.size());
    std::cout << "  \u2554" << std::string(TW-4,'=') << "\u2557\n"
              << "  \u2551" << title
              << std::string(tpad < 0 ? 0 : tpad,' ') << "\u2551\n"
              << "  \u255a" << std::string(TW-4,'=') << "\u255d\n" << RESET;

    cycSep("\xe2\x95\x94", "\xe2\x95\xa6", "\xe2\x95\x97", '=');

    // Header row
    std::cout << BOLD << BCYAN << "  \u2551 "
              << std::left  << std::setw(CC_MODE) << "MODE"       << " \u2551 "
              << std::right << std::setw(CC_CNT)  << "COUNT"      << " \u2551 "
              << std::right << std::setw(CC_MIN)  << "MIN (cyc)"  << " \u2551 "
              << std::right << std::setw(CC_MAX)  << "MAX (cyc)"  << " \u2551 "
              << std::right << std::setw(CC_AVG)  << "AVG (cyc)"
              << " \u2551\n" << RESET;

    cycSep("\xe2\x95\xa0", "\xe2\x95\xac", "\xe2\x95\xa3", '=');

    bool even = true;
    for (const auto* rec : allCycleRecords()) {
        bool noData = !rec->hasData();
        const char* rc = noData ? "\033[1m\033[90m" : (even ? BWHITE : WHITE);

        auto cstr = [&](uint64_t v) -> std::string {
            return noData ? std::string(CC_MIN, '-') : fmtCycles(v);
        };
        auto astr = [&]() -> std::string {
            if (noData) return std::string(CC_AVG, '-');
            std::ostringstream o;
            o << std::fixed << std::setprecision(0) << rec->cycleAvg;
            std::string s = o.str();
            // add thousand separators
            int ip = static_cast<int>(s.size()) - 3;
            while (ip > 0) { s.insert(ip, ","); ip -= 3; }
            return s;
        };

        std::string sMin  = cstr(rec->cycleMin);
        std::string sMax  = cstr(rec->cycleMax);
        std::string sAvg  = astr();
        std::string sCnt  = noData ? std::string(CC_CNT,'-') : std::to_string(rec->cycleCount);

        // Right-align all number strings
        while (static_cast<int>(sCnt.size()) < CC_CNT)  sCnt  = " " + sCnt;
        while (static_cast<int>(sMin.size()) < CC_MIN)  sMin  = " " + sMin;
        while (static_cast<int>(sMax.size()) < CC_MAX)  sMax  = " " + sMax;
        while (static_cast<int>(sAvg.size()) < CC_AVG)  sAvg  = " " + sAvg;

        std::cout << BOLD << BCYAN << "  \u2551 " << RESET
                  << rc << BOLD
                  << std::left  << std::setw(CC_MODE) << rec->modeName
                  << BCYAN << " \u2551 " << rc << sCnt
                  << BCYAN << " \u2551 " << rc << sMin
                  << BCYAN << " \u2551 " << rc << sMax
                  << BCYAN << " \u2551 " << rc << sAvg
                  << BOLD << BCYAN << " \u2551\n" << RESET;

        if (rec != allCycleRecords().back())
            cycSep("\xe2\x95\x9f", "\xe2\x95\xab", "\xe2\x95\xa2", '-');

        even = !even;
    }
    cycSep("\xe2\x95\x9a", "\xe2\x95\xa9", "\xe2\x95\x9d", '=');

    std::cout << BOLD << BWHITE
              << "  Note: cycles = raw TSC ticks (processor clock counter).\n"
              << "        Lower is faster.  '--' = mode not exercised this session.\n"
              << RESET << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// displayModuleCycleTable  — terminal output for per-module CPU cycle analysis
// ─────────────────────────────────────────────────────────────────────────────

void PerformanceMonitor::displayModuleCycleTable() const {
    auto recs = allModuleCycleRecords();
    if (recs.empty()) {
        std::cout << BYELLOW
                  << "  (No per-module cycle data recorded this session.)\n"
                  << RESET;
        return;
    }

    std::cout << "\n" << BOLD << BGREEN;
    int TW = CC_MODE + CC_CNT + CC_MIN + CC_MAX + CC_AVG + 5*3 + 2;
    std::string title = "  MODULE CPU CYCLE ANALYSIS  (core compute only — I/O excluded)";
    int tpad = TW - 4 - static_cast<int>(title.size());
    std::cout << "  \u2554" << std::string(TW-4,'=') << "\u2557\n"
              << "  \u2551" << title
              << std::string(tpad < 0 ? 0 : tpad,' ') << "\u2551\n"
              << "  \u255a" << std::string(TW-4,'=') << "\u255d\n" << RESET;

    cycSep("\xe2\x95\x94", "\xe2\x95\xa6", "\xe2\x95\x97", '=');

    std::cout << BOLD << BCYAN << "  \u2551 "
              << std::left  << std::setw(CC_MODE) << "MODULE"      << " \u2551 "
              << std::right << std::setw(CC_CNT)  << "COUNT"       << " \u2551 "
              << std::right << std::setw(CC_MIN)  << "MIN (cyc)"   << " \u2551 "
              << std::right << std::setw(CC_MAX)  << "MAX (cyc)"   << " \u2551 "
              << std::right << std::setw(CC_AVG)  << "AVG (cyc)"
              << " \u2551\n" << RESET;

    cycSep("\xe2\x95\xa0", "\xe2\x95\xac", "\xe2\x95\xa3", '=');

    // Keep only the 6 sensor modules in the terminal display
    static const std::array<const char*, 6> kDisplaySensors = {
        "Engine Sensor", "Battery Sensor", "Speed Sensor",
        "Tire Sensor", "Door Sensor", "Seatbelt Sensor"
    };
    std::vector<const CpuCycleRecord*> sensorRecs;
    for (const auto* r : recs) {
        for (const auto* m : kDisplaySensors)
            if (r->modeName == m) { sensorRecs.push_back(r); break; }
    }

    bool even = true;
    for (const auto* rec : sensorRecs) {
        bool noData = !rec->hasData();
        const char* rc = noData ? "\033[1m\033[90m" : (even ? BWHITE : WHITE);

        auto cstr = [&](uint64_t v) -> std::string {
            return noData ? std::string(CC_MIN, '-') : fmtCycles(v);
        };
        auto astr = [&]() -> std::string {
            if (noData) return std::string(CC_AVG, '-');
            std::ostringstream o;
            o << std::fixed << std::setprecision(0) << rec->cycleAvg;
            std::string s = o.str();
            int ip = static_cast<int>(s.size()) - 3;
            while (ip > 0) { s.insert(ip, ","); ip -= 3; }
            return s;
        };

        std::string sMin = cstr(rec->cycleMin);
        std::string sMax = cstr(rec->cycleMax);
        std::string sAvg = astr();
        std::string sCnt = noData ? std::string(CC_CNT,'-') : std::to_string(rec->cycleCount);

        while (static_cast<int>(sCnt.size()) < CC_CNT) sCnt = " " + sCnt;
        while (static_cast<int>(sMin.size()) < CC_MIN) sMin = " " + sMin;
        while (static_cast<int>(sMax.size()) < CC_MAX) sMax = " " + sMax;
        while (static_cast<int>(sAvg.size()) < CC_AVG) sAvg = " " + sAvg;

        std::cout << BOLD << BCYAN << "  \u2551 " << RESET
                  << rc << BOLD
                  << std::left  << std::setw(CC_MODE) << rec->modeName
                  << BCYAN << " \u2551 " << rc << sCnt
                  << BCYAN << " \u2551 " << rc << sMin
                  << BCYAN << " \u2551 " << rc << sMax
                  << BCYAN << " \u2551 " << rc << sAvg
                  << BOLD << BCYAN << " \u2551\n" << RESET;

        if (rec != sensorRecs.back())
            cycSep("\xe2\x95\x9f", "\xe2\x95\xab", "\xe2\x95\xa2", '-');

        even = !even;
    }
    cycSep("\xe2\x95\x9a", "\xe2\x95\xa9", "\xe2\x95\x9d", '=');

    std::cout << BOLD << BWHITE
              << "  Note: measures pure computational logic only.\n"
              << "        std::cout, file I/O, and sleep() are excluded.\n"
              << RESET << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// exportReport  — write complete report to file
// ─────────────────────────────────────────────────────────────────────────────

static std::string nowStr() {
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return buf;
}

void PerformanceMonitor::exportReport(const std::string& filename) const {
    std::ofstream out(filename);
    if (!out) return;

    out << "SMART VEHICLE PERFORMANCE REPORT\n";
    out << "Generated : " << nowStr() << "\n";
    out << std::string(80,'=') << "\n\n";

    // ── Section 1: Wall-clock timing ─────────────────────────────────────────
    out << "SECTION 1: TIMING METRICS  (wall-clock, milliseconds)\n";
    out << std::string(80,'-') << "\n";
    out << std::left
        << std::setw(28) << "MODULE"
        << std::right
        << std::setw(12) << "MIN(ms)"
        << std::setw(12) << "MAX(ms)"
        << std::setw(12) << "AVG(ms)"
        << std::setw(10) << "COUNT"
        << "\n";
    out << std::string(80,'-') << "\n";

    if (metrics_.empty()) {
        out << "  (no timing data — live simulation was not run this session)\n";
    } else {
        for (const auto& [name, m] : metrics_) {
            double avg = m.count ? m.totalMs / m.count : 0.0;
            out << std::left  << std::setw(28) << name
                << std::fixed << std::setprecision(3)
                << std::right << std::setw(12) << m.minMs
                << std::setw(12) << m.maxMs
                << std::setw(12) << avg
                << std::setw(10) << m.count
                << "\n";
        }
    }

    // ── Section 2: Per-Test-Case Module Performance ───────────────────────────
    out << "\n\nSECTION 2: PER-TEST-CASE MODULE PERFORMANCE\n";
    out << std::string(80,'-') << "\n";
    out << "  Measurement: TSC delta per sensor module per JSON test-case invocation.\n";
    out << "  Columns     : Min / Max / Avg CPU cycles across all runs of that module.\n";
    out << std::string(80,'-') << "\n";

    // The 6 sensor modules to report (in display order)
    static const std::array<const char*, 6> kSensorModules = {
        "Engine Sensor", "Battery Sensor", "Speed Sensor",
        "Tire Sensor", "Door Sensor", "Seatbelt Sensor"
    };

    if (perTestModuleResults_.empty()) {
        out << "  (no per-module per-test-case data — JSON mode was not exercised)\n";
    } else {
        // Group results by tick, preserving tick order
        struct TickData {
            int         tick = 0;
            std::string description;
            std::map<std::string, std::vector<uint64_t>> moduleCycles;
        };
        std::vector<TickData> ticks;
        std::map<int,int> tickIndex;

        for (const auto& r : perTestModuleResults_) {
            bool isSensor = false;
            for (const auto* m : kSensorModules)
                if (r.moduleName == m) { isSensor = true; break; }
            if (!isSensor) continue;

            if (tickIndex.find(r.tick) == tickIndex.end()) {
                tickIndex[r.tick] = static_cast<int>(ticks.size());
                TickData td;
                td.tick = r.tick;
                td.description = r.description;
                ticks.push_back(td);
            }
            ticks[tickIndex[r.tick]].moduleCycles[r.moduleName].push_back(r.cycles);
        }

        const int colMod = 18;
        const int colVal = 10;

        for (const auto& td : ticks) {
            out << "\n=========== TEST CASE " << td.tick
                << " : " << td.description << " ===========\n\n";

            out << std::left  << std::setw(colMod) << "Module"
                << std::right << std::setw(colVal)  << "Min"
                << std::right << std::setw(colVal)  << "Max"
                << std::right << std::setw(colVal)  << "Avg"
                << "\n";
            out << std::string(51, '-') << "\n";

            for (const auto* modName : kSensorModules) {
                auto it = td.moduleCycles.find(modName);
                if (it == td.moduleCycles.end() || it->second.empty()) {
                    out << std::left  << std::setw(colMod) << modName
                        << std::right << std::setw(colVal)  << "--"
                        << std::right << std::setw(colVal)  << "--"
                        << std::right << std::setw(colVal)  << "--"
                        << "\n";
                    continue;
                }
                const auto& vals = it->second;
                uint64_t mn = *std::min_element(vals.begin(), vals.end());
                uint64_t mx = *std::max_element(vals.begin(), vals.end());
                double   av = static_cast<double>(
                                  std::accumulate(vals.begin(), vals.end(), uint64_t(0)))
                              / static_cast<double>(vals.size());
                std::ostringstream avSS;
                avSS << std::fixed << std::setprecision(0) << av;
                out << std::left  << std::setw(colMod) << modName
                    << std::right << std::setw(colVal)  << mn
                    << std::right << std::setw(colVal)  << mx
                    << std::right << std::setw(colVal)  << avSS.str()
                    << "\n";
            }
            out << std::string(51, '=') << "\n";
        }
    }
    out << "\n";

    // ── Section 3: Module sizes ───────────────────────────────────────────────
    if (!sizes_.empty()) {
        out << "\n\nSECTION 3: MODULE SIZE REPORT  (source-file bytes)\n";
        out << std::string(80,'-') << "\n";
        out << std::left  << std::setw(22) << "MODULE"
            << std::right << std::setw(14) << "SRC (.cpp)"
            << std::right << std::setw(14) << "INC (.hpp)"
            << std::right << std::setw(12) << "TOTAL"
            << "\n";
        out << std::string(80,'-') << "\n";

        auto fmt = [](std::size_t b) -> std::string {
            std::ostringstream o;
            if (b >= 1024) o << std::fixed << std::setprecision(1) << b/1024.0 << " KB";
            else           o << b << " B";
            return o.str();
        };

        std::size_t grand = 0;
        for (const auto& [name, sz] : sizes_) {
            grand += sz.total();
            out << std::left  << std::setw(22) << name
                << std::right << std::setw(14) << fmt(sz.srcBytes)
                << std::right << std::setw(14) << fmt(sz.incBytes)
                << std::right << std::setw(12) << fmt(sz.total())
                << "\n";
        }
        out << std::string(80,'-') << "\n";
        out << std::left  << std::setw(50) << "GRAND TOTAL"
            << std::right << std::setw(12) << fmt(grand) << "\n";
    }

    out << "\n" << std::string(80,'=') << "\n";
    out << "END OF REPORT\n";
}

// Global instance
PerformanceMonitor g_performanceMonitor;
