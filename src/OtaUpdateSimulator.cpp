/**
 * @file    OtaUpdateSimulator.cpp
 * @brief   Implementation of OtaUpdateSimulator — full OTA firmware update
 *          lifecycle simulation for the Smart Vehicle ECU.
 *
 * @author  Visteon C++ Hackathon Team
 * @version 1.0
 */

#include "OtaUpdateSimulator.hpp"
#include "Dashboard.hpp"   // Color namespace

#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <random>
#include <functional>
#include <ctime>
#include <limits>
#include <chrono>
#include <thread>

using namespace Color;

// ─────────────────────────────────────────────────────────────────────────────
//  Thread-local RNG
// ─────────────────────────────────────────────────────────────────────────────

static std::mt19937& rng() {
    thread_local std::mt19937 gen{std::random_device{}()};
    return gen;
}
static double randDouble() {
    thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng());
}
static int randInt(int lo, int hi) {
    std::uniform_int_distribution<int> d(lo, hi);
    return d(rng());
}

// ─────────────────────────────────────────────────────────────────────────────
//  OtaState helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string otaStateToString(OtaState s)
{
    switch (s) {
    case OtaState::IDLE:        return "IDLE";
    case OtaState::CHECKING:    return "CHECKING";
    case OtaState::DOWNLOADING: return "DOWNLOADING";
    case OtaState::VERIFYING:   return "VERIFYING";
    case OtaState::READY:       return "READY";
    case OtaState::INSTALLING:  return "INSTALLING";
    case OtaState::REBOOTING:   return "REBOOTING";
    case OtaState::SUCCESS:     return "SUCCESS";
    case OtaState::FAILED:      return "FAILED";
    }
    return "UNKNOWN";
}

const char* otaStateColour(OtaState s)
{
    switch (s) {
    case OtaState::IDLE:        return BCYAN;
    case OtaState::CHECKING:    return BYELLOW;
    case OtaState::DOWNLOADING: return BBLUE;
    case OtaState::VERIFYING:   return BYELLOW;
    case OtaState::READY:       return BGREEN;
    case OtaState::INSTALLING:  return BMAGENTA;
    case OtaState::REBOOTING:   return BYELLOW;
    case OtaState::SUCCESS:     return BGREEN;
    case OtaState::FAILED:      return BRED;
    }
    return BWHITE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  FirmwareVersion
// ─────────────────────────────────────────────────────────────────────────────

std::string FirmwareVersion::toString() const
{
    return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
}

FirmwareVersion FirmwareVersion::nextPatch() const
{
    return { major, minor, patch + 1 };
}

FirmwareVersion FirmwareVersion::nextMinor() const
{
    return { major, minor + 1, 0 };
}

// ─────────────────────────────────────────────────────────────────────────────
//  OtaPackage
// ─────────────────────────────────────────────────────────────────────────────

double OtaPackage::downloadPercent() const
{
    if (totalChunks == 0) return 0.0;
    return (static_cast<double>(downloadedChunks) /
            static_cast<double>(totalChunks)) * 100.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  OtaUpdateSimulator — construction
// ─────────────────────────────────────────────────────────────────────────────

OtaUpdateSimulator::OtaUpdateSimulator()
{
    // Initialise the default ECU module fleet (automotive-style IDs)
    modules_ = {
        { "BCM",  "Body Control Module",       {1, 4, 2}, {1, 4, 1}, false },
        { "ECU",  "Engine Control Unit",        {3, 1, 0}, {3, 0, 9}, false },
        { "ADAS", "Advanced Driver Assist Sys", {2, 0, 5}, {2, 0, 4}, false },
        { "TCU",  "Telematics Gateway Unit",    {1, 2, 1}, {1, 2, 0}, false },
        { "IC",   "Instrument Cluster",         {1, 0, 3}, {1, 0, 2}, false },
    };
}

// ─────────────────────────────────────────────────────────────────────────────
//  Private static helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string OtaUpdateSimulator::nowStr()
{
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
    return buf;
}

std::string OtaUpdateSimulator::generateChecksum(const std::string& pkgId,
                                                  const FirmwareVersion& v)
{
    // Deterministic pseudo-hash using std::hash (simulation only)
    std::size_t h = std::hash<std::string>{}(pkgId + v.toString());
    std::ostringstream os;
    os << std::hex << std::setfill('0');
    for (int i = 0; i < 4; ++i) {
        os << std::setw(16) << (h ^ (h << (i * 7 + 3)));
    }
    return os.str().substr(0, 64); // 64-char hex ~ SHA-256
}

std::string OtaUpdateSimulator::generateSignature(const std::string& pkgId)
{
    std::size_t h = std::hash<std::string>{}("SIG:" + pkgId);
    std::ostringstream os;
    os << std::hex << std::setfill('0') << std::setw(16) << h
       << std::setw(16) << (h ^ 0xDEADBEEFCAFEBABEULL);
    return os.str();
}

std::string OtaUpdateSimulator::generateReleaseNotes(const std::string& moduleId,
                                                       const FirmwareVersion& from,
                                                       const FirmwareVersion& to)
{
    if (moduleId == "BCM")
        return "Improved door-lock response time; fixed HVAC fan speed rounding error.";
    if (moduleId == "ECU")
        return "Fuel injection timing calibration update; reduced cold-start emissions.";
    if (moduleId == "ADAS")
        return "Enhanced lane-keep assist sensitivity; pedestrian detection range +15%.";
    if (moduleId == "TCU")
        return "TLS 1.3 upgrade for OTA channel; improved 5G handoff stability.";
    if (moduleId == "IC")
        return "Speedometer animation smoothing; corrected fuel gauge non-linearity.";
    return "General stability improvements. " + from.toString() + " → " + to.toString();
}

EcuModule* OtaUpdateSimulator::findModule(const std::string& id)
{
    for (auto& m : modules_)
        if (m.id == id) return &m;
    return nullptr;
}

std::string OtaUpdateSimulator::progressBar(double pct, int width)
{
    int filled = static_cast<int>((pct / 100.0) * width);
    if (filled < 0) filled = 0;
    if (filled > width) filled = width;
    std::string bar = "[";
    for (int i = 0; i < filled; ++i)      bar += "\xe2\x96\x88"; // █
    for (int i = 0; i < (width - filled); ++i) bar += "\xe2\x96\x91"; // ░
    bar += "] ";
    std::ostringstream pctStr;
    pctStr << std::fixed << std::setprecision(0) << pct << "%";
    bar += pctStr.str();
    return bar;
}

void OtaUpdateSimulator::logEvent(const std::string& moduleId,
                                   OtaState s,
                                   const std::string& detail)
{
    OtaHistoryEntry e;
    e.timestamp = nowStr();
    e.moduleId  = moduleId;
    e.state     = s;
    e.detail    = detail;
    if (history_.size() >= static_cast<std::size_t>(HISTORY_CAPACITY))
        history_.pop_front();
    history_.push_back(e);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Query API
// ─────────────────────────────────────────────────────────────────────────────

OtaState OtaUpdateSimulator::currentState() const
{
    std::lock_guard<std::mutex> lk(otaMutex_);
    return state_;
}

std::vector<EcuModule> OtaUpdateSimulator::getModules() const
{
    std::lock_guard<std::mutex> lk(otaMutex_);
    return modules_;
}

OtaPackage OtaUpdateSimulator::currentPackage() const
{
    std::lock_guard<std::mutex> lk(otaMutex_);
    return activePackage_;
}

OtaStats OtaUpdateSimulator::getStats() const
{
    std::lock_guard<std::mutex> lk(otaMutex_);
    return stats_;
}

std::vector<OtaHistoryEntry> OtaUpdateSimulator::getHistory(std::size_t n) const
{
    std::lock_guard<std::mutex> lk(otaMutex_);
    std::vector<OtaHistoryEntry> result;
    std::size_t sz = history_.size();
    std::size_t start = (sz > n) ? sz - n : 0;
    // Return in reverse (newest first)
    for (std::size_t i = sz; i > start; --i)
        result.push_back(history_[i - 1]);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
//  checkForUpdates
// ─────────────────────────────────────────────────────────────────────────────

int OtaUpdateSimulator::checkForUpdates()
{
    std::lock_guard<std::mutex> lk(otaMutex_);

    state_ = OtaState::CHECKING;
    ++stats_.totalChecks;
    logEvent("-", OtaState::CHECKING, "Polling OTA server at ota.visteon-cloud.example/v2");

    // Randomly choose one module that has an update available
    // (In a real system the server would respond with a manifest)
    std::vector<std::size_t> candidates;
    for (std::size_t i = 0; i < modules_.size(); ++i)
        candidates.push_back(i);

    // 70% chance we find an update
    if (randDouble() > 0.70) {
        state_ = OtaState::IDLE;
        logEvent("-", OtaState::IDLE, "Server poll complete — no updates available");
        return 0;
    }

    // Pick a random module
    std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
    std::size_t idx = pick(rng());
    EcuModule& mod  = modules_[idx];

    // Decide patch vs minor bump
    FirmwareVersion newVer = (randDouble() < 0.7)
                             ? mod.currentVersion.nextPatch()
                             : mod.currentVersion.nextMinor();

    std::string pkgId = mod.id + "-" + newVer.toString() + "-OTA";

    activePackage_.packageId        = pkgId;
    activePackage_.targetModuleId   = mod.id;
    activePackage_.newVersion       = newVer;
    activePackage_.totalChunks      = static_cast<std::size_t>(TOTAL_CHUNKS);
    activePackage_.downloadedChunks = 0;
    activePackage_.packageSizeKB    = static_cast<std::size_t>(TOTAL_CHUNKS * CHUNK_SIZE_KB);
    activePackage_.checksum         = generateChecksum(pkgId, newVer);
    activePackage_.signature        = generateSignature(pkgId);
    activePackage_.releaseNotes     = generateReleaseNotes(
                                          mod.id, mod.currentVersion, newVer);

    mod.updatePending = true;
    hasActivePackage_ = true;
    ++stats_.packagesFound;

    logEvent(mod.id, OtaState::CHECKING,
             "Package found: " + pkgId + "  (" +
             mod.currentVersion.toString() + " → " + newVer.toString() + ")  " +
             std::to_string(activePackage_.packageSizeKB) + " kB");
    return 1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  startDownload  —  chunked download with animated progress bar
// ─────────────────────────────────────────────────────────────────────────────

void OtaUpdateSimulator::startDownload(std::ostream& os)
{
    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        if (state_ != OtaState::CHECKING || !hasActivePackage_) {
            os << BRED << "  [OTA] No package pending download.\n" << RESET;
            return;
        }
        state_ = OtaState::DOWNLOADING;
        activePackage_.downloadedChunks = 0;
        logEvent(activePackage_.targetModuleId, OtaState::DOWNLOADING,
                 "Download started: " + activePackage_.packageId);
    }

    os << "\n" << BBLUE << BOLD
       << "  ╔══════════════════════════════════════════════════════════════╗\n"
       << "  ║   OTA DOWNLOAD  —  " << std::left << std::setw(44)
       << activePackage_.packageId << "║\n"
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << RESET;

    // Simulate chunk-by-chunk download with ~30 ms per chunk
    std::size_t total = activePackage_.totalChunks;
    for (std::size_t chunk = 1; chunk <= total; ++chunk) {
        // Simulate network jitter
        std::this_thread::sleep_for(std::chrono::milliseconds(25 + randInt(0, 20)));

        {
            std::lock_guard<std::mutex> lk(otaMutex_);
            activePackage_.downloadedChunks = chunk;
        }

        double pct  = (static_cast<double>(chunk) / static_cast<double>(total)) * 100.0;
        std::string bar = progressBar(pct, 36);
        std::size_t kbSoFar = chunk * static_cast<std::size_t>(CHUNK_SIZE_KB);

        os << BBLUE << BOLD << "  ║ " << RESET
           << BCYAN << "  " << bar
           << "  " << std::right << std::setw(4) << kbSoFar << " / "
           << activePackage_.packageSizeKB << " kB"
           << RESET;

        // Pad to box width
        std::ostringstream line;
        line << "  " << bar << "  " << std::right << std::setw(4)
             << kbSoFar << " / " << activePackage_.packageSizeKB << " kB";
        int pad = 62 - static_cast<int>(line.str().size());
        os << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ')
           << BBLUE << BOLD << "║\r" << RESET;
        os.flush();
    }

    os << "\n" << BBLUE << BOLD
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << "  ║" << BGREEN << "  ✔  Download complete — "
       << activePackage_.packageSizeKB << " kB received"
       << std::string(62 - 26 - std::to_string(activePackage_.packageSizeKB).size(), ' ')
       << BBLUE << BOLD << "║\n"
       << "  ╚══════════════════════════════════════════════════════════════╝\n"
       << RESET << "\n";

    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        stats_.totalDownloadKB += static_cast<long>(activePackage_.packageSizeKB);
        logEvent(activePackage_.targetModuleId, OtaState::DOWNLOADING,
                 "Download complete: " + std::to_string(activePackage_.packageSizeKB) + " kB");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  verifyPackage
// ─────────────────────────────────────────────────────────────────────────────

bool OtaUpdateSimulator::verifyPackage(std::ostream& os)
{
    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        if (state_ != OtaState::DOWNLOADING || !hasActivePackage_) {
            os << BRED << "  [OTA] No downloaded package to verify.\n" << RESET;
            return false;
        }
        if (activePackage_.downloadedChunks < activePackage_.totalChunks) {
            os << BRED << "  [OTA] Download incomplete — cannot verify.\n" << RESET;
            return false;
        }
        state_ = OtaState::VERIFYING;
        logEvent(activePackage_.targetModuleId, OtaState::VERIFYING,
                 "Verification started for: " + activePackage_.packageId);
    }

    os << "\n" << BYELLOW << BOLD
       << "  ╔══════════════════════════════════════════════════════════════╗\n"
       << "  ║   OTA VERIFICATION                                           ║\n"
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << RESET;

    auto verifyStep = [&](const std::string& label, bool ok) {
        std::string mark = ok ? "\xe2\x9c\x94  " : "\xe2\x9c\x98  "; // ✔ / ✘
        std::string line = "  " + mark + label;
        int pad = 62 - static_cast<int>(line.size());
        os << BYELLOW << BOLD << "  ║" << RESET
           << (ok ? BGREEN : BRED) << line
           << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ')
           << BYELLOW << BOLD << "║\n" << RESET;
    };

    // Step 1: Package integrity (chunk count)
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    verifyStep("Chunk count integrity     [" +
               std::to_string(activePackage_.downloadedChunks) + "/" +
               std::to_string(activePackage_.totalChunks) + " OK]", true);

    // Step 2: SHA-256 checksum (10% simulated failure)
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    bool checksumOk = (randDouble() > CHECKSUM_FAIL_RATE);
    verifyStep("SHA-256 checksum          [" +
               activePackage_.checksum.substr(0, 16) + "...]", checksumOk);

    // Step 3: RSA-4096 signature (only run if checksum passed)
    bool sigOk = false;
    if (checksumOk) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        sigOk = true; // Signature always passes if checksum did
        verifyStep("RSA-4096 signature        [" +
                   activePackage_.signature.substr(0, 16) + "...]", sigOk);
    } else {
        verifyStep("RSA-4096 signature        [SKIPPED — checksum failed]", false);
    }

    bool passed = checksumOk && sigOk;

    os << BYELLOW << BOLD
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << "  ║" << RESET
       << (passed ? BGREEN : BRED)
       << (passed ? "  ✔  Verification PASSED — package is authentic and complete"
                  : "  ✘  Verification FAILED — checksum mismatch detected")
       << std::string(passed ? 4 : 2, ' ')
       << BYELLOW << BOLD << "║\n"
       << "  ╚══════════════════════════════════════════════════════════════╝\n"
       << RESET << "\n";

    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        if (passed) {
            state_ = OtaState::READY;
            logEvent(activePackage_.targetModuleId, OtaState::READY,
                     "Package verified OK — ready to install");
        } else {
            state_ = OtaState::FAILED;
            ++stats_.failedUpdates;
            logEvent(activePackage_.targetModuleId, OtaState::FAILED,
                     "Verification failed — checksum mismatch");
        }
    }

    if (!passed) {
        rollback(os);
    }
    return passed;
}

// ─────────────────────────────────────────────────────────────────────────────
//  installUpdate
// ─────────────────────────────────────────────────────────────────────────────

bool OtaUpdateSimulator::installUpdate(double currentSpeed, std::ostream& os)
{
    // WP.29 R156 safety gate: vehicle must be stationary
    if (currentSpeed > 0.0) {
        os << "\n" << BRED << BOLD
           << "  ╔══════════════════════════════════════════════════════════════╗\n"
           << "  ║   OTA INSTALL BLOCKED — SAFETY GATE                          ║\n"
           << "  ╠══════════════════════════════════════════════════════════════╣\n"
           << "  ║  Vehicle speed = " << std::fixed << std::setprecision(1) << currentSpeed
           << " km/h.  Must be 0 km/h to install.      ║\n"
           << "  ║  Park the vehicle and try again.                              ║\n"
           << "  ╚══════════════════════════════════════════════════════════════╝\n"
           << RESET << "\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        if (state_ != OtaState::READY || !hasActivePackage_) {
            os << BRED << "  [OTA] No verified package ready for install.\n" << RESET;
            return false;
        }
        state_ = OtaState::INSTALLING;
        logEvent(activePackage_.targetModuleId, OtaState::INSTALLING,
                 "Flash write started: " + activePackage_.packageId);
    }

    os << "\n" << BMAGENTA << BOLD
       << "  ╔══════════════════════════════════════════════════════════════╗\n"
       << "  ║   OTA INSTALL  —  WRITING TO FLASH                           ║\n"
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << "  ║  ⚠  Do NOT power off the vehicle during installation!        ║\n"
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << RESET;

    // Simulate flash write in stages
    const std::vector<std::string> stages = {
        "Erasing flash sectors ...",
        "Writing firmware image  ...",
        "Verifying written data  ...",
        "Updating boot metadata  ..."
    };

    bool flashOk = true;
    for (std::size_t i = 0; i < stages.size(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150 + randInt(0, 100)));

        // 5% chance of flash write error (simulated hardware fault)
        bool stageOk = (randDouble() > FLASH_FAIL_RATE);

        // Only inject failure on the actual write stage
        if (i == 1 && !stageOk) {
            flashOk = false;
        }
        bool ok = (i != 1) ? true : flashOk;

        double stagePct = (static_cast<double>(i + 1) / static_cast<double>(stages.size())) * 100.0;
        std::string bar  = progressBar(stagePct, 24);
        std::string line = "  " + bar + "  " + stages[i];
        int pad = 62 - static_cast<int>(line.size());

        os << BMAGENTA << BOLD << "  ║" << RESET
           << (ok ? BCYAN : BRED) << line
           << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ')
           << BMAGENTA << BOLD << "║\n" << RESET;

        if (!flashOk) break;
    }

    if (!flashOk) {
        os << BMAGENTA << BOLD
           << "  ╠══════════════════════════════════════════════════════════════╣\n"
           << "  ║" << BRED
           << "  ✘  Flash write error — rolling back to previous version      "
           << BMAGENTA << BOLD << "║\n"
           << "  ╚══════════════════════════════════════════════════════════════╝\n"
           << RESET << "\n";

        {
            std::lock_guard<std::mutex> lk(otaMutex_);
            state_ = OtaState::FAILED;
            ++stats_.failedUpdates;
            logEvent(activePackage_.targetModuleId, OtaState::FAILED,
                     "Flash write error during install");
        }
        rollback(os);
        return false;
    }

    // ── REBOOTING phase ───────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        state_ = OtaState::REBOOTING;
        logEvent(activePackage_.targetModuleId, OtaState::REBOOTING,
                 "ECU rebooting to activate new firmware");
    }

    os << BMAGENTA << BOLD
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << "  ║" << BYELLOW
       << "  ↺  ECU rebooting ...  activating " << std::left << std::setw(28)
       << activePackage_.newVersion.toString()
       << BMAGENTA << BOLD << "║\n"
       << RESET;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ── SUCCESS ───────────────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        EcuModule* mod = findModule(activePackage_.targetModuleId);
        if (mod) {
            mod->previousVersion = mod->currentVersion;
            mod->currentVersion  = activePackage_.newVersion;
            mod->updatePending   = false;
        }
        state_ = OtaState::SUCCESS;
        ++stats_.successfulUpdates;
        logEvent(activePackage_.targetModuleId, OtaState::SUCCESS,
                 "Update applied: " + activePackage_.newVersion.toString() +
                 " — rollback window open for 24h");
    }

    os << "  ║" << BGREEN
       << "  ✔  Install SUCCESS — firmware " << std::left << std::setw(32)
       << activePackage_.newVersion.toString()
       << BMAGENTA << BOLD << "║\n"
       << "  ╚══════════════════════════════════════════════════════════════╝\n"
       << RESET << "\n";

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  rollback
// ─────────────────────────────────────────────────────────────────────────────

void OtaUpdateSimulator::rollback(std::ostream& os)
{
    os << "\n" << BRED << BOLD
       << "  ╔══════════════════════════════════════════════════════════════╗\n"
       << "  ║   OTA ROLLBACK — RESTORING PREVIOUS VERSION                   ║\n"
       << "  ╠══════════════════════════════════════════════════════════════╣\n"
       << RESET;

    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        ++stats_.rollbacks;

        if (hasActivePackage_) {
            EcuModule* mod = findModule(activePackage_.targetModuleId);
            if (mod) {
                std::string prev = mod->previousVersion.toString();
                // previousVersion is already the good one; current stays unchanged
                // (we haven't actually bumped it on failure)
                mod->updatePending = false;

                std::string line = "  ↩  " + mod->id + " restored → " + prev;
                int pad = 62 - static_cast<int>(line.size());
                os << BRED << BOLD << "  ║" << RESET
                   << BYELLOW << line
                   << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ')
                   << BRED << BOLD << "║\n" << RESET;

                logEvent(mod->id, OtaState::FAILED,
                         "Rollback: keeping version " + mod->currentVersion.toString());
            }
        }

        hasActivePackage_ = false;
        state_            = OtaState::FAILED;
    }

    os << BRED << BOLD
       << "  ╚══════════════════════════════════════════════════════════════╝\n"
       << RESET << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  resetToIdle
// ─────────────────────────────────────────────────────────────────────────────

void OtaUpdateSimulator::resetToIdle()
{
    std::lock_guard<std::mutex> lk(otaMutex_);
    state_            = OtaState::IDLE;
    hasActivePackage_ = false;
    activePackage_    = OtaPackage{};
    logEvent("-", OtaState::IDLE, "State machine reset to IDLE");
}

// ─────────────────────────────────────────────────────────────────────────────
//  display
// ─────────────────────────────────────────────────────────────────────────────

void OtaUpdateSimulator::display(std::ostream& os, double currentSpeed) const
{
    constexpr int W = 78;

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
           << col << line << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ') << RESET
           << BOLD << col << "\u2551\n" << RESET;
    };
    auto kvRow = [&](const char* col, const std::string& key, const std::string& val,
                     const char* valCol) {
        std::string line = "  " + key;
        int padMid = 28 - static_cast<int>(line.size());
        if (padMid < 1) padMid = 1;
        line += std::string(static_cast<std::size_t>(padMid), ' ') + ": ";
        int padEnd = W - static_cast<int>(line.size()) - static_cast<int>(val.size());
        os << BOLD << col << "  \u2551" << RESET
           << BCYAN << line << RESET
           << valCol << BOLD << val << RESET
           << std::string(padEnd < 0 ? 0 : static_cast<std::size_t>(padEnd), ' ')
           << BOLD << col << "\u2551\n" << RESET;
    };
    auto divider = [&](const char* col) {
        os << BOLD << col << "  \u2560" << std::string(W, '-') << "\u2563\n" << RESET;
    };

    // ── Take a consistent snapshot ────────────────────────────────────────
    OtaState               snapState;
    std::vector<EcuModule> snapModules;
    OtaPackage             snapPkg;
    bool                   snapHasPkg;
    OtaStats               snapStats;
    std::vector<OtaHistoryEntry> snapHistory;

    {
        std::lock_guard<std::mutex> lk(otaMutex_);
        snapState   = state_;
        snapModules = modules_;
        snapPkg     = activePackage_;
        snapHasPkg  = hasActivePackage_;
        snapStats   = stats_;
        // Last 10 history entries newest-first
        std::size_t sz    = history_.size();
        std::size_t start = (sz > 10) ? sz - 10 : 0;
        for (std::size_t i = sz; i > start; --i)
            snapHistory.push_back(history_[i - 1]);
    }

    // ── Header: state badge ───────────────────────────────────────────────
    const char* stCol = otaStateColour(snapState);
    std::string title = "OTA UPDATE SIMULATOR  |  State: " + otaStateToString(snapState);
    hdr(title, stCol);

    // ── Speed safety status ───────────────────────────────────────────────
    std::ostringstream speedStr;
    speedStr << std::fixed << std::setprecision(1) << currentSpeed << " km/h";
    bool speedSafe = (currentSpeed <= 0.0);
    kvRow(stCol, "Vehicle Speed",
          speedStr.str() + (speedSafe ? "  (install allowed)" : "  (must park to install)"),
          speedSafe ? BGREEN : BRED);

    // ── Active package info ───────────────────────────────────────────────
    if (snapHasPkg) {
        divider(stCol);
        textRow(BCYAN, "ACTIVE PACKAGE");
        divider(stCol);
        kvRow(stCol, "Package ID",     snapPkg.packageId,                  BWHITE);
        kvRow(stCol, "Target Module",  snapPkg.targetModuleId,              BYELLOW);
        kvRow(stCol, "New Version",    snapPkg.newVersion.toString(),        BGREEN);
        kvRow(stCol, "Size",           std::to_string(snapPkg.packageSizeKB) + " kB", BWHITE);

        if (snapState == OtaState::DOWNLOADING && snapPkg.totalChunks > 0) {
            std::string bar = progressBar(snapPkg.downloadPercent(), 32);
            kvRow(stCol, "Download", bar, BCYAN);
        }
        kvRow(stCol, "Checksum",  snapPkg.checksum.substr(0, 32) + "...", BWHITE);
        kvRow(stCol, "Release Notes", snapPkg.releaseNotes,               BCYAN);
    }

    // ── ECU Module fleet table ────────────────────────────────────────────
    divider(stCol);
    textRow(BCYAN, "ECU MODULE FLEET");
    divider(stCol);

    // Table header
    textRow(BCYAN, "  ID     Name                       Current     Pending");
    divider(stCol);

    for (const auto& m : snapModules) {
        const char* rowCol = m.updatePending ? BYELLOW : BGREEN;
        std::string line;

        // ID (6 chars)
        std::string idPad = m.id; while (static_cast<int>(idPad.size()) < 6) idPad += ' ';
        // Name (28 chars)
        std::string namePad = m.name; while (static_cast<int>(namePad.size()) < 28) namePad += ' ';
        // Version (12 chars)
        std::string verPad = "v" + m.currentVersion.toString();
        while (static_cast<int>(verPad.size()) < 12) verPad += ' ';
        // Pending
        std::string pend = m.updatePending ? "UPDATE AVAIL" : "UP TO DATE  ";

        line = "  " + idPad + namePad + verPad + pend;
        int pad = W - static_cast<int>(line.size());
        os << BOLD << stCol << "  \u2551" << RESET
           << rowCol << line
           << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ') << RESET
           << BOLD << stCol << "\u2551\n" << RESET;
    }

    // ── Statistics ────────────────────────────────────────────────────────
    divider(stCol);
    textRow(BCYAN, "OTA STATISTICS");
    divider(stCol);
    kvRow(stCol, "Server Polls",       std::to_string(snapStats.totalChecks),       BWHITE);
    kvRow(stCol, "Packages Found",     std::to_string(snapStats.packagesFound),     BWHITE);
    kvRow(stCol, "Successful Updates", std::to_string(snapStats.successfulUpdates), BGREEN);
    kvRow(stCol, "Failed Updates",     std::to_string(snapStats.failedUpdates),
          snapStats.failedUpdates > 0 ? BRED : BGREEN);
    kvRow(stCol, "Rollbacks",          std::to_string(snapStats.rollbacks),
          snapStats.rollbacks > 0 ? BYELLOW : BGREEN);
    kvRow(stCol, "Total Downloaded",   std::to_string(snapStats.totalDownloadKB) + " kB", BWHITE);

    // ── Audit trail ───────────────────────────────────────────────────────
    if (!snapHistory.empty()) {
        divider(stCol);
        textRow(BCYAN, "AUDIT TRAIL  (last 10 events, newest first)");
        divider(stCol);
        for (const auto& e : snapHistory) {
            const char* eCol = otaStateColour(e.state);
            std::string line = "  [" + e.timestamp + "]  ";
            std::string modPad = "[" + e.moduleId + "] ";
            while (static_cast<int>(modPad.size()) < 7) modPad += ' ';
            std::string statePad = otaStateToString(e.state);
            while (static_cast<int>(statePad.size()) < 12) statePad += ' ';
            line += modPad + statePad + "  " + e.detail;
            if (static_cast<int>(line.size()) > W - 2)
                line = line.substr(0, static_cast<std::size_t>(W - 5)) + "...";
            int pad = W - static_cast<int>(line.size());
            os << BOLD << stCol << "  \u2551" << RESET
               << eCol << line
               << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ') << RESET
               << BOLD << stCol << "\u2551\n" << RESET;
        }
    }

    foot(stCol);
    os << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  runInteractiveMenu
// ─────────────────────────────────────────────────────────────────────────────

void OtaUpdateSimulator::runInteractiveMenu(std::ostream& os, double currentSpeed)
{
    while (true) {
        display(os, currentSpeed);

        OtaState st = currentState();

        // Build action menu based on current state
        os << BCYAN << BOLD
           << "  ╔══════════════════════════════════════════════════════════════╗\n"
           << "  ║   OTA ACTIONS                                                ║\n"
           << "  ╠══════════════════════════════════════════════════════════════╣\n"
           << RESET;

        auto menuLine = [&](const std::string& opt, bool enabled = true) {
            const char* col = enabled ? BGREEN : BCYAN;
            std::string line = "   " + opt;
            int pad = 64 - static_cast<int>(line.size());
            os << BCYAN << BOLD << "  ║" << RESET
               << col << line
               << std::string(pad < 0 ? 0 : static_cast<std::size_t>(pad), ' ') << RESET
               << BCYAN << BOLD << "║\n" << RESET;
        };

        menuLine("[1]  Check for updates",    st == OtaState::IDLE || st == OtaState::SUCCESS || st == OtaState::FAILED);
        menuLine("[2]  Start download",       st == OtaState::CHECKING);
        menuLine("[3]  Verify package",       st == OtaState::DOWNLOADING);
        menuLine("[4]  Install update",       st == OtaState::READY);
        menuLine("[5]  Manual rollback",      st == OtaState::SUCCESS || st == OtaState::READY);
        menuLine("[6]  Reset to IDLE",        true);
        menuLine("[7]  Back to main menu",    true);

        os << BCYAN << BOLD
           << "  ╚══════════════════════════════════════════════════════════════╝\n"
           << RESET
           << BWHITE << BOLD << "  ▶  Enter Choice [1-7]: " << RESET;

        int choice = 0;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            os << BRED << "\n  [!] Invalid input.\n" << RESET;
            continue;
        }

        switch (choice) {
        case 1: {
            int found = checkForUpdates();
            if (found == 0) {
                os << "\n" << BCYAN << BOLD
                   << "  ✔  No updates available at this time.\n" << RESET;
            } else {
                os << "\n" << BGREEN << BOLD
                   << "  ✔  " << found << " package(s) found — select [2] to download.\n"
                   << RESET;
            }
            break;
        }
        case 2:
            startDownload(os);
            break;
        case 3:
            verifyPackage(os);
            break;
        case 4:
            installUpdate(currentSpeed, os);
            break;
        case 5:
            rollback(os);
            break;
        case 6:
            resetToIdle();
            os << "\n" << BCYAN << BOLD << "  ✔  State reset to IDLE.\n" << RESET;
            break;
        case 7:
            return;
        default:
            os << BRED << "\n  [!] Invalid choice — enter 1 to 7.\n" << RESET;
        }

        // Pause after each action for readability
        os << "\n" << BWHITE << BOLD << "  Press [Enter] to continue...\n" << RESET;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
    }
}
