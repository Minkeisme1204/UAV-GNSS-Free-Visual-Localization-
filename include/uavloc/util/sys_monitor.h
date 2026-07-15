#pragma once

#include <chrono>

namespace uavloc {
namespace util {

// One process-performance sample produced by SysMonitor::sample().
struct SysStats {
    double t_sec       = 0.0;   // seconds since the monitor was constructed/reset
    float  cpu_percent = 0.0f;  // 100 * delta(utime+stime)/CLK_TCK / delta(wall);
                                // may exceed 100 when the process runs multiple threads
    float  rss_mb      = 0.0f;  // resident set size (MB)
};

// /proc-based process performance monitor (plain POSIX file reads, Linux only).
// Keeps the previous CPU-tick / wall-clock reference so each sample() call yields
// the average CPU utilisation since the previous call.
class SysMonitor {
public:
    SysMonitor();  // captures t0 + the initial CPU tick count

    // Re-anchors t0 and the CPU tick / wall-clock reference.
    void reset();

    // Takes one sample (delta vs the previous sample/reset). Returns false —
    // leaving `out` untouched and the internal state unchanged — when /proc
    // cannot be read or no wall time has elapsed.
    bool sample(SysStats& out);

    // Raw readers, usable standalone:
    // Total CPU time (utime + stime) of this process in clock ticks, from
    // /proc/self/stat. Returns false when the file cannot be parsed.
    static bool read_cpu_ticks(long& ticks_out);
    // Resident set size (MB) of this process, from the VmRSS line (kB) of
    // /proc/self/status. Returns false when the line is absent.
    static bool read_rss_mb(float& rss_mb_out);

private:
    long last_ticks_ = 0;
    std::chrono::steady_clock::time_point t0_;
    std::chrono::steady_clock::time_point last_wall_;
};

}  // namespace util
}  // namespace uavloc
