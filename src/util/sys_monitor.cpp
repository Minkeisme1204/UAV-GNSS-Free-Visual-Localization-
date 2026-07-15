#include "uavloc/util/sys_monitor.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <unistd.h>  // sysconf(_SC_CLK_TCK)

namespace uavloc {
namespace util {

bool SysMonitor::read_cpu_ticks(long& ticks_out) {
    std::ifstream stat_file("/proc/self/stat");
    if (!stat_file.is_open()) return false;
    std::string line;
    std::getline(stat_file, line);
    // Field 2 (comm) is parenthesised and may contain spaces — resume the
    // whitespace split after the LAST ')'. utime/stime are then the 12th/13th
    // tokens (stat fields 14/15, 1-based).
    const std::size_t close = line.rfind(')');
    if (close == std::string::npos) return false;
    std::istringstream rest(line.substr(close + 1));
    std::string tok;
    long utime = 0, stime = 0;
    for (int i = 1; rest >> tok; ++i) {
        if (i == 12) utime = std::atol(tok.c_str());
        if (i == 13) { stime = std::atol(tok.c_str()); break; }
    }
    ticks_out = utime + stime;
    return true;
}

bool SysMonitor::read_rss_mb(float& rss_mb_out) {
    std::ifstream status_file("/proc/self/status");
    if (!status_file.is_open()) return false;
    std::string line;
    while (std::getline(status_file, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream fields(line.substr(6));
            long kb = 0;
            fields >> kb;
            rss_mb_out = static_cast<float>(kb) / 1024.0f;
            return true;
        }
    }
    return false;
}

SysMonitor::SysMonitor() { reset(); }

void SysMonitor::reset() {
    t0_        = std::chrono::steady_clock::now();
    last_wall_ = t0_;
    last_ticks_ = 0;
    read_cpu_ticks(last_ticks_);  // best-effort; sample() re-checks readability
}

bool SysMonitor::sample(SysStats& out) {
    const long ticks_per_sec = sysconf(_SC_CLK_TCK);
    if (ticks_per_sec <= 0) return false;

    const auto now = std::chrono::steady_clock::now();
    long  ticks  = 0;
    float rss_mb = 0.0f;
    if (!read_cpu_ticks(ticks) || !read_rss_mb(rss_mb)) {
        return false;
    }
    const double wall_sec =
        std::chrono::duration<double>(now - last_wall_).count();
    if (wall_sec <= 0.0) {
        return false;
    }

    out.t_sec = std::chrono::duration<double>(now - t0_).count();
    out.cpu_percent = static_cast<float>(
        100.0 * static_cast<double>(ticks - last_ticks_) /
        static_cast<double>(ticks_per_sec) / wall_sec);
    out.rss_mb = rss_mb;

    last_wall_  = now;
    last_ticks_ = ticks;
    return true;
}

}  // namespace util
}  // namespace uavloc
