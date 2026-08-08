#include "eval_common.h"

#include "uavloc/util/scoped_timer.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>

namespace uavloc {
namespace eval {

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(v.begin(), v.end());
    std::size_t rank =
        static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())));
    if (rank == 0) rank = 1;
    return v[std::min(rank, v.size()) - 1];
}

double least_squares_slope(const std::vector<double>& x, const std::vector<double>& y) {
    const std::size_t n = std::min(x.size(), y.size());
    if (n < 2) return std::numeric_limits<double>::quiet_NaN();
    double mx = 0.0, my = 0.0;
    for (std::size_t i = 0; i < n; ++i) { mx += x[i]; my += y[i]; }
    mx /= static_cast<double>(n);
    my /= static_cast<double>(n);
    double sxy = 0.0, sxx = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        sxy += (x[i] - mx) * (y[i] - my);
        sxx += (x[i] - mx) * (x[i] - mx);
    }
    if (sxx <= 0.0) return std::numeric_limits<double>::quiet_NaN();
    return sxy / sxx;
}

double aligned_rms_2d(const std::vector<Eigen::Vector2d>& src,
                      const std::vector<Eigen::Vector2d>& dst,
                      double& yaw_deg_out) {
    const std::size_t n = src.size();
    if (n < 2) {
        yaw_deg_out = std::numeric_limits<double>::quiet_NaN();
        return std::numeric_limits<double>::quiet_NaN();
    }
    Eigen::Vector2d s_mean = Eigen::Vector2d::Zero();
    Eigen::Vector2d d_mean = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < n; ++i) { s_mean += src[i]; d_mean += dst[i]; }
    s_mean /= static_cast<double>(n);
    d_mean /= static_cast<double>(n);

    Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
    for (std::size_t i = 0; i < n; ++i)
        C += (dst[i] - d_mean) * (src[i] - s_mean).transpose();

    const double yaw = std::atan2(C(1, 0) - C(0, 1), C(0, 0) + C(1, 1));
    yaw_deg_out = yaw * 180.0 / M_PI;
    Eigen::Matrix2d R;
    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw),  std::cos(yaw);
    const Eigen::Vector2d t = d_mean - R * s_mean;

    double sq_sum = 0.0;
    for (std::size_t i = 0; i < n; ++i)
        sq_sum += (R * src[i] + t - dst[i]).squaredNorm();
    return std::sqrt(sq_sum / static_cast<double>(n));
}

std::string profile_csv_path(const std::string& out_csv) {
    const std::size_t slash = out_csv.find_last_of('/');
    const std::string dir  = (slash == std::string::npos)
                                 ? std::string()
                                 : out_csv.substr(0, slash + 1);
    std::string       stem = (slash == std::string::npos)
                                 ? out_csv
                                 : out_csv.substr(slash + 1);
    const std::size_t dot  = stem.find_last_of('.');
    if (dot != std::string::npos && dot > 0) {
        stem = stem.substr(0, dot);
    }
    return dir + "profile_" + stem + ".csv";
}

void dump_profile(const std::string& path) {
    using uavloc::util::ProfileStage;
    using uavloc::util::Profiler;
    using uavloc::util::StageStat;
    constexpr std::size_t NUM_STAGES = static_cast<std::size_t>(ProfileStage::COUNT);

    const auto by_thread = Profiler::snapshot_by_thread();
    if (by_thread.empty()) {
        spdlog::warn("test_full_flight: profiling enabled but no samples recorded");
        return;
    }

    std::ofstream pcsv(path);
    if (!pcsv.is_open()) {
        spdlog::error("test_full_flight: cannot open profile CSV '{}'", path);
        return;
    }
    pcsv << "thread,stage,count,total_ms,mean_ms,max_ms,percent_of_thread\n";
    for (const auto& entry : by_thread) {
        // Denominator: the thread's ROOT stages only (their durations are
        // disjoint), so nested stages never inflate the reference total.
        double thread_total_ms = 0.0;
        for (std::size_t i = 0; i < NUM_STAGES; ++i) {
            const auto stage = static_cast<ProfileStage>(i);
            if (uavloc::util::parent_stage(stage) == stage) {
                thread_total_ms += entry.second[i].total_ms;
            }
        }
        for (std::size_t i = 0; i < NUM_STAGES; ++i) {
            const StageStat& s = entry.second[i];
            if (s.count == 0) continue;
            const double mean = s.total_ms / static_cast<double>(s.count);
            const double pct  = (thread_total_ms > 0.0)
                                    ? 100.0 * s.total_ms / thread_total_ms : 0.0;
            pcsv << entry.first << ','
                 << uavloc::util::to_string(static_cast<ProfileStage>(i)) << ','
                 << s.count << ',' << s.total_ms << ',' << mean << ','
                 << s.max_ms << ',' << pct << '\n';
        }
    }
    pcsv.close();

    // Merged summary, sorted by total_ms (descending).
    const auto total = Profiler::snapshot_total();
    const double frame_total_ms =
        total[static_cast<std::size_t>(ProfileStage::PROCESS_FRAME_TOTAL)].total_ms;

    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < NUM_STAGES; ++i) {
        if (total[i].count > 0) order.push_back(i);
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) {
                  return total[a].total_ms > total[b].total_ms;
              });

    spdlog::info("==== profiling summary (sorted by total_ms) ====");
    spdlog::info("  {:<22} {:>8} {:>12} {:>10} {:>10} {:>7}",
                 "stage", "calls", "total_ms", "mean_ms", "max_ms", "%frame");
    for (std::size_t i : order) {
        const StageStat& s = total[i];
        const double mean = s.total_ms / static_cast<double>(s.count);
        const double pct  = (frame_total_ms > 0.0)
                                ? 100.0 * s.total_ms / frame_total_ms : 0.0;
        spdlog::info("  {:<22} {:>8} {:>12.1f} {:>10.3f} {:>10.3f} {:>7.1f}",
                     uavloc::util::to_string(static_cast<ProfileStage>(i)),
                     s.count, s.total_ms, mean, s.max_ms, pct);
    }

    // Unaccounted: the part of PROCESS_FRAME_TOTAL not covered by its level-1
    // children (glue code + profiling overhead itself).
    double children_ms = 0.0;
    for (std::size_t i = 0; i < NUM_STAGES; ++i) {
        const auto stage = static_cast<ProfileStage>(i);
        if (stage != ProfileStage::PROCESS_FRAME_TOTAL &&
            uavloc::util::parent_stage(stage) == ProfileStage::PROCESS_FRAME_TOTAL) {
            children_ms += total[i].total_ms;
        }
    }
    const double unaccounted_ms  = frame_total_ms - children_ms;
    const double unaccounted_pct = (frame_total_ms > 0.0)
                                       ? 100.0 * unaccounted_ms / frame_total_ms : 0.0;
    spdlog::info("  {:<22} {:>8} {:>12.1f} {:>10} {:>10} {:>7.1f}",
                 "unaccounted", "-", unaccounted_ms, "-", "-", unaccounted_pct);
    spdlog::info("  profile csv       = '{}'", path);
    spdlog::info("===============================================");
}

} // namespace eval
} // namespace uavloc
