#pragma once

// eval_common — the scoring ruler shared by the evaluation drivers (S8).
//
// Every number a driver reports about accuracy or runtime has to be produced
// by the SAME code, or two reports cannot be compared. Before S8 the stage
// profiler dump and the scoring maths lived inside tests/test_full_flight.cpp,
// which meant any second driver would have re-implemented them — and a
// percentile that switches from nearest-rank to interpolation silently
// invalidates every p95 already published.
//
// ⚠ This file is a MECHANICAL extraction of those helpers. The bodies are
// byte-for-byte the ones test_full_flight.cpp carried, deliberately including
// the parts that could be written better (see the S8 report); changing any of
// them breaks comparability with the stored baselines.
//
// It lives under tests/ rather than in include/uavloc/: these are evaluation
// helpers for the drivers, not product API, and putting them in a module would
// make them installed public headers of libuavloc.

#include <Eigen/Core>

#include <string>
#include <vector>

namespace uavloc {
namespace eval {

//! Nearest-rank percentile of a sample (copy taken on purpose: sorts locally).
double percentile(std::vector<double> v, double p);

//! Least-squares slope of y on x (G1a: does the error still grow with the
//! distance flown?). NaN when the sample is degenerate.
double least_squares_slope(const std::vector<double>& x, const std::vector<double>& y);

//! 4-DoF (yaw + translation) least-squares alignment of 2-D source points onto
//! 2-D destination points: closed-form optimal yaw from the 2x2 correlation,
//! then the centroid-matching translation. Returns the aligned RMS.
double aligned_rms_2d(const std::vector<Eigen::Vector2d>& src,
                      const std::vector<Eigen::Vector2d>& dst,
                      double& yaw_deg_out);

//! "<dir>/profile_<stem>.csv" for an output CSV path "<dir>/<stem>.csv".
std::string profile_csv_path(const std::string& out_csv);

//! Write the per-thread stage table to `path` and log a merged summary sorted
//! by total_ms, plus the "unaccounted" residual of the per-frame total.
void dump_profile(const std::string& path);

} // namespace eval
} // namespace uavloc
