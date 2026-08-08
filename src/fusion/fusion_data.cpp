#include "uavloc/fusion/fusion_data.h"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <limits>

namespace uavloc::fusion {

double horizontal_accuracy_m(const Eigen::Matrix2d& xy_covariance,
                             bool                   covariance_valid) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!covariance_valid || !xy_covariance.allFinite()) {
        return nan;
    }
    // Largest semi-axis of the 1-sigma ellipse = sqrt of the largest
    // eigenvalue of the (symmetric) 2x2 covariance.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(xy_covariance);
    if (eig.info() != Eigen::Success) {
        return nan;
    }
    const double lambda_max = eig.eigenvalues().maxCoeff();
    if (!(lambda_max > 0.0)) {
        // A covariance must be positive definite; a non-positive eigenvalue
        // means the figure is unusable. Reporting sqrt(0) = 0 here would claim
        // perfect certainty — see the header warning.
        return nan;
    }
    return std::sqrt(lambda_max);
}

double horizontal_accuracy_m(const FusionResult& result) {
    return horizontal_accuracy_m(result.xy_covariance, result.covariance_valid);
}

FusionHealth next_fusion_health(FusionHealth current,
                                double       sigma_xy_m,
                                bool         covariance_valid,
                                double       converged_sigma_m,
                                double       drifting_sigma_m) {
    // No usable self-assessment ⇒ the estimator is (back) in start-up: it
    // cannot claim to be converged on evidence it does not have.
    if (!covariance_valid || !std::isfinite(sigma_xy_m)) {
        return FusionHealth::INITIALIZING;
    }
    switch (current) {
        case FusionHealth::INITIALIZING:
            return sigma_xy_m < converged_sigma_m ? FusionHealth::CONVERGED
                                                  : FusionHealth::INITIALIZING;
        case FusionHealth::CONVERGED:
            return sigma_xy_m > drifting_sigma_m ? FusionHealth::DRIFTING
                                                 : FusionHealth::CONVERGED;
        case FusionHealth::DRIFTING:
            // Deliberately the CONVERGED threshold: recovery must cross the
            // whole hysteresis band, so a sigma sitting between the two
            // thresholds keeps the current state instead of oscillating.
            return sigma_xy_m < converged_sigma_m ? FusionHealth::CONVERGED
                                                  : FusionHealth::DRIFTING;
    }
    return current;
}

} // namespace uavloc::fusion
