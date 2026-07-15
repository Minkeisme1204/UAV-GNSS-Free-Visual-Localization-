#pragma once

#include "uavloc/common/type.h"

#include <g2o/core/base_vertex.h>
#include <g2o/types/slam3d/se3quat.h>

namespace uavloc {
namespace vo {
namespace optimize {
namespace internal {
namespace se3 {

class ShotVertex final : public g2o::BaseVertex<6, g2o::SE3Quat> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ShotVertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update_) override;
};

inline ShotVertex::ShotVertex()
    : g2o::BaseVertex<6, g2o::SE3Quat>() {}

inline bool ShotVertex::ShotVertex::read(std::istream& is) {
    Vec7_t estimate;
    for (unsigned int i = 0; i < 7; ++i) {
        is >> estimate(i);
    }
    g2o::SE3Quat g2o_cam_pose_wc;
    g2o_cam_pose_wc.fromVector(estimate);
    setEstimate(g2o_cam_pose_wc.inverse());
    return true;
}

inline bool ShotVertex::ShotVertex::write(std::ostream& os) const {
    g2o::SE3Quat g2o_cam_pose_wc(estimate().inverse());
    for (unsigned int i = 0; i < 7; ++i) {
        os << g2o_cam_pose_wc[i] << " ";
    }
    return os.good();
}

inline void ShotVertex::setToOriginImpl() {
    _estimate = g2o::SE3Quat();
}

inline void ShotVertex::oplusImpl(const double* update_) {
    Eigen::Map<const Vec6_t> update(update_);
    setEstimate(g2o::SE3Quat::exp(update) * estimate());
}

} // namespace se3
} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
