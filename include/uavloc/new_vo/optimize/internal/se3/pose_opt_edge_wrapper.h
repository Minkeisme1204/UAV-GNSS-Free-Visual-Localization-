#pragma once

#include "uavloc/new_vo/camera/perspective_camera.h"
#include "uavloc/new_vo/optimize/internal/se3/perspective_pose_opt_edge.h"

#include <g2o/core/robust_kernel_impl.h>

namespace uavloc {
namespace vo {

namespace data {
class Landmark;
} // namespace data

namespace camera {
class Base;
} // namespace camera

namespace optimize {
namespace internal {
namespace se3 {

class PoseOptEdgeWrapper {
public:
    PoseOptEdgeWrapper() = delete;

    PoseOptEdgeWrapper(const camera::Base* camera, ShotVertex* shot_vtx, const Vec3_t& pos_w,
                          const unsigned int idx, const float obs_x, const float obs_y, const float obs_x_right,
                          const float inv_sigma_sq, const float sqrt_chi_sq);

    virtual ~PoseOptEdgeWrapper() = default;

    bool is_inlier() const;

    bool is_outlier() const;

    void set_as_inlier() const;

    void set_as_outlier() const;

    bool depth_is_positive() const;

    g2o::OptimizableGraph::Edge* edge_;

    const camera::Base* camera_;
    const unsigned int idx_;
    const bool is_monocular_;
};

inline PoseOptEdgeWrapper::PoseOptEdgeWrapper(const camera::Base* camera, ShotVertex* shot_vtx, const Vec3_t& pos_w,
                                                    const unsigned int idx, const float obs_x, const float obs_y, const float obs_x_right,
                                                    const float inv_sigma_sq, const float sqrt_chi_sq)
    : camera_(camera), idx_(idx), is_monocular_(obs_x_right < 0) {
    // Perspective is the only camera model (see camera::PerspectiveCamera).
    {
        const auto c = static_cast<const camera::PerspectiveCamera*>(camera_);
        if (is_monocular_) {
            auto edge = new MonoPerspectivePoseOptEdge();

            const Vec2_t obs{obs_x, obs_y};
            edge->setMeasurement(obs);
            edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);

            edge->fx_ = c->fx_;
            edge->fy_ = c->fy_;
            edge->cx_ = c->cx_;
            edge->cy_ = c->cy_;

            edge->pos_w_ = pos_w;

            edge->setVertex(0, shot_vtx);

            edge_ = edge;
        }
        else {
            auto edge = new StereoPerspectivePoseOptEdge();

            const Vec3_t obs{obs_x, obs_y, obs_x_right};
            edge->setMeasurement(obs);
            edge->setInformation(Mat33_t::Identity() * inv_sigma_sq);

            edge->fx_ = c->fx_;
            edge->fy_ = c->fy_;
            edge->cx_ = c->cx_;
            edge->cy_ = c->cy_;
            edge->focal_x_baseline_ = camera_->focal_x_baseline_;

            edge->pos_w_ = pos_w;

            edge->setVertex(0, shot_vtx);

            edge_ = edge;
        }
    }

    // loss functionを設定
    auto huber_kernel = new g2o::RobustKernelHuber();
    huber_kernel->setDelta(sqrt_chi_sq);
    edge_->setRobustKernel(huber_kernel);
}

inline bool PoseOptEdgeWrapper::is_inlier() const {
    return edge_->level() == 0;
}

inline bool PoseOptEdgeWrapper::is_outlier() const {
    return edge_->level() != 0;
}

inline void PoseOptEdgeWrapper::set_as_inlier() const {
    edge_->setLevel(0);
}

inline void PoseOptEdgeWrapper::set_as_outlier() const {
    edge_->setLevel(1);
}

inline bool PoseOptEdgeWrapper::depth_is_positive() const {
    if (is_monocular_) {
        return static_cast<MonoPerspectivePoseOptEdge*>(edge_)->MonoPerspectivePoseOptEdge::depth_is_positive();
    }
    return static_cast<StereoPerspectivePoseOptEdge*>(edge_)->StereoPerspectivePoseOptEdge::depth_is_positive();
}

} // namespace se3
} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
