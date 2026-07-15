#pragma once

#include "uavloc/new_vo/camera/perspective_camera.h"
#include "uavloc/new_vo/optimize/internal/se3/perspective_reproj_edge.h"

#include <g2o/core/robust_kernel_impl.h>

#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Landmark;
} // namespace data

namespace optimize {
namespace internal {
namespace se3 {

template<typename T>
class ReprojEdgeWrapper {
public:
    ReprojEdgeWrapper() = delete;

    ReprojEdgeWrapper(const std::shared_ptr<T>& shot, ShotVertex* shot_vtx,
                        const std::shared_ptr<data::Landmark>& lm, LandmarkVertex* lm_vtx,
                        const unsigned int idx, const float obs_x, const float obs_y, const float obs_x_right,
                        const float inv_sigma_sq, const float sqrt_chi_sq, const bool use_huber_loss = true);

    virtual ~ReprojEdgeWrapper() = default;

    bool is_inlier() const;

    bool is_outlier() const;

    void set_as_inlier() const;

    void set_as_outlier() const;

    bool depth_is_positive() const;

    g2o::OptimizableGraph::Edge* edge_;

    camera::Base* camera_;
    std::shared_ptr<T> shot_;
    std::shared_ptr<data::Landmark> lm_;
    const unsigned int idx_;
    const bool is_monocular_;
};

template<typename T>
inline ReprojEdgeWrapper<T>::ReprojEdgeWrapper(const std::shared_ptr<T>& shot, ShotVertex* shot_vtx,
                                                   const std::shared_ptr<data::Landmark>& lm, LandmarkVertex* lm_vtx,
                                                   const unsigned int idx, const float obs_x, const float obs_y, const float obs_x_right,
                                                   const float inv_sigma_sq, const float sqrt_chi_sq, const bool use_huber_loss)
    : camera_(shot->camera_), shot_(shot), lm_(lm), idx_(idx), is_monocular_(obs_x_right < 0) {
    // Perspective is the only camera model (see camera::PerspectiveCamera).
    {
        auto c = static_cast<camera::PerspectiveCamera*>(camera_);
        if (is_monocular_) {
            auto edge = new MonoPerspectiveReprojEdge();

            const Vec2_t obs{obs_x, obs_y};
            edge->setMeasurement(obs);
            edge->setInformation(Mat22_t::Identity() * inv_sigma_sq);

            edge->fx_ = c->fx_;
            edge->fy_ = c->fy_;
            edge->cx_ = c->cx_;
            edge->cy_ = c->cy_;

            edge->setVertex(0, lm_vtx);
            edge->setVertex(1, shot_vtx);

            edge_ = edge;
        }
        else {
            auto edge = new StereoPerspectiveReprojEdge();

            const Vec3_t obs{obs_x, obs_y, obs_x_right};
            edge->setMeasurement(obs);
            edge->setInformation(Mat33_t::Identity() * inv_sigma_sq);

            edge->fx_ = c->fx_;
            edge->fy_ = c->fy_;
            edge->cx_ = c->cx_;
            edge->cy_ = c->cy_;
            edge->focal_x_baseline_ = camera_->focal_x_baseline_;

            edge->setVertex(0, lm_vtx);
            edge->setVertex(1, shot_vtx);

            edge_ = edge;
        }
    }

    // loss functionを設定
    if (use_huber_loss) {
        auto huber_kernel = new g2o::RobustKernelHuber();
        huber_kernel->setDelta(sqrt_chi_sq);
        edge_->setRobustKernel(huber_kernel);
    }
}

template<typename T>
inline bool ReprojEdgeWrapper<T>::is_inlier() const {
    return edge_->level() == 0;
}

template<typename T>
inline bool ReprojEdgeWrapper<T>::is_outlier() const {
    return edge_->level() != 0;
}

template<typename T>
inline void ReprojEdgeWrapper<T>::set_as_inlier() const {
    edge_->setLevel(0);
}

template<typename T>
inline void ReprojEdgeWrapper<T>::set_as_outlier() const {
    edge_->setLevel(1);
}

template<typename T>
inline bool ReprojEdgeWrapper<T>::depth_is_positive() const {
    if (is_monocular_) {
        return static_cast<MonoPerspectiveReprojEdge*>(edge_)->MonoPerspectiveReprojEdge::depth_is_positive();
    }
    return static_cast<StereoPerspectiveReprojEdge*>(edge_)->StereoPerspectiveReprojEdge::depth_is_positive();
}

} // namespace se3
} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
