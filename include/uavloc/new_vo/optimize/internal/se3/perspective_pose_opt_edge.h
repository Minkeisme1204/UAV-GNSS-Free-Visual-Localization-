#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/optimize/internal/landmark_vertex.h"
#include "uavloc/new_vo/optimize/internal/se3/shot_vertex.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

namespace uavloc {
namespace vo {
namespace optimize {
namespace internal {
namespace se3 {

class MonoPerspectivePoseOptEdge final : public g2o::BaseUnaryEdge<2, Vec2_t, ShotVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MonoPerspectivePoseOptEdge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;

    bool depth_is_positive() const;

    Vec2_t cam_project(const Vec3_t& pos_c) const;

    Vec3_t pos_w_;
    double fx_, fy_, cx_, cy_;
};

inline MonoPerspectivePoseOptEdge::MonoPerspectivePoseOptEdge()
    : g2o::BaseUnaryEdge<2, Vec2_t, ShotVertex>() {}

inline bool MonoPerspectivePoseOptEdge::read(std::istream& is) {
    for (unsigned int i = 0; i < 2; ++i) {
        is >> _measurement(i);
    }
    for (int i = 0; i < information().rows(); ++i) {
        for (int j = i; j < information().cols(); ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

inline bool MonoPerspectivePoseOptEdge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 2; ++i) {
        os << measurement()(i) << " ";
    }
    for (int i = 0; i < information().rows(); ++i) {
        for (int j = i; j < information().cols(); ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

inline void MonoPerspectivePoseOptEdge::computeError() {
    const auto v1 = static_cast<const ShotVertex*>(_vertices.at(0));
    const Vec2_t obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(pos_w_));
}

inline void MonoPerspectivePoseOptEdge::linearizeOplus() {
    auto vi = static_cast<ShotVertex*>(_vertices.at(0));
    const g2o::SE3Quat& cam_pose_cw = vi->ShotVertex::estimate();
    const Vec3_t pos_c = cam_pose_cw.map(pos_w_);

    const auto x = pos_c(0);
    const auto y = pos_c(1);
    const auto z = pos_c(2);
    const auto z_sq = z * z;

    _jacobianOplusXi(0, 0) = x * y / z_sq * fx_;
    _jacobianOplusXi(0, 1) = -(1.0 + (x * x / z_sq)) * fx_;
    _jacobianOplusXi(0, 2) = y / z * fx_;
    _jacobianOplusXi(0, 3) = -1.0 / z * fx_;
    _jacobianOplusXi(0, 4) = 0;
    _jacobianOplusXi(0, 5) = x / z_sq * fx_;

    _jacobianOplusXi(1, 0) = (1.0 + y * y / z_sq) * fy_;
    _jacobianOplusXi(1, 1) = -x * y / z_sq * fy_;
    _jacobianOplusXi(1, 2) = -x / z * fy_;
    _jacobianOplusXi(1, 3) = 0.0;
    _jacobianOplusXi(1, 4) = -1.0 / z * fy_;
    _jacobianOplusXi(1, 5) = y / z_sq * fy_;
}

inline bool MonoPerspectivePoseOptEdge::depth_is_positive() const {
    const auto v1 = static_cast<const ShotVertex*>(_vertices.at(0));
    return 0 < (v1->estimate().map(pos_w_))(2);
}

inline Vec2_t MonoPerspectivePoseOptEdge::cam_project(const Vec3_t& pos_c) const {
    return {fx_ * pos_c(0) / pos_c(2) + cx_, fy_ * pos_c(1) / pos_c(2) + cy_};
}

class StereoPerspectivePoseOptEdge : public g2o::BaseUnaryEdge<3, Vec3_t, ShotVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoPerspectivePoseOptEdge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;

    bool depth_is_positive() const;

    Vec3_t cam_project(const Vec3_t& pos_c) const;

    Vec3_t pos_w_;
    double fx_, fy_, cx_, cy_, focal_x_baseline_;
};

inline StereoPerspectivePoseOptEdge::StereoPerspectivePoseOptEdge()
    : g2o::BaseUnaryEdge<3, Vec3_t, ShotVertex>() {}

inline bool StereoPerspectivePoseOptEdge::read(std::istream& is) {
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _measurement(i);
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

inline bool StereoPerspectivePoseOptEdge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 3; ++i) {
        os << measurement()(i) << " ";
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

inline void StereoPerspectivePoseOptEdge::computeError() {
    const auto v1 = static_cast<const ShotVertex*>(_vertices.at(0));
    const Vec3_t obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(pos_w_));
}

inline void StereoPerspectivePoseOptEdge::linearizeOplus() {
    auto vi = static_cast<ShotVertex*>(_vertices.at(0));
    const g2o::SE3Quat& cam_pose_cw = vi->ShotVertex::estimate();
    const Vec3_t pos_c = cam_pose_cw.map(pos_w_);

    const auto x = pos_c(0);
    const auto y = pos_c(1);
    const auto z = pos_c(2);
    const auto z_sq = z * z;

    _jacobianOplusXi(0, 0) = x * y / z_sq * fx_;
    _jacobianOplusXi(0, 1) = -(1.0 + (x * x / z_sq)) * fx_;
    _jacobianOplusXi(0, 2) = y / z * fx_;
    _jacobianOplusXi(0, 3) = -1.0 / z * fx_;
    _jacobianOplusXi(0, 4) = 0.0;
    _jacobianOplusXi(0, 5) = x / z_sq * fx_;

    _jacobianOplusXi(1, 0) = (1.0 + y * y / z_sq) * fy_;
    _jacobianOplusXi(1, 1) = -x * y / z_sq * fy_;
    _jacobianOplusXi(1, 2) = -x / z * fy_;
    _jacobianOplusXi(1, 3) = 0.0;
    _jacobianOplusXi(1, 4) = -1.0 / z * fy_;
    _jacobianOplusXi(1, 5) = y / z_sq * fy_;

    _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - focal_x_baseline_ * y / z_sq;
    _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) + focal_x_baseline_ * x / z_sq;
    _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2);
    _jacobianOplusXi(2, 3) = _jacobianOplusXi(0, 3);
    _jacobianOplusXi(2, 4) = 0.0;
    _jacobianOplusXi(2, 5) = _jacobianOplusXi(0, 5) - focal_x_baseline_ / z_sq;
}

inline bool StereoPerspectivePoseOptEdge::depth_is_positive() const {
    const auto v1 = static_cast<const ShotVertex*>(_vertices.at(0));
    return 0 < (v1->estimate().map(pos_w_))(2);
}

inline Vec3_t StereoPerspectivePoseOptEdge::cam_project(const Vec3_t& pos_c) const {
    const double reproj_x = fx_ * pos_c(0) / pos_c(2) + cx_;
    return {reproj_x, fy_ * pos_c(1) / pos_c(2) + cy_, reproj_x - focal_x_baseline_ / pos_c(2)};
}

} // namespace se3
} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
