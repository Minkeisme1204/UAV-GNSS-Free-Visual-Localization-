#pragma once

#include "uavloc/common/type.h"

#include <g2o/core/base_vertex.h>

namespace uavloc {
namespace vo {
namespace optimize {
namespace internal {

class LandmarkVertex final : public g2o::BaseVertex<3, Vec3_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LandmarkVertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update) override;
};

inline LandmarkVertex::LandmarkVertex()
    : g2o::BaseVertex<3, Vec3_t>() {}

inline bool LandmarkVertex::read(std::istream& is) {
    Vec3_t lv;
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _estimate(i);
    }
    return true;
}

inline bool LandmarkVertex::write(std::ostream& os) const {
    const Vec3_t pos_w = estimate();
    for (unsigned int i = 0; i < 3; ++i) {
        os << pos_w(i) << " ";
    }
    return os.good();
}

inline void LandmarkVertex::setToOriginImpl() {
    _estimate.fill(0);
}

inline void LandmarkVertex::oplusImpl(const double* update) {
    Eigen::Map<const Vec3_t> v(update);
    _estimate += v;
}

} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
