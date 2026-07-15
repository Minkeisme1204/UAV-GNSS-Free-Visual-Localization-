#pragma once

#include "uavloc/new_vo/optimize/pose_optimizer.h"

#include "uavloc/common/type.h"

namespace uavloc {
namespace vo {

namespace data {
class Frame;
struct FrameObservation;
class Keyframe;
} // namespace data

namespace camera {
class Base;
} // namespace camera

namespace feature {
struct OrbParams;
} // namespace feature

namespace optimize {

class PoseOptimizerG2o : public PoseOptimizer {
public:
    /**
     * Constructor
     * @param num_trials_robust
     * @param num_trials
     * @param num_each_iter
     */
    explicit PoseOptimizerG2o(
        unsigned int num_trials_robust = 2,
        unsigned int num_trials = 2,
        unsigned int num_each_iter = 10);

    /**
     * Destructor
     */
    virtual ~PoseOptimizerG2o() = default;

    /**
     * Perform pose optimization
     * @param frm
     * @return
     */
    unsigned int optimize(const data::Frame& frm, Mat44_t& optimized_pose, std::vector<bool>& outlier_flags) const override;
    unsigned int optimize(const data::Keyframe* keyfrm, Mat44_t& optimized_pose, std::vector<bool>& outlier_flags) const override;

    unsigned int optimize(const Mat44_t& cam_pose_cw, const data::FrameObservation& frm_obs,
                          const feature::OrbParams* OrbParams,
                          const camera::Base* camera,
                          const std::vector<std::shared_ptr<data::Landmark>>& landmarks,
                          Mat44_t& optimized_pose,
                          std::vector<bool>& outlier_flags) const override;

private:
    //! Number of robust optimization (with outlier rejection) attempts
    const unsigned int num_trials_robust_ = 2;

    //! Number of optimization (with outlier rejection) attempts
    const unsigned int num_trials_ = 2;

    //! Maximum number of iterations for each optimization
    const unsigned int num_each_iter_ = 10;
};

} // namespace optimize
}} // namespace vo // namespace uavloc
