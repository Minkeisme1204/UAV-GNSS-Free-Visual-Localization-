#include "uavloc/util/converter.h"

#include <g2o/types/slam3d/se3quat.h>

// Bodies for the converter helpers referenced by new_vo (data/map_database uses
// inverse_pose; optimize/ + module/ use the g2o/SE3 helpers). Ported faithfully
// from stella_vslam::util::converter.

namespace uavloc {
namespace util {

g2o::SE3Quat Converter::to_g2o_SE3(const Mat44_t& pose) {
    const Mat33_t rot = pose.block<3, 3>(0, 0);
    const Vec3_t trans = pose.block<3, 1>(0, 3);
    return g2o::SE3Quat{rot, trans};
}

Mat44_t Converter::to_eigen_mat(const g2o::SE3Quat& g2o_SE3) {
    return g2o_SE3.to_homogeneous_matrix();
}

Mat44_t Converter::to_eigen_pose(const Mat33_t& rot, const Vec3_t& trans) {
    Mat44_t pose = Mat44_t::Identity();
    pose.block<3, 3>(0, 0) = rot;
    pose.block<3, 1>(0, 3) = trans;
    return pose;
}

Mat33_t Converter::to_skew_symmetric_mat(const Vec3_t& vec) {
    Mat33_t skew;
    skew << 0, -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;
    return skew;
}

Mat44_t Converter::inverse_pose(const Mat44_t& pose_cw) {
    const Mat33_t rot_cw = pose_cw.block<3, 3>(0, 0);
    const Vec3_t trans_cw = pose_cw.block<3, 1>(0, 3);
    const Mat33_t rot_wc = rot_cw.transpose();
    const Vec3_t cam_center = -rot_wc * trans_cw;
    Mat44_t pose_wc = Mat44_t::Identity();
    pose_wc.block<3, 3>(0, 0) = rot_wc;
    pose_wc.block<3, 1>(0, 3) = cam_center;
    return pose_wc;
}

} // namespace util
} // namespace uavloc
