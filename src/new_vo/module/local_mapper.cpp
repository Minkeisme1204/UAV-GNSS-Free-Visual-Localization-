#include "uavloc/new_vo/module/local_mapper.h"

#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/data/map_database.h"
#include "uavloc/new_vo/match/fuse.h"
#include "uavloc/new_vo/match/robust.h"
#include "uavloc/new_vo/module/local_map_cleaner.h"
#include "uavloc/new_vo/module/two_view_triangulator.h"
#include "uavloc/new_vo/optimize/local_bundle_adjuster.h"
#include "uavloc/new_vo/optimize/local_bundle_adjuster_factory.h"
#include "uavloc/new_vo/solve/essential_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <spdlog/spdlog.h>

namespace uavloc {
namespace vo {
namespace module {

LocalMapper::LocalMapper(data::MapDatabase* map_db, const VOConfig& config)
    : map_db_(map_db),
      local_map_cleaner_(new module::LocalMapCleaner(config.vo_node, map_db)),
      local_bundle_adjuster_(optimize::LocalBundleAdjusterFactory::create(config.vo_node)),
      erase_temporal_keyframes_(config.erase_temporal_keyframes),
      num_temporal_keyframes_(config.num_temporal_keyframes),
      num_covisibilities_for_landmark_generation_(config.num_covisibilities_for_landmark_generation),
      num_covisibilities_for_landmark_fusion_(config.num_covisibilities_for_landmark_fusion),
      parallax_deg_thr_(config.triangulation_parallax_deg_thr),
      residual_rad_thr_(config.triangulation_residual_deg_thr * static_cast<float>(M_PI) / 180.0f),
      baseline_dist_thr_ratio_(config.baseline_dist_thr_ratio),
      diag_enabled_(std::getenv("UAVLOC_VO_DIAG") != nullptr) {}

LocalMapper::~LocalMapper() = default;

void LocalMapper::map(const std::shared_ptr<data::Keyframe>& cur_keyfrm,
                      ReplacedLandmarks& replaced_lms,
                      bool* force_stop_flag,
                      bool skip_local_ba,
                      std::function<bool()> abort_landmark_gen) {
    replaced_lms.clear();
    if (!cur_keyfrm) {
        return;
    }

    // Register the keyframe (queue landmarks, update graph, add to the map DB).
    store_new_keyframe(cur_keyfrm);

    // Remove landmarks that turned out unreliable.
    local_map_cleaner_->remove_invalid_landmarks(cur_keyfrm->id_);

    // Triangulate new landmarks against the covisibility neighbors.
    create_new_landmarks(cur_keyfrm, abort_landmark_gen);

    // Merge duplicate landmarks between the keyframe and its covisibilities.
    fuse_landmark_duplication(cur_keyfrm, replaced_lms);

    // Refresh the covisibility graph after new landmark associations.
    cur_keyfrm->graph_node_->update_connections(map_db_->get_min_num_shared_lms());

    // Local bundle adjustment (needs a non-trivial window of keyframes). Skipped
    // under async backpressure so the mapping queue can drain.
    if (2 < map_db_->get_num_keyframes() && !skip_local_ba) {
        bool local_abort = false;
        local_bundle_adjuster_->optimize(map_db_, cur_keyfrm,
                                         force_stop_flag ? force_stop_flag : &local_abort);
    }
    else if (skip_local_ba) {
        spdlog::debug("LocalMapper: skipped localBA under load");
    }

    // Temporal mapping (mirrors stella mapping_module.cc:210-237): erase
    // temporal keyframes (id > fixed threshold) once the current keyframe is
    // more than num_temporal_keyframes_ ids ahead — the bounded-VO sliding
    // window. Off by default (erase_temporal_keyframes_ = false).
    if (erase_temporal_keyframes_) {
        for (const auto& keyfrm : map_db_->get_all_keyframes()) {
            if (keyfrm->id_ <= map_db_->get_fixed_keyframe_id_threshold()) {
                continue;
            }

            // erase temporal keyframes after a period of time
            if (keyfrm->id_ > map_db_->get_fixed_keyframe_id_threshold()
                && cur_keyfrm->id_ > keyfrm->id_ + num_temporal_keyframes_) {
                const auto cur_landmarks = keyfrm->get_landmarks();
                keyfrm->prepare_for_erasing(map_db_);
                for (const auto& lm : cur_landmarks) {
                    if (!lm) {
                        continue;
                    }
                    if (lm->will_be_erased()) {
                        continue;
                    }
                    if (!lm->has_representative_descriptor()) {
                        lm->compute_descriptor();
                    }
                    if (!lm->has_valid_prediction_parameters()) {
                        lm->update_mean_normal_and_obs_scale_variance();
                    }
                }
            }
        }
    }

    // Cull redundant keyframes.
    local_map_cleaner_->remove_redundant_keyframes(cur_keyfrm);
}

void LocalMapper::reset() {
    local_map_cleaner_->reset();
}

void LocalMapper::store_new_keyframe(const std::shared_ptr<data::Keyframe>& cur_keyfrm) {
    const auto cur_lms = cur_keyfrm->get_landmarks();
    for (const auto& lm : cur_lms) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        auto lm_nonconst = lm;
        local_map_cleaner_->add_fresh_landmark(lm_nonconst);
    }

    cur_keyfrm->graph_node_->update_connections(map_db_->get_min_num_shared_lms());
    map_db_->add_keyframe(cur_keyfrm);
}

void LocalMapper::create_new_landmarks(const std::shared_ptr<data::Keyframe>& cur_keyfrm,
                                       const std::function<bool()>& abort_landmark_gen) {
    const auto cur_covisibilities =
        cur_keyfrm->graph_node_->get_top_n_covisibilities(num_covisibilities_for_landmark_generation_);

    match::Robust robust_matcher(0.95f, false);

    const Vec3_t cur_cam_center = cur_keyfrm->get_trans_wc();

    for (const auto& ngh_keyfrm : cur_covisibilities) {
        // Interrupt landmark generation when a newer keyframe is queued so the
        // async mapping queue can drain (async backpressure).
        if (abort_landmark_gen && abort_landmark_gen()) {
            break;
        }

        if (!ngh_keyfrm || ngh_keyfrm->will_be_erased()) {
            continue;
        }

        const Vec3_t ngh_cam_center = ngh_keyfrm->get_trans_wc();
        const Vec3_t baseline_vec = ngh_cam_center - cur_cam_center;
        const double baseline_dist = baseline_vec.norm();

        // Skip pairs whose baseline is small relative to the scene scale.
        const float median_scale_in_ngh = ngh_keyfrm->compute_median_depth(true);
        if (baseline_dist < baseline_dist_thr_ratio_ * static_cast<double>(median_scale_in_ngh)) {
            if (diag_enabled_) {
                spdlog::info("DIAG: lmgen skip pair cur={} ngh={} baseline={} thr={}",
                             cur_keyfrm->id_, ngh_keyfrm->id_, baseline_dist,
                             baseline_dist_thr_ratio_ * static_cast<double>(median_scale_in_ngh));
            }
            continue;
        }

        // Essential matrix from the two known camera poses (1 = neighbor, 2 = current).
        const Mat33_t E_ngh_to_cur = solve::EssentialSolver::create_E_21(
            ngh_keyfrm->get_rot_cw(), ngh_keyfrm->get_trans_cw(),
            cur_keyfrm->get_rot_cw(), cur_keyfrm->get_trans_cw());

        std::vector<std::pair<unsigned int, unsigned int>> matches;
        robust_matcher.match_for_triangulation(cur_keyfrm, ngh_keyfrm, E_ngh_to_cur, matches, residual_rad_thr_);

        triangulate_with_two_keyframes(cur_keyfrm, ngh_keyfrm, matches);
    }
}

void LocalMapper::fuse_landmark_duplication(const std::shared_ptr<data::Keyframe>& cur_keyfrm,
                                            ReplacedLandmarks& replaced_lms) {
    const auto fuse_tgt_keyfrms =
        cur_keyfrm->graph_node_->get_top_n_covisibilities(num_covisibilities_for_landmark_fusion_);

    // Fixed Lowe ratio mirrors the stella_vslam fusion constant.
    match::Fuse fuse_matcher(0.6f);
    constexpr float margin = 3.0f;

    // Resolve a duplication or add a fresh association discovered by the matcher.
    const auto resolve_matches =
        [&](const std::shared_ptr<data::Keyframe>& tgt_keyfrm,
            std::unordered_map<std::shared_ptr<data::Landmark>, std::shared_ptr<data::Landmark>>& duplicated_lms,
            std::unordered_map<unsigned int, std::shared_ptr<data::Landmark>>& new_connections) {
            for (const auto& lms_pair : duplicated_lms) {
                auto lm_to_replace = lms_pair.first;
                auto lm_in_keyfrm = lms_pair.second;
                // Replace with the more reliable 3D point (= more observed).
                if (lm_to_replace->num_observations() < lm_in_keyfrm->num_observations()) {
                    std::swap(lm_to_replace, lm_in_keyfrm);
                }
                if (lm_to_replace->id_ != lm_in_keyfrm->id_) {
                    replaced_lms[lm_in_keyfrm] = lm_to_replace;
                    lm_in_keyfrm->replace(lm_to_replace, map_db_);
                    if (!lm_to_replace->has_representative_descriptor()) {
                        lm_to_replace->compute_descriptor();
                    }
                    if (!lm_to_replace->has_valid_prediction_parameters()) {
                        lm_to_replace->update_mean_normal_and_obs_scale_variance();
                    }
                }
            }

            for (const auto& idx_lm : new_connections) {
                const auto best_idx = idx_lm.first;
                auto lm = idx_lm.second;
                while (replaced_lms.count(lm)) {
                    lm = replaced_lms[lm];
                }
                lm->connect_to_keyframe(tgt_keyfrm, best_idx);
                lm->update_mean_normal_and_obs_scale_variance();
                lm->compute_descriptor();
            }
        };

    // Reproject the current keyframe's landmarks into each covisibility target.
    {
        const auto cur_landmarks = cur_keyfrm->get_landmarks();
        for (const auto& fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            if (!fuse_tgt_keyfrm || fuse_tgt_keyfrm->will_be_erased()) {
                continue;
            }
            std::unordered_map<std::shared_ptr<data::Landmark>, std::shared_ptr<data::Landmark>> duplicated_lms;
            std::unordered_map<unsigned int, std::shared_ptr<data::Landmark>> new_connections;
            const Mat33_t rot_cw = fuse_tgt_keyfrm->get_rot_cw();
            const Vec3_t trans_cw = fuse_tgt_keyfrm->get_trans_cw();
            fuse_matcher.detect_duplication(fuse_tgt_keyfrm, rot_cw, trans_cw, cur_landmarks,
                                            margin, duplicated_lms, new_connections, true);
            resolve_matches(fuse_tgt_keyfrm, duplicated_lms, new_connections);
        }
    }

    // Reproject the covisibility targets' landmarks into the current keyframe.
    {
        id_ordered_set<std::shared_ptr<data::Landmark>> candidate_landmarks_to_fuse;
        for (const auto& fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            if (!fuse_tgt_keyfrm || fuse_tgt_keyfrm->will_be_erased()) {
                continue;
            }
            for (const auto& lm : fuse_tgt_keyfrm->get_landmarks()) {
                if (!lm || lm->will_be_erased()) {
                    continue;
                }
                candidate_landmarks_to_fuse.insert(lm);
            }
        }

        std::unordered_map<std::shared_ptr<data::Landmark>, std::shared_ptr<data::Landmark>> duplicated_lms;
        std::unordered_map<unsigned int, std::shared_ptr<data::Landmark>> new_connections;
        const Mat33_t rot_cw = cur_keyfrm->get_rot_cw();
        const Vec3_t trans_cw = cur_keyfrm->get_trans_cw();
        fuse_matcher.detect_duplication(cur_keyfrm, rot_cw, trans_cw, candidate_landmarks_to_fuse,
                                        margin, duplicated_lms, new_connections, true);
        resolve_matches(cur_keyfrm, duplicated_lms, new_connections);
    }
}

void LocalMapper::triangulate_with_two_keyframes(const std::shared_ptr<data::Keyframe>& keyfrm_1,
                                                 const std::shared_ptr<data::Keyframe>& keyfrm_2,
                                                 const std::vector<std::pair<unsigned int, unsigned int>>& matches) {
    const module::TwoViewTriangulator triangulator(keyfrm_1, keyfrm_2, parallax_deg_thr_);

    // Diagnostics: world-frame rotations hoisted out of the loop so each match's
    // ray parallax can be classified against the same threshold triangulate() uses.
    const Mat33_t rot_wc_1 = keyfrm_1->get_rot_cw().transpose();
    const Mat33_t rot_wc_2 = keyfrm_2->get_rot_cw().transpose();
    const double cos_parallax_thr =
        std::cos(static_cast<double>(parallax_deg_thr_) * M_PI / 180.0);
    std::vector<double> parallax_degs;
    unsigned int diag_passed = 0, diag_rej_parallax = 0, diag_rej_other = 0;

    for (const auto& match : matches) {
        const auto idx_1 = match.first;
        const auto idx_2 = match.second;

        Vec3_t pos_w;
        const bool ok = triangulator.triangulate(idx_1, idx_2, pos_w);

        if (diag_enabled_) {
            const Vec3_t ray_w_1 = rot_wc_1 * keyfrm_1->frm_obs_.bearings_.at(idx_1);
            const Vec3_t ray_w_2 = rot_wc_2 * keyfrm_2->frm_obs_.bearings_.at(idx_2);
            const double cos_p = ray_w_1.dot(ray_w_2);
            parallax_degs.push_back(std::acos(std::clamp(cos_p, -1.0, 1.0)) * 180.0 / M_PI);
            const bool parallax_ok = (0.0 < cos_p && cos_p < cos_parallax_thr);
            if (ok) {
                ++diag_passed;
            }
            else if (!parallax_ok) {
                ++diag_rej_parallax;
            }
            else {
                ++diag_rej_other;
            }
        }

        if (!ok) {
            continue;
        }

        auto lm = std::make_shared<data::Landmark>(map_db_->next_landmark_id_++, pos_w, keyfrm_1);

        lm->connect_to_keyframe(keyfrm_1, idx_1);
        lm->connect_to_keyframe(keyfrm_2, idx_2);

        lm->compute_descriptor();
        lm->update_mean_normal_and_obs_scale_variance();

        map_db_->add_landmark(lm);
        local_map_cleaner_->add_fresh_landmark(lm);
    }

    if (diag_enabled_) {
        double median_parallax = 0.0;
        if (!parallax_degs.empty()) {
            std::sort(parallax_degs.begin(), parallax_degs.end());
            median_parallax = parallax_degs.at(parallax_degs.size() / 2);
        }
        const double baseline =
            (keyfrm_2->get_trans_wc() - keyfrm_1->get_trans_wc()).norm();
        spdlog::info("DIAG: lmgen pair cur={} ngh={} matches={} median_parallax_deg={} "
                     "passed={} rej_parallax={} rej_other={} baseline={}",
                     keyfrm_1->id_, keyfrm_2->id_, matches.size(), median_parallax,
                     diag_passed, diag_rej_parallax, diag_rej_other, baseline);
    }
}

} // namespace module
} // namespace vo
} // namespace uavloc
