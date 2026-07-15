#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/data/frame.h"
#include "uavloc/new_vo/data/keyframe.h"
#include "uavloc/new_vo/optimize/internal/se3/shot_vertex.h"

#include <unordered_map>
#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Frame;
class Keyframe;
} // namespace data

namespace optimize {
namespace internal {
namespace se3 {

class ShotVertexContainer {
public:
    //! Constructor
    explicit ShotVertexContainer(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve = 50);

    //! Destructor
    virtual ~ShotVertexContainer() = default;

    //! Create and return the g2o vertex created from the specified Frame
    ShotVertex* create_vertex(data::Frame* frm, const bool is_constant);

    //! Create and return the g2o vertex created from the specified Keyframe
    ShotVertex* create_vertex(const std::shared_ptr<data::Keyframe>& keyfrm, const bool is_constant);

    //! Create and return the g2o vertex created from shot ID and camera pose
    ShotVertex* create_vertex(const unsigned int id, const Mat44_t& pose_cw, const bool is_constant);

    //! Get vertex corresponding with the specified Frame
    ShotVertex* get_vertex(data::Frame* frm) const;

    //! Get vertex corresponding with the specified Keyframe
    ShotVertex* get_vertex(const std::shared_ptr<data::Keyframe>& keyfrm) const;

    //! Get vertex corresponding with the specified shot (Frame/Keyframe) ID
    ShotVertex* get_vertex(const unsigned int id) const;

    //! Convert Frame ID to vertex ID
    unsigned int get_vertex_id(data::Frame* frm) const;

    //! Convert Keyframe ID to vertex ID
    unsigned int get_vertex_id(const std::shared_ptr<data::Keyframe>& keyfrm) const;

    //! Convert shot (Frame/Keyframe) ID to vertex ID
    unsigned int get_vertex_id(unsigned int id) const;

    //! Convert vertex ID to shot (Frame/Keyframe) ID
    unsigned int get_id(ShotVertex* vtx);

    //! Convert vertex ID to shot (Frame/Keyframe) ID
    unsigned int get_id(unsigned int vtx_id) const;

    //! Contains the specified Keyframe or not
    bool contain(const std::shared_ptr<data::Keyframe>& keyfrm) const;

    // iterators to sweep shot vertices
    using iterator = std::unordered_map<unsigned int, ShotVertex*>::iterator;
    using const_iterator = std::unordered_map<unsigned int, ShotVertex*>::const_iterator;
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

private:
    const std::shared_ptr<unsigned int> offset_ = nullptr;

    //! key: vertex ID, value: vertex
    std::unordered_map<unsigned int, ShotVertex*> vtx_container_;

    //! key: Frame/Keyframe ID, value: vertex ID
    std::unordered_map<unsigned int, unsigned int> vtx_id_container_;

    //! key: vertex ID, value: Frame/Keyframe ID
    std::unordered_map<unsigned int, unsigned int> id_container_;
};

inline ShotVertexContainer::ShotVertexContainer(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve)
    : offset_(offset) {
    vtx_container_.reserve(num_reserve);
    vtx_id_container_.reserve(num_reserve);
    id_container_.reserve(num_reserve);
}

inline ShotVertex* ShotVertexContainer::create_vertex(data::Frame* frm, const bool is_constant) {
    return create_vertex(frm->id_, frm->get_pose_cw(), is_constant);
}

inline ShotVertex* ShotVertexContainer::create_vertex(const std::shared_ptr<data::Keyframe>& keyfrm, const bool is_constant) {
    return create_vertex(keyfrm->id_, keyfrm->get_pose_cw(), is_constant);
}

inline ShotVertex* ShotVertexContainer::create_vertex(const unsigned int id, const Mat44_t& pose_cw, const bool is_constant) {
    // vertexを作成
    const auto vtx_id = *offset_;
    (*offset_)++;
    auto vtx = new ShotVertex();
    vtx->setId(vtx_id);
    vtx->setEstimate(util::Converter::to_g2o_SE3(pose_cw));
    vtx->setFixed(is_constant);
    // databaseに登録
    id_container_[vtx_id] = id;
    vtx_id_container_[id] = vtx_id;
    vtx_container_[id] = vtx;
    // 作成したvertexをreturn
    return vtx;
}

inline ShotVertex* ShotVertexContainer::get_vertex(data::Frame* frm) const {
    return get_vertex(frm->id_);
}

inline ShotVertex* ShotVertexContainer::get_vertex(const std::shared_ptr<data::Keyframe>& keyfrm) const {
    return get_vertex(keyfrm->id_);
}

inline ShotVertex* ShotVertexContainer::get_vertex(const unsigned int id) const {
    return vtx_container_.at(id);
}

inline unsigned int ShotVertexContainer::get_vertex_id(data::Frame* frm) const {
    return get_vertex_id(frm->id_);
}

inline unsigned int ShotVertexContainer::get_vertex_id(const std::shared_ptr<data::Keyframe>& keyfrm) const {
    return get_vertex_id(keyfrm->id_);
}

inline unsigned int ShotVertexContainer::get_vertex_id(unsigned int id) const {
    return vtx_id_container_.at(id);
}

inline unsigned int ShotVertexContainer::get_id(ShotVertex* vtx) {
    return get_id(vtx->id());
}

inline unsigned int ShotVertexContainer::get_id(unsigned int vtx_id) const {
    return id_container_.at(vtx_id);
}

inline bool ShotVertexContainer::contain(const std::shared_ptr<data::Keyframe>& keyfrm) const {
    return 0 != vtx_container_.count(keyfrm->id_);
}

inline ShotVertexContainer::iterator ShotVertexContainer::begin() {
    return vtx_container_.begin();
}

inline ShotVertexContainer::const_iterator ShotVertexContainer::begin() const {
    return vtx_container_.begin();
}

inline ShotVertexContainer::iterator ShotVertexContainer::end() {
    return vtx_container_.end();
}

inline ShotVertexContainer::const_iterator ShotVertexContainer::end() const {
    return vtx_container_.end();
}

} // namespace se3
} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
