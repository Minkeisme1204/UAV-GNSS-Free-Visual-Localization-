#pragma once

#include "uavloc/common/type.h"
#include "uavloc/new_vo/data/landmark.h"
#include "uavloc/new_vo/optimize/internal/landmark_vertex.h"

#include <unordered_map>
#include <memory>

namespace uavloc {
namespace vo {

namespace data {
class Landmark;
} // namespace data

namespace optimize {
namespace internal {

class LandmarkVertexContainer {
public:
    //! Constructor
    explicit LandmarkVertexContainer(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve = 200);

    //! Destructor
    virtual ~LandmarkVertexContainer() = default;

    //! Create and return the g2o vertex created from the specified Landmark
    LandmarkVertex* create_vertex(const std::shared_ptr<data::Landmark>& lm, const bool is_constant);

    //! Create and return the g2o vertex created from the specified Landmark
    LandmarkVertex* create_vertex(const unsigned int id, const Vec3_t& pos_w, const bool is_constant);

    //! Get vertex corresponding with the specified Landmark
    LandmarkVertex* get_vertex(const std::shared_ptr<data::Landmark>& lm) const;

    //! Get vertex corresponding with the specified Landmark ID
    LandmarkVertex* get_vertex(const unsigned int id) const;

    //! Convert Landmark to vertex ID
    unsigned int get_vertex_id(const std::shared_ptr<data::Landmark>& lm) const;

    //! Convert Landmark ID to vertex ID
    unsigned int get_vertex_id(const unsigned int id) const;

    //! Convert vertex to Landmark ID
    unsigned int get_id(LandmarkVertex* vtx) const;

    //! Convert vertex ID to Landmark ID
    unsigned int get_id(const unsigned int vtx_id) const;

    //! Contains the specified Landmark or not
    bool contain(const std::shared_ptr<data::Landmark>& lm) const;

    // iterators to sweep Landmark vertices
    using iterator = std::unordered_map<unsigned int, LandmarkVertex*>::iterator;
    using const_iterator = std::unordered_map<unsigned int, LandmarkVertex*>::const_iterator;
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

private:
    //! vertex ID = offset + Landmark ID
    const std::shared_ptr<unsigned int> offset_ = nullptr;

    //! key: Landmark ID, value: vertex
    std::unordered_map<unsigned int, LandmarkVertex*> vtx_container_;

    //! key: Landmark ID, value: vertex ID
    std::unordered_map<unsigned int, unsigned int> vtx_id_container_;

    //! key: vertex ID, value: Frame/Keyframe ID
    std::unordered_map<unsigned int, unsigned int> id_container_;
};

inline LandmarkVertexContainer::LandmarkVertexContainer(const std::shared_ptr<unsigned int> offset, const unsigned int num_reserve)
    : offset_(offset) {
    vtx_container_.reserve(num_reserve);
    vtx_id_container_.reserve(num_reserve);
    id_container_.reserve(num_reserve);
}

inline LandmarkVertex* LandmarkVertexContainer::create_vertex(const std::shared_ptr<data::Landmark>& lm, const bool is_constant) {
    return create_vertex(lm->id_, lm->get_pos_in_world(), is_constant);
}

inline LandmarkVertex* LandmarkVertexContainer::create_vertex(const unsigned int id, const Vec3_t& pos_w, const bool is_constant) {
    // vertexを作成
    const auto vtx_id = *offset_;
    (*offset_)++;
    auto vtx = new LandmarkVertex();
    vtx->setId(vtx_id);
    vtx->setEstimate(pos_w);
    vtx->setFixed(is_constant);
    vtx->setMarginalized(true);
    // databaseに登録
    id_container_[vtx_id] = id;
    vtx_id_container_[id] = vtx_id;
    vtx_container_[id] = vtx;
    // 作成したvertexをreturn
    return vtx;
}

inline LandmarkVertex* LandmarkVertexContainer::get_vertex(const std::shared_ptr<data::Landmark>& lm) const {
    return get_vertex(lm->id_);
}

inline LandmarkVertex* LandmarkVertexContainer::get_vertex(const unsigned int id) const {
    return vtx_container_.at(id);
}

inline unsigned int LandmarkVertexContainer::get_vertex_id(const std::shared_ptr<data::Landmark>& lm) const {
    return get_vertex_id(lm->id_);
}

inline unsigned int LandmarkVertexContainer::get_vertex_id(const unsigned int id) const {
    return vtx_id_container_.at(id);
}

inline unsigned int LandmarkVertexContainer::get_id(LandmarkVertex* vtx) const {
    return get_id(vtx->id());
}

inline unsigned int LandmarkVertexContainer::get_id(const unsigned int vtx_id) const {
    return id_container_.at(vtx_id);
}

inline bool LandmarkVertexContainer::contain(const std::shared_ptr<data::Landmark>& lm) const {
    return 0 != vtx_container_.count(lm->id_);
}

inline LandmarkVertexContainer::iterator LandmarkVertexContainer::begin() {
    return vtx_container_.begin();
}

inline LandmarkVertexContainer::const_iterator LandmarkVertexContainer::begin() const {
    return vtx_container_.begin();
}

inline LandmarkVertexContainer::iterator LandmarkVertexContainer::end() {
    return vtx_container_.end();
}

inline LandmarkVertexContainer::const_iterator LandmarkVertexContainer::end() const {
    return vtx_container_.end();
}

} // namespace internal
} // namespace optimize
}} // namespace vo // namespace uavloc
