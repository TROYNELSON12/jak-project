#include "fr3_to_gltf.h"

#include <unordered_map>

#include "common/custom_data/Tfrag3Data.h"
#include "common/math/Vector.h"
#include "common/math/geometry.h"

#include "decompiler/level_extractor/tfrag_tie_fixup.h"

#include "third-party/tiny_gltf/tiny_gltf.h"

#include "third-party/stb_image/stb_image_write.h"

#include "common/log/log.h"

namespace {

/*!
 * Remove 4096 meter scaling from a transformation matrix.
 */
math::Matrix4f unscale_translation(const math::Matrix4f& in) {
  auto out = in;
  for (int i = 0; i < 3; i++) {
    out(i, 3) /= 4096.;
  }
  return out;
}

/*!
 * Convert fr3 format indices (strip format, with UINT32_MAX as restart) to unstripped tris.
 * Assumes that this is the tfrag/tie format of stripping. Will flip tris as needed so the faces
 * in this fragment all point a consistent way. However, the entire frag may be flipped.
 */
void unstrip_tfrag(const std::vector<u32>& stripped_indices,
                       const std::vector<math::Vector3f>& positions,
                       std::vector<u32>& unstripped,
                       std::vector<u32>& old_to_new_start) {
  fixup_and_unstrip_tfrag(stripped_indices, positions, unstripped, old_to_new_start);
}

void unstrip_tie(const std::vector<u32>& stripped_indices,
                 const std::vector<math::Vector3f>& positions,
                 const std::vector<tfrag3::PreloadedVertex>& vertices,
                 std::vector<u32>& unstripped,
                 std::vector<u32>& old_to_new_start) {
  fixup_and_unstrip_tie(stripped_indices, positions, vertices, unstripped, old_to_new_start);
}

/*!
 * Convert shrub strips. This doesn't assume anything about the strips.
 */
void unstrip_shrub_draws(const std::vector<u32>& stripped_indices,
                         std::vector<u32>& unstripped,
                         std::vector<u32>& draw_to_start,
                         std::vector<u32>& draw_to_count,
                         const std::vector<tfrag3::ShrubDraw>& draws) {
  for (auto& draw : draws) {
    draw_to_start.push_back(unstripped.size());

    for (size_t i = 2; i < draw.num_indices; i++) {
      int idx = i + draw.first_index_index;
      u32 a = stripped_indices[idx];
      u32 b = stripped_indices[idx - 1];
      u32 c = stripped_indices[idx - 2];
      if (a == UINT32_MAX || b == UINT32_MAX || c == UINT32_MAX) {
        continue;
      }
      unstripped.push_back(a);
      unstripped.push_back(b);
      unstripped.push_back(c);
    }
    draw_to_count.push_back(unstripped.size() - draw_to_start.back());
  }
}

void unstrip_tie_wind(std::vector<u32>& unstripped,
                      std::vector<std::vector<u32>>& draw_to_starts,
                      std::vector<std::vector<u32>>& draw_to_counts,
                      const std::vector<tfrag3::InstancedStripDraw>& draws) {
  for (auto& draw : draws) {
    auto& starts = draw_to_starts.emplace_back();
    auto& counts = draw_to_counts.emplace_back();

    int grp_offset = 0;

    for (const auto& grp : draw.instance_groups) {
      starts.push_back(unstripped.size());

      for (size_t i = grp_offset + 2; i < grp_offset + grp.num; i++) {
        u32 a = draw.vertex_index_stream.at(i);
        u32 b = draw.vertex_index_stream.at(i - 1);
        u32 c = draw.vertex_index_stream.at(i - 2);
        if (a == UINT32_MAX || b == UINT32_MAX || c == UINT32_MAX) {
          continue;
        }
        unstripped.push_back(a);
        unstripped.push_back(b);
        unstripped.push_back(c);
      }
      counts.push_back(unstripped.size() - starts.back());
      grp_offset += grp.num;
    }
  }
}

/*!
 * Remap indices from unpacked shrub vertices to proto vertices.
 * Also track which instance group each index came from.
 * unpacked_idx -> proto_idx mapping based on instance_groups.
 */
void remap_shrub_indices_to_proto(const std::vector<u32>& unpacked_indices,
                                   const std::vector<tfrag3::PackedShrubVertices::InstanceGroup>& groups,
                                   std::vector<u32>& proto_indices,
                                   std::vector<u32>& index_to_group_id) {
  proto_indices.reserve(unpacked_indices.size());
  index_to_group_id.reserve(unpacked_indices.size());

  // Build cumulative offset map for unpacked vertices
  std::vector<u32> unpacked_offsets;
  u32 offset = 0;
  for (const auto& grp : groups) {
    unpacked_offsets.push_back(offset);
    offset += (grp.end_vert - grp.start_vert);
  }

  // Remap each index
  for (u32 unpacked_idx : unpacked_indices) {
    // Find which instance group this vertex belongs to
    for (size_t grp_idx = 0; grp_idx < groups.size(); grp_idx++) {
      u32 grp_start = unpacked_offsets[grp_idx];
      u32 grp_end = (grp_idx + 1 < groups.size()) ? unpacked_offsets[grp_idx + 1]
                                                    : offset;

      if (unpacked_idx >= grp_start && unpacked_idx < grp_end) {
        // This vertex belongs to group grp_idx
        u32 offset_in_group = unpacked_idx - grp_start;
        u32 proto_idx = groups[grp_idx].start_vert + offset_in_group;
        proto_indices.push_back(proto_idx);
        index_to_group_id.push_back(grp_idx);
        break;
      }
    }
  }
}

/*!
 * Convert merc strips. Doesn't assume anything about strips. Output is [effect][draw] format (done
 * for each model)
 */
void unstrip_merc_draws(const std::vector<u32>& stripped_indices,
                        const tfrag3::MercModel& model,
                        const std::vector<tfrag3::MercVertex>& vertices,
                        std::vector<u32>& unstripped,
                        std::vector<std::vector<u32>>& draw_to_start,
                        std::vector<std::vector<u32>>& draw_to_count) {
  for (auto& effect : model.effects) {
    auto& effect_dts = draw_to_start.emplace_back();
    auto& effect_dtc = draw_to_count.emplace_back();
    for (auto& draw : effect.all_draws) {
      effect_dts.push_back(unstripped.size());

      for (size_t i = 2; i < draw.index_count; i++) {
        int idx = i + draw.first_index;
        u32 a = stripped_indices[idx];
        u32 b = stripped_indices[idx - 1];
        u32 c = stripped_indices[idx - 2];
        if (a == UINT32_MAX || b == UINT32_MAX || c == UINT32_MAX) {
          continue;
        }

        auto& va = vertices[a];
        auto& vb = vertices[b];
        auto& vc = vertices[c];

        math::Vector3f na(va.normal[0], va.normal[1], va.normal[2]);
        math::Vector3f nb(vb.normal[0], vb.normal[1], vb.normal[2]);
        math::Vector3f nc(vc.normal[0], vc.normal[1], vc.normal[2]);

        math::Vector3f avg_normal = (na + nb + nc).normalized();

        math::Vector3f pa(va.pos[0], va.pos[1], va.pos[2]);
        math::Vector3f pb(vb.pos[0], vb.pos[1], vb.pos[2]);
        math::Vector3f pc(vc.pos[0], vc.pos[1], vc.pos[2]);

        math::Vector3f edge1 = pb - pa;
        math::Vector3f edge2 = pc - pa;
        math::Vector3f face_normal = edge1.cross(edge2).normalized();

        if (face_normal.dot(avg_normal) < 0.0f) {
          unstripped.push_back(a);
          unstripped.push_back(c);
          unstripped.push_back(b);
        } else {
          unstripped.push_back(a);
          unstripped.push_back(b);
          unstripped.push_back(c);
        }
      }
      effect_dtc.push_back(unstripped.size() - effect_dts.back());
    }
  }
}

/*!
 * Get just the xyz positions from a preloaded vertex vector.
 */
std::vector<math::Vector3f> extract_positions(const std::vector<tfrag3::PreloadedVertex>& vtx) {
  std::vector<math::Vector3f> result;
  for (auto& v : vtx) {
    auto& o = result.emplace_back();
    o[0] = v.x;
    o[1] = v.y;
    o[2] = v.z;
  }
  return result;
}

/*!
 * Set up a buffer for the positions of the given vertices.
 * Return the index of the accessor.
 */
template <typename T>
int make_position_buffer_accessor(const std::vector<T>& vertices, tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 3 * vertices.size());

  // and fill it
  u8* buffer_ptr = buffer.data.data();
  for (const auto& vtx : vertices) {
    if constexpr (std::is_same<T, tfrag3::MercVertex>::value) {
      float xyz[3] = {vtx.pos[0] / 4096.f, vtx.pos[1] / 4096.f, vtx.pos[2] / 4096.f};
      memcpy(buffer_ptr, xyz, 3 * sizeof(float));
      buffer_ptr += 3 * sizeof(float);
    } else {
      float xyz[3] = {vtx.x / 4096.f, vtx.y / 4096.f, vtx.z / 4096.f};
      memcpy(buffer_ptr, xyz, 3 * sizeof(float));
      buffer_ptr += 3 * sizeof(float);
    }
  }

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC3;

  return accessor_idx;
}

/*!
 * Set up a buffer for the texture coordinates of the given vertices, multiplying by scale.
 * Return the index of the accessor.
 */
template <typename T>
int make_tex_buffer_accessor(const std::vector<T>& vertices, tinygltf::Model& model, float scale) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 2 * vertices.size());

  // and fill it
  u8* buffer_ptr = buffer.data.data();
  for (const auto& vtx : vertices) {
    if constexpr (std::is_same<T, tfrag3::MercVertex>::value) {
      float st[2] = {vtx.st[0] * scale, vtx.st[1] * scale};
      memcpy(buffer_ptr, st, 2 * sizeof(float));
      buffer_ptr += 2 * sizeof(float);
    } else {
      float st[2] = {vtx.s * scale, vtx.t * scale};
      memcpy(buffer_ptr, st, 2 * sizeof(float));
      buffer_ptr += 2 * sizeof(float);
    }
  }

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC2;

  return accessor_idx;
}

/*!
 * Set up a buffer of vertex colors for the given time of day index, for tfrag.
 * Uses the time of day texture to look up colors.
 */
int make_color_buffer_accessor(const std::vector<tfrag3::PreloadedVertex>& vertices,
                               tinygltf::Model& model,
                               const tfrag3::TfragTree& tfrag_tree,
                               int time_of_day) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());
  std::vector<float> floats;

  for (size_t i = 0; i < vertices.size(); i++) {
    for (int j = 0; j < 4; j++) {
      floats.push_back(((float)tfrag_tree.colors.read(vertices[i].color_index, time_of_day, j)) / 255.f);
    }
    //floats.push_back(1.f);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;
  accessor.maxValues = {1, 1, 1};
  accessor.minValues = {0, 0, 0};

  return accessor_idx;
}

/*!
 * Set up a buffer of vertex colors for the given time of day index, for tie.
 * Uses the time of day texture to look up colors.
 */
int make_color_buffer_accessor(const std::vector<tfrag3::PreloadedVertex>& vertices,
                               tinygltf::Model& model,
                               const tfrag3::TieTree& tie_tree,
                               int time_of_day) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());
  std::vector<float> floats;

  for (size_t i = 0; i < vertices.size(); i++) {
    for (int j = 0; j < 4; j++) {
      floats.push_back(((float)tie_tree.colors.read(vertices[i].color_index, time_of_day, j)) / 255.f);
    }
    //floats.push_back(1.f);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;
  accessor.maxValues = {1, 1, 1};
  accessor.minValues = {0, 0, 0};

  return accessor_idx;
}

int make_color_buffer_accessor(const std::vector<tfrag3::MercVertex>& vertices,
                               tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());
  std::vector<float> floats;

  for (size_t i = 0; i < vertices.size(); i++) {
    for (int j = 0; j < 3; j++) {
      floats.push_back(((float)vertices[i].rgba[j]) / 255.f);
    }
    floats.push_back(1.f);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;
  accessor.maxValues = {1, 1, 1};
  accessor.minValues = {0, 0, 0};

  return accessor_idx;
}

/*!
 * Set up a buffer of vertex colors for the given time of day index, for shrub.
 * Uses the time of day texture to look up colors.
 */
int make_color_buffer_accessor(const std::vector<tfrag3::ShrubGpuVertex>& vertices,
                               tinygltf::Model& model,
                               const tfrag3::ShrubTree& shrub_tree,
                               int time_of_day) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());
  std::vector<float> floats;

  for (size_t i = 0; i < vertices.size(); i++) {
    for (int j = 0; j < 4; j++) {
      floats.push_back(
          ((float)shrub_tree.time_of_day_colors.read(vertices[i].color_index, time_of_day, j)) /
          255.f);
    }
    //floats.push_back(1.f);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;
  accessor.maxValues = {1, 1, 1};
  accessor.minValues = {0, 0, 0};

  return accessor_idx;
}

/*!
 * Set up a buffer for the env colors of the given vertices.
 * Return the index of the accessor.
 */
int make_env_color_buffer_accessor(const std::vector<tfrag3::PreloadedVertex>& vertices,
                                   tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());
  std::vector<float> floats;

  // and fill it
  for (size_t i = 0; i < vertices.size(); i++) {
    floats.push_back(vertices[i].r / 255.0f);
    floats.push_back(vertices[i].g / 255.0f);
    floats.push_back(vertices[i].b / 255.0f);
    floats.push_back(vertices[i].a / 255.0f);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;

  return accessor_idx;
}

/*!
 * Set up a buffer for the shrub colors of the given vertices.
 * Return the index of the accessor.
 */
int make_shrub_color_buffer_accessor(const std::vector<tfrag3::ShrubGpuVertex>& vertices,
                                     tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());
  std::vector<float> floats;

  // and fill it
  for (size_t i = 0; i < vertices.size(); i++) {
    for (int j = 0; j < 3; j++) {
      floats.push_back(vertices[i].rgba_base[j] / 255.0f);
    }
    floats.push_back(1);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;

  return accessor_idx;
}

int make_normal_buffer_accessor(const std::vector<tfrag3::PreloadedVertex>& vertices,
                                tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 3 * vertices.size());
  std::vector<float> floats;

  // and fill it
  for (size_t i = 0; i < vertices.size(); i++) {
    floats.push_back(vertices[i].nx);
    floats.push_back(vertices[i].ny);
    floats.push_back(vertices[i].nz);
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC3;

  return accessor_idx;
}

int make_normal_buffer_accessor(const std::vector<tfrag3::MercVertex>& vertices, tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 3 * vertices.size());
  std::vector<float> floats;

  // and fill it
  for (size_t i = 0; i < vertices.size(); i++) {
    for (int j = 0; j < 3; j++) {
      floats.push_back(vertices[i].normal[j]);
    }
  }
  memcpy(buffer.data.data(), floats.data(), sizeof(float) * floats.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC3;

  return accessor_idx;
}

/*!
 * Create a tinygltf buffer and buffer view for indices, and convert to gltf format.
 * The map can be used to go from slots in the old index buffer to new.
 */
int make_tfrag_index_buffer_view(const std::vector<u32>& indices,
                                 const std::vector<math::Vector3f>& positions,
                                 tinygltf::Model& model,
                                 std::vector<u32>& map_out) {
  std::vector<u32> unstripped;
  unstrip_tfrag(indices, positions, unstripped, map_out);

  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * unstripped.size());

  // and fill it
  memcpy(buffer.data.data(), unstripped.data(), buffer.data.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
  return buffer_view_idx;
}

int make_tie_index_buffer_view(const std::vector<u32>& indices,
                               const std::vector<math::Vector3f>& positions,
                               const std::vector<tfrag3::PreloadedVertex>& vertices,
                               tinygltf::Model& model,
                               std::vector<u32>& map_out) {
  std::vector<u32> unstripped;
  unstrip_tie(indices, positions, vertices, unstripped, map_out);

  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * unstripped.size());

  // and fill it
  memcpy(buffer.data.data(), unstripped.data(), buffer.data.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
  return buffer_view_idx;
}

int make_tie_wind_index_buffer_view(const std::vector<tfrag3::InstancedStripDraw>& draws,
                                    tinygltf::Model& model,
                                    std::vector<std::vector<u32>>& draw_to_starts,
                                    std::vector<std::vector<u32>>& draw_to_counts) {
  std::vector<u32> unstripped;
  unstrip_tie_wind(unstripped, draw_to_starts, draw_to_counts, draws);

  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * unstripped.size());

  // and fill it
  memcpy(buffer.data.data(), unstripped.data(), buffer.data.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
  return buffer_view_idx;
}

int make_shrub_index_buffer_accessor(const std::vector<u32>& indices,
                                      const std::vector<tfrag3::ShrubDraw>& draws,
                                      tinygltf::Model& model,
                                      std::vector<u32>& draw_to_start,
                                      std::vector<u32>& draw_to_count) {
  // Build unstripped indices for each draw
  std::vector<u32> unstripped;
  unstrip_shrub_draws(indices, unstripped, draw_to_start, draw_to_count, draws);

  // Create a buffer for all indices
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * unstripped.size());
  memcpy(buffer.data.data(), unstripped.data(), buffer.data.size());

  // Create a buffer view
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

  return buffer_view_idx;
}

int make_merc_index_buffer_view(const std::vector<u32>& indices,
                                const tfrag3::MercModel& mmodel,
                                const std::vector<tfrag3::MercVertex>& vertices,
                                tinygltf::Model& model,
                                std::vector<std::vector<u32>>& draw_to_start,
                                std::vector<std::vector<u32>>& draw_to_count) {
  std::vector<u32> unstripped;
  unstrip_merc_draws(indices, mmodel, vertices, unstripped, draw_to_start, draw_to_count);

  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * unstripped.size());

  // and fill it
  memcpy(buffer.data.data(), unstripped.data(), buffer.data.size());

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
  return buffer_view_idx;
}

int make_index_buffer_accessor(tinygltf::Model& model,
                               const tfrag3::StripDraw& draw,
                               const std::vector<u32>& idx_map,
                               int buffer_view_idx) {
  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = sizeof(u32) * idx_map.at(draw.unpacked.idx_of_first_idx_in_full_buffer);
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
  accessor.count = draw.num_triangles * 3;
  accessor.type = TINYGLTF_TYPE_SCALAR;

  return accessor_idx;
}

int make_index_buffer_accessor(tinygltf::Model& model, u32 start, u32 count, int buffer_view_idx) {
  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;

  accessor.byteOffset = sizeof(u32) * start;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
  accessor.count = count;
  accessor.type = TINYGLTF_TYPE_SCALAR;

  return accessor_idx;
}

int add_image_for_tex(const tfrag3::Level& level,
                      tinygltf::Model& model,
                      int tex_idx,
                      std::unordered_map<int, int>& tex_image_map) {
  const auto& existing = tex_image_map.find(tex_idx);
  if (existing != tex_image_map.end()) {
    return existing->second;
  }

  auto& tex = level.textures.at(tex_idx);
  int image_idx = (int)model.images.size();
  auto& image = model.images.emplace_back();
  image.pixel_type = TINYGLTF_TEXTURE_TYPE_UNSIGNED_BYTE;
  image.width = tex.w;
  image.height = tex.h;
  image.image.resize(tex.data.size() * 4);
  image.bits = 8;
  image.component = 4;
  image.mimeType = "image/png";
  image.name = tex.debug_name;
  memcpy(image.image.data(), tex.data.data(), tex.data.size() * 4);

  tex_image_map[tex_idx] = image_idx;
  return image_idx;
}

int add_material_for_tex(const tfrag3::Level& level,
                         tinygltf::Model& model,
                         int tex_idx,
                         std::unordered_map<int, int>& tex_image_map,
                         const DrawMode& draw_mode) {
  if (tex_idx < 0) {
    // anim textures, just use default material
    return 0;
  }
  int mat_idx = (int)model.materials.size();
  auto& mat = model.materials.emplace_back();
  auto& tex = level.textures.at(tex_idx);

  mat.name = fmt::format("{}-{}", tex.debug_name, fmt::format("{:x}", draw_mode.as_int()));
  mat.doubleSided = true;
  // the 2.0 here compensates for the ps2's weird blending where 0.5 behaves like 1.0
  mat.pbrMetallicRoughness.baseColorFactor = {1.0, 1.0, 1.0, 1.0};
  mat.pbrMetallicRoughness.baseColorTexture.texCoord = 0;  // TEXCOORD_0, I think
  mat.pbrMetallicRoughness.baseColorTexture.index = model.textures.size();
  mat.alphaMode = draw_mode.get_ab_enable() ? "BLEND" : "MASK";
  // the foreground and background renderers both use this cutoff
  mat.alphaCutoff = (float)0x26 / 255.f;
  auto& gltf_texture = model.textures.emplace_back();
  gltf_texture.name = tex.debug_name;
  gltf_texture.sampler = model.samplers.size();
  auto& sampler = model.samplers.emplace_back();
  sampler.minFilter = draw_mode.get_filt_enable() ? TINYGLTF_TEXTURE_FILTER_LINEAR
                                                  : TINYGLTF_TEXTURE_FILTER_NEAREST;
  sampler.magFilter = draw_mode.get_filt_enable() ? TINYGLTF_TEXTURE_FILTER_LINEAR
                                                  : TINYGLTF_TEXTURE_FILTER_NEAREST;
  sampler.wrapS = draw_mode.get_clamp_s_enable() ? TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE
                                                 : TINYGLTF_TEXTURE_WRAP_REPEAT;
  sampler.wrapT = draw_mode.get_clamp_t_enable() ? TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE
                                                 : TINYGLTF_TEXTURE_WRAP_REPEAT;
  sampler.name = tex.debug_name;

  gltf_texture.source = add_image_for_tex(level, model, tex_idx, tex_image_map);

  return mat_idx;
}

/*!
 * Export shrub time-of-day color texture as PNG
 * Layout: width = color_count, height = 8 (time-of-day palettes)
 */
void export_shrub_tod_texture(const tfrag3::PackedTimeOfDay& tod,
                               const fs::path& output_dir,
                               const std::string& level_name,
                               const std::string& ToD_name) {
  tinygltf::Image image;
  image.width = tod.color_count;
  image.height = 8;
  image.component = 4;  // RGBA
  image.bits = 8;
  image.pixel_type = TINYGLTF_TEXTURE_TYPE_UNSIGNED_BYTE;
  image.image.resize(tod.color_count * 8 * 4);

  // Fill texture data: [palette][color_index] layout
  for (u32 palette = 0; palette < 8; palette++) {
    for (u32 color_idx = 0; color_idx < tod.color_count; color_idx++) {
      u32 pixel_idx = (palette * tod.color_count + color_idx) * 4;
      image.image[pixel_idx + 0] = tod.read(color_idx, palette, 0);  // R
      image.image[pixel_idx + 1] = tod.read(color_idx, palette, 1);  // G
      image.image[pixel_idx + 2] = tod.read(color_idx, palette, 2);  // B
      image.image[pixel_idx + 3] = tod.read(color_idx, palette, 3);  // A
    }
  }

  // Save as PNG using stb_image_write
  auto tod_path =
      output_dir /
      fmt::format("{}_{}_tod.png", level_name.substr(0, level_name.find_last_of('-')), ToD_name);
  file_util::create_dir_if_needed_for_file(tod_path);
  
  int result = stbi_write_png(tod_path.string().c_str(), 
                              image.width, image.height, 
                              image.component, 
                              image.image.data(), 
                              image.width * image.component);
  
  if (result) {
    lg::info("Exported shrub ToD texture: {} ({}x{} colors)", tod_path.string(), 
             tod.color_count, 8);
  } else {
    lg::error("Failed to export shrub ToD texture: {}", tod_path.string());
  }
}

const int kMaxColor = 8;
/*!
 * Add the given tfrag data to a node under tfrag_root.
 */
void add_tfrag(const tfrag3::Level& level,
               const tfrag3::TfragTree& tfrag_in,
               tinygltf::Model& model,
               std::unordered_map<int, int>& tex_image_map,
               u32 index) {
  // copy and unpack in place
  tfrag3::TfragTree tfrag = tfrag_in;
  tfrag.unpack();

  // we'll make a Node, Mesh, Primitive, then add the data to the primitive.
  int node_idx = (int)model.nodes.size();
  auto& node = model.nodes.emplace_back();
  model.scenes.at(0).nodes.push_back(node_idx);

  int mesh_idx = (int)model.meshes.size();
  auto& mesh = model.meshes.emplace_back();
  node.mesh = mesh_idx;
  node.name = fmt::format("Tfrag_{}_{}", tfrag3::tfrag_tree_names[(int)tfrag_in.kind], index);

  int position_buffer_accessor = make_position_buffer_accessor(tfrag.unpacked.vertices, model);
  int texture_buffer_accessor = make_tex_buffer_accessor(tfrag.unpacked.vertices, model, 1.f);
  std::vector<u32> index_map;
  int index_buffer_view = make_tfrag_index_buffer_view(
      tfrag.unpacked.indices, extract_positions(tfrag.unpacked.vertices), model, index_map);
  int colors[kMaxColor];

  for (int i = 0; i < kMaxColor; i++) {
    colors[i] = make_color_buffer_accessor(tfrag.unpacked.vertices, model, tfrag, i);
  }

  for (auto& draw : tfrag.draws) {
    auto& prim = mesh.primitives.emplace_back();
    prim.material = add_material_for_tex(level, model, draw.tree_tex_id, tex_image_map, draw.mode);
    prim.indices = make_index_buffer_accessor(model, draw, index_map, index_buffer_view);
    prim.attributes["POSITION"] = position_buffer_accessor;
    prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
    for (int i = 0; i < kMaxColor; i++) {
      prim.attributes[fmt::format("COLOR_{}", i)] = colors[i];
    }
    prim.mode = TINYGLTF_MODE_TRIANGLES;
  }
}

void add_tie(const tfrag3::Level& level,
             const tfrag3::TieTree& tie_in,
             tinygltf::Model& model,
             std::unordered_map<int, int>& tex_image_map,
             u32 index) {
  // copy and unpack in place
  tfrag3::TieTree tie = tie_in;
  tie.unpack();

  // we'll make a Node, Mesh, Primitive, then add the data to the primitive.
  int node_idx = (int)model.nodes.size();
  auto& node = model.nodes.emplace_back();
  model.scenes.at(0).nodes.push_back(node_idx);

  // Do not create a top-level mesh here; child meshes for each category will be created
  // (this avoids leaving an empty mesh referenced by the root node, which can invalidate
  // the GLTF for some importers).
  node.name = fmt::format("TIE_Root_{}", index);
  //node.name = fmt::format("TIE_{}_{}", tfrag3::tfrag_tree_names[(int)tie_in.kind], index);

  int position_buffer_accessor = make_position_buffer_accessor(tie.unpacked.vertices, model);
  int normal_buffer_accessor = make_normal_buffer_accessor(tie.unpacked.vertices, model);
  int texture_buffer_accessor = make_tex_buffer_accessor(tie.unpacked.vertices, model, 1.f);
  std::vector<u32> index_map;
  int index_buffer_view = make_tie_index_buffer_view( tie.unpacked.indices, extract_positions(tie.unpacked.vertices), tie.unpacked.vertices, model, index_map);
  int colors[kMaxColor];

  for (int i = 0; i < kMaxColor; i++) {
    colors[i] = make_color_buffer_accessor(tie.unpacked.vertices, model, tie, i);
  }
  int env_color_buffer_accessor = make_env_color_buffer_accessor(tie.unpacked.vertices, model);

  for (int cat = 0; cat < tfrag3::kNumTieCategories; cat++) {
    u32 start = tie.category_draw_indices[cat];
    u32 end = tie.category_draw_indices[cat + 1];
    if (start >= end) {
      continue;
    }

    // create a child node+mesh for this category so it can be named
    int c_node_idx = (int)model.nodes.size();
    auto& c_node = model.nodes.emplace_back();
    model.nodes[node_idx].children.push_back(c_node_idx);
    int c_mesh_idx = (int)model.meshes.size();
    auto& c_mesh = model.meshes.emplace_back();
    c_node.mesh = c_mesh_idx;
    c_node.name = fmt::format("TIE_{}_{}", tfrag3::kTieCategoryNames[cat], index);

    // add all draws in this category as primitives on the child mesh
    for (u32 draw_idx = start; draw_idx < end; draw_idx++) {
      const auto& draw = tie.static_draws.at(draw_idx);
      auto& prim = c_mesh.primitives.emplace_back();

      prim.material = add_material_for_tex(level, model, draw.tree_tex_id, tex_image_map, draw.mode);
      prim.indices = make_index_buffer_accessor(model, draw, index_map, index_buffer_view);
      prim.attributes["POSITION"] = position_buffer_accessor;
      prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
      for (int i = 0; i < kMaxColor; i++) {
        prim.attributes[fmt::format("COLOR_{}", i)] = colors[i];
      }
      prim.attributes[fmt::format("COLOR_{}", kMaxColor + 0)] = env_color_buffer_accessor;
      prim.attributes["NORMAL"] = normal_buffer_accessor;
      prim.mode = TINYGLTF_MODE_TRIANGLES;
    }
  }

  if (!tie.instanced_wind_draws.empty()) {
    return;
    std::vector<std::vector<u32>> draw_to_starts, draw_to_counts;
    int wind_index_buffer_view = make_tie_wind_index_buffer_view(tie.instanced_wind_draws, model,
                                                                 draw_to_starts, draw_to_counts);

    for (size_t draw_idx = 0; draw_idx < tie.instanced_wind_draws.size(); draw_idx++) {
      const auto& wind_draw = tie.instanced_wind_draws[draw_idx];
      int mat =
          add_material_for_tex(level, model, wind_draw.tree_tex_id, tex_image_map, wind_draw.mode);

      // Create a single mesh for all instances of this draw
      int c_mesh_idx = (int)model.meshes.size();
      auto& c_mesh = model.meshes.emplace_back();

      for (size_t grp_idx = 0; grp_idx < wind_draw.instance_groups.size(); grp_idx++) {
        const auto& grp = wind_draw.instance_groups[grp_idx];

        u16 wind_index = tie.wind_instance_info.at(grp.instance_idx).wind_idx;
        float stiffness = tie.wind_instance_info.at(grp.instance_idx).stiffness;

        //lg::info("TIE draw {} has wind index of {} and stiffness {}", draw_idx, wind_index, stiffness);

        int c_node_idx = (int)model.nodes.size();
        auto& c_node = model.nodes.emplace_back();
        model.nodes[node_idx].children.push_back(c_node_idx);
        c_node.mesh = c_mesh_idx;
        c_node.name = fmt::format("TIE_Wind,Windex:{},Stiffness:{};", wind_index, stiffness);

        const auto& info = tie.wind_instance_info.at(grp.instance_idx);
        for (int i = 0; i < 4; i++) {
          float scale = i == 3 ? (1.f / 4096.f) : 1.f;
          for (int j = 0; j < 4; j++) {
            c_node.matrix.push_back(scale * info.matrix[i][j]);
          }
        }

        auto& prim = c_mesh.primitives.emplace_back();
        prim.material = mat;
        prim.indices = make_index_buffer_accessor(model, draw_to_starts.at(draw_idx).at(grp_idx),
                                                  draw_to_counts.at(draw_idx).at(grp_idx),
                                                  wind_index_buffer_view);
        prim.attributes["POSITION"] = position_buffer_accessor;
        prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
        for (int i = 0; i < kMaxColor; i++) {
          prim.attributes[fmt::format("COLOR_{}", i)] = colors[i];
        }
        prim.mode = TINYGLTF_MODE_TRIANGLES;
      }
    }
  }
}

//void add_shrub_old(const tfrag3::Level& level,
//               const tfrag3::ShrubTree& shrub_in,
//               tinygltf::Model& model,
//               std::unordered_map<int, int>& tex_image_map,
//               u32 index) {
//  // copy and unpack in place
//  tfrag3::ShrubTree shrub = shrub_in;
//  shrub.unpackExtractor();
//
//  // we'll make a Node, Mesh, Primitive, then add the data to the primitive.
//  int node_idx = (int)model.nodes.size();
//  auto& node = model.nodes.emplace_back();
//  model.scenes.at(0).nodes.push_back(node_idx);
//  node.name = fmt::format("Shrub_Root_{}", index);
//
//
//  int position_buffer_accessor = make_position_buffer_accessor(shrub.unpacked.vertices, model);
//  int texture_buffer_accessor = make_tex_buffer_accessor(shrub.unpacked.vertices, model, 1.f / 4096.f);
//  int colors[kMaxColor];
//  for (int i = 0; i < kMaxColor; i++) {
//    colors[i] = make_color_buffer_accessor(shrub.unpacked.vertices, model, shrub, i);
//  }
//  int base_color_accessor = make_shrub_color_buffer_accessor(shrub.unpacked.vertices, model);
//
//  std::vector<std::vector<u32>> draw_to_starts, draw_to_counts, draw_to_group_ids;
//  int index_buffer_view = make_shrub_index_buffer_view_grouped(shrub.indices, shrub.static_draws,
//                                                               shrub.packed_vertices.instance_groups,
//                                                               model, draw_to_starts,
//                                                               draw_to_counts, draw_to_group_ids);
//
//  for (size_t draw_idx = 0; draw_idx < shrub.static_draws.size(); draw_idx++) {
//    const auto& draw = shrub.static_draws[draw_idx];
//    int mat = add_material_for_tex(level, model, draw.tree_tex_id, tex_image_map, draw.mode);
//
//    // Create a single mesh for all instances of this draw
//    int c_mesh_idx = (int)model.meshes.size();
//    auto& c_mesh = model.meshes.emplace_back();
//
//    for (size_t local_grp = 0; local_grp < draw_to_starts[draw_idx].size(); local_grp++) {
//      u32 start = draw_to_starts[draw_idx][local_grp];
//      u32 count = draw_to_counts[draw_idx][local_grp];
//      u32 group_id = draw_to_group_ids[draw_idx][local_grp];
//
//      u16 wind_index = shrub.packed_vertices.instance_groups.at(group_id).wind_idx;
//      float stiffness = shrub.packed_vertices.instance_groups.at(group_id).stiffness;
//
//      //lg::info("Shrub draw {} group {} has wind index of {} and stiffness {}", draw_idx, group_id, wind_index, stiffness);
//
//      int c_node_idx = (int)model.nodes.size();
//      auto& c_node = model.nodes.emplace_back();
//      model.nodes[node_idx].children.push_back(c_node_idx);
//      c_node.mesh = c_mesh_idx;
//      ASSERT(draw.proto_idx < shrub.proto_names.size());
//      std::string shrub_name = shrub.proto_names[draw.proto_idx];
//      c_node.name = fmt::format("Shrub_{},Windex:{},Stiffness:{};", shrub_name.substr(0, shrub_name.find_last_of('.')), wind_index, stiffness);
//
//      const auto& inst_grp = shrub.packed_vertices.instance_groups.at(group_id);
//      const auto& info = shrub.packed_vertices.matrices.at(inst_grp.matrix_idx);
//
//      for (int i = 0; i < 4; i++) {
//        float scale = i == 3 ? (1.f / 4096.f) : 1.f;
//        for (int j = 0; j < 4; j++) {
//          c_node.matrix.push_back(scale * info[i][j]);
//        }
//      }
//
//      auto& prim = c_mesh.primitives.emplace_back();
//      prim.material = mat;
//      prim.indices = make_index_buffer_accessor(model, start, count, index_buffer_view);
//      prim.attributes["POSITION"] = position_buffer_accessor;
//      prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
//      for (int i = 0; i < kMaxColor; i++) {
//        prim.attributes[fmt::format("COLOR_{}", i)] = colors[i];
//      }
//      prim.attributes[fmt::format("COLOR_{}", kMaxColor + 0)] = base_color_accessor;
//      prim.mode = TINYGLTF_MODE_TRIANGLES;
//    }
//  }
//}

void add_shrub(const tfrag3::Level& level,
               const tfrag3::ShrubTree& shrub_in,
               tinygltf::Model& model,
               std::unordered_map<int, int>& tex_image_map,
               u32 index) {
  // Use proto vertices directly and remap indices to reference them
  // This avoids the massive vertex duplication from unpacking

  // Create root node
  int node_idx = (int)model.nodes.size();
  auto& node = model.nodes.emplace_back();
  model.scenes.at(0).nodes.push_back(node_idx);
  node.name = fmt::format("Shrub_Root_{}", index);

  // Convert proto vertices to GPU format
  std::vector<tfrag3::ShrubGpuVertex> gpu_vertices;
  for (const auto& proto_vtx : shrub_in.packed_vertices.vertices) {
    auto& gpu_vtx = gpu_vertices.emplace_back();
    gpu_vtx.x = proto_vtx.x;
    gpu_vtx.y = proto_vtx.y;
    gpu_vtx.z = proto_vtx.z;
    gpu_vtx.s = proto_vtx.s;
    gpu_vtx.t = proto_vtx.t;
    gpu_vtx.pad0 = 0;
    gpu_vtx.color_index = 0;
    gpu_vtx.pad1 = 0;
    memcpy(gpu_vtx.rgba_base, proto_vtx.rgba, 3);
    gpu_vtx.pad2 = 0;
  }

  // Create buffer accessors for vertex data (using proto vertices)
  int position_buffer_accessor = make_position_buffer_accessor(gpu_vertices, model);
  int texture_buffer_accessor = make_tex_buffer_accessor(gpu_vertices, model, 1.f / 4096.f);
  int colors[kMaxColor];
  //for (int i = 0; i < kMaxColor; i++) {
  //  colors[i] = make_color_buffer_accessor(gpu_vertices, model, shrub_in, i);
  //}
  int base_color_accessor = make_shrub_color_buffer_accessor(gpu_vertices, model);

  // Build index buffer - unstrip the strip format indices and remap to proto vertices
  std::vector<u32> unstripped;
  std::vector<u32> draw_to_start_flat, draw_to_count_flat;
  unstrip_shrub_draws(shrub_in.indices, unstripped, draw_to_start_flat, draw_to_count_flat,
                      shrub_in.static_draws);

  // Remap indices from unpacked vertex ordering to proto vertex indices
  std::vector<u32> proto_indices;
  std::vector<u32> index_to_group_id;
  remap_shrub_indices_to_proto(unstripped, shrub_in.packed_vertices.instance_groups, proto_indices,
                                index_to_group_id);

  // Create buffer for remapped indices
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * proto_indices.size());
  memcpy(buffer.data.data(), proto_indices.data(), buffer.data.size());

  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;

  // For each static_draw (proto), create one mesh
  std::vector<int> draw_to_mesh_idx(shrub_in.static_draws.size());
  for (size_t draw_idx = 0; draw_idx < shrub_in.static_draws.size(); draw_idx++) {
    const auto& draw = shrub_in.static_draws[draw_idx];
    int material = add_material_for_tex(level, model, draw.tree_tex_id, tex_image_map, draw.mode);

    int c_mesh_idx = (int)model.meshes.size();
    draw_to_mesh_idx[draw_idx] = c_mesh_idx;  // Store the actual mesh index
    auto& c_mesh = model.meshes.emplace_back();

    // Create a single primitive for this draw with all its indices
    auto& prim = c_mesh.primitives.emplace_back();
    prim.material = material;
    prim.indices = make_index_buffer_accessor(model, draw_to_start_flat[draw_idx],
                                              draw_to_count_flat[draw_idx], buffer_view_idx);
    prim.attributes["POSITION"] = position_buffer_accessor;
    prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
    //for (int i = 0; i < kMaxColor; i++) {
    //  prim.attributes[fmt::format("COLOR_{}", i)] = colors[i];
    //}
    prim.attributes["COLOR_0"] = base_color_accessor;
    prim.mode = TINYGLTF_MODE_TRIANGLES;
  }

  // Map each instance group to its first occurrence in the index_to_group_id
  // This tells us which draw owns each instance group
  std::vector<int> group_to_draw(shrub_in.packed_vertices.instance_groups.size(), -1);
  for (size_t draw_idx = 0; draw_idx < shrub_in.static_draws.size(); draw_idx++) {
    u32 start = draw_to_start_flat[draw_idx];
    u32 count = draw_to_count_flat[draw_idx];
    for (u32 i = start; i < start + count && i < index_to_group_id.size(); i++) {
      u32 grp_id = index_to_group_id[i];
      if (group_to_draw[grp_id] == -1) {
        group_to_draw[grp_id] = draw_idx;
      }
    }
  }

  // For each instance group, create a node with its transformation
  for (size_t inst_grp_idx = 0; inst_grp_idx < shrub_in.packed_vertices.instance_groups.size();
       inst_grp_idx++) {
    const auto& inst_grp = shrub_in.packed_vertices.instance_groups.at(inst_grp_idx);

    int draw_idx_for_this_group = group_to_draw[inst_grp_idx];
    if (draw_idx_for_this_group < 0) {
      continue;  // Skip if no draw found
    }

    int c_mesh_idx = draw_to_mesh_idx[draw_idx_for_this_group];  // Use the stored mesh index

    // Create node for this instance
    int c_node_idx = (int)model.nodes.size();
    auto& c_node = model.nodes.emplace_back();
    model.nodes[node_idx].children.push_back(c_node_idx);
    c_node.mesh = c_mesh_idx;

    // Set node name
    const auto& draw = shrub_in.static_draws[draw_idx_for_this_group];
    if (draw.proto_idx < shrub_in.proto_names.size()) {
      std::string shrub_name = shrub_in.proto_names[draw.proto_idx];
      c_node.name = fmt::format("Shrub_{},C:{},W:{},S:{};",
                                shrub_name.substr(0, shrub_name.find_last_of('.')),
                                inst_grp.color_index, inst_grp.wind_idx, inst_grp.stiffness);
    } else {
      c_node.name = fmt::format("Shrub_Instance_{}", inst_grp_idx);
    }

    // Apply transformation matrix
    const auto& mat = shrub_in.packed_vertices.matrices.at(inst_grp.matrix_idx);
    for (int i = 0; i < 4; i++) {
      float scale = i == 3 ? (1.f / 4096.f) : 1.f;
      for (int j = 0; j < 4; j++) {
        c_node.matrix.push_back(scale * mat[i][j]);
      }
    }
  }
}

int make_weights_accessor(const std::vector<tfrag3::MercVertex>& vertices, tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());

  // and fill it
  u8* buffer_ptr = buffer.data.data();
  for (const auto& vtx : vertices) {
    float weights[4] = {vtx.weights[0], vtx.weights[1], vtx.weights[2], 0};
    memcpy(buffer_ptr, weights, 4 * sizeof(float));
    buffer_ptr += 4 * sizeof(float);
  }

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;
  return accessor_idx;
}

int make_bones_accessor(const std::vector<tfrag3::MercVertex>& vertices, tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 4 * vertices.size());

  // and fill it
  u8* buffer_ptr = buffer.data.data();
  for (const auto& vtx : vertices) {
    s32 indices[4];
    for (int i = 0; i < 3; i++) {
      indices[i] = vtx.mats[i] ? vtx.mats[i] - 1 : 0;
    }
    indices[3] = 0;
    memcpy(buffer_ptr, indices, 4 * sizeof(s32));
    buffer_ptr += 4 * sizeof(s32);
  }

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;  // blender doesn't support INT...
  accessor.count = vertices.size();
  accessor.type = TINYGLTF_TYPE_VEC4;
  return accessor_idx;
}

int make_inv_matrix_bind_poses(const std::vector<level_tools::Joint>& joints,
                               tinygltf::Model& model) {
  // first create a buffer:
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(float) * 16 * joints.size());

  // and fill it
  for (int m = 0; m < (int)joints.size(); m++) {
    auto matrix = unscale_translation(joints[m].bind_pose_T_w);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        memcpy(buffer.data.data() + sizeof(float) * (i * 4 + j + m * 16), &matrix(j, i), 4);
      }
    }
  }

  // create a view of this buffer
  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;  // tightly packed
  buffer_view.target = TINYGLTF_TARGET_ARRAY_BUFFER;

  int accessor_idx = (int)model.accessors.size();
  auto& accessor = model.accessors.emplace_back();
  accessor.bufferView = buffer_view_idx;
  accessor.byteOffset = 0;
  accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
  accessor.count = joints.size();
  accessor.type = TINYGLTF_TYPE_MAT4;
  return accessor_idx;
}

void add_merc(const tfrag3::Level& level,
              const std::map<std::string, level_tools::ArtData>& art_data,
              const tfrag3::MercModel& mmodel,
              tinygltf::Model& model,
              std::unordered_map<int, int>& tex_image_map) {
  const auto& mverts = level.merc_data.vertices;

  // create position and uv buffers
  int position_buffer_accessor = make_position_buffer_accessor(mverts, model);
  int texture_buffer_accessor = make_tex_buffer_accessor(mverts, model, 1.f);

  std::vector<std::vector<u32>> draw_to_start, draw_to_count;
  int index_buffer_view = make_merc_index_buffer_view(level.merc_data.indices, mmodel, mverts, model,
                                                      draw_to_start, draw_to_count);
  int colors = make_color_buffer_accessor(mverts, model);

  int normal_buffer_accessor = make_normal_buffer_accessor(mverts, model);

  auto joints_accessor = make_bones_accessor(mverts, model);
  auto weights_accessor = make_weights_accessor(mverts, model);

  const auto& art = art_data.find(mmodel.name);
  int node_idx = (int)model.nodes.size();
  auto& node = model.nodes.emplace_back();
  model.scenes.at(0).nodes.push_back(node_idx);
  node.name = mmodel.name;
  int mesh_idx = (int)model.meshes.size();
  auto& mesh = model.meshes.emplace_back();
  mesh.name = node.name;
  node.mesh = mesh_idx;

  if (art != art_data.end() && !art->second.joint_group.empty()) {
    node.skin = model.skins.size();
    auto& skin = model.skins.emplace_back();
    const auto& game_bones = art->second.joint_group;
    int n_bones = game_bones.size();
    std::vector<std::vector<int>> children(n_bones);
    for (size_t i = 0; i < game_bones.size(); i++) {
      if (game_bones[i].parent_idx >= 0) {
        children.at(game_bones[i].parent_idx).push_back(i);
      }
    }
    skin.skeleton = model.nodes.size();
    for (int i = 0; i < n_bones; i++) {
      const auto& gbone = game_bones[i];
      skin.joints.push_back(skin.skeleton + i);
      auto& snode = model.nodes.emplace_back();
      snode.name = gbone.name;

      // bind pose is bind_T_w
      // for glb we want bind_parent_T_bind_child
      // so bindp_T_w * inverse(bindc_T_w)
      math::Matrix4f matrix;
      if (gbone.parent_idx >= 0) {
        matrix = unscale_translation(game_bones.at(gbone.parent_idx).bind_pose_T_w) *
                 inverse(unscale_translation(gbone.bind_pose_T_w));

      } else {
        // I think this value is ignored anyway.
        for (int r = 0; r < 4; r++) {
          for (int c = 0; c < 4; c++) {
            matrix(r, c) = (r == c) ? 1 : 0;
          }
        }
      }

      for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
          snode.matrix.push_back(matrix(c, r));
        }
      }
      for (auto child : children.at(i)) {
        snode.children.push_back(skin.skeleton + child);
      }
    }
    ASSERT(skin.skeleton + n_bones == (int)model.nodes.size());
    skin.inverseBindMatrices = make_inv_matrix_bind_poses(game_bones, model);
  }

  std::vector<int> all_target_pos_accessors;
  std::vector<int> all_target_norm_accessors;

  for (size_t effect_idx = 0; effect_idx < mmodel.effects.size(); effect_idx++) {
    const auto& effect = mmodel.effects[effect_idx];
    for (size_t draw_idx = 0; draw_idx < effect.all_draws.size(); draw_idx++) {
      const auto& draw = effect.all_draws[draw_idx];
      auto& prim = mesh.primitives.emplace_back();
      prim.material =
          add_material_for_tex(level, model, draw.tree_tex_id, tex_image_map, draw.mode);
      prim.indices =
          make_index_buffer_accessor(model, draw_to_start[effect_idx][draw_idx],
                                     draw_to_count[effect_idx][draw_idx], index_buffer_view);
      prim.attributes["POSITION"] = position_buffer_accessor;
      prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
      prim.attributes["COLOR_0"] = colors;
      prim.attributes["NORMAL"] = normal_buffer_accessor;
      prim.attributes["JOINTS_0"] = joints_accessor;
      prim.attributes["WEIGHTS_0"] = weights_accessor;
      prim.mode = TINYGLTF_MODE_TRIANGLES;
    }

    //Thus code is hot garbage.
    continue;

    // Handle BLERC (morph targets) for this effect if present
    if (!effect.mod.blerc.int_data.empty() && !effect.mod.vertices.empty()) {
      lg::info("BLERC: Processing effect {} with {} mod vertices, {} blerc int_data entries",
               effect_idx, effect.mod.vertices.size(), effect.mod.blerc.int_data.size());

      // BLERC (Blend-shape) data format (from Tfrag3Data.h):
      // int data, per vertex:
      // [tgt0_idx, tgt1_idx, ..., terminator, dest]
      // float data, per vertex:
      // [base, tgt0, tgt1, ...]

      // final vertex position is:
      // base + sum(tgtn * weights[tgtn_idx])
      
      // Map each mod vertex to a global vertex so we can output deltas in global vertex space
      std::vector<int> mod_to_global(effect.mod.vertices.size(), -1);
      
      // Build a hash map of all global vertices for O(1) lookup
      std::unordered_map<std::string, std::vector<int>> global_vert_map;
      for (size_t gi = 0; gi < mverts.size(); gi++) {
        const auto& gv = mverts[gi];
        std::string key(reinterpret_cast<const char*>(&gv), sizeof(gv));
        global_vert_map[key].push_back((int)gi);
      }

      // For each mod vertex, find its corresponding global vertex by raw byte comparison
      // (they should be exact copies from extraction)
      int mapped_count = 0;
      for (size_t mi = 0; mi < effect.mod.vertices.size(); mi++) {
        const auto& mv = effect.mod.vertices[mi];
        std::string key(reinterpret_cast<const char*>(&mv), sizeof(mv));
        auto it = global_vert_map.find(key);
        if (it != global_vert_map.end() && !it->second.empty()) {
          mod_to_global[mi] = it->second[0];
          mapped_count++;
        }
      }
      lg::info("BLERC: Mapped {} / {} mod vertices to global indices", mapped_count,
               effect.mod.vertices.size());

      // Count how many targets we have by scanning int_data
      int num_targets = 0;
      for (auto idx : effect.mod.blerc.int_data) {
        if (idx == (u32)tfrag3::Blerc::kTargetIdxTerminator)
          continue;
        num_targets = std::max(num_targets, (int)idx + 1);
      }
      lg::info("BLERC: Detected {} target shapes", num_targets);

      if (num_targets > 0) {
        // We'll build position and normal deltas for each target.
        // These are indexed by GLOBAL vertex index (same as base mesh indices)
        // so that morph targets align with the primitive's vertex references.
        int global_vert_count = (int)mverts.size();
        std::vector<std::vector<float>> pos_deltas(num_targets,
                                                   std::vector<float>(global_vert_count * 3, 0.0f));
        std::vector<std::vector<float>> norm_deltas(num_targets,
                                                    std::vector<float>(global_vert_count * 3, 0.0f));

        // Parse BLERC int_data and float_data together.
        // The format is per-vertex: [base_float_idx, tgt0_idx, tgt1_idx, ..., terminator, dest, ...]
        int fidx = 0;  // index into float_data
        int iidx = 0;  // index into int_data
        const auto& fdata = effect.mod.blerc.float_data;
        const auto& idata = effect.mod.blerc.int_data;
        int processed_vertices = 0;
        int skipped_vertices = 0;

        while (iidx < (int)idata.size()) {
          if (fidx >= (int)fdata.size()) {
            lg::warn("BLERC: fidx {} >= fdata.size() {}", fidx, fdata.size());
            break;
          }

          // Skip the base vertex float data (we don't need it for deltas)
          fidx++;

          // Collect all (target_index, float_data_ptr) pairs for this vertex
          std::vector<std::pair<int, const tfrag3::BlercFloatData*>> targets;
          while (iidx < (int)idata.size() && idata[iidx] != tfrag3::Blerc::kTargetIdxTerminator) {
            int tidx = (int)idata[iidx++];
            if (fidx >= (int)fdata.size()) {
              lg::warn("BLERC: ran out of float data while reading targets");
              break;
            }
            targets.emplace_back(tidx, &fdata[fidx++]);
          }

          // Expect terminator
          if (iidx >= (int)idata.size()) {
            lg::warn("BLERC: iidx {} >= idata.size() {}", iidx, idata.size());
            break;
          }
          iidx++;  // skip terminator

          // Get the destination mod vertex index
          if (iidx >= (int)idata.size()) {
            lg::warn("BLERC: no dest after terminator");
            break;
          }
          int dest_mod_idx = (int)idata[iidx++];

          // Map mod vertex to global vertex
          if (dest_mod_idx < 0 || dest_mod_idx >= (int)effect.mod.vertices.size()) {
            lg::warn("BLERC: invalid dest_mod_idx {} (mod has {} vertices)", dest_mod_idx,
                     effect.mod.vertices.size());
            skipped_vertices++;
            continue;
          }
          int global_idx = mod_to_global[dest_mod_idx];
          if (global_idx < 0) {
            lg::warn("BLERC: mod vertex {} has no global mapping", dest_mod_idx);
            skipped_vertices++;
            continue;
          }

          // Store the deltas for each target at this global vertex index
          for (auto& [tidx, tdata] : targets) {
            if (tidx < 0 || tidx >= num_targets) {
              lg::warn("BLERC: invalid target index {} (model has {} targets)", tidx, num_targets);
              continue;
            }
            int base_idx = global_idx * 3;
            pos_deltas[tidx][base_idx + 0] = tdata->v[0];
            pos_deltas[tidx][base_idx + 1] = tdata->v[1];
            pos_deltas[tidx][base_idx + 2] = tdata->v[2];
            norm_deltas[tidx][base_idx + 0] = tdata->v[4];
            norm_deltas[tidx][base_idx + 1] = tdata->v[5];
            norm_deltas[tidx][base_idx + 2] = tdata->v[6];
          }
          processed_vertices++;
        }
        lg::info("BLERC: Processed {} vertices, skipped {}", processed_vertices, skipped_vertices);

        std::vector<int> target_pos_accessors;
        std::vector<int> target_norm_accessors;
        std::vector<std::pair<size_t, size_t>> pos_target_ranges; // byte ranges per target
        std::vector<std::pair<size_t, size_t>> norm_target_ranges; // byte ranges per target

        std::vector<unsigned char> pos_buf_data;
        std::vector<unsigned char> norm_buf_data;

        const float blerc_scale = 1.0f;
        int kept = 0;
        for (int t = 0; t < num_targets; t++) {
          bool all_zero = true;
          // Check if this target has any non-zero pos or normal delta
          for (int vi = 0; vi < global_vert_count; vi++) {
            float px = pos_deltas[t][vi * 3 + 0];
            float py = pos_deltas[t][vi * 3 + 1];
            float pz = pos_deltas[t][vi * 3 + 2];
            float nx = norm_deltas[t][vi * 3 + 0];
            float ny = norm_deltas[t][vi * 3 + 1];
            float nz = norm_deltas[t][vi * 3 + 2];
            if (px != 0.0f || py != 0.0f || pz != 0.0f || nx != 0.0f || ny != 0.0f || nz != 0.0f) {
              all_zero = false;
              break;
            }
          }
          //if (all_zero) continue;

          // append position floats (scaled) to pos_buf_data
          size_t pos_offset = pos_buf_data.size();
          pos_target_ranges.emplace_back(pos_offset, (size_t)global_vert_count * 3 * sizeof(float));
          pos_buf_data.resize(pos_buf_data.size() + (size_t)global_vert_count * 3 * sizeof(float));
          float* pos_ptr = reinterpret_cast<float*>(pos_buf_data.data() + pos_offset);
          for (int vi = 0; vi < global_vert_count; vi++) {
            pos_ptr[vi * 3 + 0] = pos_deltas[t][vi * 3 + 0] * blerc_scale;
            pos_ptr[vi * 3 + 1] = pos_deltas[t][vi * 3 + 1] * blerc_scale;
            pos_ptr[vi * 3 + 2] = pos_deltas[t][vi * 3 + 2] * blerc_scale;
          }

          // append normal floats (scaled) to norm_buf_data
          size_t norm_offset = norm_buf_data.size();
          norm_target_ranges.emplace_back(norm_offset, (size_t)global_vert_count * 3 * sizeof(float));
          norm_buf_data.resize(norm_buf_data.size() + (size_t)global_vert_count * 3 * sizeof(float));
          float* norm_ptr = reinterpret_cast<float*>(norm_buf_data.data() + norm_offset);
          for (int vi = 0; vi < global_vert_count; vi++) {
            norm_ptr[vi * 3 + 0] = norm_deltas[t][vi * 3 + 0] * blerc_scale;
            norm_ptr[vi * 3 + 1] = norm_deltas[t][vi * 3 + 1] * blerc_scale;
            norm_ptr[vi * 3 + 2] = norm_deltas[t][vi * 3 + 2] * blerc_scale;
          }

          kept++;
        }

        lg::info("BLERC: Kept {} / {} non-empty targets", kept, num_targets);

        // If we have any kept targets, create two buffers and create bufferViews/accessors
        if (kept > 0) {
          int pos_buffer_idx = (int)model.buffers.size();
          model.buffers.emplace_back();
          model.buffers.back().data = pos_buf_data;

          int norm_buffer_idx = (int)model.buffers.size();
          model.buffers.emplace_back();
          model.buffers.back().data = norm_buf_data;

          // Create bufferViews and accessors for each kept target in the same order we appended them
          for (size_t ti = 0; ti < pos_target_ranges.size(); ti++) {
            auto [pos_off, pos_len] = pos_target_ranges[ti];
            auto [norm_off, norm_len] = norm_target_ranges[ti];

            int pbv = (int)model.bufferViews.size();
            model.bufferViews.emplace_back();
            auto& pbview = model.bufferViews.back();
            pbview.buffer = pos_buffer_idx;
            pbview.byteOffset = (int)pos_off;
            pbview.byteLength = (int)pos_len;
            pbview.byteStride = 0;
            pbview.target = TINYGLTF_TARGET_ARRAY_BUFFER;

            int pacc = (int)model.accessors.size();
            model.accessors.emplace_back();
            auto& paccessor = model.accessors.back();
            paccessor.bufferView = pbv;
            paccessor.byteOffset = 0;
            paccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
            paccessor.count = global_vert_count;  // MUST match base mesh vertex count
            paccessor.type = TINYGLTF_TYPE_VEC3;
            target_pos_accessors.push_back(pacc);

            int nbv = (int)model.bufferViews.size();
            model.bufferViews.emplace_back();
            auto& nbview = model.bufferViews.back();
            nbview.buffer = norm_buffer_idx;
            nbview.byteOffset = (int)norm_off;
            nbview.byteLength = (int)norm_len;
            nbview.byteStride = 0;
            nbview.target = TINYGLTF_TARGET_ARRAY_BUFFER;

            int nacc = (int)model.accessors.size();
            model.accessors.emplace_back();
            auto& naccessor = model.accessors.back();
            naccessor.bufferView = nbv;
            naccessor.byteOffset = 0;
            naccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
            naccessor.count = global_vert_count;  // MUST match base mesh vertex count
            naccessor.type = TINYGLTF_TYPE_VEC3;
            target_norm_accessors.push_back(nacc);
          }
        }

        for (int t = 0; t < (int)target_pos_accessors.size(); t++) {
          all_target_pos_accessors.push_back(target_pos_accessors[t]);
          all_target_norm_accessors.push_back(target_norm_accessors[t]);
        }
        lg::info("BLERC: Appended {} targets for effect {} (total now {})",
                 target_pos_accessors.size(), effect_idx, all_target_pos_accessors.size());
      }
    }
  }

  if (!all_target_pos_accessors.empty()) {
    lg::info("BLERC: Attaching combined {} targets to {} primitives",
             all_target_pos_accessors.size(), mesh.primitives.size());
    for (size_t pi = 0; pi < mesh.primitives.size(); pi++) {
      auto& prim = mesh.primitives[pi];
      for (size_t t = 0; t < all_target_pos_accessors.size(); t++) {
        std::map<std::string, int> tgt;
        tgt["POSITION"] = all_target_pos_accessors[t];
        tgt["NORMAL"] = all_target_norm_accessors[t];
        prim.targets.push_back(tgt);
      }
      lg::info("BLERC: Primitive {} now has {} target shapes", pi, prim.targets.size());
    }

    mesh.weights = std::vector<double>(all_target_pos_accessors.size(), 0.0);
    lg::info("BLERC: Set mesh.weights to {} entries", mesh.weights.size());
  }
}

}  // namespace

/*!
 * Export the background geometry (tie, tfrag, shrub) to a GLTF binary format (.glb) file.
 */
void save_level_background_as_gltf(const tfrag3::Level& level, const fs::path& glb_file) {
  // the top level container for everything is the model.
  tinygltf::Model model;

  // a "scene" is a traditional scene graph, made up of Nodes.
  // sadly, attempting to nest stuff makes the blender importer unhappy, so we just dump
  // everything into the top level.
  model.scenes.emplace_back();

  // hack, add a default material.
  tinygltf::Material mat;
  mat.pbrMetallicRoughness.baseColorFactor = {1.0f, 0.9f, 0.9f, 1.0f};
  mat.doubleSided = true;
  model.materials.push_back(mat);

  std::unordered_map<int, int> tex_image_map;

  // add all hi-lod tfrag trees
  for (u32 i = 0; i < level.tfrag_trees.at(0).size(); i++) {
    const auto& tfrag = level.tfrag_trees.at(0).at(i);
    add_tfrag(level, tfrag, model, tex_image_map, i);
    export_shrub_tod_texture(tfrag.colors, glb_file.parent_path(),
                             glb_file.stem().string(), fmt::format("tfrag{}", i));
  }

  for (u32 i = 0; i < level.tie_trees.at(0).size(); i++) {
    const auto& tie = level.tie_trees.at(0).at(i);
    add_tie(level, tie, model, tex_image_map, i);
    export_shrub_tod_texture(tie.colors, glb_file.parent_path(),
                             glb_file.stem().string(), fmt::format("tie{}", i));
  }

  for (u32 i = 0; i < level.shrub_trees.size(); i++) {
    const auto& shrub = level.shrub_trees.at(i);
    add_shrub(level, shrub, model, tex_image_map, i);
    export_shrub_tod_texture(shrub.time_of_day_colors, glb_file.parent_path(),
                             glb_file.stem().string(), fmt::format("shrub{}", i));
  }

  model.asset.generator = "opengoal";
  tinygltf::TinyGLTF gltf;
  gltf.WriteGltfSceneToFile(&model, glb_file.string(),
                            true,   // embedImages
                            true,   // embedBuffers
                            true,   // pretty print
                            true);  // write binary
}

void save_level_foreground_as_gltf(const tfrag3::Level& level,
                                   const std::map<std::string, level_tools::ArtData>& art_data,
                                   const fs::path& glb_path) {
  for (size_t model_idx = 0; model_idx < level.merc_data.models.size(); model_idx++) {
    const auto& mmodel = level.merc_data.models[model_idx];

    // the top level container for everything is the model.
    tinygltf::Model model;

    // a "scene" is a traditional scene graph, made up of Nodes.
    // sadly, attempting to nest stuff makes the blender importer unhappy, so we just dump
    // everything into the top level.
    model.scenes.emplace_back();

    // hack, add a default material.
    tinygltf::Material mat;
    mat.pbrMetallicRoughness.baseColorFactor = {1.0f, 0.9f, 0.9f, 1.0f};
    mat.doubleSided = true;
    model.materials.push_back(mat);

    std::unordered_map<int, int> tex_image_map;

    add_merc(level, art_data, mmodel, model, tex_image_map);

    model.asset.generator = "opengoal";

    auto glb_file = glb_path / fmt::format("{}.glb", mmodel.name);
    file_util::create_dir_if_needed_for_file(glb_file);

    tinygltf::TinyGLTF gltf;
    gltf.WriteGltfSceneToFile(&model, glb_file.string(),
                              true,   // embedImages
                              true,   // embedBuffers
                              true,   // pretty print
                              true);  // write binary
  }
}
