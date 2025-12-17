#include "fr3_to_gltf.h"

#include <unordered_map>

#include "common/custom_data/Tfrag3Data.h"
#include "common/math/Vector.h"
#include "common/math/geometry.h"

#include "decompiler/level_extractor/tfrag_tie_fixup.h"

#include "third-party/tiny_gltf/tiny_gltf.h"

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
 * Convert merc strips. Doesn't assume anything about strips. Output is [effect][draw] format (done
 * for each model)
 */
void unstrip_merc_draws(const std::vector<u32>& stripped_indices,
                        const tfrag3::MercModel& model,
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
        unstripped.push_back(a);
        unstripped.push_back(b);
        unstripped.push_back(c);
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
    for (int j = 0; j < 3; j++) {
      floats.push_back(
          ((float)shrub_tree.time_of_day_colors.read(vertices[i].color_index, time_of_day, j)) /
          255.f);
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

float unpack_s10(u32 num) {
  // chop to ten bits.
  s32 snum = static_cast<s32>(num & 0x3FF);

  // flip if negative bit (10)
  if (snum & 0x200) {
    snum |= ~0x3FF;
  }

  float result = static_cast<float>(snum);
  return result / 511.0f;
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
    floats.push_back(unpack_s10(vertices[i].nor));
    floats.push_back(unpack_s10(vertices[i].nor >> 10));
    floats.push_back(unpack_s10(vertices[i].nor >> 20));
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

/*!
 * Create a tinygltf buffer and buffer view for indices, and convert to gltf format.
 * The map can be used to go from slots in the old index buffer to new.
 */
int make_shrub_index_buffer_view(const std::vector<u32>& indices,
                                 const std::vector<tfrag3::ShrubDraw>& draws,
                                 tinygltf::Model& model,
                                 std::vector<u32>& draw_to_start,
                                 std::vector<u32>& draw_to_count) {
  std::vector<u32> unstripped;
  unstrip_shrub_draws(indices, unstripped, draw_to_start, draw_to_count, draws);

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

int make_shrub_index_buffer_view_grouped(const std::vector<u32>& indices,
                                         const std::vector<tfrag3::ShrubDraw>& draws,
                                         const std::vector<tfrag3::PackedShrubVertices::InstanceGroup>& groups,
                                         tinygltf::Model& model,
                                         std::vector<std::vector<u32>>& draw_to_starts,
                                         std::vector<std::vector<u32>>& draw_to_counts,
                                         std::vector<std::vector<u32>>& draw_to_group_ids) {
  // Build group offsets in the expanded (unpacked) vertex ordering.
  std::vector<u32> group_offsets;
  group_offsets.reserve(groups.size());
  u32 running = 0;
  for (const auto& g : groups) {
    group_offsets.push_back(running);
    u32 size = g.end_vert - g.start_vert;
    running += size;
  }

  auto find_group = [&](u32 vtx_idx) -> int {
    // binary search the group_offsets to find greatest offset <= vtx_idx
    int lo = 0;
    int hi = (int)group_offsets.size() - 1;
    if (group_offsets.empty()) return -1;
    if (vtx_idx < group_offsets[0]) return -1;
    while (lo <= hi) {
      int mid = (lo + hi) / 2;
      u32 off = group_offsets[mid];
      u32 next_off = (mid + 1 < (int)group_offsets.size()) ? group_offsets[mid + 1] : running;
      if (vtx_idx >= off && vtx_idx < next_off) return mid;
      if (vtx_idx < off) hi = mid - 1;
      else lo = mid + 1;
    }
    return -1;
  };

  std::vector<u32> unstripped;

  // For each draw, collect triangles per group, then append them so they are grouped
  // (draw -> group -> triangles). This mirrors the tie wind exporter behavior.
  draw_to_starts.resize(draws.size());
  draw_to_counts.resize(draws.size());
  draw_to_group_ids.resize(draws.size());

  for (size_t d = 0; d < draws.size(); d++) {
    const auto& draw = draws[d];

    // temporary buckets per group (only push for groups that get tris)
    std::vector<std::vector<u32>> per_group(groups.size());

    // iterate over the raw index stream for this draw (strip-format)
    for (size_t i = 2; i < draw.num_indices; i++) {
      int idx = (int)(i + draw.first_index_index);
      u32 a = indices[idx];
      u32 b = indices[idx - 1];
      u32 c = indices[idx - 2];
      if (a == UINT32_MAX || b == UINT32_MAX || c == UINT32_MAX) {
        continue;
      }
      int ga = find_group(a);
      int gb = find_group(b);
      int gc = find_group(c);
      // triangles should all come from same group. If not, put them in group 0 as fallback.
      int gid = (ga == gb && gb == gc) ? ga : (ga >= 0 ? ga : (gb >= 0 ? gb : gc));
      if (gid < 0) continue;
      per_group[gid].push_back(a);
      per_group[gid].push_back(b);
      per_group[gid].push_back(c);
    }

    // now append groups that have data into the main unstripped buffer, record starts/counts
    for (size_t g = 0; g < per_group.size(); g++) {
      if (per_group[g].empty()) continue;
      draw_to_group_ids[d].push_back((u32)g);
      draw_to_starts[d].push_back(unstripped.size());
      draw_to_counts[d].push_back(per_group[g].size());
      unstripped.insert(unstripped.end(), per_group[g].begin(), per_group[g].end());
    }
  }

  // create buffer and bufferView
  int buffer_idx = (int)model.buffers.size();
  auto& buffer = model.buffers.emplace_back();
  buffer.data.resize(sizeof(u32) * unstripped.size());
  memcpy(buffer.data.data(), unstripped.data(), buffer.data.size());

  int buffer_view_idx = (int)model.bufferViews.size();
  auto& buffer_view = model.bufferViews.emplace_back();
  buffer_view.buffer = buffer_idx;
  buffer_view.byteOffset = 0;
  buffer_view.byteLength = buffer.data.size();
  buffer_view.byteStride = 0;
  buffer_view.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
  return buffer_view_idx;
}

int make_merc_index_buffer_view(const std::vector<u32>& indices,
                                const tfrag3::MercModel& mmodel,
                                tinygltf::Model& model,
                                std::vector<std::vector<u32>>& draw_to_start,
                                std::vector<std::vector<u32>>& draw_to_count) {
  std::vector<u32> unstripped;
  unstrip_merc_draws(indices, mmodel, unstripped, draw_to_start, draw_to_count);

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
    std::vector<std::vector<u32>> draw_to_starts, draw_to_counts;
    int wind_index_buffer_view = make_tie_wind_index_buffer_view(tie.instanced_wind_draws, model,
                                                                 draw_to_starts, draw_to_counts);

    for (size_t draw_idx = 0; draw_idx < tie.instanced_wind_draws.size(); draw_idx++) {
      const auto& wind_draw = tie.instanced_wind_draws[draw_idx];
      int mat =
          add_material_for_tex(level, model, wind_draw.tree_tex_id, tex_image_map, wind_draw.mode);
      for (size_t grp_idx = 0; grp_idx < wind_draw.instance_groups.size(); grp_idx++) {
        const auto& grp = wind_draw.instance_groups[grp_idx];
        int c_node_idx = (int)model.nodes.size();
        auto& c_node = model.nodes.emplace_back();
        model.nodes[node_idx].children.push_back(c_node_idx);
        int c_mesh_idx = (int)model.meshes.size();
        auto& c_mesh = model.meshes.emplace_back();
        c_node.mesh = c_mesh_idx;
        c_node.name = fmt::format("TIE_WindMesh_{}_{}_{}", index, draw_idx, grp_idx);
        auto& prim = c_mesh.primitives.emplace_back();

        const auto& info = tie.wind_instance_info.at(grp.instance_idx);
        for (int i = 0; i < 4; i++) {
          float scale = i == 3 ? (1.f / 4096.f) : 1.f;
          for (int j = 0; j < 4; j++) {
            c_node.matrix.push_back(scale * info.matrix[i][j]);
          }
        }

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

void add_shrub(const tfrag3::Level& level,
               const tfrag3::ShrubTree& shrub_in,
               tinygltf::Model& model,
               std::unordered_map<int, int>& tex_image_map,
               u32 index) {
  // copy and unpack in place
  tfrag3::ShrubTree shrub = shrub_in;
  shrub.unpackExtractor();

  // we'll make a Node, Mesh, Primitive, then add the data to the primitive.
  int node_idx = (int)model.nodes.size();
  auto& node = model.nodes.emplace_back();
  model.scenes.at(0).nodes.push_back(node_idx);
  node.name = fmt::format("Shrub_Tree_{}", index);


  int position_buffer_accessor = make_position_buffer_accessor(shrub.unpacked.vertices, model);
  int texture_buffer_accessor = make_tex_buffer_accessor(shrub.unpacked.vertices, model, 1.f / 4096.f);
  int colors[kMaxColor];
  for (int i = 0; i < kMaxColor; i++) {
    colors[i] = make_color_buffer_accessor(shrub.unpacked.vertices, model, shrub, i);
  }
  int base_color_accessor = make_shrub_color_buffer_accessor(shrub.unpacked.vertices, model);

  std::vector<std::vector<u32>> draw_to_starts, draw_to_counts, draw_to_group_ids;
  int index_buffer_view = make_shrub_index_buffer_view_grouped(shrub.indices, shrub.static_draws,
                                                               shrub.packed_vertices.instance_groups,
                                                               model, draw_to_starts,
                                                               draw_to_counts, draw_to_group_ids);

  for (size_t draw_idx = 0; draw_idx < shrub.static_draws.size(); draw_idx++) {
    const auto& draw = shrub.static_draws[draw_idx];
    int mat = add_material_for_tex(level, model, draw.tree_tex_id, tex_image_map, draw.mode);

    for (size_t local_grp = 0; local_grp < draw_to_starts[draw_idx].size(); local_grp++) {
      u32 start = draw_to_starts[draw_idx][local_grp];
      u32 count = draw_to_counts[draw_idx][local_grp];
      u32 group_id = draw_to_group_ids[draw_idx][local_grp];

      int c_node_idx = (int)model.nodes.size();
      auto& c_node = model.nodes.emplace_back();
      model.nodes[node_idx].children.push_back(c_node_idx);
      int c_mesh_idx = (int)model.meshes.size();
      auto& c_mesh = model.meshes.emplace_back();
      c_node.mesh = c_mesh_idx;
      ASSERT(draw.proto_idx < shrub.proto_names.size());
      c_node.name = fmt::format("Shrub_{}_{}_{}", shrub.proto_names[draw.proto_idx], draw_idx, group_id);

      const auto& inst_grp = shrub.packed_vertices.instance_groups.at(group_id);
      const auto& info = shrub.packed_vertices.matrices.at(inst_grp.matrix_idx);

      for (int i = 0; i < 4; i++) {
        float scale = i == 3 ? (1.f / 4096.f) : 1.f;
        for (int j = 0; j < 4; j++) {
          c_node.matrix.push_back(scale * info[i][j]);
        }
      }

      auto& prim = c_mesh.primitives.emplace_back();
      prim.material = mat;
      prim.indices = make_index_buffer_accessor(model, start, count, index_buffer_view);
      prim.attributes["POSITION"] = position_buffer_accessor;
      prim.attributes["TEXCOORD_0"] = texture_buffer_accessor;
      for (int i = 0; i < kMaxColor; i++) {
        prim.attributes[fmt::format("COLOR_{}", i)] = colors[i];
      }
      prim.attributes[fmt::format("COLOR_{}", kMaxColor + 0)] = base_color_accessor;
      prim.mode = TINYGLTF_MODE_TRIANGLES;
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
  int index_buffer_view = make_merc_index_buffer_view(level.merc_data.indices, mmodel, model,
                                                      draw_to_start, draw_to_count);
  int colors = make_color_buffer_accessor(mverts, model);

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
      prim.attributes["JOINTS_0"] = joints_accessor;
      prim.attributes["WEIGHTS_0"] = weights_accessor;
      prim.mode = TINYGLTF_MODE_TRIANGLES;
    }
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
  }

  for (u32 i = 0; i < level.tie_trees.at(0).size(); i++) {
    const auto& tie = level.tie_trees.at(0).at(i);
    add_tie(level, tie, model, tex_image_map, i);
  }

  for (u32 i = 0; i < level.shrub_trees.size(); i++) {
    const auto& shrub = level.shrub_trees.at(i);
    add_shrub(level, shrub, model, tex_image_map, i);
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
