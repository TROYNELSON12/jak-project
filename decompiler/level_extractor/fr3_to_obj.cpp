#include "fr3_to_obj.h"

#include "common/custom_data/Tfrag3Data.h"

#include "third-party/tiny_gltf/tiny_gltf.h"
#include "common/math/Vector.h"


#include "third-party/stb_image/stb_image_write.h"

#include "common/log/log.h"

#include "extract_tfrag.h"


/*!
 * Export shrub time-of-day color texture as PNG
 * Layout: width = color_count, height = 8 (time-of-day palettes)
 */
void export_tod_texture(const tfrag3::PackedTimeOfDay& tod,
                              const fs::path& output_dir,
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

  auto tod_path =
      output_dir /
      fmt::format("{}-tod.png", ToD_name);
  file_util::create_dir_if_needed_for_file(tod_path);

  int result = stbi_write_png(tod_path.string().c_str(), image.width, image.height, image.component,
                              image.image.data(), image.width * image.component);

  if (!result) {
    lg::error("Failed to export ToD texture: {}", tod_path.string());
  }
}

nlohmann::json export_level_textures(const tfrag3::Level& level,
                                  const fs::path& output_dir,
                                  std::unordered_set<int> tex_map) {
  nlohmann::json tex_info;
  //std::string result = "Level texture index table:\n";
  for (int i = 0; i < level.textures.size(); i++) {
    if (!tex_map.contains(i))
      continue;
    auto& tex = level.textures[i];
    tex_info[std::to_string(i)] = tex.debug_name;
    //result += fmt::format("{}/{}\n", i, tex.debug_name);
    tinygltf::Image image;
    image.pixel_type = TINYGLTF_TEXTURE_TYPE_UNSIGNED_BYTE;
    image.width = tex.w;
    image.height = tex.h;
    image.image.resize(tex.data.size() * 4);
    image.bits = 8;
    image.component = 4;
    image.mimeType = "image/png";
    image.name = tex.debug_name;
    memcpy(image.image.data(), tex.data.data(), tex.data.size() * 4);

    // Save as PNG using stb_image_write
    auto tod_path = output_dir / fmt::format("{}.png", image.name);
    file_util::create_dir_if_needed_for_file(tod_path);

    int result = stbi_write_png(tod_path.string().c_str(), image.width, image.height,
                                image.component, image.image.data(), image.width * image.component);

    if (!result) {
      lg::error("Failed to export texture: {}", tod_path.string());
    }
  }
  return tex_info;
}

std::string add_tfrag(const tfrag3::Level& level,
                      const tfrag3::TfragTree& tfrag,
                      const fs::path& output_dir,
                      std::unordered_set<int>& tex_map,
                      u32 index) {
  std::string returnResult;

  std::vector<math::Vector4f> verts;
  std::vector<math::Vector<float, 2>> txcrds;
  std::vector<float> tods;
  std::vector<std::vector<math::Vector<int, 3>>> faces;
  std::vector<u32> DrawModes;
  std::vector<u32> Textures;

  std::string kind = tfrag3::tfrag_tree_names[(int)tfrag.kind];

  returnResult += fmt::format("TFrag tree {} info:\n", index);
  returnResult += fmt::format("kind:\n{}\n", kind);

  constexpr float kClusterSize = 4096 * 40;  // 100 in-game meters
  constexpr float kMasterOffset = 12000 * 4096;
  constexpr float rescale = kClusterSize / UINT16_MAX;

  for (auto& vertex : tfrag.packed_vertices.vertices) {
    auto& cluster = tfrag.packed_vertices.cluster_origins[vertex.cluster_idx];
    math::Vector4f v;
    float cx = -kMasterOffset + kClusterSize * cluster.x();
    float cy = -kMasterOffset + kClusterSize * cluster.y();
    float cz = -kMasterOffset + kClusterSize * cluster.z();
    v.x() = cx + vertex.xoff * rescale;
    v.y() = cy + vertex.yoff * rescale;
    v.z() = cz + vertex.zoff * rescale;
    verts.push_back(v / 4096.0f);
    txcrds.push_back(math::Vector<float, 2>{vertex.s, vertex.t} / 1024.0f);
    tods.push_back(vertex.color_index);
  }
  faces.resize(tfrag.draws.size());
  for (int draw_idx = 0; draw_idx < tfrag.draws.size(); draw_idx++) {
    auto& stripdraw = tfrag.draws[draw_idx];

    DrawModes.push_back(stripdraw.mode.as_int());
    Textures.push_back(stripdraw.tree_tex_id);
    tex_map.insert(stripdraw.tree_tex_id);

    for (auto& run : stripdraw.runs) {
      bool flip = false;
      for (u32 i = 0; i < run.length - 2; ++i) {
        u32 a = run.vertex0 + i + 0;
        u32 b = run.vertex0 + i + 1;
        u32 c = run.vertex0 + i + 2;

        if (flip)
          faces[draw_idx].push_back({a, c, b});
        else
          faces[draw_idx].push_back({a, b, c});

        flip = !flip;
      }
    }
  }

  std::string result;
  for (auto& vert : verts) {
    result += fmt::format("v {} {} {}\n", vert.x(), vert.y(), vert.z());
  }
  for (auto& tc : txcrds) {
    result += fmt::format("vt {} {}\n", tc.x(), tc.y());
  }
  for (auto& tod : tods) {
    result += fmt::format("t {}\n", tod);
  }
  for (int draw_idx = 0; draw_idx < tfrag.draws.size(); draw_idx++) {
    result += fmt::format("d texture:{} drawmode:{}\n", Textures[draw_idx], DrawModes[draw_idx]);
    for (auto& face : faces[draw_idx]) {
      result += fmt::format("f {}/{} {}/{} {}/{}\n", face.x() + 1, face.x() + 1, face.y() + 1,
                            face.y() + 1, face.z() + 1, face.z() + 1);
    }
  }
  auto file_name = fmt::format("tfrag-{}-{}", kind, index);
  auto file_path = output_dir / fmt::format("{}.obj", file_name);
  file_util::create_dir_if_needed_for_file(file_path);
  //file_util::write_text_file(file_path, result);
  //export_tod_texture(tfrag.colors, output_dir, file_name);
  return returnResult;
}

std::string add_tie(const tfrag3::Level& level,
                    const tfrag3::TieTree& tie,
                    const fs::path& output_dir,
                    std::unordered_set<int>& tex_map,
                    u32 index) {
  std::string returnResult;

  const auto& packed = tie.packed_vertices;

  lg::info(fmt::format("TIE matrix group count: {}", packed.matrix_groups.size()));

  std::vector<math::Vector4f> verts;
  std::vector<math::Vector<float, 2>> txcrds;
  std::vector<math::Vector<float, 3>> norms;
  std::vector<math::Vector<s8, 4>> colors;

  verts.reserve(packed.vertices.size());
  txcrds.reserve(packed.vertices.size());
  norms.reserve(packed.vertices.size());
  colors.reserve(packed.vertices.size());

  for (auto& vtx : packed.vertices) {
    verts.emplace_back(vtx.x / 4096.f, vtx.y / 4096.f, vtx.z / 4096.f, 1.f);
    txcrds.push_back({vtx.s / 1024.f, vtx.t / 1024.f});
    norms.push_back({vtx.nx < 1, vtx.ny < 1, vtx.nz < 1});
    colors.push_back({vtx.r, vtx.g, vtx.b, vtx.a});
  }

  std::vector<std::vector<math::Vector<int, 3>>> faces(tie.static_draws.size());
  std::vector<u32> DrawModes;
  std::vector<u32> Textures;

  for (int draw_idx = 0; draw_idx < tie.static_draws.size(); draw_idx++) {
    const auto& draw = tie.static_draws[draw_idx];

    DrawModes.push_back(draw.mode.as_int());
    Textures.push_back(draw.tree_tex_id);
    tex_map.insert(draw.tree_tex_id);

    for (auto& run : draw.runs) {
      bool flip = false;
      for (u32 i = 0; i < run.length - 2; i++) {
        u32 a = run.vertex0 + i;
        u32 b = run.vertex0 + i + 1;
        u32 c = run.vertex0 + i + 2;

        if (flip)
          faces[draw_idx].push_back({a, c, b});
        else
          faces[draw_idx].push_back({a, b, c});

        flip = !flip;
      }
    }
  }
  if (!tie.instanced_wind_draws.empty()) {
    for (size_t draw_idx = 0; draw_idx < tie.instanced_wind_draws.size(); draw_idx++) {
      const auto& wind_draw = tie.instanced_wind_draws[draw_idx];
      DrawModes.push_back(wind_draw.mode.as_int());
      Textures.push_back(wind_draw.tree_tex_id);
      tex_map.insert(wind_draw.tree_tex_id);
    }
  }

  int proto_counter = 0;

  for (int mg = 0; mg < packed.matrix_groups.size(); mg++) {
    const auto& grp = packed.matrix_groups[mg];

    if (grp.matrix_idx != -1)
      continue;  // skip instances

    u32 start = grp.start_vert;
    u32 end = grp.end_vert;

    std::vector<math::Vector<int, 3>> proto_faces;

    for (int draw_idx = 0; draw_idx < faces.size(); draw_idx++) {
      for (auto f : faces[draw_idx]) {
        if (f.x() >= start && f.x() < end && f.y() >= start && f.y() < end && f.z() >= start &&
            f.z() < end) {
          f.x() -= start;
          f.y() -= start;
          f.z() -= start;
          proto_faces.push_back(f);
        }
      }
    }

    if (proto_faces.empty())
      continue;

    std::unordered_set<int> used;
    used.reserve(proto_faces.size() * 3);

    for (auto& f : proto_faces) {
      used.insert(f.x());
      used.insert(f.y());
      used.insert(f.z());
    }

    std::vector<int> remap(end - start, -1);
    std::vector<math::Vector4f> out_verts;
    std::vector<math::Vector<float, 2>> out_uvs;
    std::vector<math::Vector<float, 3>> out_norms;
    std::vector<math::Vector<s8, 4>> out_colors;

    out_verts.reserve(used.size());
    out_uvs.reserve(used.size());
    out_norms.reserve(used.size());
    out_colors.reserve(used.size());

    int next = 0;
    for (int old : used) {
      remap[old] = next++;
      out_verts.push_back(verts[start + old]);
      out_uvs.push_back(txcrds[start + old]);
      out_norms.push_back(norms[start + old]);
      out_colors.push_back(colors[start + old]);
    }

    for (auto& f : proto_faces) {
      f.x() = remap[f.x()];
      f.y() = remap[f.y()];
      f.z() = remap[f.z()];
    }

    std::string result;

    for (auto& v : out_verts)
      result += fmt::format("v {} {} {}\n", v.x(), v.y(), v.z());

    for (auto& uv : out_uvs)
      result += fmt::format("vt {} {}\n", uv.x(), uv.y());

    for (auto& norm : out_norms)
      result += fmt::format("vn {} {} {}\n", norm.x(), norm.y(), norm.z());

    for (auto& color : out_colors)
      result += fmt::format("vc {} {} {} {}\n", color.x(), color.y(), color.z(), color.w());

    for (auto& f : proto_faces)
      result += fmt::format("f {} {} {}\n", f.x() + 1, f.y() + 1, f.z() + 1);

    auto file_name = fmt::format("tie-{}-proto-{}", index, proto_counter++);
    auto file_path = output_dir / fmt::format("{}.obj", file_name);

    file_util::create_dir_if_needed_for_file(file_path);
    //file_util::write_text_file(file_path, result);
  }

  //export_tod_texture(tie.colors, output_dir, fmt::format("tie-{}", index));
  return returnResult;
}



std::string add_shrub(const tfrag3::Level& level,
                      const tfrag3::ShrubTree& shrub,
                      const fs::path& output_dir,
                      std::unordered_set<int>& tex_map,
                      u32 index) {
  std::string returnResult;

  std::vector<math::Vector4f> verts;
  std::vector<math::Vector<float, 2>> txcrds;
  std::vector<math::Vector<u8, 4>> rgba;
  std::vector<std::vector<math::Vector<int, 3>>> faces;
  std::vector<u32> DrawModes;
  std::vector<u32> Textures;

  for (auto& vertex : shrub.packed_vertices.vertices) {
    math::Vector4f v;
    v.x() = vertex.x;
    v.y() = vertex.y;
    v.z() = vertex.z;
    verts.push_back(v / 4096.0f);
    txcrds.push_back(math::Vector<float, 2>{vertex.s, vertex.t} / 1024.0f);
  }
  faces.resize(shrub.static_draws.size());
  for (int draw_idx = 0; draw_idx <shrub.static_draws.size(); draw_idx++) {
    auto& stripdraw = shrub.static_draws[draw_idx];

    DrawModes.push_back(stripdraw.mode.as_int());
    Textures.push_back(stripdraw.tree_tex_id);
    tex_map.insert(stripdraw.tree_tex_id);

    //for (auto& run : stripdraw.runs) {
    //  bool flip = false;
    //  for (u32 i = 0; i < run.length - 2; ++i) {
    //    u32 a = run.vertex0 + i + 0;
    //    u32 b = run.vertex0 + i + 1;
    //    u32 c = run.vertex0 + i + 2;
    //
    //    if (flip)
    //      faces[draw_idx].push_back({a, c, b});
    //    else
    //      faces[draw_idx].push_back({a, b, c});
    //
    //    flip = !flip;
    //  }
    //}
  }

  std::string result;
  for (auto& vert : verts) {
    result += fmt::format("v {} {} {}\n", vert.x(), vert.y(), vert.z());
  }
  for (auto& tc : txcrds) {
    result += fmt::format("vt {} {}\n", tc.x(), tc.y());
  }
  for (int draw_idx = 0; draw_idx < shrub.static_draws.size(); draw_idx++) {
    result += fmt::format("d texture:{} drawmode:{}\n", Textures[draw_idx], DrawModes[draw_idx]);
    for (auto& face : faces[draw_idx]) {
      result += fmt::format("f {} {} {}\n", face.x() + 1, face.y() + 1, face.z() + 1);
    }
  }

  auto file_name = fmt::format("tie-{}", index);
  auto file_path = output_dir / fmt::format("{}.obj", file_name);
  file_util::create_dir_if_needed_for_file(file_path);
  //file_util::write_text_file(file_path, result);
  //export_tod_texture(shrub.time_of_day_colors, output_dir, file_name);
  return returnResult;
}

/*!
 * Export the background geometry (tie, tfrag, shrub) to a obj format (.obj) file.
 */
void save_level_background_as_obj(const tfrag3::Level& level, const fs::path& obj_path, nlohmann::json& level_json) {

  std::string result = fmt::format("Name:\n{}\n", level.level_name);
  std::unordered_set<int> tex_map;

  // add all hi-lod tfrag trees
  result += fmt::format("Tfrag tree count:\n{}\n", level.tfrag_trees.at(0).size());
  for (u32 i = 0; i < level.tfrag_trees.at(0).size(); i++) {
    const auto& tfrag = level.tfrag_trees.at(0).at(i);
    result += add_tfrag(level, tfrag, obj_path / "tfrag", tex_map, i);
    export_tod_texture(tfrag.colors, obj_path / "palettes", fmt::format("tfrag-{}", i));
  }

  result += fmt::format("TIE tree count:\n{}\n", level.tie_trees.at(0).size());
  for (u32 i = 0; i < level.tie_trees.at(0).size(); i++) {
    const auto& tie = level.tie_trees.at(0).at(i);
    result += add_tie(level, tie, obj_path / "tie", tex_map, i);
    export_tod_texture(tie.colors, obj_path / "palettes", fmt::format("tie-{}", i));
  }

  result += fmt::format("Shrub tree count:\n{}\n", level.shrub_trees.size());
  for (u32 i = 0; i < level.shrub_trees.size(); i++) {
    const auto& shrub = level.shrub_trees.at(i);
    result += add_shrub(level, shrub, obj_path / "shrub", tex_map, i);
    export_tod_texture(shrub.time_of_day_colors, obj_path / "palettes", fmt::format("shrub-{}", i));
  }

  level_json["texture-index-table"] = export_level_textures(level, obj_path / "textures", tex_map);

  //auto file_path = obj_path / fmt::format("{}-render-info.jfp", level.level_name);
  //file_util::create_dir_if_needed_for_file(file_path);
  //file_util::write_text_file(file_path, result);
}
