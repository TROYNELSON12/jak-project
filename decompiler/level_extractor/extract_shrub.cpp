#include "extract_shrub.h"

#include <array>

#include "common/log/log.h"
#include "common/util/FileUtil.h"

#include "decompiler/ObjectFile/LinkedObjectFile.h"
#include "decompiler/level_extractor/extract_common.h"

namespace decompiler {
using namespace level_tools;

std::array<math::Vector4f, 4> extract_shrub_matrix(const u16* data) {
  std::array<math::Vector4f, 4> result;
  for (int i = 0; i < 4; i++) {
    s32 x = data[12 + i];
    x <<= 16;
    x >>= 10;
    result[3][i] = x;
  }

  for (int vec = 0; vec < 3; vec++) {
    for (int i = 0; i < 4; i++) {
      s32 x = data[vec * 4 + i];
      x <<= 16;
      x >>= 16;
      result[vec][i] = (float)x / 4096.f;
    }
  }

  return result;
}

struct ShrubVertex {
  math::Vector<float, 3> xyz;
  math::Vector<float, 2> st;
  math::Vector<u8, 3> rgba_generic;
  bool adc = false;

  bool operator==(const ShrubVertex& other) const {
    return xyz.x() == other.xyz.x() &&
           xyz.y() == other.xyz.y() &&
           xyz.z() == other.xyz.z() &&
           st.x() == other.st.x() &&
           st.y() == other.st.y() &&
           rgba_generic.x() == other.rgba_generic.x() &&
           rgba_generic.y() == other.rgba_generic.y() &&
           rgba_generic.z() == other.rgba_generic.z();
  }

  struct hash {
    std::size_t operator()(const ShrubVertex& x) const {
      return std::hash<float>()(x.xyz.x()) ^
             std::hash<float>()(x.xyz.y()) ^
             std::hash<float>()(x.xyz.z()) ^
             std::hash<float>()(x.st.x()) ^
             std::hash<float>()(x.st.y()) ^
             std::hash<u8>()(x.rgba_generic.x()) ^
             std::hash<u8>()(x.rgba_generic.y()) ^
             std::hash<u8>()(x.rgba_generic.z());
    }
  };
};
struct DrawSettings {
  DrawMode mode;
  u32 combo_tex;
};
struct ShrubDraw {
  u32 start_vtx_idx = -1;
  AdGifData adgif;
  DrawSettings settings;
  std::vector<ShrubVertex> vertices;
};

struct ShrubFrag {
  std::vector<ShrubDraw> draws;
};

struct ShrubInstanceInfo {
  u32 proto_idx;
  u32 color_idx;
  std::array<math::Vector4f, 4> mat;
  math::Vector4f bsphere;
  u16 wind_index;
};

struct ShrubProtoInfo {
  std::string name;
  u32 flags;
  std::vector<ShrubFrag> frags;
  std::vector<ShrubInstanceInfo> instances;
  float stiffness;
};

int get_or_add_vertex(const ShrubVertex& v,
                      std::vector<ShrubVertex>& unique_verts,
                      std::unordered_map<ShrubVertex, int, ShrubVertex::hash>& lookup) {
  auto it = lookup.find(v);
  if (it != lookup.end())
    return it->second;

  int idx = unique_verts.size();
  unique_verts.push_back(v);
  lookup[v] = idx;
  return idx;
}

std::string debug_dump_proto_to_obj(const ShrubProtoInfo& proto,
                                    const TextureDB& tdb,
                                    tfrag3::Level& lev) {
  //std::vector<math::Vector<float, 3>> verts;
  //std::vector<math::Vector<float, 2>> tcs;
  //std::vector<math::Vector<u8, 3>> cols;

  std::vector<ShrubVertex> unique_verts;
  std::unordered_map<ShrubVertex, int, ShrubVertex::hash> lookup;

  struct MatKey {
    u32 tex;
    u32 mode;

    bool operator==(const MatKey& o) const { return tex == o.tex && mode == o.mode; }
  };

  struct MatKeyHash {
    size_t operator()(const MatKey& k) const { return (size_t(k.tex) << 32) ^ size_t(k.mode); }
  };

  std::unordered_map<MatKey, std::vector<math::Vector<int, 3>>, MatKeyHash> grouped_faces;

  for (auto& frag : proto.frags) {
    for (auto& strip : frag.draws) {
      // what texture are we using?
      u32 combo_tex = strip.settings.combo_tex;

      // try looking it up in the existing textures that we have in the C++ renderer data.
      // (this is shared with tfrag)
      u32 idx_in_lev_data = UINT32_MAX;
      for (u32 i = 0; i < lev.textures.size(); i++) {
        if (lev.textures[i].combo_id == combo_tex) {
          idx_in_lev_data = i;
          break;
        }
      }

      if (idx_in_lev_data == UINT32_MAX) {
        // didn't find it, have to add a new one texture.
        auto tex_it = tdb.textures.find(combo_tex);
        if (tex_it == tdb.textures.end()) {
          bool ok_to_miss = false;  // for TIE, there's no missing textures.
          if (ok_to_miss) {
            // we're missing a texture, just use the first one.
            tex_it = tdb.textures.begin();
          } else {
            ASSERT_MSG(
                false,
                fmt::format(
                    "texture {} wasn't found. make sure it is loaded somehow. You may need to "
                    "include ART.DGO or GAME.DGO in addition to the level DGOs for shared "
                    "textures. tpage is {} id is {} (0x{:x})",
                    combo_tex, combo_tex >> 16, combo_tex & 0xffff, combo_tex & 0xffff));
          }
        }
        // add a new texture to the level data
        idx_in_lev_data = lev.textures.size();
        lev.textures.emplace_back();
        auto& new_tex = lev.textures.back();
        new_tex.combo_id = combo_tex;
        new_tex.w = tex_it->second.w;
        new_tex.h = tex_it->second.h;
        new_tex.debug_name = tex_it->second.name;
        new_tex.debug_tpage_name = tdb.tpage_names.at(tex_it->second.page);
        new_tex.data = tex_it->second.rgba_bytes;
      }

      MatKey key{idx_in_lev_data, strip.settings.mode.as_int()};

      // add verts...
      ASSERT(strip.vertices.size() >= 3);

      int vert_idx = 0;

      int vtx_idx_queue[3];

      int q_idx = 0;
      int startup = 0;
      while (vert_idx < (int)strip.vertices.size()) {
        ShrubVertex v;
        v.xyz = strip.vertices.at(vert_idx).xyz / 4096;
        v.st = strip.vertices.at(vert_idx).st / 4096;
        v.rgba_generic = strip.vertices.at(vert_idx).rgba_generic;

        int idx = get_or_add_vertex(v, unique_verts, lookup);

        idx++;
        vtx_idx_queue[q_idx++] = idx;

        // wrap the index
        if (q_idx == 3) {
          q_idx = 0;
        }

        // bump the startup
        if (startup < 3) {
          startup++;
        }

        if (startup >= 3 && strip.vertices.at(vert_idx).adc) {
          grouped_faces[key].push_back(
              math::Vector<int, 3>{vtx_idx_queue[0], vtx_idx_queue[1], vtx_idx_queue[2]});
        }
        vert_idx++;
      }
    }
  }

  std::string result;
  for (auto& v : unique_verts) {
    result += fmt::format("v {} {} {}\n", v.xyz.x(), v.xyz.y(), v.xyz.z());
  }
  for (auto& v : unique_verts) {
    auto& col = v.rgba_generic;
    result += fmt::format("vc {} {} {} {}\n", col.x(), col.y(), col.z(), 0);
  }
  for (auto& v : unique_verts) {
    result += fmt::format("vt {} {}\n", v.st.x(), v.st.y());
  }
  for (auto& [key, face_list] : grouped_faces) {
    result += fmt::format("d texture:{} drawmode:{}\n", key.tex, key.mode);

    for (auto& f : face_list) {
      result += fmt::format("f {} {} {}\n", f.x(), f.y(), f.z());
    }
  }

  return result;
}

namespace {
/*!
 * adgif shader texture id's can be "remapped". I think it allows textures to be shared.
 * So far we haven't seen this feature used, but we do have the texture map and we check it here.
 */
u32 remap_texture(u32 original, const std::vector<level_tools::TextureRemap>& map) {
  auto masked = original & 0xffffff00;
  for (auto& t : map) {
    if (t.original_texid == masked) {
      ASSERT_MSG(false, "OKAY! remapped!");
      return t.new_texid | 20;
    }
  }
  return original;
}
}  // namespace

DrawSettings adgif_to_draw_mode(const AdGifData& ad,
                                const TextureDB& tdb,
                                const std::vector<level_tools::TextureRemap>& map,
                                int count,
                                bool alpha_tpage_flag) {
  // initialize draw mode
  DrawMode current_mode;
  current_mode.set_at(true);
  current_mode.set_alpha_test(DrawMode::AlphaTest::GEQUAL);
  current_mode.set_aref(0x26);
  current_mode.set_alpha_fail(GsTest::AlphaFail::KEEP);
  current_mode.set_zt(true);
  current_mode.set_depth_test(GsTest::ZTest::GEQUAL);
  current_mode.set_depth_write_enable(true);  // todo, is this actual true
  current_mode.set_alpha_blend(DrawMode::AlphaBlend::SRC_SRC_SRC_SRC);
  current_mode.enable_fog();
  current_mode.set_ab(false);

  if (alpha_tpage_flag) {
    current_mode.set_alpha_test(DrawMode::AlphaTest::NEVER);
    current_mode.set_aref(0);
    current_mode.set_alpha_fail(GsTest::AlphaFail::FB_ONLY);
    current_mode.set_ab(true);
  }

  // ADGIF 0
  bool weird = (u8)ad.tex0_addr != (u32)GsRegisterAddress::TEX0_1;
  if (weird) {
    lg::info("----------------  WEIRD: 0x{:x}", ad.tex0_addr);
    lg::info("i have {} verts", count);
  } else {
    if (ad.tex0_data == 0) {
      current_mode.set_decal(false);
    } else if (ad.tex0_data == 0x8'0000'0000) {
      current_mode.set_decal(true);
    } else {
      ASSERT(false);
    }
  }

  // tw/th

  // ADGIF 1
  ASSERT((u8)ad.tex1_addr == (u32)GsRegisterAddress::TEX1_1);
  u32 original_tex = ad.tex1_addr;
  u32 new_tex = remap_texture(original_tex, map);
  // try remapping it
  if (original_tex != new_tex) {
    lg::info("map from 0x{:x} to 0x{:x}", original_tex, new_tex);
  }
  // texture the texture page/texture index, and convert to a PC port texture ID
  u32 tpage = new_tex >> 20;
  u32 tidx = (new_tex >> 8) & 0b1111'1111'1111;
  u32 tex_combo = (((u32)tpage) << 16) | tidx;
  // look up the texture to make sure it's valid
  auto tex = tdb.textures.find(tex_combo);
  ASSERT(tex != tdb.textures.end());
  if (weird) {
    lg::info("tex: {}", tex->second.name);
  }

  // ADGIF 2
  ASSERT((u8)ad.mip_addr == (u32)GsRegisterAddress::MIPTBP1_1);

  // ADGIF 3
  ASSERT((u8)ad.clamp_addr == (u32)GsRegisterAddress::CLAMP_1);
  {
    bool clamp_s = ad.clamp_data & 0b001;
    bool clamp_t = ad.clamp_data & 0b100;
    current_mode.set_clamp_s_enable(clamp_s);
    current_mode.set_clamp_t_enable(clamp_t);
  }

  u64 final_alpha;

  // ADGIF 4
  if ((u8)ad.alpha_addr == (u32)GsRegisterAddress::ALPHA_1) {
    final_alpha = ad.alpha_data;
  } else {
    ASSERT(false);
    // ASSERT((u8)ad.alpha_addr == (u32)GsRegisterAddress::MIPTBP2_1);
  }

  GsAlpha reg(final_alpha);
  auto a = reg.a_mode();
  auto b = reg.b_mode();
  auto c = reg.c_mode();
  auto d = reg.d_mode();
  if (a == GsAlpha::BlendMode::SOURCE && b == GsAlpha::BlendMode::DEST &&
      c == GsAlpha::BlendMode::SOURCE && d == GsAlpha::BlendMode::DEST) {
    current_mode.set_alpha_blend(DrawMode::AlphaBlend::SRC_DST_SRC_DST);
  } else if (a == GsAlpha::BlendMode::SOURCE && b == GsAlpha::BlendMode::ZERO_OR_FIXED &&
             c == GsAlpha::BlendMode::SOURCE && d == GsAlpha::BlendMode::DEST) {
    current_mode.set_alpha_blend(DrawMode::AlphaBlend::SRC_0_SRC_DST);
  } else if (a == GsAlpha::BlendMode::ZERO_OR_FIXED && b == GsAlpha::BlendMode::SOURCE &&
             c == GsAlpha::BlendMode::SOURCE && d == GsAlpha::BlendMode::DEST) {
    current_mode.set_alpha_blend(DrawMode::AlphaBlend::ZERO_SRC_SRC_DST);
  } else if (a == GsAlpha::BlendMode::SOURCE && b == GsAlpha::BlendMode::DEST &&
             c == GsAlpha::BlendMode::ZERO_OR_FIXED && d == GsAlpha::BlendMode::DEST) {
    current_mode.set_alpha_blend(DrawMode::AlphaBlend::SRC_DST_FIX_DST);
  } else if (a == GsAlpha::BlendMode::SOURCE && b == GsAlpha::BlendMode::SOURCE &&
             c == GsAlpha::BlendMode::SOURCE && d == GsAlpha::BlendMode::SOURCE) {
    current_mode.set_alpha_blend(DrawMode::AlphaBlend::SRC_SRC_SRC_SRC);
  } else if (a == GsAlpha::BlendMode::SOURCE && b == GsAlpha::BlendMode::ZERO_OR_FIXED &&
             c == GsAlpha::BlendMode::DEST && d == GsAlpha::BlendMode::DEST) {
    current_mode.set_alpha_blend(DrawMode::AlphaBlend::SRC_0_DST_DST);
  } else {
    // unsupported blend: a 0 b 2 c 2 d 1
    // lg::error("unsupported blend: a {} b {} c {} d {}", (int)a, (int)b, (int)c, (int)d);
    //      ASSERT(false);
  }

  return {current_mode, tex_combo};
}

ShrubProtoInfo extract_proto(const shrub_types::PrototypeBucketShrub& proto,
                             const TextureDB& tdb,
                             const std::vector<level_tools::TextureRemap>& map,
                             const fs::path& output_dir,
                             tfrag3::Level& lev,
                             GameVersion version) {
  ShrubProtoInfo result;
  result.name = proto.name;
  result.stiffness = proto.stiffness;
  result.flags = proto.flags;
  for (int frag_idx = 0; frag_idx < proto.generic_geom.length; frag_idx++) {
    auto& frag_out = result.frags.emplace_back();
    auto& frag = proto.generic_geom.shrubs.at(frag_idx);

    std::vector<AdGifData> adgif_data;
    adgif_data.resize(frag.textures.size() / sizeof(AdGifData));
    memcpy(adgif_data.data(), frag.textures.data(), frag.textures.size());

    if (frag_idx == 0 && proto.name == "vil2-cattail.mb") {
      lg::info("Skipping broken village2 thing");
      continue;
    }

    for (size_t i = 0; i < adgif_data.size(); i++) {
      auto& draw = frag_out.draws.emplace_back();
      draw.adgif = adgif_data[i];

      const auto& ag = adgif_data[i];
      int count = (ag.tex1_addr >> 32) & 0xfff;  // drop the eop flag
      draw.start_vtx_idx = ((ag.tex0_addr >> 32) & 0xffff) / 3;

      if (i > 0) {
        auto& prev_draw = frag_out.draws[frag_out.draws.size() - 2];
        ASSERT(prev_draw.start_vtx_idx + prev_draw.vertices.size() + 3 == draw.start_vtx_idx);
      }

      for (int vert_idx = 0; vert_idx < count; vert_idx++) {
        auto& vert_out = draw.vertices.emplace_back();
        s16 vert_data[3];
        memcpy(vert_data, frag.vtx.data() + sizeof(u16) * 3 * (vert_idx + draw.start_vtx_idx),
               3 * sizeof(u16));
        vert_out.xyz = math::Vector3f(vert_data[0], vert_data[1], vert_data[2]);

        s16 st_data[2];
        memcpy(st_data, frag.stq.data() + sizeof(u16) * 2 * (vert_idx + draw.start_vtx_idx),
               2 * sizeof(u16));
        vert_out.st = math::Vector2f(st_data[0], st_data[1]);
        vert_out.adc = (st_data[0] & 1) == 0;  // adc in the low bit of texture coordinate

        memcpy(vert_out.rgba_generic.data(), frag.col.data() + 3 * (vert_idx + draw.start_vtx_idx),
               3);
        ASSERT(3 * (vert_idx + draw.start_vtx_idx) + 3 <= frag.col.size());
      }

      bool alpha_tpage_flag = false;
      if (version > GameVersion::Jak1) {
        alpha_tpage_flag = proto.flags & 0x4;  // tpage-alpha
      }
      draw.settings = adgif_to_draw_mode(ag, tdb, map, count, alpha_tpage_flag);
    }

    ASSERT(frag.vtx_cnt * 3 * sizeof(u16) <= frag.vtx.size());
  }

  if (proto.stiffness == 0)
    return result;

  auto file_path = output_dir / fmt::format("{}.obj", proto.name);
  file_util::write_text_file(
      file_path,
      debug_dump_proto_to_obj(result, tdb, lev));
      
  return result;
}

void extract_instance(const shrub_types::InstanceShrubbery& inst,
                      std::vector<ShrubProtoInfo>& protos) {
  ShrubInstanceInfo result;
  result.proto_idx = inst.bucket_index;
  for (int i = 0; i < 4; i++) {
    result.bsphere[i] = inst.bsphere.data[i];
  }

  // from ee asm
  result.mat = extract_shrub_matrix(inst.origin.data);
  result.mat[3][0] += result.bsphere[0];
  result.mat[3][1] += result.bsphere[1];
  result.mat[3][2] += result.bsphere[2];
  result.wind_index = inst.wind_index;

  result.mat[0][3] = 0.f;
  result.color_idx = inst.color_indices / 4;

  protos.at(result.proto_idx).instances.push_back(result);
}

/*!
 * Transform a point in a prototype to the actual point location in the game world.
 */
math::Vector<float, 3> transform_shrub(const std::array<math::Vector4f, 4> mat,
                                       const math::Vector3f& pt) {
  auto temp = mat[0] * pt.x() + mat[1] * pt.y() + mat[2] * pt.z() + mat[3];
  math::Vector3f result;
  result.x() = temp.x();
  result.y() = temp.y();
  result.z() = temp.z();
  return result;
}

struct ShrubAllVertex {
  math::Vector<float, 3> xyz;
  math::Vector<float, 2> st;
  math::Vector<u8, 3> rgba_generic;
  u32 tod_index;
  bool adc = false;

  bool operator==(const ShrubAllVertex& other) const {
    return xyz.x() == other.xyz.x() &&
           xyz.y() == other.xyz.y() &&
           xyz.z() == other.xyz.z() &&
           st.x() == other.st.x() &&
           st.y() == other.st.y() &&
           rgba_generic.x() == other.rgba_generic.x() &&
           rgba_generic.y() == other.rgba_generic.y() &&
           rgba_generic.z() == other.rgba_generic.z() &&
           tod_index == other.tod_index;
  }

  struct hash {
    std::size_t operator()(const ShrubAllVertex& x) const {
      return std::hash<float>()(x.xyz.x()) ^
             std::hash<float>()(x.xyz.y()) ^
             std::hash<float>()(x.xyz.z()) ^
             std::hash<float>()(x.st.x()) ^
             std::hash<float>()(x.st.y()) ^
             std::hash<u8>()(x.rgba_generic.x()) ^
             std::hash<u8>()(x.rgba_generic.y()) ^
             std::hash<u8>()(x.rgba_generic.z()) ^
             std::hash<u32>()(x.tod_index);
    }
  };
};

int get_or_add_vertex(const ShrubAllVertex& v,
                      std::vector<ShrubAllVertex>& unique_verts,
                      std::unordered_map<ShrubAllVertex, int, ShrubAllVertex::hash>& lookup) {
  auto it = lookup.find(v);
  if (it != lookup.end())
    return it->second;

  int idx = unique_verts.size();
  unique_verts.push_back(v);
  lookup[v] = idx;
  return idx;
}

/*!
 * Dump the entire tie tree to an obj. Used to debug the transform_tie function. If we get this
 * right, it should fit in with .obj's produced from the tfrag debug.
 */
std::string dump_full_to_obj(const std::vector<ShrubProtoInfo>& protos,
                             const TextureDB& tdb,
                             tfrag3::Level& lev) {
  //std::vector<math::Vector<float, 3>> verts;
  //std::vector<math::Vector<int, 3>> faces;

  std::vector<ShrubAllVertex> unique_verts;
  std::unordered_map<ShrubAllVertex, int, ShrubAllVertex::hash> lookup;

  struct MatKey {
    u32 tex;
    u32 mode;

    bool operator==(const MatKey& o) const { return tex == o.tex && mode == o.mode; }
  };

  struct MatKeyHash {
    size_t operator()(const MatKey& k) const { return (size_t(k.tex) << 32) ^ size_t(k.mode); }
  };

  std::unordered_map<MatKey, std::vector<math::Vector<int, 3>>, MatKeyHash> grouped_faces;

  for (auto& proto : protos) {
    if (proto.stiffness != 0)
      continue;
    for (auto& inst : proto.instances) {
      auto& mat = inst.mat;
      for (auto& frag : proto.frags) {
        for (auto& strip : frag.draws) {
          // what texture are we using?
          u32 combo_tex = strip.settings.combo_tex;

          // try looking it up in the existing textures that we have in the C++ renderer data.
          // (this is shared with tfrag)
          u32 idx_in_lev_data = UINT32_MAX;
          for (u32 i = 0; i < lev.textures.size(); i++) {
            if (lev.textures[i].combo_id == combo_tex) {
              idx_in_lev_data = i;
              break;
            }
          }

          if (idx_in_lev_data == UINT32_MAX) {
            // didn't find it, have to add a new one texture.
            auto tex_it = tdb.textures.find(combo_tex);
            if (tex_it == tdb.textures.end()) {
              bool ok_to_miss = false;  // for TIE, there's no missing textures.
              if (ok_to_miss) {
                // we're missing a texture, just use the first one.
                tex_it = tdb.textures.begin();
              } else {
                ASSERT_MSG(
                    false,
                    fmt::format(
                        "texture {} wasn't found. make sure it is loaded somehow. You may need to "
                        "include ART.DGO or GAME.DGO in addition to the level DGOs for shared "
                        "textures. tpage is {} id is {} (0x{:x})",
                        combo_tex, combo_tex >> 16, combo_tex & 0xffff, combo_tex & 0xffff));
              }
            }
            // add a new texture to the level data
            idx_in_lev_data = lev.textures.size();
            lev.textures.emplace_back();
            auto& new_tex = lev.textures.back();
            new_tex.combo_id = combo_tex;
            new_tex.w = tex_it->second.w;
            new_tex.h = tex_it->second.h;
            new_tex.debug_name = tex_it->second.name;
            new_tex.debug_tpage_name = tdb.tpage_names.at(tex_it->second.page);
            new_tex.data = tex_it->second.rgba_bytes;
          }

          MatKey key{idx_in_lev_data, strip.settings.mode.as_int()};

          // add verts...
          ASSERT(strip.vertices.size() >= 3);

          int vert_idx = 0;

          int vtx_idx_queue[3];

          int q_idx = 0;
          int startup = 0;
          while (vert_idx < (int)strip.vertices.size()) {
            ShrubAllVertex v;
            v.xyz = transform_shrub(mat, strip.vertices.at(vert_idx).xyz) / 4096;
            v.st = strip.vertices.at(vert_idx).st / 4096;
            v.rgba_generic = strip.vertices.at(vert_idx).rgba_generic;
            v.tod_index = inst.color_idx;

            int idx = get_or_add_vertex(v, unique_verts, lookup);

            idx++;
            vtx_idx_queue[q_idx++] = idx;

            // wrap the index
            if (q_idx == 3) {
              q_idx = 0;
            }

            // bump the startup
            if (startup < 3) {
              startup++;
            }

            if (startup >= 3 && strip.vertices.at(vert_idx).adc) {
              grouped_faces[key].push_back(
                  math::Vector<int, 3>{vtx_idx_queue[0], vtx_idx_queue[1], vtx_idx_queue[2]});
            }
            vert_idx++;
          }
        }
      }
    }
  }

  std::string result;
  for (auto& v : unique_verts) {
    result += fmt::format("v {} {} {}\n", v.xyz.x(), v.xyz.y(), v.xyz.z());
  }
  for (auto& v : unique_verts) {
    result += fmt::format("vt {} {}\n", v.st.x(), v.st.y());
  }
  for (auto& v : unique_verts) {
    result += fmt::format("vp {}\n", v.tod_index);
  }
  for (auto& v : unique_verts) {
    auto& col = v.rgba_generic;
    result += fmt::format("vc {} {} {} {}\n", col.x(), col.y(), col.z(), 0);
  }
  for (auto& [key, face_list] : grouped_faces) {
    result += fmt::format("d texture:{} drawmode:{}\n", key.tex, key.mode);

    for (auto& f : face_list) {
      result += fmt::format("f {} {} {}\n", f.x(), f.y(), f.z());
    }
  }

  return result;
}

void make_draws(tfrag3::Level& lev,
                tfrag3::ShrubTree& tree_out,
                const std::vector<ShrubProtoInfo>& protos,
                const TextureDB& tdb) {
  std::vector<std::vector<u32>> indices_regrouped_by_draw;
  std::unordered_map<u32, std::vector<u32>> static_draws_by_tex;
  size_t global_vert_counter = 0;
  for (u32 proto_idx = 0; proto_idx < protos.size(); proto_idx++) {
    auto& proto = protos[proto_idx];
    // packed_vert_indices[frag][draw] = {start, end}
    std::vector<std::vector<std::pair<int, int>>> packed_vert_indices;

    for (size_t frag_idx = 0; frag_idx < proto.frags.size(); frag_idx++) {
      auto& frag_inds = packed_vert_indices.emplace_back();
      auto& frag = proto.frags[frag_idx];

      for (auto& draw : frag.draws) {
        int start = tree_out.packed_vertices.vertices.size();
        for (auto& vert : draw.vertices) {
          tree_out.packed_vertices.vertices.push_back(
              {vert.xyz.x(),
               vert.xyz.y(),
               vert.xyz.z(),
               vert.st.x(),
               vert.st.y(),
               {vert.rgba_generic[0], vert.rgba_generic[1], vert.rgba_generic[2]}});
        }
        int end = tree_out.packed_vertices.vertices.size();
        frag_inds.emplace_back(start, end);
      }
    }

    for (auto& inst : proto.instances) {
      u32 matrix_idx = tree_out.packed_vertices.matrices.size();
      tree_out.packed_vertices.matrices.push_back(inst.mat);

      for (size_t frag_idx = 0; frag_idx < proto.frags.size(); frag_idx++) {
        auto& frag = proto.frags[frag_idx];  // shared info for all instances of this frag

        for (size_t draw_idx = 0; draw_idx < frag.draws.size(); draw_idx++) {
          auto& draw = frag.draws[draw_idx];

          // what texture are we using?
          u32 combo_tex = draw.settings.combo_tex;

          // try looking it up in the existing textures that we have in the C++ renderer data.
          // (this is shared with tfrag)
          u32 idx_in_lev_data = UINT32_MAX;
          for (u32 i = 0; i < lev.textures.size(); i++) {
            if (lev.textures[i].combo_id == combo_tex) {
              idx_in_lev_data = i;
              break;
            }
          }

          if (idx_in_lev_data == UINT32_MAX) {
            // didn't find it, have to add a new one texture.
            auto tex_it = tdb.textures.find(combo_tex);
            if (tex_it == tdb.textures.end()) {
              bool ok_to_miss = false;  // for TIE, there's no missing textures.
              if (ok_to_miss) {
                // we're missing a texture, just use the first one.
                tex_it = tdb.textures.begin();
              } else {
                ASSERT_MSG(
                    false,
                    fmt::format(
                        "texture {} wasn't found. make sure it is loaded somehow. You may need to "
                        "include ART.DGO or GAME.DGO in addition to the level DGOs for shared "
                        "textures. tpage is {} id is {} (0x{:x})",
                        combo_tex, combo_tex >> 16, combo_tex & 0xffff, combo_tex & 0xffff));
              }
            }
            // add a new texture to the level data
            idx_in_lev_data = lev.textures.size();
            lev.textures.emplace_back();
            auto& new_tex = lev.textures.back();
            new_tex.combo_id = combo_tex;
            new_tex.w = tex_it->second.w;
            new_tex.h = tex_it->second.h;
            new_tex.debug_name = tex_it->second.name;
            new_tex.debug_tpage_name = tdb.tpage_names.at(tex_it->second.page);
            new_tex.data = tex_it->second.rgba_bytes;
          }

          DrawMode mode = draw.settings.mode;

          // okay, we now have a texture and draw mode, let's see if we can add to an existing...
          auto existing_draws_in_tex = static_draws_by_tex.find(idx_in_lev_data);
          tfrag3::ShrubDraw* draw_to_add_to = nullptr;
          std::vector<u32>* verts_to_add_to = nullptr;
          if (existing_draws_in_tex != static_draws_by_tex.end()) {
            for (auto idx : existing_draws_in_tex->second) {
              auto& candidate_draw_out = tree_out.static_draws.at(idx);
              if (candidate_draw_out.mode == mode && (!tree_out.has_per_proto_visibility_toggle ||
                                                      candidate_draw_out.proto_idx == proto_idx)) {
                draw_to_add_to = &tree_out.static_draws[idx];
                verts_to_add_to = &indices_regrouped_by_draw[idx];
              }
            }
          }

          if (!draw_to_add_to) {
            // nope, need to create a new draw
            tree_out.static_draws.emplace_back();
            static_draws_by_tex[idx_in_lev_data].push_back(tree_out.static_draws.size() - 1);
            draw_to_add_to = &tree_out.static_draws.back();
            draw_to_add_to->mode = mode;
            draw_to_add_to->tree_tex_id = idx_in_lev_data;
            draw_to_add_to->proto_idx = proto_idx;
            verts_to_add_to = &indices_regrouped_by_draw.emplace_back();
          }

          // now we have a draw, time to add vertices
          // draw_to_add_to->num_triangles += draw.vertices.size() - 2;
          tfrag3::PackedShrubVertices::InstanceGroup grp;
          grp.matrix_idx = matrix_idx;
          grp.color_index = inst.color_idx;
          grp.start_vert = packed_vert_indices.at(frag_idx).at(draw_idx).first;
          grp.end_vert = packed_vert_indices.at(frag_idx).at(draw_idx).second;
          grp.wind_idx = inst.wind_index;
          grp.stiffness = proto.stiffness;
          tree_out.packed_vertices.instance_groups.push_back(grp);

          for (size_t vidx = 0; vidx < draw.vertices.size(); vidx++) {
            if (draw.vertices[vidx].adc) {
              verts_to_add_to->push_back(vidx + global_vert_counter);
              draw_to_add_to->num_triangles++;
            } else {
              verts_to_add_to->push_back(UINT32_MAX);
              verts_to_add_to->push_back(vidx + global_vert_counter - 1);
              verts_to_add_to->push_back(vidx + global_vert_counter);
            }
          }
          verts_to_add_to->push_back(UINT32_MAX);
          global_vert_counter += draw.vertices.size();
        }
      }
    }
  }

  for (size_t didx = 0; didx < tree_out.static_draws.size(); didx++) {
    auto& draw = tree_out.static_draws[didx];
    auto& inds = indices_regrouped_by_draw[didx];
    draw.num_triangles = clean_up_vertex_indices(inds);
    draw.num_indices = inds.size();
    draw.first_index_index = tree_out.indices.size();
    tree_out.indices.insert(tree_out.indices.end(), inds.begin(), inds.end());
  }

  tree_out.packed_vertices.total_vertex_count = global_vert_counter;
}

nlohmann::json vectorm_shrub_json(const math::Vector4f v) {
  nlohmann::json result;
  for (int i = 0; i < 4; i++) {
    result.push_back(v[i] / 4096.f);
  }
  return result;
}

auto to_shrub_json_matrix = [](const std::array<math::Vector4f, 4>& mat) {
  nlohmann::json j = nlohmann::json::array();
  for (int i = 0; i < 4; i++) {
    if (i != 3)
      j.push_back({mat[i].x(), mat[i].y(), mat[i].z(), mat[i].w()});
    else
      j.push_back({mat[i].x() / 4096.0f, mat[i].y() / 4096.0f, mat[i].z() / 4096.0f, mat[i].w()});
  }
  return j;
};

nlohmann::json extract_shrub(const shrub_types::DrawableTreeInstanceShrub* tree,
                             const std::string& debug_name,
                             const std::vector<level_tools::TextureRemap>& map,
                             const TextureDB& tex_db,
                             const std::vector<std::pair<int, int>>& /*expected_missing_textures*/,
                             tfrag3::Level& out,
                             bool dump_level,
                             GameVersion version) {
  nlohmann::json shrub_tree_info;
  //shrub_tree_info = nlohmann::json::array();

  auto& tree_out = out.shrub_trees.emplace_back();

  if (version > GameVersion::Jak1) {
    tree_out.has_per_proto_visibility_toggle = true;
  }

  auto& protos = tree->info.prototype_inline_array_shrub;
  std::vector<ShrubProtoInfo> proto_info;
  auto obj_path = file_util::get_jak_project_dir() / "decompiler_out" /
                  game_version_names[version] / "levels" / out.level_name /
                  fmt::format("{}-background", out.level_name);
  auto shrub_path = obj_path / "shrub";
  auto shrub_proto_path = shrub_path / "protos";
  file_util::create_dir_if_needed(obj_path);
  file_util::create_dir_if_needed(shrub_path);
  file_util::create_dir_if_needed(shrub_proto_path);
  for (auto& proto : protos.data) {
    proto_info.push_back(extract_proto(proto, tex_db, map, shrub_proto_path, out, version));
    tree_out.proto_names.push_back(proto.name);
  }

  for (auto& arr : tree->discovered_arrays) {
    auto as_shrubs = dynamic_cast<shrub_types::DrawableInlineArrayInstanceShrub*>(arr.get());
    if (as_shrubs) {
      for (auto& inst : as_shrubs->instances) {
        extract_instance(inst, proto_info);
      }
    }
  }
  shrub_tree_info["protos"] = nlohmann::json::array();
  for (auto& proto : proto_info) {
    if (proto.stiffness == 0)
      continue;
    nlohmann::json proto_json;
    proto_json["name"] = proto.name;
    proto_json["flags"] = proto.flags;
    proto_json["stiffness"] = proto.stiffness;
    proto_json["instances"] = nlohmann::json::array();
    for (auto& inst : proto.instances) {
      nlohmann::json inst_json;
      inst_json["bsphere"] = vectorm_shrub_json(inst.bsphere);
      inst_json["proto-idx"] = inst.proto_idx;
      inst_json["color-idx"] = inst.color_idx;
      inst_json["wind-index"] = inst.wind_index;
      inst_json["mat"] = to_shrub_json_matrix(inst.mat);
      proto_json["instances"].push_back(inst_json);
    }
    shrub_tree_info["protos"].push_back(proto_json);
  }

  //file_util::write_text_file(obj_path / fmt::format("{}_info.json", debug_name),
  //                           shrub_tree_info.dump(2));

  // time of day colors
  tree_out.time_of_day_colors = pack_colors(tree->time_of_day);

  make_draws(out, tree_out, proto_info, tex_db);

  if (dump_level) {
    std::string allShrub = dump_full_to_obj(proto_info,tex_db, out);
    std::string idx = debug_name.substr(debug_name.find_first_of('-') + 1, 1);
    file_util::write_text_file(shrub_path / fmt::format("shrub-{}.obj", idx), allShrub);
  }
  return shrub_tree_info;
}
}  // namespace decompiler
