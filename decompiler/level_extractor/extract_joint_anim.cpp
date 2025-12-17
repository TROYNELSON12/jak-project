#include "extract_joint_anim.h"

#include "common/math/geometry.h"

#include "decompiler/util/goal_data_reader.h"

#include "common/log/log.h"
#include "common/util/FileUtil.h"

#include "third-party/json.hpp"

#include <algorithm>
#include <string>

namespace {
// Minimal base64 encoder (no newlines, padded with '=')
std::string base64_encode(const u8* data, size_t len) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((len + 2) / 3) * 4);
  for (size_t i = 0; i < len; i += 3) {
    const u32 b0 = data[i];
    const u32 b1 = (i + 1 < len) ? data[i + 1] : 0;
    const u32 b2 = (i + 2 < len) ? data[i + 2] : 0;
    const u32 chunk = (b0 << 16) | (b1 << 8) | b2;
    out.push_back(kAlphabet[(chunk >> 18) & 0x3F]);
    out.push_back(kAlphabet[(chunk >> 12) & 0x3F]);
    out.push_back(i + 1 < len ? kAlphabet[(chunk >> 6) & 0x3F] : '=');
    out.push_back(i + 2 < len ? kAlphabet[chunk & 0x3F] : '=');
  }
  return out;
}

std::string encode_vec4_base64(const math::Vector4f* data, size_t count) {
  return base64_encode(reinterpret_cast<const u8*>(data), count * sizeof(math::Vector4f));
}
}  // namespace

namespace decompiler {

void extract_joint_anim(const ObjectFileData& ag_data,
                        const DecompilerTypeSystem& dts,
                        GameVersion version,
                        std::map<std::string, level_tools::ArtData>& out,
                        bool isStream) {
  if (version == GameVersion::Jak1) {
    auto locations = find_objects_with_type(ag_data.linked_data, "art-joint-anim");
    if (isStream && locations.empty()) {
      lg::info("No art-joint-anim found in stream object: {}", ag_data.name_in_dgo);
    }
    for (auto loc : locations) {
      TypedRef ref(Ref{&ag_data.linked_data, 0, loc * 4}, dts.ts.lookup_type("art-joint-anim"));

      std::string name = read_string_field(ref, "name", dts, false);
    
      // Use reference to directly modify the map entry
      // For streams with multiple characters, include character name in key to avoid overwriting
      std::string key = isStream ? (name + "-" + ag_data.name_in_dgo) : name;
      auto& art_data = out[key];

      art_data.art_name = name;
      art_data.art_group_name = ag_data.name_in_dgo;
    
      auto& anim = art_data.joint_anims.emplace_back();
    
      // Extract fields from art base type
      anim.name = read_string_field(ref, "name", dts, false);
      anim.length = read_plain_data_field<int32_t>(ref, "length", dts);
      
      if (isStream) {
        lg::info("  Found stream anim: {} in {} (length: {})", anim.name, ag_data.name_in_dgo, anim.length);
      }

      if (anim.name == "sidekick-death-0202")
        lg::info("gottem");
      
    
      // Extract eye-anim-data (optional pointer to merc-eye-anim-block)
      if (get_word_kind_for_field(ref, "eye-anim-data", dts) == LinkedWord::PTR) {
        Ref eye_data_ref = deref_label(get_field_ref(ref, "eye-anim-data", dts));
        TypedRef teye_ref(eye_data_ref, dts.ts.lookup_type("merc-eye-anim-block"));
    
        anim.eye_anim_data.max_frame = read_plain_data_field<u16>(teye_ref, "max-frame", dts);
    
        // Access the data array field (merc-eye-anim-frame array)
        Ref eye_frames_ref = get_field_ref(teye_ref, "data", dts);
        const int eye_frame_count = (anim.eye_anim_data.max_frame + 1) * 2;
        anim.eye_anim_data.data.reserve(eye_frame_count);
        for (int i = 0; i < eye_frame_count; i++) {
          TypedRef frame_ref(eye_frames_ref, dts.ts.lookup_type("merc-eye-anim-frame"));
    
          auto& frame = anim.eye_anim_data.data.emplace_back();
          frame.pupil_trans_x = read_plain_data_field<s8>(frame_ref, "pupil-trans-x", dts);
          frame.pupil_trans_y = read_plain_data_field<s8>(frame_ref, "pupil-trans-y", dts);
          frame.blink = read_plain_data_field<s8>(frame_ref, "blink", dts);
          frame.iris_scale = read_plain_data_field<s8>(frame_ref, "iris-scale", dts);
          frame.pupil_scale = read_plain_data_field<s8>(frame_ref, "pupil-scale", dts);
          frame.lid_scale = read_plain_data_field<s8>(frame_ref, "lid-scale", dts);
    
          // Advance to next frame
          eye_frames_ref.byte_offset += dts.ts.lookup_type("merc-eye-anim-frame")->get_size_in_memory();
        }
      }

    
      // Extract basic fields
      anim.speed = read_plain_data_field<float>(ref, "speed", dts);
      anim.artist_base = read_plain_data_field<float>(ref, "artist-base", dts);
      anim.artist_step = read_plain_data_field<float>(ref, "artist-step", dts);
      anim.master_art_group_name = read_string_field(ref, "master-art-group-name", dts, false);
      anim.master_art_group_index = read_plain_data_field<int32_t>(ref, "master-art-group-index", dts);
      
    
      // Extract frames (need to do this before blerc-data)
      Ref frames_ref = deref_label(get_field_ref(ref, "frames", dts));
      TypedRef tframe(frames_ref, dts.ts.lookup_type("joint-anim-compressed-control"));
      anim.frames.num_frames = read_plain_data_field<u32>(tframe, "num-frames", dts);
      anim.frames.fixed_qwc = read_plain_data_field<u32>(tframe, "fixed-qwc", dts);
      anim.frames.frame_qwc = read_plain_data_field<u32>(tframe, "frame-qwc", dts);
      
      if (isStream) {
        lg::info("    Stream anim has {} frames", anim.frames.num_frames);
      }

      Ref fixed_ref = deref_label(get_field_ref(tframe, "fixed", dts));
      TypedRef tfixed(fixed_ref, dts.ts.lookup_type("joint-anim-compressed-fixed"));

      Ref hdr_ref = get_field_ref(tfixed, "hdr", dts);
      TypedRef thdr(hdr_ref, dts.ts.lookup_type("joint-anim-compressed-hdr"));

      Ref control_bits_ref = get_field_ref(thdr, "control-bits", dts);
      memcpy_from_plain_data((u8*)anim.frames.fixed.hdr.control_bits, control_bits_ref, 14 * sizeof(u32));
      anim.frames.fixed.hdr.num_joints = read_plain_data_field<u32>(thdr, "num-joints", dts);
      anim.frames.fixed.hdr.matrix_bits = read_plain_data_field<u32>(thdr, "matrix-bits", dts);
      
      anim.frames.fixed.offset_64 = read_plain_data_field<u32>(tfixed, "offset-64", dts);
      anim.frames.fixed.offset_32 = read_plain_data_field<u32>(tfixed, "offset-32", dts);
      anim.frames.fixed.offset_16 = read_plain_data_field<u32>(tfixed, "offset-16", dts);
      anim.frames.fixed.reserved = read_plain_data_field<u32>(tfixed, "reserved", dts);
      
      u32 fixed_data_qwc = anim.frames.fixed_qwc > 5 ? (anim.frames.fixed_qwc - 5) : 0;
      Ref fixed_data_ref = get_field_ref(tfixed, "data", dts);
      u32 fixed_data_vectors = std::min(fixed_data_qwc, 133u);
      memcpy_from_plain_data((u8*)anim.frames.fixed.data, fixed_data_ref, fixed_data_vectors * sizeof(math::Vector4f));

      // Extract per-frame compressed data array
      Ref frames_data_ref = get_field_ref(tframe, "data", dts);
      for (u32 frame_idx = 0; frame_idx < anim.frames.num_frames; frame_idx++) {
        Ref frame_ref = deref_label(frames_data_ref);
        TypedRef tframe_ref(frame_ref, dts.ts.lookup_type("joint-anim-compressed-frame"));

        auto& frame = anim.frames.frames.emplace_back();
        frame.offset_64 = read_plain_data_field<u32>(tframe_ref, "offset-64", dts);
        frame.offset_32 = read_plain_data_field<u32>(tframe_ref, "offset-32", dts);
        frame.offset_16 = read_plain_data_field<u32>(tframe_ref, "offset-16", dts);
        frame.reserved = read_plain_data_field<u32>(tframe_ref, "reserved", dts);

        // Copy frame payload (frame_qwc includes the 1 qword header with offsets, subtract it)
        Ref frame_data_ref = get_field_ref(tframe_ref, "data", dts);
        u32 frame_data_qwc = anim.frames.frame_qwc - 1;
        u32 frame_data_vectors = std::min(frame_data_qwc, 133u);
        memcpy_from_plain_data((u8*)frame.data,
                               frame_data_ref,
                               frame_data_vectors * sizeof(math::Vector4f));

        // advance to next frame pointer in the dynamic array
        frames_data_ref.byte_offset += sizeof(u32);
      }
    
      // Extract data
      Ref data_ref = get_field_ref(ref, "data", dts);
      for (int i = 0; i < anim.length; i++) {
        auto& compressed_anim = anim.data.emplace_back();
    
        Ref janim_ref = deref_label(data_ref);
        janim_ref.byte_offset -= 4;  // Adjust for type tag of basic
        TypedRef tjanim = typed_ref_from_basic(janim_ref, dts);
    
        compressed_anim.name = read_string_field(tjanim, "name", dts, true);
        compressed_anim.number = read_plain_data_field<int16_t>(tjanim, "number", dts);
        compressed_anim.length = read_plain_data_field<int16_t>(tjanim, "length", dts);
    
        // Move to next pointer in the array
        data_ref.byte_offset += sizeof(u32);
      }

      // Extract blerc-data (optional pointer to uint8 array)
      // blerc-data contains per-frame blend shape weights for facial animation
      // Format: array of uint8, organized as [frame_0_targets...][frame_1_targets...]
      // Each frame has one uint8 per blend target (from merc-ctrl header blend-target-count)
      // Values are offset: stored_value = actual_weight + 64

      const int num_frames = anim.frames.num_frames;
      Ref blerc_field_ref = get_field_ref(ref, "blerc-data", dts);
      auto blerc_kind = get_word_kind_for_field(ref, "blerc-data", dts);

      // Use art_group_name (which is ag_data.name_in_dgo) to safely look up blerc count
      int blend_target_count = out[anim.master_art_group_name].blerc_blend_target_count;

      if (blend_target_count > 0 && num_frames > 0) {
        // blerc-data is a (pointer uint8), but can also be a symbol.
        // The symbol is typically '#f' (false/null), meaning no blerc data is available.
        // Typically a SYM_PTR when no blerc, becomes PTR when blerc.
        if (blerc_kind == decompiler::LinkedWord::PTR) {
          // Direct pointer to byte array of blend shape weights
          Ref blerc_data_ref = deref_label(blerc_field_ref);
          
          anim.blerc_blend_target_count = blend_target_count;
          const int total_bytes = num_frames * blend_target_count;
          anim.blerc_data.resize(num_frames);
          anim.blerc_flat.assign(total_bytes, 0);
          
          for (int frame = 0; frame < num_frames; frame++) {
            anim.blerc_data[frame].resize(blend_target_count);
            memcpy_from_plain_data(anim.blerc_data[frame].data(), blerc_data_ref, blend_target_count);
            std::copy(anim.blerc_data[frame].begin(), anim.blerc_data[frame].end(),
                      anim.blerc_flat.begin() + (frame * blend_target_count));
            blerc_data_ref.byte_offset += blend_target_count;
          }
          
          lg::info("  Extracted blerc-data: {} frames × {} targets", num_frames, blend_target_count);
        } else if (blerc_kind == decompiler::LinkedWord::SYM_PTR) {
          // Symbol pointer, typically '#f' when no data
          anim.blerc_blend_target_count = 0;
        }
      }

      //logging
      if (false) {
        lg::info("Found art-joint-anim of {} at location {}", name, loc);

        //art base type
        lg::info("  length: {}", anim.length);

        //eye-anim-block
        lg::info("  eye max-frame: {}", anim.eye_anim_data.max_frame);

        //basic
        lg::info("  speed: {}", anim.speed);
        lg::info("  artist-base: {}", anim.artist_base);
        lg::info("  artist-step: {}", anim.artist_step);
        lg::info("  master-art-group-name: {}", anim.master_art_group_name);
        lg::info("  master-art-group-index: {}", anim.master_art_group_index);

        // blerc data
        if (anim.blerc_blend_target_count > 0 && !anim.blerc_data.empty()) {
          lg::info("  blerc-data: {} frames × {} targets", 
                   anim.blerc_data.size(), anim.blerc_blend_target_count);
        }

        //frames
        lg::info("  frames: num-frames={}, fixed-qwc={}, frame-qwc={}", 
                 anim.frames.num_frames, anim.frames.fixed_qwc, anim.frames.frame_qwc);
        lg::info("    fixed: num-joints={}, matrix-bits=0x{:x}", 
                 anim.frames.fixed.hdr.num_joints, anim.frames.fixed.hdr.matrix_bits);
        lg::info("    extracted {} frame data arrays", anim.frames.frames.size());

        //data
        for (int i = 0; i < anim.length; i++)
          lg::info("    joint-anim-compressed[{}]: name={}, number={}, length={}",
                   i, anim.data[i].name, anim.data[i].number, anim.data[i].length);
      }
    }
  } else {
    lg::warn("Unsupported game for art-joint-anim extraction.");
  }
}

void export_anim_as_json(const std::string dgo_name,
                         GameVersion version,
                         std::map<std::string, level_tools::ArtData>& anims,
                         bool isStream) {
  // Serialize animations to JSON for debugging
  if (version == GameVersion::Jak1) {
    for (const auto& entry : anims) {
      if (!entry.second.joint_anims.empty()) {
        const auto& anim = entry.second.joint_anims[0];

        nlohmann::json json;
        //art
        json["name"] = anim.name;
        json["length"] = anim.length;


        //eye-anim-data
        if (anim.eye_anim_data.max_frame > 0) {
          json["eye-anim-data"]["max-frame"] = anim.eye_anim_data.max_frame;
          json["eye-anim-data"]["data"] = nlohmann::json::array();
          for (int i = 0; i < anim.eye_anim_data.max_frame * 2; i++) {
            nlohmann::json data;
            data["pupil-trans-x"] = anim.eye_anim_data.data[i].pupil_trans_x;
            data["pupil-trans-y"] = anim.eye_anim_data.data[i].pupil_trans_y;
            data["blink"] = anim.eye_anim_data.data[i].blink;
            data["iris-scale"] = anim.eye_anim_data.data[i].iris_scale;
            data["pupil-scale"] = anim.eye_anim_data.data[i].pupil_scale;
            data["lid-scale"] = anim.eye_anim_data.data[i].lid_scale;
            json["eye-anim-data"]["data"].push_back(data);
          }
        }


        //basic
        json["speed"] = anim.speed;
        json["artist-base"] = anim.artist_base;
        json["artist-step"] = anim.artist_step;
        json["master-art-group-name"] = anim.master_art_group_name;
        json["master-art-group-index"] = anim.master_art_group_index;


        // blerc-data
        if (anim.blerc_blend_target_count > 0 && !anim.blerc_data.empty()) {
          json["blerc-data"]["blend-target-count"] = anim.blerc_blend_target_count;
          json["blerc-data"]["num-frames"] = anim.frames.num_frames;
          json["blerc-data"]["data"] = anim.blerc_flat;  // flat runtime order

          json["blerc-data"]["frames"] = nlohmann::json::array();
          for (size_t frame_idx = 0; frame_idx < anim.blerc_data.size(); frame_idx++) {
            nlohmann::json frame_data = nlohmann::json::array();
            for (size_t target_idx = 0; target_idx < anim.blerc_data[frame_idx].size(); target_idx++) {
              u8 stored_value = anim.blerc_data[frame_idx][target_idx];
              // Values are stored as: actual_weight + 64
              // Final weight = (stored_value - 64) * 64
              frame_data.push_back(stored_value);
            }
            json["blerc-data"]["frames"].push_back(frame_data);
          }
          json["blerc-data"]["note"] =
              "Values are stored as offset: actual_weight = (value - 64) * 64; "
              "flat data matches runtime pointer layout (frame-major).";
        }


        //frames
        json["frames"]["num_frames"] = anim.frames.num_frames;
        json["frames"]["fixed-qwc"] = anim.frames.fixed_qwc;
        json["frames"]["frame-qwc"] = anim.frames.frame_qwc;
        
        // Export compression header
        json["frames"]["fixed"]["hdr"]["num-joints"] = anim.frames.fixed.hdr.num_joints;
        json["frames"]["fixed"]["hdr"]["matrix-bits"] = anim.frames.fixed.hdr.matrix_bits;
        json["frames"]["fixed"]["hdr"]["control-bits"] = nlohmann::json::array();
        for (int i = 0; i < 14; i++) {
          json["frames"]["fixed"]["hdr"]["control-bits"].push_back(anim.frames.fixed.hdr.control_bits[i]);
        }
        
        // Export base pose offsets
        json["frames"]["fixed"]["offset-64"] = anim.frames.fixed.offset_64;
        json["frames"]["fixed"]["offset-32"] = anim.frames.fixed.offset_32;
        json["frames"]["fixed"]["offset-16"] = anim.frames.fixed.offset_16;
        json["frames"]["fixed"]["reserved"] = anim.frames.fixed.reserved;
        
        // Export base pose vector data as raw bytes to avoid precision loss
        const u32 fixed_data_qwc = anim.frames.fixed_qwc > 5 ? (anim.frames.fixed_qwc - 5) : 0;
        const u32 fixed_data_vectors = std::min(fixed_data_qwc, 133u);
        json["frames"]["fixed"]["data"] =
          encode_vec4_base64(anim.frames.fixed.data, fixed_data_vectors);
        
        // Export frame data arrays (offset pointers and payload data)
        json["frames"]["data"] = nlohmann::json::array();
        for (const auto& frame : anim.frames.frames) {
          nlohmann::json frame_json;
          frame_json["offset-64"] = frame.offset_64;
          frame_json["offset-32"] = frame.offset_32;
          frame_json["offset-16"] = frame.offset_16;
          frame_json["reserved"] = frame.reserved;
          
          // Export frame payload vectors as raw bytes to avoid precision loss
          u32 frame_data_qwc = anim.frames.frame_qwc > 1 ? (anim.frames.frame_qwc - 1) : 0;
          u32 frame_data_vectors = std::min(frame_data_qwc, 133u);
          frame_json["data"] = encode_vec4_base64(frame.data, frame_data_vectors);
          
          json["frames"]["data"].push_back(frame_json);
        }


        //data - joint-anim-compressed entries (one per joint)
        json["data"] = nlohmann::json::array();
        for (const auto& joint_data : anim.data) {
          nlohmann::json data;
          data["name"] = joint_data.name;
          data["number"] = joint_data.number;
          data["length"] = joint_data.length;

          json["data"].push_back(data);
        }

        //export:
        // For stream animations, use the chunk name (e.g., "animation+0") to avoid overwriting
        std::string outputName;
        if (!isStream)
          outputName = dgo_name == "" ? "common" : dgo_name;
        else
          outputName = dgo_name;
        std::string json_name = isStream ? entry.first : anim.name;
        auto file_path = file_util::get_file_path({fmt::format("debug_out/animations/compressed/{}/{}/",
                                                               outputName,
                                                               anim.master_art_group_name),
                                                   fmt::format("{}.json", json_name)});
        file_util::create_dir_if_needed_for_file(file_path);
        file_util::write_text_file(file_path, json.dump(2));

        lg::info("Serialized animation {}", anim.name);
      }
    }
  }
}

}  // namespace decompiler
