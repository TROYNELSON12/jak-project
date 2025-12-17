#pragma once
#include "common/common_types.h"
#include "common/math/Vector.h"

namespace level_tools {

// levels may remap textures if they provide one that should be shared
struct TextureRemap {
  u32 original_texid;
  u32 new_texid;
};

struct Joint {
  std::string name;
  int parent_idx = -1;  // -1 for magic ROOT joint.
  math::Matrix4f bind_pose_T_w;
};

struct MercEyeAnimFrame {
  s8 pupil_trans_x;
  s8 pupil_trans_y;
  s8 blink;
  s8 pad0 = 0;  // goal layout has iris-scale at byte 4
  s8 iris_scale;
  s8 pupil_scale;
  s8 lid_scale;
  s8 pad1 = 0;  // pack-me to 8 bytes, matches overlayed uint64
};

struct MercEyeAnimBlock {
  short max_frame;
  std::vector<MercEyeAnimFrame> data;
};

struct JointAnim {
  std::string name;
  short number;
  short length;
};

struct JointAnimCompressed : JointAnim {
  //std::vector<u32> data; //possibly not needed?
};

struct JointAnimCompressedHdr {
  u32 control_bits[14];
  u32 num_joints;
  u32 matrix_bits;
};

struct JointAnimCompressedFixed {
  JointAnimCompressedHdr hdr;
  u32 offset_64;
  u32 offset_32;
  u32 offset_16;
  u32 reserved;
  math::Vector4f data[133];
};

struct JointAnimCompressedFrame {
  u32 offset_64;
  u32 offset_32;
  u32 offset_16;
  u32 reserved;
  math::Vector4f data[133];
};

struct JointAnimCompressedControl {
  u32 num_frames;
  u32 fixed_qwc;
  u32 frame_qwc;
  JointAnimCompressedFixed fixed;
  std::vector<JointAnimCompressedFrame> frames;  // actual frame data array
};

struct Art{
  std::string name;
  int length;
  u8 extra;
};

struct ArtElement : Art {
  u8 pad[12];
};

struct ArtJointAnim : ArtElement {
  MercEyeAnimBlock eye_anim_data;
  float speed;
  float artist_base;
  float artist_step;
  std::string master_art_group_name;
  int master_art_group_index;
  
  // blerc_data: per-frame blend shape weights for facial animation
  // Format: array of uint8 organized as frames × blend_targets
  // Each frame has blend_target_count values (from merc-ctrl header)
  // Values are offset by 64: actual_weight = (stored_value - 64) * 64
  std::vector<std::vector<u8>> blerc_data;  // [frame_idx][target_idx]
  std::vector<u8> blerc_flat;               // flat array in runtime order [f0 targets][f1 targets]...
  int blerc_blend_target_count = 0;  // number of blend targets (from merc-ctrl)
  
  JointAnimCompressedControl frames;
  std::vector<JointAnimCompressed> data;
};
/*!
 * Data extracted from art groups that is not needed for .FR3, but is potentially needed for other
 * stuff (skeleton export).
 */
struct ArtData {
  std::string art_group_name;
  std::string art_name;
  std::vector<Joint> joint_group;
  std::vector<ArtJointAnim> joint_anims;

  int blerc_blend_target_count = 0;
};

}  // namespace level_tools