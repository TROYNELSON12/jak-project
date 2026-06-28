#pragma once

#include "common/custom_data/Tfrag3Data.h"

#include "decompiler/level_extractor/BspHeader.h"

namespace decompiler {

bool extract_hfrag(const level_tools::BspHeader& bsp, const TextureDB& tex_db, tfrag3::Level* out);

std::string export_hfrag_to_obj(tfrag3::Hfragment  hfrag,
                                tfrag3::Level& lev,
                                const TextureDB& tdb,
                                GameVersion version);
}
