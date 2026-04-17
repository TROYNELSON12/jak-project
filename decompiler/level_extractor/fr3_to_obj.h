#include <map>
#include <unordered_set>

#include "common/custom_data/Tfrag3Data.h"
#include "common/util/FileUtil.h"

#include "decompiler/level_extractor/common_formats.h"

#include "third-party/json.hpp"

/*!
 * Export the background geometry (tie, tfrag, shrub) to a obj format (.obj) file.
 */
void save_level_background_as_obj(const tfrag3::Level& level,
                                  const fs::path& obj_path,
                                  nlohmann::json& level_json);
