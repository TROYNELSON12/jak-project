#include "extract_actors.h"

#include "common/goos/PrettyPrinter2.h"

#include "fmt/format.h"
#include "third-party/json.hpp"

namespace decompiler {

/*
 * {
"trans": [-21.6238, 20.0496, 17.1191], // translation
"etype": "fuel-cell",  // actor type
"game_task": 0, // associated game task (for powercells, etc)
"quat" : [0, 0, 0, 1], // quaternion
"bsphere": [-21.6238, 19.3496, 17.1191, 10], // bounding sphere
"lump": {
  "name":"test-fuel-cell"
}
},

 */

namespace {
nlohmann::json vectorm_json(const level_tools::Vector& v) {
  nlohmann::json result;
  for (int i = 0; i < 4; i++) {
    result.push_back(v.data[i] / 4096.f);
  }
  return result;
}

nlohmann::json vector_json(const level_tools::Vector& v) {
  nlohmann::json result;
  for (int i = 0; i < 4; i++) {
    result.push_back(v.data[i]);
  }
  return result;
}

nlohmann::json vectorm_keepw_json(const level_tools::Vector& v) {
  nlohmann::json result;
  for (int i = 0; i < 3; i++) {
    result.push_back(v.data[i] / 4096.f);
  }
  result.push_back(v.data[3]);
  return result;
}

nlohmann::json vectorm_json(const float* data) {
  nlohmann::json result;
  for (int i = 0; i < 4; i++) {
    result.push_back(data[i] / 4096.f);
  }
  return result;
}

nlohmann::json vector_json(const float* data) {
  nlohmann::json result;
  for (int i = 0; i < 4; i++) {
    result.push_back(data[i]);
  }
  return result;
}

nlohmann::json vectorm_keepw_json(const float* data) {
  nlohmann::json result;
  for (int i = 0; i < 3; i++) {
    result.push_back(data[i] / 4096.f);
  }
  result.push_back(data[3]);
  return result;
}

nlohmann::json vectorm_onlyw_json(const float* data) {
  nlohmann::json result;
  for (int i = 0; i < 3; i++) {
    result.push_back(data[i]);
  }
  result.push_back(data[3] / 4096.f);
  return result;
}

nlohmann::json vectorm_rotw_json(const float* data) {
  nlohmann::json result;
  for (int i = 0; i < 3; i++) {
    result.push_back(data[i] / 4096.f);
  }
  result.push_back(data[3] * (360.f / 65536.f));
  return result;
}

nlohmann::json strings_json(const std::vector<std::string>& data, bool prefix_quote) {
  if (data.size() == 1) {
    if (prefix_quote) {
      return fmt::format("'{}", data[0]);
    } else {
      return data[0];
    }
  } else {
    nlohmann::json result;
    for (const auto& str : data) {
      if (prefix_quote) {
        result.push_back(fmt::format("'{}", str));
      } else {
        result.push_back(str);
      }
    }
    return result;
  }
}

template <typename T>
nlohmann::json value_json(const std::vector<u8>& data, int count) {
  ASSERT(count * sizeof(T) == data.size());
  if (count == 1) {
    T v;
    memcpy(&v, data.data(), sizeof(T));
    return v;
  } else {
    nlohmann::json ret;
    T v;
    for (int i = 0; i < count; i++) {
      memcpy(&v, data.data() + i * sizeof(T), sizeof(T));
      ret.push_back(v);
    }
    return ret;
  }
}

template <typename T>
nlohmann::json valuem_json(const std::vector<u8>& data, int count) {
  ASSERT(count * sizeof(T) == data.size());
  if (count == 1) {
    T v;
    memcpy(&v, data.data(), sizeof(T));
    return v / 4096.f;
  } else {
    nlohmann::json ret;
    T v;
    for (int i = 0; i < count; i++) {
      memcpy(&v, data.data() + i * sizeof(T), sizeof(T));
      ret.push_back(v / 4096.f);
    }
    return ret;
  }
}
}  // namespace

nlohmann::json nav_mesh_json(const level_tools::Nav_Mesh& nav_mesh) {
  nlohmann::json json_nav;
  json_nav["bounds"] = vectorm_json(nav_mesh.bounds);
  json_nav["origin"] = vectorm_keepw_json(nav_mesh.origin);

  json_nav["node-count"] = nav_mesh.node_count;
  json_nav["nodes"] = nlohmann::json::array();
  for (int i = 0; i < nav_mesh.node_count; i++) {
    nlohmann::json json_node;
    json_node["center-x"] = nav_mesh.nodes[i].center_x;
    json_node["center-y"] = nav_mesh.nodes[i].center_y;
    json_node["center-z"] = nav_mesh.nodes[i].center_x;
    json_node["type"] = nav_mesh.nodes[i].type;
    json_node["parent-offset"] = nav_mesh.nodes[i].parent_offset;
    json_node["radius-x"] = nav_mesh.nodes[i].radius_x;
    json_node["radius-y"] = nav_mesh.nodes[i].radius_y;
    json_node["radius-z"] = nav_mesh.nodes[i].radius_z;
    json_node["left-offset"] = nav_mesh.nodes[i].left_offset;
    json_node["right-offset"] = nav_mesh.nodes[i].right_offset;
    json_node["scale-x"] = nav_mesh.nodes[i].scale_x;
    for (int j = 0; j < 4; j++)
      json_node["first-tris"] = nav_mesh.nodes[i].first_tris[j];
    json_node["scale-z"] = nav_mesh.nodes[i].scale_z;
    for (int j = 0; j < 4; j++)
      json_node["last-tris"] = nav_mesh.nodes[i].last_tris[j];
    json_nav["nodes"].push_back(json_node);
  }

  json_nav["vertex-count"] = nav_mesh.vertex_count;
  json_nav["vertex"] = nlohmann::json::array();
  for (int i = 0; i < nav_mesh.vertex_count; i++) {
    json_nav["vertex"].push_back(vectorm_keepw_json(nav_mesh.vertex[i]));
  }

  json_nav["poly-count"] = nav_mesh.poly_count;
  json_nav["poly"] = nlohmann::json::array();
  for (int i = 0; i < nav_mesh.poly_count; i++) {
    nlohmann::json json_poly;
    json_poly["id"] = nav_mesh.poly[i].id;
    json_poly["vertex"] = nlohmann::json::array();
    json_poly["adj-poly"] = nlohmann::json::array();
    for (int j = 0; j < 3; j++) {
      json_poly["vertex"].push_back(nav_mesh.poly[i].vertex[j]);
      json_poly["adj-poly"].push_back(nav_mesh.poly[i].adj_poly[j]);
    }
    json_poly["pat"] = nav_mesh.poly[i].pat;
    json_nav["poly"].push_back(json_poly);
  }
  return json_nav;
}

std::list<std::string> eltMeters = {  "spring-height",
                                      "vis-dist",
                                      "height-info",
                                      "distance",
                                      "cam-notice-dist",
                                      "cam-vert",
                                      "cam-horz",
                                      "idle-distance",
                                      "nearest-y-threshold",
                                      "center-point",
                                      "center-radius",
                                      "notice-dist",
                                      "trigger-height",
                                      "notice-top"};

std::string extract_actors_to_json(const level_tools::DrawableInlineArrayActor& actors) {
  nlohmann::json json;

  for (const auto& dactor : actors.drawable_actors) {
    const auto& actor = dactor.actor;
    auto& json_actor = json.emplace_back();
    json_actor["bsphere"] = vectorm_json(dactor.bsphere);
    // drawable ID?

    json_actor["trans"] = vectorm_keepw_json(actor.trans);
    json_actor["aid"] = actor.aid;  // aid

    if (actor.nav_mesh.exists) {  // nav mesh
      json_actor["nav-mesh"] = nav_mesh_json(actor.nav_mesh);
    }

    json_actor["etype"] = actor.etype;
    json_actor["game_task"] = actor.task;
    json_actor["vis-id"] = actor.vis_id;
    json_actor["quat"] = vector_json(actor.quat);
    auto& json_lump = json_actor["lump"];
    int PathCount = 0;
    int VolCount = 0;
    for (const auto& res : actor.res_list) {
      if (res.elt_type == "string") {
        json_lump[res.name] = strings_json(res.strings, false);
      } else if (res.elt_type == "symbol") {
        json_lump[res.name] = strings_json(res.strings, true);
      } else if (res.elt_type == "type") {
        // TODO: confusion with symbols
        json_lump[res.name] = strings_json(res.strings, true);
      } else if (res.elt_type == "vector") {
        // Paths and Vols get combined due to having the same key, so add an index:
        // Also they define points in world, so meter convert them.
        if (res.name == "path") {
          std::string field_name = fmt::format("{}{}", res.name, PathCount);
          PathCount++;
          const float* data = (const float*)res.inlined_storage.data();
          for (int i = 0; i < res.count; i++) {
            json_lump[field_name].push_back(vectorm_keepw_json(data + 4 * i));
          }
        } else if (res.name == "vol") {
          std::string field_name = fmt::format("{}{}", res.name, VolCount);
          VolCount++;
          const float* data = (const float*)res.inlined_storage.data();
          for (int i = 0; i < res.count; i++) {
            json_lump[field_name].push_back(vectorm_onlyw_json(data + 4 * i));
          }
        } else if (res.name == "visvol") {
          const float* data = (const float*)res.inlined_storage.data();
          for (int i = 0; i < res.count; i++) {
            json_lump["visvol"].push_back(vectorm_keepw_json(data + 4 * i));
          }
        } else if (res.name == "movie-pos") {
          const float* data = (const float*)res.inlined_storage.data();
          for (int i = 0; i < res.count; i++) {
            json_lump["movie-pos"].push_back(vectorm_rotw_json(data + 4 * i));
          }
        } else if (res.name == "nav-mesh-sphere") {
          const float* data = (const float*)res.inlined_storage.data();
          for (int i = 0; i < res.count; i++) {
            json_lump["nav-mesh-sphere"].push_back(vectorm_json(data + 4 * i));
          }
        } else if (res.name == "alt-vector") {
          if (actor.etype == "yakow") {
            const float* data = (const float*)res.inlined_storage.data();
            for (int i = 0; i < res.count; i++) {
              json_lump["alt-vector"].push_back(vectorm_rotw_json(data + 4 * i));
            }
          } else {
            const float* data = (const float*)res.inlined_storage.data();
            for (int i = 0; i < res.count; i++) {
              json_lump["alt-vector"].push_back(vectorm_keepw_json(data + 4 * i));
            }
          }
        } else {  // Regular vector:
          const float* data = (const float*)res.inlined_storage.data();
          if (res.count == 1) {
            json_lump[res.name] = vector_json(data);
          } else {
            for (int i = 0; i < res.count; i++) {
              json_lump[res.name].push_back(vector_json(data + 4 * i));
            }
          }
        }
      } else if (res.elt_type == "pair") {
        json_lump[res.name] = pretty_print::to_string(res.script);
      } else if (res.elt_type == "float") {
        auto isMeter = std::find(eltMeters.begin(), eltMeters.end(), res.name);
        if (isMeter != eltMeters.end() && !(actor.etype == "silostep" && res.name == "distance")) {
          json_lump[res.name] = valuem_json<float>(res.inlined_storage, res.count);
        } else {
          json_lump[res.name] = value_json<float>(res.inlined_storage, res.count);
        }
      } else if (res.elt_type == "int32") {
        json_lump[res.name] = value_json<int32_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "int16") {
        json_lump[res.name] = value_json<int16_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "int8") {
        json_lump[res.name] = value_json<int8_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "uint32") {
        json_lump[res.name] = value_json<uint32_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "uint16") {
        json_lump[res.name] = value_json<uint16_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "uint8") {
        json_lump[res.name] = value_json<uint8_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "actor-group") {
        // not supported.
      } else {
        ASSERT_NOT_REACHED();
      }
    }
  }

  return json.dump(2);
}

std::string extract_ambients_to_json(const level_tools::DrawableInlineArrayAmbient& actors) {
  nlohmann::json json;

  for (const auto& damb : actors.drawable_ambients) {
    const auto& ambient = damb.ambient;
    auto& json_ambient = json.emplace_back();
    json_ambient["bsphere"] = vectorm_json(damb.bsphere);
    // drawable ID?

    json_ambient["trans"] = vectorm_json(ambient.trans);
    json_ambient["aid"] = ambient.aid;  // aid

    auto& json_lump = json_ambient["lump"];

    nlohmann::json effects;
    int effectCount = 0,
        effectParamCount =
            0;  // just to keep track since names cound all be together and then params

    for (const auto& res : ambient.res_list) {
      if (res.elt_type == "string") {
        if (res.name == "name")
          json_lump[res.name] = strings_json(res.strings, false);
        else
          json_lump[res.name] = strings_json(res.strings, true);
      } else if (res.elt_type == "symbol") {
        if (res.name == "effect-name") {
          if (++effectCount > effectParamCount) {
            nlohmann::json effect;
            effect["name"] = strings_json(res.strings, false);
            effects.push_back(effect);
          } else {
            auto& effect = effects[effectCount - 1];
            effect["name"] = strings_json(res.strings, false);
          }
        } else {
          json_lump[res.name] = strings_json(res.strings, false);
        }
      } else if (res.elt_type == "type") {
        // TODO: confusion with symbols
        json_lump[res.name] = strings_json(res.strings, true);
      } else if (res.elt_type == "vector") {
        const float* data = (const float*)res.inlined_storage.data();
        if (res.count == 1) {
          json_lump[res.name] = vector_json(data);
        } else {
          for (int i = 0; i < res.count; i++) {
            json_lump[res.name].push_back(vector_json(data + 4 * i));
          }
        }
      } else if (res.elt_type == "pair") {
        json_lump[res.name] = pretty_print::to_string(res.script);
      } else if (res.elt_type == "float") {
        if (res.name == "effect-param") {
          if (++effectParamCount > effectCount) {
            nlohmann::json effect;
            effect["params"] = value_json<float>(res.inlined_storage, res.count);
            effects.push_back(effect);
          } else {
            auto& effect = effects[effectParamCount - 1];
            effect["params"] = value_json<float>(res.inlined_storage, res.count);
          }
        } else {
          json_lump[res.name] = value_json<float>(res.inlined_storage, res.count);
        }
      } else if (res.elt_type == "int32") {
        json_lump[res.name] = value_json<int32_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "int16") {
        json_lump[res.name] = value_json<int16_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "int8") {
        json_lump[res.name] = value_json<int8_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "uint32") {
        json_lump[res.name] = value_json<uint32_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "uint16") {
        json_lump[res.name] = value_json<uint16_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "uint8") {
        json_lump[res.name] = value_json<uint8_t>(res.inlined_storage, res.count);
      } else if (res.elt_type == "actor-group") {
        // not supported.
      } else {
        ASSERT_NOT_REACHED();
      }
    }

    if (effectCount || effectParamCount)
      json_lump["effects"] = effects;
  }

  return json.dump(2);
}

}  // namespace decompiler
