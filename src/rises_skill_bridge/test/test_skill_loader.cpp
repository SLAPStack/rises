// Unit tests for the ROS4HRI skill-descriptor JSON files shipped under
// rises_skill_bridge/skills/. The production SkillBridgeNode does not load
// these files at runtime -- it hardcodes ROS action servers under skill/*.
// The descriptors are consumed externally (ROS4HRI orchestrators / MCP).
//
// These tests therefore validate the descriptor *artifacts* the package
// ships: they parse the JSON with nlohmann_json, check required fields, and
// reject malformed input. The parser used here is local to the test
// translation unit; it is not a production seam.
//
// Schema (per the shipped files):
//   id              : string  -- skill identifier (required, unique)
//   version         : string  -- semver-ish (required)
//   type            : string  -- must equal "skill"
//   package         : string  -- declaring package
//   interface       : string  -- must equal "action" or "service" or "topic"
//   datatype        : string  -- ROS interface (e.g.
//   "rises_interfaces/action/X") default_path    : string  -- ROS namespace
//   (e.g. "/skill/x") description     : string  -- human-readable
//
// The parameterized LoadAllShippedSkills suite locates the shipped descriptor
// directory via, in order:
//   1) ARISE_SKILL_BRIDGE_SKILLS_DIR environment variable (pre-install runs)
//   2) ament_index_cpp::get_package_share_directory("rises_skill_bridge")
//      followed by "/skills" (post-install / ament colcon test).
// If neither is resolvable the suite reports an empty parameter set, which
// GoogleTest reports as a single skipped entry -- diagnostic, not silent.
//
// Failure-path tests use test_support::TempJsonFile to write transient input
// under TMPDIR; no real network or filesystem race is involved.

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include "test_support/temp_json.hpp"

namespace {

// Allowed enum values for the "interface" field. Matches ROS4HRI vocabulary
// observed in the shipped descriptors.
const std::unordered_set<std::string> &allowedInterfaces() {
  static const std::unordered_set<std::string> kAllowed{"action", "service",
                                                        "topic"};
  return kAllowed;
}

// Result of a single load attempt. Mirrors what a real loader would
// expose: either a parsed descriptor or a non-empty error string. Kept
// local to the test TU on purpose -- not a production type.
struct LoadResult {
  std::optional<nlohmann::json> descriptor;
  std::string error;
};

// Local validator used only by these tests. Mirrors the rules we expect a
// production loader to enforce when one is added.
LoadResult loadDescriptor(const std::string &path) {
  LoadResult out;
  std::ifstream in(path);
  if (!in) {
    out.error = "open failed: " + path;
    return out;
  }
  nlohmann::json parsed;
  try {
    in >> parsed;
  } catch (const nlohmann::json::parse_error &e) {
    out.error = std::string{"parse error: "} + e.what();
    return out;
  }
  if (!parsed.is_object()) {
    out.error = "root is not an object";
    return out;
  }
  if (!parsed.contains("id") || !parsed["id"].is_string() ||
      parsed["id"].get<std::string>().empty()) {
    out.error = "missing or empty 'id'";
    return out;
  }
  if (!parsed.contains("interface") || !parsed["interface"].is_string()) {
    out.error = "missing 'interface'";
    return out;
  }
  const std::string iface = parsed["interface"].get<std::string>();
  if (allowedInterfaces().find(iface) == allowedInterfaces().end()) {
    out.error = "invalid interface: " + iface;
    return out;
  }
  if (!parsed.contains("datatype") || !parsed["datatype"].is_string() ||
      parsed["datatype"].get<std::string>().empty()) {
    out.error = "missing or empty 'datatype'";
    return out;
  }
  out.descriptor = std::move(parsed);
  return out;
}

LoadResult loadFromString(const std::string &content) {
  test_support::TempJsonFile tmp{content};
  return loadDescriptor(tmp.path());
}

std::optional<std::string> resolveSkillsDir() {
  if (const char *env = std::getenv("ARISE_SKILL_BRIDGE_SKILLS_DIR")) {
    std::filesystem::path p{env};
    if (std::filesystem::is_directory(p)) {
      return p.string();
    }
  }
  try {
    const std::string share =
        ament_index_cpp::get_package_share_directory("rises_skill_bridge");
    std::filesystem::path candidate = std::filesystem::path{share} / "skills";
    if (std::filesystem::is_directory(candidate)) {
      return candidate.string();
    }
  } catch (const std::exception &) {
    // Pre-install: ament index entry not present yet.
  }
  return std::nullopt;
}

std::vector<std::string> discoverShippedSkillFiles() {
  std::vector<std::string> out;
  const auto dir = resolveSkillsDir();
  if (!dir) {
    return out;
  }
  for (const auto &entry : std::filesystem::directory_iterator(*dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    constexpr const char *kSuffix = ".skill.json";
    const std::size_t suffix_len = std::string_view{kSuffix}.size();
    if (name.size() < suffix_len) {
      continue;
    }
    if (name.compare(name.size() - suffix_len, suffix_len, kSuffix) == 0) {
      out.push_back(entry.path().string());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::string minimalValidJson(const std::string &id) {
  std::ostringstream os;
  os << "{"
     << "\"id\":\"" << id << "\","
     << "\"version\":\"1.0.0\","
     << "\"type\":\"skill\","
     << "\"package\":\"geofence_skills\","
     << "\"interface\":\"action\","
     << "\"datatype\":\"rises_interfaces/action/GetSafetyRadius\","
     << "\"default_path\":\"/skill/" << id << "\","
     << "\"description\":\"unit test descriptor\""
     << "}";
  return os.str();
}

} // namespace

TEST(SkillLoader, LoadValidSkillJson) {
  const LoadResult result =
      loadFromString(minimalValidJson("get_safety_radius"));
  ASSERT_TRUE(result.descriptor.has_value()) << "error: " << result.error;
  const auto &d = *result.descriptor;
  EXPECT_EQ(d.at("id").get<std::string>(), "get_safety_radius");
  EXPECT_EQ(d.at("interface").get<std::string>(), "action");
  EXPECT_EQ(d.at("datatype").get<std::string>(),
            "rises_interfaces/action/GetSafetyRadius");
  EXPECT_EQ(d.at("default_path").get<std::string>(),
            "/skill/get_safety_radius");
}

TEST(SkillLoader, RejectMissingNameField) {
  // "name" is not part of this schema; the canonical identifier field is
  // "id". A descriptor with neither must be rejected.
  const std::string json = R"({
        "version":"1.0.0","type":"skill","package":"geofence_skills",
        "interface":"action","datatype":"rises_interfaces/action/GetMapInfo",
        "default_path":"/skill/get_map_info","description":"no id"
    })";
  const LoadResult result = loadFromString(json);
  EXPECT_FALSE(result.descriptor.has_value());
  EXPECT_NE(result.error.find("'id'"), std::string::npos) << result.error;
}

TEST(SkillLoader, RejectInvalidActionType) {
  // "action_type" is not part of this schema; the canonical field is
  // "interface" (allowed values: action / service / topic). A descriptor
  // claiming an unknown interface kind must be rejected.
  const std::string json = R"({
        "id":"bogus","version":"1.0.0","type":"skill","package":"x",
        "interface":"completely_made_up",
        "datatype":"rises_interfaces/action/GetMapInfo",
        "default_path":"/skill/bogus","description":""
    })";
  const LoadResult result = loadFromString(json);
  EXPECT_FALSE(result.descriptor.has_value());
  EXPECT_NE(result.error.find("invalid interface"), std::string::npos)
      << result.error;
}

TEST(SkillLoader, RejectMalformedJson) {
  const std::string truncated =
      R"({"id":"x","interface":"action")"; // no closing brace
  const LoadResult result = loadFromString(truncated);
  EXPECT_FALSE(result.descriptor.has_value());
  EXPECT_NE(result.error.find("parse error"), std::string::npos)
      << result.error;
}

TEST(SkillLoader, RejectDuplicateSkillName) {
  // Stage two descriptors with the same id in the same logical directory
  // and confirm that an id-uniqueness check rejects the second one.
  test_support::TempJsonFile first{minimalValidJson("duplicated_id")};
  test_support::TempJsonFile second{minimalValidJson("duplicated_id")};

  std::unordered_set<std::string> seen;
  const std::vector<std::string> paths{first.path(), second.path()};

  std::vector<std::string> rejected;
  for (const auto &path : paths) {
    const LoadResult result = loadDescriptor(path);
    ASSERT_TRUE(result.descriptor.has_value()) << result.error;
    const std::string id = result.descriptor->at("id").get<std::string>();
    if (!seen.insert(id).second) {
      rejected.push_back(path);
    }
  }
  ASSERT_EQ(rejected.size(), 1u);
  EXPECT_EQ(rejected.front(), second.path());
}

// --- Parameterized: every shipped descriptor must load. -----------------

class ShippedSkillFixture : public ::testing::TestWithParam<std::string> {};

TEST_P(ShippedSkillFixture, LoadAllShippedSkills) {
  const std::string &path = GetParam();
  if (path.empty()) {
    GTEST_SKIP() << "ARISE_SKILL_BRIDGE_SKILLS_DIR not set and ament "
                    "index entry for rises_skill_bridge not found "
                    "(pre-install run?). Set the env var to the source "
                    "skills/ directory to exercise this suite.";
  }
  const LoadResult result = loadDescriptor(path);
  ASSERT_TRUE(result.descriptor.has_value())
      << "failed to load " << path << ": " << result.error;
  EXPECT_EQ(result.descriptor->at("type").get<std::string>(), "skill");
  EXPECT_EQ(result.descriptor->at("interface").get<std::string>(), "action");
  EXPECT_FALSE(result.descriptor->at("id").get<std::string>().empty());
}

namespace {
// GoogleTest requires a non-empty parameter vector. Emit a single empty
// string placeholder when no descriptors are discoverable; the test body
// converts that into a GTEST_SKIP so the diagnostic is loud, not silent.
std::vector<std::string> shippedSkillParams() {
  std::vector<std::string> v = discoverShippedSkillFiles();
  if (v.empty()) {
    v.emplace_back();
  }
  return v;
}
} // namespace

INSTANTIATE_TEST_SUITE_P(Shipped, ShippedSkillFixture,
                         ::testing::ValuesIn(shippedSkillParams()),
                         [](const ::testing::TestParamInfo<std::string> &info) {
                           if (info.param.empty()) {
                             return std::string{"unresolved"};
                           }
                           std::filesystem::path p{info.param};
                           std::string stem = p.filename().string();
                           // Replace '.' with '_' so GoogleTest accepts it as
                           // an identifier.
                           std::replace(stem.begin(), stem.end(), '.', '_');
                           return stem;
                         });

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
