#include "aos/util/mcap_logger.h"

#include "flatbuffers/reflection_generated.h"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"

#include "aos/testing/path.h"
#include "aos/util/file.h"

namespace aos::testing {
// TODO(james): Write a proper test for the McapLogger itself. However, that
// will require writing an MCAP reader (or importing an existing one).

// Confirm that the schema for the reflection.Schema table itself hasn't
// changed. reflection.Schema should be a very stable type, so this should
// need
// updating except when we change the JSON schema generation itself.
TEST(JsonSchemaTest, ReflectionSchema) {
  nlohmann::json schema_json =
      JsonSchemaForFlatbuffer({reflection::Schema::MiniReflectTypeTable()});

  std::string expected_json_str = aos::util::ReadFileToStringOrDie(
      aos::testing::ArtifactPath("aos/util/reflection_schema.json"));

  EXPECT_EQ(nlohmann::json::parse(expected_json_str), schema_json);
}

}  // namespace aos::testing
