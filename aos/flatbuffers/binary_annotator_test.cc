#include "aos/flatbuffers/binary_annotator.h"

#include "gtest/gtest.h"

#include "aos/flatbuffers/test_generated.h"
#include "aos/flatbuffers/test_schema.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"

namespace aos::fbs::testing {
inline constexpr const char *kFbsJson = R"json({ "scalar": 111 })json";
inline constexpr const char *kExpectedAnnotations = R"afb(
header:
  +0x00 | 0C 00 00 00             | UOffset32  | 0x0000000C (12) Loc: 0x0C | offset to root table `aos.fbs.testing.TestTable`

padding:
  +0x04 | 00 00                   | uint8_t[2] | ..                        | padding

vtable (aos.fbs.testing.TestTable):
  +0x06 | 06 00                   | uint16_t   | 0x0006 (6)                | size of this vtable
  +0x08 | 08 00                   | uint16_t   | 0x0008 (8)                | size of referring table
  +0x0A | 04 00                   | VOffset16  | 0x0004 (4)                | offset to field `scalar` (id: 0)

root_table (aos.fbs.testing.TestTable):
  +0x0C | 06 00 00 00             | SOffset32  | 0x00000006 (6) Loc: 0x06  | offset to vtable
  +0x10 | 6F 00 00 00             | uint32_t   | 0x0000006F (111)          | table field `scalar` (Int)
)afb";
TEST(BinaryAnnotatorTest, PrintsBinaryFromFileSchema) {
  aos::FlatbufferDetachedBuffer<testing::TestTable> fbs =
      aos::JsonToFlatbuffer<testing::TestTable>(kFbsJson);
  EXPECT_EQ(kExpectedAnnotations,
            AnnotateBinaries(
                ::aos::testing::ArtifactPath("aos/flatbuffers/test.bfbs"),
                fbs.span()));
}
TEST(BinaryAnnotatorTest, PrintsBinaryFromObjectSchema) {
  aos::FlatbufferDetachedBuffer<testing::TestTable> fbs =
      aos::JsonToFlatbuffer<testing::TestTable>(kFbsJson);
  aos::FlatbufferSpan<reflection::Schema> test_schema{TestTableSchema()};
  EXPECT_EQ(kExpectedAnnotations, AnnotateBinaries(test_schema, fbs.span()));
}
}  // namespace aos::fbs::testing
