#include "aos/events/logging/binary_annotator_wrapper.h"

#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"
#include "src/annotated_binary_text_gen.h"
#include "src/binary_annotator.h"

namespace aos::logger::testing {

std::string AnnotateBinaries(
    const aos::NonSizePrefixedFlatbuffer<reflection::Schema> &schema,
    const std::string &schema_filename,
    flatbuffers::span<uint8_t> binary_data) {
  flatbuffers::BinaryAnnotator binary_annotator(
      schema.span().data(), schema.span().size(), binary_data.data(),
      binary_data.size(), /*is_size_prefixed=*/false);

  auto annotations = binary_annotator.Annotate();

  flatbuffers::AnnotatedBinaryTextGenerator text_generator(
      flatbuffers::AnnotatedBinaryTextGenerator::Options{}, annotations,
      binary_data.data(), binary_data.size());

  text_generator.Generate(aos::testing::TestTmpDir() + "/foo.bfbs",
                          schema_filename);

  return aos::util::ReadFileToStringOrDie(aos::testing::TestTmpDir() +
                                          "/foo.afb");
}

}  // namespace aos::logger::testing
