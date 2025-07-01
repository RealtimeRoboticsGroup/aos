#ifndef AOS_FLATBUFFERS_BINARY_ANNOTATOR_H_
#define AOS_FLATBUFFERS_BINARY_ANNOTATOR_H_

#include <filesystem>

#include "flatbuffers/reflection.h"

#include "aos/flatbuffers.h"

namespace aos::fbs {

// These functions use the flatbuffers binary annotation to take in a
// non-size-prefixed flatbuffer and returns an annotated version of the
// underlying binary data.
std::string AnnotateBinaries(
    const aos::NonSizePrefixedFlatbuffer<reflection::Schema> &schema,
    flatbuffers::span<const uint8_t> binary_data);
std::string AnnotateBinaries(const std::filesystem::path &schema_bfbs_file,
                             flatbuffers::span<const uint8_t> binary_data);
}  // namespace aos::fbs
#endif  // AOS_FLATBUFFERS_BINARY_ANNOTATOR_H_
