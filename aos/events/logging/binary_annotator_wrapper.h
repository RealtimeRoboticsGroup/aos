#ifndef AOS_EVENTS_LOGGING_BINARY_ANNOTATOR_WRAPPER_H_
#define AOS_EVENTS_LOGGING_BINARY_ANNOTATOR_WRAPPER_H_

// This wrapper library encapsulates the flatbuffers binary annotator and text
// generator. On Windows, transitively including `<windows.h>` defines the
// `ERROR` macro as `0`. However, flatbuffers has an enum member
// `reflection::ERROR` or `reflection::Type::ERROR`, which gets incorrectly
// replaced by `0` if the macro is defined, causing compile errors.
//
// To prevent macro collision issues in large test files like
// logfile_utils_test.cc that include `<windows.h>`, this wrapper library
// isolates the flatbuffers headers in a clean translation unit that does not
// include `<windows.h>`, avoiding the conflict.

#include <string>
#include <string_view>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection_generated.h"

#include "aos/flatbuffers.h"

namespace aos::logger::testing {

// Uses the binary schema to annotate the provided flatbuffer binary data.
//
// @param schema The non-size-prefixed flatbuffer schema.
// @param schema_filename The filename of the schema to associate with the
// annotations.
// @param binary_data The raw binary flatbuffer data to annotate.
// @return The annotated flatbuffer representation as a string.
std::string AnnotateBinaries(
    const aos::NonSizePrefixedFlatbuffer<reflection::Schema> &schema,
    const std::string &schema_filename, flatbuffers::span<uint8_t> binary_data);

}  // namespace aos::logger::testing

#endif  // AOS_EVENTS_LOGGING_BINARY_ANNOTATOR_WRAPPER_H_
