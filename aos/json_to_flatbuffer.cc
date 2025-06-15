#include "aos/json_to_flatbuffer.h"

#include <cstddef>
#include <cstdio>
#include <string_view>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/minireflect.h"

#include "aos/flatbuffer_utils.h"
#include "aos/json_tokenizer.h"
#include "aos/util/string_formatting.h"

// TODO(austin): Can we just do an Offset<void> ?  It doesn't matter, so maybe
// just say that.
//
// TODO(austin): I've yet to see how to create an ET_UTYPE, so I don't know what
// one is and how to test it.  So everything rejects it.

namespace aos {
namespace {

// Class to hold one of the 3 json types for an array.
struct Element {
  // The type.
  enum class ElementType { INT, DOUBLE, OFFSET, STRUCT };

  // Constructs an Element holding an integer.
  Element(absl::int128 new_int_element)
      : int_element(new_int_element), type(ElementType::INT) {}
  // Constructs an Element holding an double.
  Element(double new_double_element)
      : double_element(new_double_element), type(ElementType::DOUBLE) {}
  // Constructs an Element holding an Offset.
  Element(flatbuffers::Offset<flatbuffers::String> new_offset_element)
      : offset_element(new_offset_element), type(ElementType::OFFSET) {}
  // Constructs an Element holding a struct.
  Element(std::vector<uint8_t> struct_data)
      : /*initialize the union member to keep the compiler happy*/ int_element(
            0),
        struct_data(std::move(struct_data)),
        type(ElementType::STRUCT) {}

  // Union for the various datatypes.
  union {
    absl::int128 int_element;
    double double_element;
    flatbuffers::Offset<flatbuffers::String> offset_element;
  };
  // Because we can't know the maximum size of any potential structs at
  // compile-time, we will use a vector to store the vector data inline.
  // If you were to do a reinterpret_cast<StructType*>(struct_data.data()) then
  // you would have an instance of the struct in question.
  std::vector<uint8_t> struct_data;

  // And an enum signaling which one is in use.
  ElementType type;
};

// Structure to represent a field element.
struct FieldElement {
  FieldElement(int new_field_index, absl::int128 int_element)
      : element(int_element), field_index(new_field_index) {}
  FieldElement(int new_field_index, double double_element)
      : element(double_element), field_index(new_field_index) {}
  FieldElement(int new_field_index,
               flatbuffers::Offset<flatbuffers::String> offset_element)
      : element(offset_element), field_index(new_field_index) {}
  FieldElement(int new_field_index, const Element &element)
      : element(element), field_index(new_field_index) {}

  // Data to write.
  Element element;
  // Field index.  The type table which this index is for is stored outside this
  // object.
  int field_index;
};

// Adds a single element.  This assumes that vectors have been dealt with
// already.  Returns true on success.
bool AddSingleElement(FlatbufferType type, const FieldElement &field_element,
                      ::std::vector<bool> *fields_in_use,
                      flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(FlatbufferType type, int field_index,
                      absl::int128 int_value,
                      flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(FlatbufferType type, int field_index, double double_value,
                      flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(FlatbufferType type, int field_index,
                      flatbuffers::Offset<flatbuffers::String> offset_element,
                      flatbuffers::FlatBufferBuilder *fbb);
bool AddSingleElement(FlatbufferType type, int field_index,
                      const std::vector<uint8_t> &struct_data,
                      flatbuffers::FlatBufferBuilder *fbb);

template <typename T, typename U>
void SetMemory(U value, uint8_t *destination) {
  // destination may be poorly aligned. As such, we should not simply do
  // *reinterpret_cast<T*>(destination) = value directly.
  const T casted = static_cast<T>(value);
  memcpy(destination, &casted, sizeof(T));
}

bool SetStructElement(FlatbufferType type, int field_index, absl::int128 value,
                      uint8_t *destination) {
  const flatbuffers::ElementaryType elementary_type =
      type.FieldElementaryType(field_index);
  switch (elementary_type) {
    case flatbuffers::ET_CHAR:
      SetMemory<int8_t>(value, destination);
      break;
    case flatbuffers::ET_UCHAR:
      SetMemory<uint8_t>(value, destination);
      break;
    case flatbuffers::ET_SHORT:
      SetMemory<int16_t>(value, destination);
      break;
    case flatbuffers::ET_USHORT:
      SetMemory<uint16_t>(value, destination);
      break;
    case flatbuffers::ET_INT:
      SetMemory<int32_t>(value, destination);
      break;
    case flatbuffers::ET_UINT:
      SetMemory<uint32_t>(value, destination);
      break;
    case flatbuffers::ET_LONG:
      SetMemory<int64_t>(value, destination);
      break;
    case flatbuffers::ET_ULONG:
      SetMemory<uint64_t>(value, destination);
      break;
    case flatbuffers::ET_BOOL:
      SetMemory<bool>(value, destination);
      break;
    case flatbuffers::ET_FLOAT:
      SetMemory<float>(value, destination);
      break;
    case flatbuffers::ET_DOUBLE:
      SetMemory<double>(value, destination);
      break;
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_SEQUENCE: {
      const std::string_view name = type.FieldName(field_index);
      fprintf(stderr,
              "Mismatched type for field '%.*s'. Got: integer, expected %s\n",
              static_cast<int>(name.size()), name.data(),
              ElementaryTypeName(elementary_type));
      return false;
    }
  }
  return true;
}

bool SetStructElement(FlatbufferType type, int field_index, double value,
                      uint8_t *destination) {
  const flatbuffers::ElementaryType elementary_type =
      type.FieldElementaryType(field_index);
  switch (elementary_type) {
    case flatbuffers::ET_FLOAT:
      SetMemory<float>(value, destination);
      break;
    case flatbuffers::ET_DOUBLE:
      SetMemory<double>(value, destination);
      break;
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_SEQUENCE: {
      const std::string_view name = type.FieldName(field_index);
      fprintf(stderr,
              "Mismatched type for field '%.*s'. Got: integer, expected %s\n",
              static_cast<int>(name.size()), name.data(),
              ElementaryTypeName(elementary_type));
      return false;
    }
  }
  return true;
}

// Writes an array of FieldElement (with the definition in "type") to the
// builder.  Returns the offset of the resulting table.
std::optional<Element> WriteObject(FlatbufferType type,
                                   const ::std::vector<FieldElement> &elements,
                                   flatbuffers::FlatBufferBuilder *fbb) {
  // End of a nested object!  Add it.
  if (type.IsTable()) {
    const flatbuffers::uoffset_t start = fbb->StartTable();

    ::std::vector<bool> fields_in_use(type.NumberFields(), false);

    for (const FieldElement &field_element : elements) {
      AddSingleElement(type, field_element, &fields_in_use, fbb);
    }

    return Element{
        flatbuffers::Offset<flatbuffers::String>{fbb->EndTable(start)}};
  } else if (type.IsStruct()) {
    // In order to write an inline struct, we need to fill out each field at the
    // correct position inline in memory. In order to do this, we retrieve the
    // offset/size of each field, and directly populate that memory with the
    // relevant value.
    std::vector<uint8_t> buffer(type.InlineSize(), 0);
    for (size_t field_index = 0;
         field_index < static_cast<size_t>(type.NumberFields());
         ++field_index) {
      auto it = std::find_if(elements.begin(), elements.end(),
                             [field_index](const FieldElement &field) {
                               return field.field_index ==
                                      static_cast<int>(field_index);
                             });
      if (it == elements.end()) {
        fprintf(stderr,
                "All fields must be specified for struct types (field %s "
                "missing).\n",
                type.FieldName(field_index).data());
        return std::nullopt;
      }

      uint8_t *field_data = buffer.data() + type.StructFieldOffset(field_index);
      const size_t field_size = type.FieldInlineSize(field_index);
      switch (it->element.type) {
        case Element::ElementType::INT:
          if (!SetStructElement(type, field_index, it->element.int_element,
                                field_data)) {
            return std::nullopt;
          }
          break;
        case Element::ElementType::DOUBLE:
          if (!SetStructElement(type, field_index, it->element.double_element,
                                field_data)) {
            return std::nullopt;
          }
          break;
        case Element::ElementType::STRUCT:
          CHECK_EQ(field_size, it->element.struct_data.size());
          memcpy(field_data, it->element.struct_data.data(), field_size);
          break;
        case Element::ElementType::OFFSET:
          LOG(FATAL)
              << "This should be unreachable; structs cannot contain offsets.";
          break;
      }
    }
    return Element{buffer};
  }
  LOG(FATAL) << "Unimplemented.";
}

// Class to parse JSON into a flatbuffer.
//
// The basic strategy is that we need to do everything backwards.  So we need to
// build up what we need to do fully in memory, then do it.
//
// The driver for this is that strings need to be fully created before the
// tables that use them.  Same for sub messages.  But, we only know we have them
// all when the structure ends.  So, store each sub message in a
// FieldElement and put them in the table at the end when we finish up
// each message.  Same goes for vectors.
class JsonParser {
 public:
  JsonParser(flatbuffers::FlatBufferBuilder *fbb) : fbb_(fbb) {}
  ~JsonParser() {}

  // Parses the json into a flatbuffer.  Returns either an empty vector on
  // error, or a vector with the flatbuffer data in it.
  flatbuffers::Offset<flatbuffers::Table> Parse(const std::string_view data,
                                                FlatbufferType type) {
    flatbuffers::uoffset_t end = 0;
    bool result = DoParse(type, data, &end);

    if (result) {
      // On success, finish the table and build the vector.
      return flatbuffers::Offset<flatbuffers::Table>(end);
    } else {
      return flatbuffers::Offset<flatbuffers::Table>(0);
    }
  }

 private:
  // Getters for in_vector (at the current level of the stack)
  bool in_vector() const { return !stack_.back().vector_elements.empty(); }

  // Parses the flatbuffer.  This is a second method so we can do easier
  // cleanup at the top level.  Returns true on success.
  bool DoParse(FlatbufferType type, const std::string_view data,
               flatbuffers::uoffset_t *table_end);

  // Adds *_value for the provided field.  If we are in a vector, queues the
  // data up in vector_elements.  Returns true on success.
  bool AddElement(int field_index, absl::int128 int_value);
  bool AddElement(int field_index, double double_value);
  bool AddElement(int field_index, const ::std::string &data);

  // Finishes a vector for the provided field index.  Returns true on success.
  bool FinishVector(int field_index);

  // Pushes an element as part of a vector.  Returns true on success.
  bool PushElement(flatbuffers::ElementaryType elementary_type,
                   absl::int128 int_value);
  bool PushElement(flatbuffers::ElementaryType elementary_type,
                   double double_value);
  bool PushElement(flatbuffers::ElementaryType elementary_type,
                   flatbuffers::Offset<flatbuffers::String> offset_value);
  bool PushElement(const FlatbufferType &type,
                   const std::vector<uint8_t> &struct_data);
  flatbuffers::FlatBufferBuilder *fbb_;

  // This holds the state information that is needed as you recurse into
  // nested structures.
  struct FlatBufferContext {
    // Type of the current type.
    FlatbufferType type;
    // The field index of the current field.
    int field_index;
    // Name of the current field.
    ::std::string field_name;

    // Field elements that need to be inserted.
    ::std::vector<FieldElement> elements;

    // For scalar types (not strings, and not nested tables), the vector ends
    // up being implemented as a start and end, and a block of data.  So we
    // can't just push offsets in as we go.  We either need to reproduce the
    // logic inside flatbuffers, or build up vectors of the data.  Vectors
    // will be a bit of extra stack space, but whatever.
    //
    // Strings and nested structures are vectors of offsets.
    // into the vector. Once you get to the end, you build up a vector and
    // push that into the field.
    //
    // If non-empty, we are in a vector.
    //
    // An entry in vector_elements will be added whenever we enter a JSON array
    // and removed when we exit the JSON array. vector_elements should only ever
    // be two elements at the most---flatbuffers do not currently support
    // arbitrarily-nested vectors, so the only situation where we have even 2
    // vector_elements elements are for representing non-unicode strings that
    // happen to be inside of a vector (which can end up looking like `[ "Valid
    // String", [ 255 ], "<-- Non-Unicode String" ]`
    ::std::vector<::std::vector<Element>> vector_elements;
  };
  ::std::vector<FlatBufferContext> stack_;
};

bool JsonParser::DoParse(FlatbufferType type, const std::string_view data,
                         flatbuffers::uoffset_t *table_end) {
  ::std::vector<FlatbufferType> stack;

  Tokenizer t(data);

  // Main loop.  Run until we get an end.
  while (true) {
    Tokenizer::TokenType token = t.Next();

    switch (token) {
      case Tokenizer::TokenType::kEnd:
        if (stack_.size() != 0) {
          fprintf(stderr, "Failed to unwind stack all the way\n");
          return false;
        } else {
          return true;
        }
        break;
      case Tokenizer::TokenType::kError:
        fprintf(stderr, "Encountered an error in the tokenizer\n");
        return false;
        break;

      case Tokenizer::TokenType::kStartObject:  // {
        if (stack_.size() == 0) {
          stack_.push_back({type, -1, "", {}, {}});
        } else {
          int field_index = stack_.back().field_index;

          if (!stack_.back().type.FieldIsSequence(field_index)) {
            fprintf(stderr, "Field '%s' is not a sequence\n",
                    stack_.back().field_name.c_str());
            return false;
          }

          if (in_vector() != stack_.back().type.FieldIsRepeating(field_index)) {
            fprintf(stderr,
                    "Field '%s' is%s supposed to be a vector, but is a %s.\n",
                    stack_.back().field_name.c_str(), in_vector() ? " not" : "",
                    in_vector() ? "vector" : "bare object");
            return false;
          }

          stack_.push_back(
              {stack_.back().type.FieldType(field_index), -1, "", {}, {}});
        }
        break;
      case Tokenizer::TokenType::kEndObject:  // }
        if (stack_.size() == 0) {
          // Somehow we popped more than we pushed.  Error.
          fprintf(stderr, "Empty stack\n");
          return false;
        } else {
          // End of a nested object!  Add it.
          std::optional<Element> object =
              WriteObject(stack_.back().type, stack_.back().elements, fbb_);
          if (!object.has_value()) {
            return false;
          }

          // We now want to talk about the parent structure.  Pop the child.
          stack_.pop_back();

          if (stack_.size() == 0) {
            CHECK_EQ(static_cast<int>(object->type),
                     static_cast<int>(Element::ElementType::OFFSET))
                << ": JSON parsing only supports parsing flatbuffer tables.";
            // Instead of queueing it up in the stack, return it through the
            // passed in variable.
            *table_end = object->offset_element.o;
          } else {
            // And now we can add it.
            const int field_index = stack_.back().field_index;

            // Do the right thing if we are in a vector.
            if (in_vector()) {
              CHECK_EQ(1u, stack_.back().vector_elements.size())
                  << ": Lost track of vector_elements book-keeping; when we "
                     "have a vector of objects vector_elements should just "
                     "have one entry.";
              stack_.back().vector_elements[0].emplace_back(
                  std::move(object.value()));
            } else {
              stack_.back().elements.emplace_back(field_index,
                                                  std::move(object.value()));
            }
          }
        }
        break;

      case Tokenizer::TokenType::kStartArray:  // [
        if (stack_.size() == 0) {
          fprintf(stderr,
                  "We don't support an array of structs at the root level.\n");
          return false;
        }
        // Sanity check that we aren't trying to make more than a single nested
        // layer of vectors (and that single layer of nesting is only permitted
        // for vectors of strings).
        if (in_vector() && stack_.back().vector_elements.size() >= 2u) {
          fprintf(stderr, "We don't support vectors of vectors.\n");
          return false;
        }
        stack_.back().vector_elements.emplace_back();
        break;
      case Tokenizer::TokenType::kEndArray: {  // ]
        if (!in_vector()) {
          fprintf(stderr, "Encountered ']' with no prior '['.\n");
          return false;
        }

        const int field_index = stack_.back().field_index;

        if (!FinishVector(field_index)) return false;
      } break;

      case Tokenizer::TokenType::kTrueValue:   // true
      case Tokenizer::TokenType::kFalseValue:  // false
      case Tokenizer::TokenType::kNumberValue: {
        bool is_int = true;
        double double_value;
        absl::int128 int_value;
        if (token == Tokenizer::TokenType::kTrueValue) {
          int_value = 1;
        } else if (token == Tokenizer::TokenType::kFalseValue) {
          int_value = 0;
        } else if (!t.FieldAsInt(&int_value)) {
          if (t.FieldAsDouble(&double_value)) {
            is_int = false;
          } else {
            fprintf(stderr, "Got a invalid number '%s'\n",
                    t.field_value().c_str());
            return false;
          }
        }

        const int field_index = stack_.back().field_index;

        if (is_int) {
          // No need to get too stressed about bool vs int.  Convert them all.
          absl::int128 val = int_value;
          if (!AddElement(field_index, val)) return false;
        } else {
          if (!AddElement(field_index, double_value)) return false;
        }
      } break;
      case Tokenizer::TokenType::kStringValue:  // string value
      {
        const int field_index = stack_.back().field_index;

        if (!AddElement(field_index, t.field_value())) return false;
      } break;

      case Tokenizer::TokenType::kNullValue: {
        if (stack_.size() == 0) {
          fprintf(stderr, "We don't support null at the root level.\n");
          return false;
        }
        stack_.back().field_name = t.field_name();
      } break;

      case Tokenizer::TokenType::kField:  // field name
      {
        stack_.back().field_name = t.field_name();
        stack_.back().field_index =
            stack_.back().type.FieldIndex(stack_.back().field_name.c_str());

        if (stack_.back().field_index == -1) {
          fprintf(stderr, "Invalid field name '%s'\n",
                  stack_.back().field_name.c_str());
          return false;
        }
      } break;
    }
  }
  return false;
}

bool JsonParser::AddElement(int field_index, absl::int128 int_value) {
  // We specifically allow strings to be represented as integer vectors.
  const bool field_permits_int_vector =
      stack_.back().type.FieldIsRepeating(field_index) ||
      stack_.back().type.FieldElementaryType(field_index) ==
          flatbuffers::ET_STRING;
  if (field_permits_int_vector != in_vector()) {
    fprintf(stderr,
            "Type and json disagree on if we are in a vector or not (JSON "
            "believes that we are%s in a vector for field '%s').\n",
            in_vector() ? "" : " not",
            stack_.back().type.FieldName(field_index).data());
    return false;
  }

  if (in_vector()) {
    stack_.back().vector_elements.back().emplace_back(int_value);
  } else {
    stack_.back().elements.emplace_back(field_index, int_value);
  }
  return true;
}

bool JsonParser::AddElement(int field_index, double double_value) {
  if (stack_.back().type.FieldIsRepeating(field_index) != in_vector()) {
    fprintf(stderr,
            "Type and json disagree on if we are in a vector or not (JSON "
            "believes that we are%s in a vector for field '%s').\n",
            in_vector() ? "" : " not",
            stack_.back().type.FieldName(field_index).data());
    return false;
  }

  if (in_vector()) {
    stack_.back().vector_elements.back().emplace_back(double_value);
  } else {
    stack_.back().elements.emplace_back(field_index, double_value);
  }
  return true;
}

bool JsonParser::AddElement(int field_index, const ::std::string &data) {
  if (stack_.back().type.FieldIsRepeating(field_index) != in_vector()) {
    fprintf(stderr,
            "Type and json disagree on if we are in a vector or not (JSON "
            "believes that we are%s in a vector for field '%s').\n",
            in_vector() ? "" : " not",
            stack_.back().type.FieldName(field_index).data());
    return false;
  }

  const flatbuffers::ElementaryType elementary_type =
      stack_.back().type.FieldElementaryType(field_index);
  switch (elementary_type) {
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
      if (stack_.back().type.FieldIsEnum(field_index)) {
        // We have an enum.
        const FlatbufferType type = stack_.back().type;
        const FlatbufferType enum_type = type.FieldType(field_index);
        CHECK(enum_type.IsEnum());

        const std::optional<absl::int128> int_value = enum_type.EnumValue(data);

        if (!int_value) {
          const std::string_view name = type.FieldName(field_index);
          fprintf(stderr, "Enum value '%s' not found for field '%.*s'\n",
                  data.c_str(), static_cast<int>(name.size()), name.data());
          return false;
        }

        if (in_vector()) {
          stack_.back().vector_elements.back().emplace_back(*int_value);
        } else {
          stack_.back().elements.emplace_back(field_index, *int_value);
        }
        return true;
      }
      break;
    case flatbuffers::ET_FLOAT:
    case flatbuffers::ET_DOUBLE: {
      // We *may* have an infinity/nan value that has been encoded as a string.
      double double_value;
      // If the string successfully converted to a double, we can consider it;
      // only accept nan's/inf's that are converted this way, as people should
      // not be providing regular floats/doubles as strings.
      if (absl::SimpleAtod(data, &double_value) &&
          !std::isfinite(double_value)) {
        if (in_vector()) {
          stack_.back().vector_elements.back().emplace_back(double_value);
        } else {
          stack_.back().elements.emplace_back(field_index, double_value);
        }
        return true;
      }
      break;
    }
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      break;
  }

  if (in_vector()) {
    stack_.back().vector_elements.back().emplace_back(fbb_->CreateString(data));
  } else {
    stack_.back().elements.emplace_back(field_index, fbb_->CreateString(data));
  }
  return true;
}

bool AddSingleElement(FlatbufferType type, const FieldElement &field_element,
                      ::std::vector<bool> *fields_in_use,
                      flatbuffers::FlatBufferBuilder *fbb) {
  if ((*fields_in_use)[field_element.field_index]) {
    const std::string_view name = type.FieldName(field_element.field_index);
    fprintf(stderr, "Duplicate field: '%.*s'\n", static_cast<int>(name.size()),
            name.data());
    return false;
  }

  (*fields_in_use)[field_element.field_index] = true;

  switch (field_element.element.type) {
    case Element::ElementType::INT:
      return AddSingleElement(type, field_element.field_index,
                              field_element.element.int_element, fbb);
    case Element::ElementType::DOUBLE:
      return AddSingleElement(type, field_element.field_index,
                              field_element.element.double_element, fbb);
    case Element::ElementType::OFFSET:
      return AddSingleElement(type, field_element.field_index,
                              field_element.element.offset_element, fbb);
    case Element::ElementType::STRUCT:
      return AddSingleElement(type, field_element.field_index,
                              field_element.element.struct_data, fbb);
  }
  return false;
}

bool AddSingleElement(FlatbufferType type, int field_index,
                      absl::int128 int_value,
                      flatbuffers::FlatBufferBuilder *fbb

) {
  flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
      static_cast<flatbuffers::voffset_t>(field_index));

  const flatbuffers::ElementaryType elementary_type =
      type.FieldElementaryType(field_index);
  switch (elementary_type) {
    case flatbuffers::ET_BOOL:
      fbb->AddElement<bool>(field_offset, static_cast<bool>(int_value));
      return true;
    case flatbuffers::ET_CHAR:
      fbb->AddElement<int8_t>(field_offset, static_cast<int8_t>(int_value));
      return true;
    case flatbuffers::ET_UCHAR:
      fbb->AddElement<uint8_t>(field_offset, static_cast<uint8_t>(int_value));
      return true;
    case flatbuffers::ET_SHORT:
      fbb->AddElement<int16_t>(field_offset, static_cast<int16_t>(int_value));
      return true;
    case flatbuffers::ET_USHORT:
      fbb->AddElement<uint16_t>(field_offset, static_cast<uint16_t>(int_value));
      return true;
    case flatbuffers::ET_INT:
      fbb->AddElement<int32_t>(field_offset, static_cast<int32_t>(int_value));
      return true;
    case flatbuffers::ET_UINT:
      fbb->AddElement<uint32_t>(field_offset, static_cast<uint32_t>(int_value));
      return true;
    case flatbuffers::ET_LONG:
      fbb->AddElement<int64_t>(field_offset, static_cast<int64_t>(int_value));
      return true;
    case flatbuffers::ET_ULONG:
      fbb->AddElement<uint64_t>(field_offset, static_cast<uint64_t>(int_value));
      return true;
      // The floating point cases occur when someone specifies an integer in the
      // JSON for a double field.
    case flatbuffers::ET_FLOAT:
      fbb->AddElement<float>(field_offset, static_cast<float>(int_value));
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb->AddElement<double>(field_offset, static_cast<double>(int_value));
      return true;
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_SEQUENCE: {
      const std::string_view name = type.FieldName(field_index);
      fprintf(stderr,
              "Mismatched type for field '%.*s'. Got: integer, expected %s\n",
              static_cast<int>(name.size()), name.data(),
              ElementaryTypeName(elementary_type));
      return false;
    }
  };
  return false;
}

bool AddSingleElement(FlatbufferType type, int field_index, double double_value,
                      flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
      static_cast<flatbuffers::voffset_t>(field_index));

  const flatbuffers::ElementaryType elementary_type =
      type.FieldElementaryType(field_index);
  switch (elementary_type) {
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE: {
      const std::string_view name = type.FieldName(field_index);
      fprintf(stderr,
              "Mismatched type for field '%.*s'. Got: double, expected %s\n",
              static_cast<int>(name.size()), name.data(),
              ElementaryTypeName(elementary_type));
      return false;
    }
    case flatbuffers::ET_FLOAT:
      fbb->AddElement<float>(field_offset, double_value);
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb->AddElement<double>(field_offset, double_value);
      return true;
  }
  return false;
}

bool AddSingleElement(FlatbufferType type, int field_index,
                      flatbuffers::Offset<flatbuffers::String> offset_element,
                      flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::voffset_t field_offset = flatbuffers::FieldIndexToOffset(
      static_cast<flatbuffers::voffset_t>(field_index));

  // Vectors will always be Offset<>'s.
  if (type.FieldIsRepeating(field_index)) {
    fbb->AddOffset(field_offset, offset_element);
    return true;
  }

  const flatbuffers::ElementaryType elementary_type =
      type.FieldElementaryType(field_index);
  switch (elementary_type) {
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_FLOAT:
    case flatbuffers::ET_DOUBLE: {
      const std::string_view name = type.FieldName(field_index);
      fprintf(stderr,
              "Mismatched type for field '%.*s'. Got: string, expected %s\n",
              static_cast<int>(name.size()), name.data(),
              ElementaryTypeName(elementary_type));
      return false;
    }
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      fbb->AddOffset(field_offset, offset_element);
      return true;
  }
  return false;
}

bool AddSingleElement(FlatbufferType type, int field_index,
                      const std::vector<uint8_t> &data,
                      flatbuffers::FlatBufferBuilder *fbb) {
  // Structs are always inline.
  // We have to do somewhat manual serialization to get the struct into place,
  // since the regular FlatBufferBuilder assumes that you will know the type of
  // the struct that you are constructing at compile time.
  fbb->Align(type.FieldType(field_index).Alignment());
  fbb->PushBytes(data.data(), data.size());
  fbb->AddStructOffset(flatbuffers::FieldIndexToOffset(
                           static_cast<flatbuffers::voffset_t>(field_index)),
                       fbb->GetSize());
  return true;
}

bool JsonParser::FinishVector(int field_index) {
  const FlatbufferType &current_type = stack_.back().type;
  CHECK(!stack_.back().vector_elements.empty());
  const std::vector<Element> vector_elements =
      std::move(stack_.back().vector_elements.back());
  stack_.back().vector_elements.pop_back();

  // Vectors have a start (unfortunately which needs to know the size)
  //
  // When the JSON we are parsing was generated with use_standard_json set,
  // strings may be represented as vectors of integers, in which case we will
  // serialize them as if they are a vector of bytes; since this is bypassing
  // some of the "normal" logic we end up needing to special-case it here
  // (because it is simpler to write the logic, we check here if we are in the
  // stage where we are constructing the parent vector).
  if (current_type.FieldIsRepeating(field_index) &&
      stack_.back().vector_elements.empty()) {
    const size_t inline_size = current_type.FieldInlineSize(field_index);
    const size_t alignment = current_type.FieldInlineAlignment(field_index);
    fbb_->StartVector(vector_elements.size(), inline_size,
                      /*align=*/alignment);
  } else {
    // Strings have 1-byte elements with only a 1-byte alignment requirement.
    fbb_->StartVector(vector_elements.size(), /*element size*/ 1u,
                      /*align*/ 1u);
  }

  const flatbuffers::ElementaryType elementary_type =
      stack_.back().type.FieldElementaryType(field_index);

  // Then the data (in reverse order for some reason...)
  for (size_t i = vector_elements.size(); i > 0;) {
    const Element &element = vector_elements[--i];
    switch (element.type) {
      case Element::ElementType::INT:
        if (!PushElement(elementary_type, element.int_element)) return false;
        break;
      case Element::ElementType::DOUBLE:
        if (!PushElement(elementary_type, element.double_element)) return false;
        break;
      case Element::ElementType::OFFSET:
        if (!PushElement(elementary_type, element.offset_element)) return false;
        break;
      case Element::ElementType::STRUCT:
        if (!PushElement(stack_.back().type.FieldType(field_index),
                         element.struct_data))
          return false;
        break;
    }
  }

  // Then an End which is placed into the buffer the same as any other offset.
  // Note that the type of the Offset being flatbuffers::String is immaterial.
  const flatbuffers::Offset<flatbuffers::String> vector_offset =
      fbb_->EndVector(vector_elements.size());
  if (in_vector()) {
    stack_.back().vector_elements.back().emplace_back(vector_offset);
  } else {
    stack_.back().elements.emplace_back(field_index, vector_offset);
  }
  return true;
}

bool JsonParser::PushElement(flatbuffers::ElementaryType elementary_type,
                             absl::int128 int_value) {
  switch (elementary_type) {
    case flatbuffers::ET_BOOL:
      fbb_->PushElement<bool>(static_cast<bool>(int_value));
      return true;
    case flatbuffers::ET_CHAR:
      fbb_->PushElement<int8_t>(static_cast<int8_t>(int_value));
      return true;
    case flatbuffers::ET_UCHAR:
      fbb_->PushElement<uint8_t>(static_cast<uint8_t>(int_value));
      return true;
    case flatbuffers::ET_SHORT:
      fbb_->PushElement<int16_t>(static_cast<int16_t>(int_value));
      return true;
    case flatbuffers::ET_USHORT:
      fbb_->PushElement<uint16_t>(static_cast<uint16_t>(int_value));
      return true;
    case flatbuffers::ET_INT:
      fbb_->PushElement<int32_t>(static_cast<int32_t>(int_value));
      return true;
    case flatbuffers::ET_UINT:
      fbb_->PushElement<uint32_t>(static_cast<uint32_t>(int_value));
      return true;
    case flatbuffers::ET_LONG:
      fbb_->PushElement<int64_t>(static_cast<int64_t>(int_value));
      return true;
    case flatbuffers::ET_ULONG:
      fbb_->PushElement<uint64_t>(static_cast<uint64_t>(int_value));
      return true;
    case flatbuffers::ET_FLOAT:
      fbb_->PushElement<float>(static_cast<float>(int_value));
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb_->PushElement<double>(static_cast<double>(int_value));
      return true;
    case flatbuffers::ET_STRING:
      // Strings may be represented as vectors of integers, in which case we
      // will serialize them as if they are a vector of bytes.
      fbb_->PushElement<uint8_t>(static_cast<uint8_t>(int_value));
      return true;
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_SEQUENCE:
      fprintf(stderr,
              "Mismatched type for field '%s'. Got: integer, expected %s\n",
              stack_.back().field_name.c_str(),
              ElementaryTypeName(elementary_type));
      return false;
  };
  return false;
}

bool JsonParser::PushElement(flatbuffers::ElementaryType elementary_type,
                             double double_value) {
  switch (elementary_type) {
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      fprintf(stderr,
              "Mismatched type for field '%s'. Got: double, expected %s\n",
              stack_.back().field_name.c_str(),
              ElementaryTypeName(elementary_type));
      return false;
    case flatbuffers::ET_FLOAT:
      fbb_->PushElement<float>(double_value);
      return true;
    case flatbuffers::ET_DOUBLE:
      fbb_->PushElement<double>(double_value);
      return true;
  }
  return false;
}

bool JsonParser::PushElement(const FlatbufferType &type,
                             const std::vector<uint8_t> &struct_data) {
  // To add a struct to a vector, we just need to get the relevant bytes pushed
  // straight into the builder. The FlatBufferBuilder normally expects that you
  // will know the type of your struct at compile-time, so doesn't have a
  // first-class way to do this.
  fbb_->Align(type.Alignment());
  fbb_->PushBytes(struct_data.data(), struct_data.size());
  return true;
}

bool JsonParser::PushElement(
    flatbuffers::ElementaryType elementary_type,
    flatbuffers::Offset<flatbuffers::String> offset_value) {
  switch (elementary_type) {
    case flatbuffers::ET_UTYPE:
    case flatbuffers::ET_BOOL:
    case flatbuffers::ET_CHAR:
    case flatbuffers::ET_UCHAR:
    case flatbuffers::ET_SHORT:
    case flatbuffers::ET_USHORT:
    case flatbuffers::ET_INT:
    case flatbuffers::ET_UINT:
    case flatbuffers::ET_LONG:
    case flatbuffers::ET_ULONG:
    case flatbuffers::ET_FLOAT:
    case flatbuffers::ET_DOUBLE:
      fprintf(stderr,
              "Mismatched type for field '%s'. Got: sequence, expected %s\n",
              stack_.back().field_name.c_str(),
              ElementaryTypeName(elementary_type));
      return false;
    case flatbuffers::ET_STRING:
    case flatbuffers::ET_SEQUENCE:
      fbb_->PushElement(offset_value);
      return true;
  }
  return false;
}

}  // namespace

flatbuffers::Offset<flatbuffers::Table> JsonToFlatbuffer(
    const std::string_view data, FlatbufferType type,
    flatbuffers::FlatBufferBuilder *fbb) {
  JsonParser p(fbb);
  return p.Parse(data, type);
}

flatbuffers::DetachedBuffer JsonToFlatbuffer(const std::string_view data,
                                             FlatbufferType type) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  const flatbuffers::Offset<flatbuffers::Table> result =
      JsonToFlatbuffer(data, type, &fbb);
  if (result.o != 0) {
    fbb.Finish(result);

    return fbb.Release();
  } else {
    // Otherwise return an empty vector.
    return flatbuffers::DetachedBuffer();
  }
}

namespace {

enum class UseStandardJson { kYes, kNo };

// A visitor which manages skipping the contents of vectors that are longer than
// a specified threshold.
class TruncatingStringVisitor : public flatbuffers::IterationVisitor {
 public:
  TruncatingStringVisitor(size_t max_vector_size, std::string delimiter,
                          bool quotes, std::string indent, bool vdelimited,
                          UseStandardJson standard_json,
                          std::optional<int> float_precision)
      : max_vector_size_(max_vector_size),
        to_string_(delimiter, quotes, indent, vdelimited),
        use_standard_json_(standard_json == UseStandardJson::kYes),
        float_precision_(float_precision) {}
  ~TruncatingStringVisitor() override {}

  void StartSequence() override {
    if (should_skip()) return;
    to_string_.StartSequence();
  }
  void EndSequence() override {
    if (should_skip()) return;
    to_string_.EndSequence();
  }
  void Field(size_t field_idx, size_t set_idx, flatbuffers::ElementaryType type,
             bool is_repeating, const flatbuffers::TypeTable *type_table,
             const char *name, const uint8_t *val) override {
    if (should_skip()) return;
    to_string_.Field(field_idx, set_idx, type, is_repeating, type_table, name,
                     val);
  }
  void UType(uint8_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.UType(value, name);
  }
  void Bool(bool value) override {
    if (should_skip()) return;
    to_string_.Bool(value);
  }
  void Char(int8_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.Char(value, name);
  }
  void UChar(uint8_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.UChar(value, name);
  }
  void Short(int16_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.Short(value, name);
  }
  void UShort(uint16_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.UShort(value, name);
  }
  void Int(int32_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.Int(value, name);
  }
  void UInt(uint32_t value, const char *name) override {
    if (should_skip()) return;
    to_string_.UInt(value, name);
  }
  void Long(int64_t value) override {
    if (should_skip()) return;
    to_string_.Long(value);
  }
  void ULong(uint64_t value) override {
    if (should_skip()) return;
    to_string_.ULong(value);
  }

  // Because the flatbuffers::IterationVisitor interface (and thus both this
  // function and `to_string_`) uses separate `Float()` and `Double()` methods
  // for floats versus doubles we end up having a bunch of common code for the
  // two implementations which is only different in a couple of names that are
  // somewhat nested.
  // This code could be shared via templating, but given the nature of the code,
  // it seems clearer to use a macro.
#define STRINGIFY_FLOAT(type, Method)                                       \
  void Method(type value) override {                                        \
    if (should_skip()) return;                                              \
    if (use_standard_json_ && !std::isfinite(value)) {                      \
      /* When using standards-compliant JSON, use strings to represent      \
       * numbers.*/                                                         \
      if (std::isnan(value)) {                                              \
        to_string_.s += std::signbit(value) ? "\"-nan\"" : "\"nan\"";       \
      } else {                                                              \
        DCHECK(std::isinf(value))                                           \
            << "All non-finite, non-nan floats should be infinite; got "    \
            << value;                                                       \
        to_string_.s += std::signbit(value) ? "\"-inf\"" : "\"inf\"";       \
      }                                                                     \
      return;                                                               \
    }                                                                       \
    if (value == 0.0 && std::signbit(value)) {                              \
      to_string_.s += "-0.0";                                               \
      return;                                                               \
    }                                                                       \
    if (float_precision_.has_value()) {                                     \
      to_string_.s +=                                                       \
          util::FormatFloat(static_cast<double>(value), *float_precision_); \
    } else {                                                                \
      to_string_.Method(value);                                             \
    }                                                                       \
  }

  STRINGIFY_FLOAT(float, Float);
  STRINGIFY_FLOAT(double, Double);
#undef STRINGIFY_FLOAT

  void String(const flatbuffers::String *value) override {
    if (should_skip()) return;
    if (!aos::util::ValidateUtf8(value->string_view()) && use_standard_json_) {
      // If the input string contains non-UTF-8 characters, and
      // standards-compliant JSON has been requested, we will render
      // the string as a uint8 vector.
      StartVector(value->size());
      for (size_t index = 0; index < value->size(); ++index) {
        Element(index, flatbuffers::ElementaryType::ET_UCHAR,
                /*type_table=*/nullptr, /*val (appears to be unused)=*/nullptr);
        UChar(value->Get(index), /*enum name*/ nullptr);
      }
      EndVector();
      return;
    }
    to_string_.String(value);
  }
  void Unknown(const uint8_t *value) override {
    if (should_skip()) return;
    to_string_.Unknown(value);
  }
  void Element(size_t i, flatbuffers::ElementaryType type,
               const flatbuffers::TypeTable *type_table,
               const uint8_t *val) override {
    if (should_skip()) return;
    to_string_.Element(i, type, type_table, val);
  }

  virtual void StartVector(size_t size) override {
    if (should_skip()) {
      ++skip_levels_;
      return;
    }
    if (size > max_vector_size_) {
      ++skip_levels_;
      to_string_.s += "[ \"... " + std::to_string(size) + " elements ...\" ]";
      return;
    }
    to_string_.StartVector(size);
  }
  virtual void EndVector() override {
    if (should_skip()) {
      --skip_levels_;
      return;
    }
    to_string_.EndVector();
  }

  std::string &string() { return to_string_.s; }

 private:
  bool should_skip() const { return skip_levels_ > 0; }

  const size_t max_vector_size_;
  flatbuffers::ToStringVisitor to_string_;
  int skip_levels_ = 0;
  bool use_standard_json_;
  std::optional<int> float_precision_;
};

}  // namespace

::std::string TableFlatbufferToJson(const flatbuffers::Table *t,
                                    const ::flatbuffers::TypeTable *typetable,
                                    JsonOptions json_options) {
  // It is pretty common to get passed in a nullptr when a test fails.  Rather
  // than CHECK, return a more user friendly result.
  if (t == nullptr) {
    return "null";
  }
  TruncatingStringVisitor tostring_visitor(
      json_options.max_vector_size, json_options.multi_line ? "\n" : " ", true,
      json_options.multi_line ? " " : "", json_options.multi_line,
      json_options.use_standard_json ? UseStandardJson::kYes
                                     : UseStandardJson::kNo,
      json_options.float_precision);
  flatbuffers::IterateObject(reinterpret_cast<const uint8_t *>(t), typetable,
                             &tostring_visitor);
  return tostring_visitor.string();
}

}  // namespace aos
