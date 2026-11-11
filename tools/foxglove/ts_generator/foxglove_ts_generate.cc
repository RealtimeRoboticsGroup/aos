// Generates Foxglove-compatible TypeScript type definitions from FlatBuffer
// reflection schemas (.bfbs files).  The generated types are simplified
// versions of the FlatBuffer types, suitable for use in Foxglove extensions.
//
// Usage:
//   foxglove_ts_generate --reflection_bfbs=schema.bfbs
//       --output_file=output_foxglove.ts --base_file_name=my_schema.fbs
//       --import="aos/configuration.fbs=./aos/configuration_foxglove"

#include <stdlib.h>

#include <algorithm>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "flatbuffers/reflection_generated.h"

#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"

ABSL_FLAG(std::string, reflection_bfbs, "",
          "Path to the .bfbs reflection file.");
ABSL_FLAG(std::string, output_file, "", "Path to the output .ts file.");
ABSL_FLAG(std::string, base_file_name, "",
          "Name of the .fbs file to generate code for, used as a hint when "
          "the schema has no root table.");
ABSL_FLAG(std::vector<std::string>, import, {},
          "Import mappings in the form 'fbs_path=ts_import_path'. "
          "For example: 'aos/configuration.fbs=./aos/configuration_foxglove'. "
          "This flag can be repeated.");

namespace foxglove_ts {
namespace {

// Returns the short type name from a fully qualified FlatBuffer name.
// e.g., "aos.timing.Sender" -> "Sender"
std::string ShortName(std::string_view full_name) {
  const size_t pos = full_name.rfind('.');
  if (pos == std::string_view::npos) {
    return std::string(full_name);
  }
  return std::string(full_name.substr(pos + 1));
}

// Maps a FlatBuffer scalar base type to a TypeScript type string.
std::string ScalarTsType(reflection::BaseType type) {
  switch (type) {
    case reflection::BaseType::Bool:
      return "boolean";
    case reflection::BaseType::Byte:
    case reflection::BaseType::UByte:
    case reflection::BaseType::Short:
    case reflection::BaseType::UShort:
    case reflection::BaseType::Int:
    case reflection::BaseType::UInt:
    case reflection::BaseType::Float:
    case reflection::BaseType::Double:
      return "number";
    case reflection::BaseType::Long:
    case reflection::BaseType::ULong:
      return "bigint";
    case reflection::BaseType::String:
      return "string";
    default:
      ABSL_LOG(FATAL) << "Unsupported scalar base type: "
                      << static_cast<int>(type);
      return "unknown";
  }
}

// Returns true if the given base type is a scalar (not a compound type).
bool IsScalar(reflection::BaseType type) {
  switch (type) {
    case reflection::BaseType::Bool:
    case reflection::BaseType::Byte:
    case reflection::BaseType::UByte:
    case reflection::BaseType::Short:
    case reflection::BaseType::UShort:
    case reflection::BaseType::Int:
    case reflection::BaseType::UInt:
    case reflection::BaseType::Long:
    case reflection::BaseType::ULong:
    case reflection::BaseType::Float:
    case reflection::BaseType::Double:
      return true;
    default:
      return false;
  }
}

// Resolves the TypeScript type for a field, given the schema.
// Returns the TS type string and whether the type needs an import from another
// file.
struct ResolvedType {
  std::string ts_type;
  // If non-empty, this type is declared in a different .fbs file and needs
  // to be imported.
  std::string declaration_file;
  // If non-empty, a comment that should be emitted on the line above the
  // field that resolved to this type.  Used to flag surprising mappings
  // (e.g. a `[Color]` field where `Color : ubyte` becomes `Uint8Array`).
  std::string field_comment;
};

ResolvedType ResolveFieldType(const reflection::Schema *schema,
                              const reflection::Field *field) {
  const reflection::Type *type = field->type();
  const reflection::BaseType base_type = type->base_type();

  if (base_type == reflection::BaseType::String) {
    return {.ts_type = "string", .declaration_file = "", .field_comment = ""};
  }

  // Enums have a scalar base type but a non-negative index.
  if (IsScalar(base_type) && type->index() >= 0) {
    const reflection::Enum *enum_def = schema->enums()->Get(type->index());
    const std::string short_name = ShortName(enum_def->name()->string_view());
    std::string decl_file;
    if (enum_def->declaration_file() != nullptr) {
      decl_file = enum_def->declaration_file()->str();
    }
    return {.ts_type = short_name,
            .declaration_file = decl_file,
            .field_comment = ""};
  }

  if (IsScalar(base_type)) {
    return {.ts_type = ScalarTsType(base_type),
            .declaration_file = "",
            .field_comment = ""};
  }

  if (base_type == reflection::BaseType::Obj) {
    const reflection::Object *obj = schema->objects()->Get(type->index());
    const std::string short_name = ShortName(obj->name()->string_view());
    std::string decl_file;
    if (obj->declaration_file() != nullptr) {
      decl_file = obj->declaration_file()->str();
    }
    return {.ts_type = short_name,
            .declaration_file = decl_file,
            .field_comment = ""};
  }

  if (base_type == reflection::BaseType::Union) {
    // Unions are not well-supported in Foxglove; treat as unknown.
    return {.ts_type = "unknown", .declaration_file = "", .field_comment = ""};
  }

  if (base_type == reflection::BaseType::Vector ||
      base_type == reflection::BaseType::Array) {
    const reflection::BaseType element_type = type->element();
    // For vectors, `type->index()` is the enum index when the element is an
    // enum, and -1 otherwise.
    const bool element_is_enum = IsScalar(element_type) && type->index() >= 0;

    // [ubyte] (raw or enum-backed) is serialized as Uint8Array because
    // FlatBuffers packs a contiguous byte array.  When the element type is an
    // enum we emit an explanatory comment so the surprising mapping is
    // self-documenting in the generated .ts file.
    if (element_type == reflection::BaseType::UByte) {
      if (element_is_enum) {
        const reflection::Enum *enum_def = schema->enums()->Get(type->index());
        const std::string enum_name =
            ShortName(enum_def->name()->string_view());
        // No declaration_file is returned: the generated field is typed as
        // `Uint8Array` and the enum name is not referenced, so there's
        // nothing to import.
        return {.ts_type = "Uint8Array",
                .declaration_file = "",
                .field_comment = "// Serialized as Uint8Array because the " +
                                 enum_name +
                                 " enum's underlying type is `ubyte`."};
      }
      return {
          .ts_type = "Uint8Array", .declaration_file = "", .field_comment = ""};
    }

    // Vectors of non-ubyte enums map to a regular TS array of the enum type.
    if (element_is_enum) {
      const reflection::Enum *enum_def = schema->enums()->Get(type->index());
      const std::string enum_name = ShortName(enum_def->name()->string_view());
      std::string decl_file;
      if (enum_def->declaration_file() != nullptr) {
        decl_file = enum_def->declaration_file()->str();
      }
      return {.ts_type = enum_name + "[]",
              .declaration_file = decl_file,
              .field_comment = ""};
    }

    if (IsScalar(element_type)) {
      return {.ts_type = ScalarTsType(element_type) + "[]",
              .declaration_file = "",
              .field_comment = ""};
    }

    if (element_type == reflection::BaseType::String) {
      return {
          .ts_type = "string[]", .declaration_file = "", .field_comment = ""};
    }

    if (element_type == reflection::BaseType::Obj) {
      const reflection::Object *obj = schema->objects()->Get(type->index());
      const std::string short_name = ShortName(obj->name()->string_view());
      std::string decl_file;
      if (obj->declaration_file() != nullptr) {
        decl_file = obj->declaration_file()->str();
      }
      return {.ts_type = short_name + "[]",
              .declaration_file = decl_file,
              .field_comment = ""};
    }

    return {
        .ts_type = "unknown[]", .declaration_file = "", .field_comment = ""};
  }

  if (base_type == reflection::BaseType::UType) {
    return {.ts_type = "number", .declaration_file = "", .field_comment = ""};
  }

  return {.ts_type = "unknown", .declaration_file = "", .field_comment = ""};
}

// Determines the "base" file -- the .fbs file we're generating code for.
std::string_view DetermineBaseFile(const reflection::Schema *schema,
                                   std::string_view base_file_name) {
  // Try to find the file from the root table first.
  if (schema->root_table() != nullptr &&
      schema->root_table()->declaration_file() != nullptr) {
    return schema->root_table()->declaration_file()->string_view();
  }

  // Fall back to matching against base_file_name.
  // Check objects first.
  if (schema->objects() != nullptr) {
    for (const reflection::Object *obj : *schema->objects()) {
      if (obj->declaration_file() != nullptr) {
        std::string_view decl = obj->declaration_file()->string_view();
        if (decl.size() >= base_file_name.size() &&
            decl.substr(decl.size() - base_file_name.size()) ==
                base_file_name) {
          return decl;
        }
      }
    }
  }

  // Check enums.
  if (schema->enums() != nullptr) {
    for (const reflection::Enum *e : *schema->enums()) {
      if (e->declaration_file() != nullptr) {
        std::string_view decl = e->declaration_file()->string_view();
        if (decl.size() >= base_file_name.size() &&
            decl.substr(decl.size() - base_file_name.size()) ==
                base_file_name) {
          return decl;
        }
      }
    }
  }

  return base_file_name;
}

// Parses import mappings from command-line flags.
// Returns a map from .fbs declaration_file path to TypeScript import path.
std::map<std::string, std::string> ParseImportMappings(
    const std::vector<std::string> &imports) {
  std::map<std::string, std::string> result;
  for (const std::string &mapping : imports) {
    const size_t eq_pos = mapping.find('=');
    ABSL_CHECK(eq_pos != std::string::npos)
        << "Invalid import mapping (missing '='): " << mapping;
    const std::string fbs_path = mapping.substr(0, eq_pos);
    const std::string ts_path = mapping.substr(eq_pos + 1);
    result[fbs_path] = ts_path;
  }
  return result;
}

// Returns true if path ends with suffix at a path component boundary, so
// "//foo/data.fbs" ends with "data.fbs" but not with "a.fbs".
bool EndsWithPathComponents(std::string_view path, std::string_view suffix) {
  if (suffix.empty() || path.size() < suffix.size() ||
      path.substr(path.size() - suffix.size()) != suffix) {
    return false;
  }
  return path.size() == suffix.size() ||
         path[path.size() - suffix.size() - 1] == '/';
}

// Checks whether a declaration_file path matches a given fbs_path key, allowing
// either one to carry extra leading directories (e.g. flatc's "//" prefix).
bool DeclarationFileMatches(std::string_view declaration_file,
                            std::string_view fbs_path) {
  return EndsWithPathComponents(declaration_file, fbs_path) ||
         EndsWithPathComponents(fbs_path, declaration_file);
}

// Finds the import path for a declaration file, or returns empty if it's a
// local type.
std::string FindImportPath(
    const std::map<std::string, std::string> &import_mappings,
    std::string_view declaration_file, std::string_view base_file) {
  // If the declaration file is the same as our base file, it's local.
  if (declaration_file == base_file) {
    return "";
  }

  for (const auto &[fbs_path, ts_path] : import_mappings) {
    if (DeclarationFileMatches(declaration_file, fbs_path)) {
      return ts_path;
    }
  }

  // If we don't have a mapping, this type is external but unmapped.
  // We'll still generate the type reference but won't add an import.
  ABSL_LOG(WARNING) << "No import mapping for declaration file: "
                    << declaration_file
                    << ". Types from this file will be referenced but not "
                       "imported.";
  return "";
}

std::string GenerateTypeScript(const reflection::Schema *schema,
                               std::string_view base_file_name,
                               const std::vector<std::string> &imports) {
  const std::map<std::string, std::string> import_mappings =
      ParseImportMappings(imports);
  const std::string_view base_file = DetermineBaseFile(schema, base_file_name);

  std::ostringstream out;
  out << "// Auto-generated Foxglove-compatible TypeScript types.\n";
  out << "// Source: " << base_file << "\n";
  out << "// DO NOT EDIT.\n\n";

  // Collect all types that need to be imported, grouped by import path.
  std::map<std::string, std::vector<std::string>> needed_imports;

  // First pass: collect enums that belong to this file.
  struct EnumInfo {
    std::string name;
    const reflection::Enum *enum_def;
  };
  std::vector<EnumInfo> local_enums;

  if (schema->enums() != nullptr) {
    for (const reflection::Enum *e : *schema->enums()) {
      if (e->declaration_file() == nullptr) {
        continue;
      }
      if (e->declaration_file()->string_view() != base_file) {
        continue;
      }
      // Skip unions.
      if (e->is_union()) {
        continue;
      }
      local_enums.push_back(
          {.name = ShortName(e->name()->string_view()), .enum_def = e});
    }
  }

  // First pass: collect objects that belong to this file.
  struct ObjectInfo {
    std::string name;
    const reflection::Object *object;
  };
  std::vector<ObjectInfo> local_objects;

  if (schema->objects() != nullptr) {
    for (const reflection::Object *obj : *schema->objects()) {
      if (obj->declaration_file() == nullptr) {
        continue;
      }
      if (obj->declaration_file()->string_view() != base_file) {
        continue;
      }
      local_objects.push_back(
          {.name = ShortName(obj->name()->string_view()), .object = obj});
    }
  }

  // Second pass: collect needed imports from fields in local objects.
  for (const ObjectInfo &obj_info : local_objects) {
    const reflection::Object *obj = obj_info.object;
    if (obj->fields() == nullptr) {
      continue;
    }
    for (const reflection::Field *field : *obj->fields()) {
      if (field->deprecated()) {
        continue;
      }

      ResolvedType resolved = ResolveFieldType(schema, field);
      std::string decl_file = resolved.declaration_file;
      if (!decl_file.empty() && decl_file != base_file) {
        // Strip array suffix for import.
        std::string type_name = resolved.ts_type;
        if (type_name.size() > 2 &&
            type_name.substr(type_name.size() - 2) == "[]") {
          type_name = type_name.substr(0, type_name.size() - 2);
        }
        const std::string import_path =
            FindImportPath(import_mappings, decl_file, base_file);
        if (!import_path.empty()) {
          needed_imports[import_path].push_back(type_name);
        }
      }
    }
  }

  // Emit import statements.
  for (auto &[path, types] : needed_imports) {
    // Deduplicate types.
    std::sort(types.begin(), types.end());
    types.erase(std::unique(types.begin(), types.end()), types.end());

    out << "import { ";
    for (size_t i = 0; i < types.size(); ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << types[i];
    }
    out << " } from '" << path << "';\n";
  }
  if (!needed_imports.empty()) {
    out << "\n";
  }

  // Emit enums.
  for (const EnumInfo &enum_info : local_enums) {
    const reflection::Enum *e = enum_info.enum_def;
    out << "export enum " << enum_info.name << " {\n";
    if (e->values() != nullptr) {
      for (const reflection::EnumVal *val : *e->values()) {
        out << "\t" << val->name()->str() << " = " << val->value() << ",\n";
      }
    }
    out << "}\n\n";
  }

  // Emit tables/structs as types.
  for (const ObjectInfo &obj_info : local_objects) {
    const reflection::Object *obj = obj_info.object;
    const bool is_struct = obj->is_struct();

    out << "export type " << obj_info.name << " = {\n";

    if (obj->fields() == nullptr) {
      out << "};\n\n";
      continue;
    }

    // Sort fields by their id for stable output.
    std::vector<const reflection::Field *> sorted_fields;
    for (const reflection::Field *field : *obj->fields()) {
      if (field->deprecated()) {
        continue;
      }
      sorted_fields.push_back(field);
    }
    std::sort(sorted_fields.begin(), sorted_fields.end(),
              [](const reflection::Field *a, const reflection::Field *b) {
                return a->id() < b->id();
              });

    for (const reflection::Field *field : sorted_fields) {
      const std::string field_name = field->name()->str();

      ResolvedType resolved = ResolveFieldType(schema, field);
      std::string ts_type = resolved.ts_type;
      std::string field_comment = resolved.field_comment;

      if (!field_comment.empty()) {
        out << "\t" << field_comment << "\n";
      }

      // Table fields are optional, struct fields are required.
      const bool optional = !is_struct;
      out << "\t" << field_name;
      if (optional) {
        out << "?";
      }
      out << ": " << ts_type << ";\n";
    }

    out << "};\n\n";
  }

  return out.str();
}

}  // namespace

int Main() {
  const std::string bfbs_path = absl::GetFlag(FLAGS_reflection_bfbs);
  const std::string output_path = absl::GetFlag(FLAGS_output_file);
  const std::string base_file_name = absl::GetFlag(FLAGS_base_file_name);
  const std::vector<std::string> imports = absl::GetFlag(FLAGS_import);

  ABSL_CHECK(!bfbs_path.empty()) << "--reflection_bfbs is required";
  ABSL_CHECK(!output_path.empty()) << "--output_file is required";
  ABSL_CHECK(!base_file_name.empty()) << "--base_file_name is required";

  aos::FlatbufferVector<reflection::Schema> schema =
      aos::FileToFlatbuffer<reflection::Schema>(bfbs_path);

  const std::string ts_code =
      GenerateTypeScript(&schema.message(), base_file_name, imports);

  aos::util::WriteStringToFileOrDie(output_path, ts_code);
  return EXIT_SUCCESS;
}

}  // namespace foxglove_ts

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return foxglove_ts::Main();
}
