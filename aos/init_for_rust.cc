#include "aos/init_for_rust.h"

#include <string.h>

#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/initialize.h"

#include "aos/init.h"

extern "C" {

void aos_init_from_rust() {
  ABSL_CHECK(!aos::IsInitialized()) << "Only initialize once.";
  absl::InitializeLog();
  aos::MarkInitialized();
}

size_t aos_get_cpp_flags(aos_flag_info_t **flags_out) {
  auto info = absl::GetAllFlags();
  std::vector<aos_flag_info_t> out;
  for (const auto &flag : info) {
    const char *type = "unknown";
    if (flag.second->IsOfType<float>())
      type = "float";
    else if (flag.second->IsOfType<double>())
      type = "double";
    else if (flag.second->IsOfType<bool>())
      type = "bool";
    else if (flag.second->IsOfType<uint8_t>())
      type = "uint8_t";
    else if (flag.second->IsOfType<int8_t>())
      type = "int8_t";
    else if (flag.second->IsOfType<uint16_t>())
      type = "uint16_t";
    else if (flag.second->IsOfType<int16_t>())
      type = "int16_t";
    else if (flag.second->IsOfType<uint32_t>())
      type = "uint32_t";
    else if (flag.second->IsOfType<int32_t>())
      type = "int32_t";
    else if (flag.second->IsOfType<uint64_t>())
      type = "uint64_t";
    else if (flag.second->IsOfType<int64_t>())
      type = "int64_t";
    else if (flag.second->IsOfType<std::string>())
      type = "string";
    else if (flag.second->IsOfType<std::vector<std::string>>())
      type = "vector<string>";

    auto copy_string_view = [](absl::string_view sv) -> char * {
      char *res = (char *)malloc(sv.size() + 1);
      memcpy(res, sv.data(), sv.size());
      res[sv.size()] = '\0';
      return res;
    };

    aos_flag_info_t out_flag;
    out_flag.name = copy_string_view(flag.second->Name());
    out_flag.type = copy_string_view(type);
    out_flag.description = copy_string_view(flag.second->Help());
    out_flag.default_value = copy_string_view(flag.second->DefaultValue());
    out_flag.filename = copy_string_view(flag.second->Filename());
    out.push_back(out_flag);
  }

  size_t size = out.size();
  *flags_out = (aos_flag_info_t *)malloc(sizeof(aos_flag_info_t) * size);
  memcpy(*flags_out, out.data(), sizeof(aos_flag_info_t) * size);
  return size;
}

void aos_free_cpp_flags(aos_flag_info_t *flags, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    free(flags[i].name);
    free(flags[i].type);
    free(flags[i].description);
    free(flags[i].default_value);
    free(flags[i].filename);
  }
  free(flags);
}

bool aos_set_command_line_option(const char *name, const char *value) {
  absl::CommandLineFlag *flag = absl::FindCommandLineFlag(name);
  if (flag == nullptr) {
    return false;
  }
  std::string error;
  return flag->ParseFrom(value, &error);
}

char *aos_get_command_line_option(const char *name) {
  absl::CommandLineFlag *flag = absl::FindCommandLineFlag(name);
  if (flag == nullptr) {
    return nullptr;
  }
  return strdup(flag->CurrentValue().c_str());
}

void aos_free_command_line_option(char *value) { free(value); }
}
