#include "aos/uuid.h"

#include <sys/types.h>

#include <array>
#include <cstring>
#include <random>
#include <string_view>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#if defined(__APPLE__)
#include <sys/sysctl.h>
#endif

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <memory>
#endif

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

ABSL_FLAG(std::string, boot_uuid, "",
          "If set, override the boot UUID to have this value instead.");

namespace aos {
namespace {
void ToHex(const uint8_t *val, char *result, size_t count) {
  while (count > 0) {
    int upper = ((*val) >> 4) & 0xf;
    if (upper < 10) {
      result[0] = upper + '0';
    } else {
      result[0] = upper - 10 + 'a';
    }

    int lower = (*val) & 0xf;
    if (lower < 10) {
      result[1] = lower + '0';
    } else {
      result[1] = lower - 10 + 'a';
    }

    ++val;
    result += 2;
    --count;
  }
}

void FromHex(const char *val, uint8_t *result, size_t count) {
  while (count > 0) {
    ABSL_CHECK((val[0] >= '0' && val[0] <= '9') ||
               (val[0] >= 'a' && val[0] <= 'f'))
        << ": Invalid hex '" << val[0] << "'";
    ABSL_CHECK((val[1] >= '0' && val[1] <= '9') ||
               (val[1] >= 'a' && val[1] <= 'f'))
        << ": Invalid hex '" << val[1] << "'";

    uint8_t converted = 0;
    if (val[0] < 'a') {
      converted |= static_cast<uint8_t>(val[0] - '0') << 4;
    } else {
      converted |= (static_cast<uint8_t>(val[0] - 'a') + 0xa) << 4;
    }
    if (val[1] < 'a') {
      converted |= static_cast<uint8_t>(val[1] - '0');
    } else {
      converted |= (static_cast<uint8_t>(val[1] - 'a') + 0xa);
    }
    *result = converted;

    val += 2;
    ++result;
    --count;
  }
}

#if defined(__APPLE__)
bool TryReadDarwinBootUUID(UUID *uuid) {
  std::array<char, UUID::kStringSize + 1> buffer{};
  size_t buffer_size = buffer.size();
  if (sysctlbyname("kern.bootsessionuuid", buffer.data(), &buffer_size, nullptr,
                   0) != 0) {
    return false;
  }
  if (buffer_size < UUID::kStringSize) {
    return false;
  }

  *uuid = UUID::FromString(std::string_view(buffer.data(), UUID::kStringSize));
  return true;
}
#endif  // defined(__APPLE__)

}  // namespace

namespace internal {
std::mt19937 FullySeededRandomGenerator() {
  // Total bits that the mt19937 has internally that we could plausibly
  // initialize with.
  // The internal state ends up being ~1200 bytes, which is significantly more
  // than the 128 bits we want for UUIDs, but since we should only need to
  // generate this randomness once, it should be fine.
  // If the performance cost ends up causing issues, then we can revisit the
  // need to *fully* seed the twister.
  constexpr size_t kInternalEntropy =
      std::mt19937::state_size * sizeof(std::mt19937::result_type);
  // Number, rounded up, of random values required.
  constexpr size_t kSeedsRequired =
      ((kInternalEntropy - 1) / sizeof(std::random_device::result_type)) + 1;

  std::array<std::random_device::result_type, kSeedsRequired> random_data;
#if defined(__linux__) || defined(__APPLE__)
  // /dev/random is *much* faster than std::random_device on modern Linux.
  //
  // My AMD Ryzen 7 PRO 7840U w/ Radeon 780M Graphics takes ~3 hours with
  // random_device to generate 1<<18 seeds, and 6 seconds with /dev/urandom.
  //
  // This is async safe.  open, read, and close are async safe.
  {
    int fp = open("/dev/urandom", O_RDONLY);
    ABSL_PCHECK(fp != -1);
    size_t to_read = sizeof(std::random_device::result_type) * kSeedsRequired;
    char *data = reinterpret_cast<char *>(&random_data[0]);
    while (to_read != 0) {
      size_t was_read = read(fp, data, to_read);
      ABSL_PCHECK(was_read > 0) << "Read " << was_read;
      to_read -= was_read;
      data += was_read;
    }
    ABSL_PCHECK(close(fp) == 0);
  }
#else
  // Portable fallback for Windows.
  {
    std::random_device random_device;
// Older LLVM libstdc++'s just return 0 for the random device entropy.
#if !defined(__clang__) || (__clang_major__ > 13)
    ABSL_CHECK_EQ(sizeof(std::random_device::result_type) * 8,
                  random_device.entropy())
        << ": Does your random_device actually support generating entropy?";
#endif
    std::generate(std::begin(random_data), std::end(random_data),
                  std::ref(random_device));
  }
#endif

  std::seed_seq seeds(std::begin(random_data), std::end(random_data));
  return std::mt19937(seeds);
}
}  // namespace internal

UUID UUID::Random() {
  // thread_local to guarantee safe use of the generator itself.
  thread_local std::mt19937 gen(internal::FullySeededRandomGenerator());

  std::uniform_int_distribution<> dis(0, 255);
  UUID result;
  for (size_t i = 0; i < kDataSize; ++i) {
    result.data_[i] = dis(gen);
  }

  // Mark the reserved bits in the data that this is a uuid4, a random UUID.
  result.data_[6] = (result.data_[6] & 0x0f) | 0x40;
  result.data_[8] = (result.data_[6] & 0x3f) | 0x80;

  return result;
}

std::string UUID::ToString() const {
  std::string out;
  out.resize(UUID::kStringSize);
  CopyTo(out.data());
  return out;
}

std::ostream &operator<<(std::ostream &os, const UUID &uuid) {
  return os << uuid.ToString();
}

flatbuffers::Offset<flatbuffers::String> UUID::PackString(
    flatbuffers::FlatBufferBuilder *fbb) const {
  std::array<char, kStringSize> data;
  CopyTo(data.data());

  return fbb->CreateString(data.data(), data.size());
}

flatbuffers::Offset<flatbuffers::Vector<uint8_t>> UUID::PackVector(
    flatbuffers::FlatBufferBuilder *fbb) const {
  return fbb->CreateVector(data_.data(), data_.size());
}

void UUID::CopyTo(char *result) const {
  ToHex(&data_[0], result, 4);
  result[8] = '-';
  ToHex(&data_[4], result + 9, 2);
  result[13] = '-';
  ToHex(&data_[6], result + 14, 2);
  result[18] = '-';
  ToHex(&data_[8], result + 19, 2);
  result[23] = '-';
  ToHex(&data_[10], result + 24, 6);
}

UUID UUID::FromString(const flatbuffers::String *str) {
  return FromString(str->string_view());
}

UUID UUID::FromVector(const flatbuffers::Vector<uint8_t> *data) {
  ABSL_CHECK(data != nullptr);
  ABSL_CHECK_EQ(data->size(), kDataSize);

  UUID result;
  std::memcpy(result.data_.data(), data->Data(), kDataSize);
  return result;
}

UUID UUID::FromSpan(absl::Span<const uint8_t> data) {
  ABSL_CHECK_EQ(data.size(), kDataSize);

  UUID result;
  std::copy(data.begin(), data.end(), result.data_.begin());
  return result;
}

UUID UUID::FromString(std::string_view str) {
  ABSL_CHECK_EQ(str.size(), kStringSize);

  UUID result;
  FromHex(str.data(), result.data_.data(), 4);
  ABSL_CHECK(str.data()[8] == '-' && str.data()[13] == '-' &&
             str.data()[18] == '-' && str.data()[23] == '-')
      << ": Invalid uuid.";
  FromHex(str.data() + 9, result.data_.data() + 4, 2);
  FromHex(str.data() + 14, result.data_.data() + 6, 2);
  FromHex(str.data() + 19, result.data_.data() + 8, 2);
  FromHex(str.data() + 24, result.data_.data() + 10, 6);
  return result;
}

UUID UUID::BootUUID() {
  auto flag = absl::GetFlag(FLAGS_boot_uuid);
  if (!flag.empty()) {
    return UUID::FromString(flag);
  }

#if defined(__linux__)
  int fd = open("/proc/sys/kernel/random/boot_id", O_RDONLY);
  ABSL_PCHECK(fd != -1);

  std::array<char, kStringSize> data;
  ABSL_CHECK_EQ(static_cast<ssize_t>(kStringSize),
                read(fd, data.begin(), kStringSize));
  close(fd);

  return UUID::FromString(std::string_view(data.data(), data.size()));
#elif defined(__APPLE__)
  UUID darwin_uuid;
  if (TryReadDarwinBootUUID(&darwin_uuid)) {
    return darwin_uuid;
  }
  ABSL_LOG(FATAL)
      << "TODO: Support macOS sandboxed boot UUID fallback implementation.";
  return UUID::Zero();
#elif defined(_WIN32)
  // On Windows there is no direct boot UUID. We synthesize one by combining
  // the machine GUID with the System process (PID 4) creation time.
  // This is NTP-invariant, always available, and doesn't require admin rights.
  // Cached in a static so the value is stable for the lifetime of the process
  // (matching Linux/macOS behavior).
  static UUID windows_boot_uuid = []() {
    // Read the MachineGuid from the registry.
    HKEY hKey;
    char machine_guid[128] = {};
    DWORD guid_size = sizeof(machine_guid);
    const LONG open_status =
        RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SOFTWARE\\Microsoft\\Cryptography",
                      0, KEY_READ | KEY_WOW64_64KEY, &hKey);
    ABSL_CHECK_EQ(open_status, ERROR_SUCCESS)
        << "Failed to open Cryptography registry key: " << open_status;
    const LONG query_status =
        RegQueryValueExA(hKey, "MachineGuid", nullptr, nullptr,
                         reinterpret_cast<LPBYTE>(machine_guid), &guid_size);
    ABSL_CHECK_EQ(query_status, ERROR_SUCCESS)
        << "Failed to query MachineGuid: " << query_status;
    RegCloseKey(hKey);

    // Get the creation time of the System process (PID 4) as our boot seed.
    uint64_t stable_boot_seed = 0;

    struct UNICODE_STRING_CUSTOM {
      USHORT Length;
      USHORT MaximumLength;
      PWSTR Buffer;
    };

    struct SYSTEM_PROCESS_INFORMATION_CUSTOM {
      ULONG NextEntryOffset;
      ULONG NumberOfThreads;
      LARGE_INTEGER WorkingSetPrivateSize;
      ULONG HardFaultCount;
      ULONG NumberOfThreadsHighWatermark;
      ULONGLONG CycleTime;
      LARGE_INTEGER CreateTime;
      LARGE_INTEGER UserTime;
      LARGE_INTEGER KernelTime;
      UNICODE_STRING_CUSTOM ImageName;
      LONG BasePriority;
      HANDLE UniqueProcessId;
    };

    using NtQuerySystemInformationFn =
        LONG(WINAPI *)(ULONG SystemInformationClass, PVOID SystemInformation,
                       ULONG SystemInformationLength, PULONG ReturnLength);

    HMODULE ntdll = GetModuleHandleA("ntdll.dll");
    if (ntdll != nullptr) {
      auto nt_query = reinterpret_cast<NtQuerySystemInformationFn>(
          GetProcAddress(ntdll, "NtQuerySystemInformation"));
      if (nt_query != nullptr) {
        // Query system process information. Since the buffer size needs can be
        // large, we allocate 1MB dynamically.
        ULONG size = 1024 * 1024;
        std::unique_ptr<uint8_t[]> buffer(new uint8_t[size]);
        ULONG return_length = 0;
        // SystemProcessInformation = 5
        if (nt_query(5, buffer.get(), size, &return_length) == 0) {
          uint8_t *current = buffer.get();
          while (true) {
            auto *proc_info =
                reinterpret_cast<SYSTEM_PROCESS_INFORMATION_CUSTOM *>(current);
            uint64_t pid =
                reinterpret_cast<uint64_t>(proc_info->UniqueProcessId);
            if (pid == 4) {
              stable_boot_seed = proc_info->CreateTime.QuadPart;
              break;
            }
            if (proc_info->NextEntryOffset == 0) {
              break;
            }
            current += proc_info->NextEntryOffset;
          }
        }
      }
    }
    ABSL_CHECK(stable_boot_seed != 0)
        << "Failed to query System process (PID 4) creation time. Set "
           "--boot_uuid manually.";

    // Seed from MachineGuid + stable_boot_seed for a per-machine, per-boot
    // UUID.
    uint8_t seed_data[128 + sizeof(stable_boot_seed)];
    ABSL_CHECK_LE(guid_size + sizeof(stable_boot_seed), sizeof(seed_data));
    std::memcpy(seed_data, machine_guid, guid_size);
    std::memcpy(seed_data + guid_size, &stable_boot_seed,
                sizeof(stable_boot_seed));
    std::seed_seq seed(seed_data,
                       seed_data + guid_size + sizeof(stable_boot_seed));
    std::mt19937 gen(seed);

    UUID result;
    std::uniform_int_distribution<> dis(0, 255);
    for (size_t i = 0; i < kDataSize; ++i) {
      result.data_[i] = static_cast<uint8_t>(dis(gen));
    }
    result.data_[6] = (result.data_[6] & 0x0f) | 0x40;
    result.data_[8] = (result.data_[8] & 0x3f) | 0x80;
    return result;
  }();
  return windows_boot_uuid;
#else
#error "Unsupported platform for BootUUID"
#endif
}

}  // namespace aos
