#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// Replicate gtest's testing::Message exactly: a heap-allocated stringstream
// primed with setprecision in the constructor, with a templated operator<<.
class MessageLike {
 public:
  MessageLike() : ss_(new std::stringstream) {
    *ss_ << std::setprecision(std::numeric_limits<double>::digits10 + 2);
  }
  template <typename T>
  MessageLike &operator<<(const T &val) {
    *ss_ << val;
    return *this;
  }
  std::string GetString() const { return ss_->str(); }

 private:
  const std::unique_ptr<std::stringstream> ss_;
};

static std::string FlagToEnvVarLike(const char *flag) {
  const std::string full_flag = (MessageLike() << "gtest_" << flag).GetString();
  MessageLike env_var;
  for (size_t i = 0; i != full_flag.length(); i++) {
    env_var << static_cast<char>(toupper(full_flag.c_str()[i]));
  }
  return env_var.GetString();
}

namespace {
struct GlobalInit {
  GlobalInit() {
    value = FlagToEnvVarLike("color");
    // Mimic gtest's StringFromGTestEnv: read an env var at static-init time
    // and feed the result through an ostream. This exercises whether the
    // environment block set up by libc startup is unpoisoned by msan.
    const char *env = getenv(value.c_str());
    std::stringstream ss;
    if (env != nullptr) {
      ss << env;
    } else {
      ss << "(unset)";
    }
    resolved = ss.str();
  }
  std::string value;
  std::string resolved;
};
GlobalInit g_init;
}  // namespace

int main(int argc, char **argv) {
  std::vector<int> v;
  for (int i = 0; i < 100; ++i) {
    v.push_back(i * 2);
  }
  long sum = 0;
  for (int x : v) {
    sum += x;
  }
  if (argc > 100) {
    sum += static_cast<long>(argv[0][0]);
  }
  return (sum == 9900 && g_init.value == "GTEST_COLOR") ? 0 : 1;
}
