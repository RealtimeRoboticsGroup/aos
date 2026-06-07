#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>

int main() {
  // Case 1: stack stringstream, runtime.
  std::stringstream stack_ss;
  stack_ss << std::setprecision(5);
  std::printf("stack precision=%d\n", (int)stack_ss.precision());

  // Case 2: heap stringstream, runtime.
  auto heap_ss = std::make_unique<std::stringstream>();
  *heap_ss << std::setprecision(7);
  std::printf("heap precision=%d\n", (int)heap_ss->precision());

  return 0;
}
