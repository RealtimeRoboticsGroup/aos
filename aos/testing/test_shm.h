#ifndef AOS_TESTING_TEST_SHM_H_
#define AOS_TESTING_TEST_SHM_H_

#include <memory>
#include <stdexcept>
#include <system_error>

namespace aos::testing {

class SharedMemoryBlock {
 public:
  explicit SharedMemoryBlock(size_t size);

  // Corrected Destructor: Now actually frees the memory
  ~SharedMemoryBlock();

  // Delete copy constructors
  SharedMemoryBlock(const SharedMemoryBlock &) = delete;
  SharedMemoryBlock &operator=(const SharedMemoryBlock &) = delete;

  // Move support
  SharedMemoryBlock(SharedMemoryBlock &&other) noexcept
      : addr_(other.addr_), size_(other.size_) {
    other.addr_ = nullptr;
    other.size_ = 0;
  }

  SharedMemoryBlock &operator=(SharedMemoryBlock &&other) noexcept;

  void *get() const { return addr_; }
  size_t size() const { return size_; }

  template <typename T, typename... Args>
  T *construct(Args &&...args) {
    if (sizeof(T) > size_) {
      throw std::runtime_error("Shared memory block too small for type T");
    }
    return new (addr_) T(std::forward<Args>(args)...);
  }

 private:
  void *addr_ = nullptr;
  size_t size_ = 0;
};

}  // namespace aos::testing

#endif  // AOS_TESTING_TEST_SHM_H_
