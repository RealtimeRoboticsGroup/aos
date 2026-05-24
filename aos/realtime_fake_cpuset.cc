// Shared CpuSet implementation for non-Linux platforms (Darwin, Windows)
// that use std::bitset instead of cpu_set_t.

#include "aos/realtime.h"

namespace aos {

CpuSet::CpuSet() {}

void CpuSet::Set(int cpu) {
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    set_.set(cpu);
  }
}

void CpuSet::Clear(int cpu) {
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    set_.reset(cpu);
  }
}

void CpuSet::Clear() { set_.reset(); }

bool CpuSet::IsSet(int cpu) const {
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    return set_.test(cpu);
  }
  return false;
}

bool CpuSet::Empty() const { return set_.none(); }

bool CpuSet::operator==(const CpuSet &other) const {
  return set_ == other.set_;
}

bool CpuSet::operator!=(const CpuSet &other) const {
  return set_ != other.set_;
}

}  // namespace aos
