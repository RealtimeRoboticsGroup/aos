#ifndef AOS_STARTER_DUMMY_CGROUP_MANAGER_H_
#define AOS_STARTER_DUMMY_CGROUP_MANAGER_H_

#include "absl/log/log.h"

#include "aos/starter/starterd_lib.h"

namespace aos::starter {

class DummyMemoryCGroup : public MemoryCGroup {
 public:
  DummyMemoryCGroup(std::string_view name) : name_(name) {}
  void AddTid(pid_t pid = 0) override {
    VLOG(1) << "Adding " << pid << " to " << name_;
  }
  void SetMemoryLimit(uint64_t limit_value) override {
    VLOG(1) << "Setting memory limit of " << limit_value << " on " << name_;
  }

 private:
  std::string name_;
};

class DummyCGroupManager : public CGroupManager {
 public:
  std::unique_ptr<MemoryCGroup> MakeCGroup(
      std::string_view name, std::string_view /* user_name */) override {
    VLOG(1) << "Making CGroup " << name;
    return std::make_unique<DummyMemoryCGroup>(name);
  }
};

}  // namespace aos::starter

#endif  // AOS_STARTER_DUMMY_CGROUP_MANAGER_H_
