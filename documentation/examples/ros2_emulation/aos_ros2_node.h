#ifndef AOS_EXAMPLES_ROS2_EMULATION_AOS_ROS2_NODE_H_
#define AOS_EXAMPLES_ROS2_EMULATION_AOS_ROS2_NODE_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"

#include "aos/events/event_loop.h"

namespace rclcpp {

template <typename T>
class Publisher {
 public:
  using SharedPtr = std::shared_ptr<Publisher<T>>;

  Publisher(aos::Sender<typename T::TableType> sender)
      : sender_(std::move(sender)) {}

  void publish(const T::SharedPtr &message) { publish(*message); }

  void publish(const T *message) { publish(*message); }

  void publish(const T &message) {
    auto builder = sender_.MakeBuilder();
    builder.CheckOk(
        builder.Send(T::TableType::Pack(*builder.fbb(), &message).o));
  }

  // Public to allow incremental migration to native AOS APIs.
  aos::Sender<typename T::TableType> sender_;
};

// Empty class which has no purpose with AOS.
template <typename T>
class Subscription {
 public:
  using SharedPtr = std::shared_ptr<Subscription<T>>;
};

class TimerBase {
 public:
  using SharedPtr = std::shared_ptr<TimerBase>;

  TimerBase(aos::EventLoop *event_loop, std::chrono::nanoseconds period,
            aos::TimerHandler *timer)
      : event_loop_(event_loop), period_(period), timer_(timer) {
    event_loop->OnRun([this]() { reset(); });
  }

  void cancel() { timer_->Disable(); }

  bool is_canceled() { return timer_->IsDisabled(); }

  void reset() {
    timer_->Schedule(event_loop_->monotonic_now() + period_, period_);
  }

  // Public to allow incremental migration to native AOS APIs.
  aos::EventLoop *const event_loop_;
  const std::chrono::nanoseconds period_;
  aos::TimerHandler *const timer_;
};

// A class implementing part of the rclcpp::Node API on top of AOS.
//
// See README.md in this folder for details.
class Node {
 public:
  using SharedPtr = std::shared_ptr<Node>;

  Node(aos::EventLoop *event_loop)
      : event_loop_(event_loop), name_string_(event_loop->name()) {}
  virtual ~Node() = default;

  aos::EventLoop *event_loop() { return event_loop_; }

  const char *get_name() const { return name_string_.c_str(); }

  template <typename T>
  Publisher<T>::SharedPtr create_publisher(const std::string_view topic,
                                           int /*qos*/ = 0) {
    return std::make_shared<Publisher<T>>(
        event_loop_->MakeSender<typename T::TableType>(convert_topic(topic)));
  }

  template <typename T>
  Subscription<T>::SharedPtr create_subscription(
      const std::string_view topic, int /*qos*/,
      std::function<void(std::unique_ptr<T>)> watch) {
    event_loop_->MakeWatcher(
        convert_topic(topic),
        [watch = std::move(watch)](const T::TableType &table) {
          std::unique_ptr<T> object = std::make_unique<T>();
          table.UnPackTo(object.get());
          watch(std::move(object));
        });
    return std::make_shared<Subscription<T>>();
  }

  template <typename T>
  Subscription<T>::SharedPtr create_subscription(const std::string_view topic,
                                                 int /*qos*/,
                                                 std::function<void(T)> watch) {
    event_loop_->MakeWatcher(
        convert_topic(topic),
        [watch = std::move(watch)](const T::TableType &table) {
          T object;
          table.UnPackTo(&object);
          watch(std::move(object));
        });
    return std::make_shared<Subscription<T>>();
  }

  TimerBase::SharedPtr create_wall_timer(std::chrono::nanoseconds period,
                                         std::function<void()> callback) {
    return std::make_shared<TimerBase>(
        event_loop_, period, event_loop_->AddTimer(std::move(callback)));
  }

  // Public to allow incremental migration to native AOS APIs.
  aos::EventLoop *const event_loop_;

  std::string convert_topic(std::string_view topic) {
    if (topic.starts_with('/')) {
      return std::string(topic);
    }
    return absl::StrCat("/", topic);
  }

 private:
  const std::string name_string_;
};

}  // namespace rclcpp

#endif  // AOS_EXAMPLES_ROS2_EMULATION_AOS_ROS2_NODE_H_
