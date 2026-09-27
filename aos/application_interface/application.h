#ifndef AOS_APPLICATION_INTERFACE_APPLICATION_H_
#define AOS_APPLICATION_INTERFACE_APPLICATION_H_

namespace aos {

// Base interface for all AOS Applications. This is a very simple interface
// which type erases anything intended to run as an AosApplication so that a
// factory for it can be stored in the singleton ApplicationRegistry when
// registered via the Self-Registration pattern with
// AOS_REGISTER_APPLICATION(). The only requirement for that macro to
// successfully register an AosApplication is for it to be constructible from a
// raw aos::EventLoop pointer.
//
// For example:
//
////////////////////////////////////////////////////////////////////////////////
//
// // my_application.h
//
// namespace foo::bar {
//
// class MyAosRobotApplication : public aos::AosApplication {
//  public:
//   // This factory must exist for your application.
//   static aos::Result<std::unique_ptr<aos::AosApplication>> Create(
//       aos::EventLoop *event_loop);
// };
//
// }  // namespace foo::bar
//
////////////////////////////////////////////////////////////////////////////////
//
// // my_application.cc
//
// namespace foo::bar {
//
// aos::Result<std::unique_ptr<aos::AosApplication>>
// MyAosRobotApplication::MyAosRobotApplication(
//     aos::EventLoop *const event_loop) {
//     // Attach all the watchers and senders and fetchers you want.
// }
//
// }  // namespace foo::bar
//
// // At global namespace:
// AOS_REGISTER_APPLICATION(foo::bar::MyAosRobotApplication);
//
////////////////////////////////////////////////////////////////////////////////
class AosApplication {
 public:
  AosApplication() = default;

  AosApplication(const AosApplication &) = delete;
  AosApplication &operator=(const AosApplication &) = delete;

  AosApplication(AosApplication &&) = delete;
  AosApplication &operator=(AosApplication &&) = delete;

  virtual ~AosApplication() = default;
};

}  // namespace aos

#endif  // AOS_APPLICATION_INTERFACE_APPLICATION_H_
