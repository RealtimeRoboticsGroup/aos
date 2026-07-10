#pragma once
#include <memory>

#include "aos/events/event_loop.h"

// TODO(austin): cbindgen can generate the extern "C" block below from
// event_loop_runtime_test_lib.rs, which keeps the two sides in sync
// automatically.  That's probably not worth setting up for just this test, but
// it is the right answer for exporting a real Rust API to C++.

namespace aos::events::testing {

struct TestApplication {
  void before_sending();
  void after_sending();
};

struct TypedTestApplication {
  void before_sending();
  void after_sending();
};

struct PanicApplication {};
struct PanicOnRunApplication {};

extern "C" {
TestApplication *make_test_application_raw(aos::EventLoop *event_loop);
void test_application_before_sending(TestApplication *app);
void test_application_after_sending(TestApplication *app);
void test_application_destroy(TestApplication *app);

TypedTestApplication *make_typed_test_application_raw(
    aos::EventLoop *event_loop);
void typed_test_application_before_sending(TypedTestApplication *app);
void typed_test_application_after_sending(TypedTestApplication *app);
void typed_test_application_destroy(TypedTestApplication *app);

PanicApplication *make_panic_application_raw(aos::EventLoop *event_loop);
void panic_application_destroy(PanicApplication *app);

PanicOnRunApplication *make_panic_on_run_application_raw(
    aos::EventLoop *event_loop);
void panic_on_run_application_destroy(PanicOnRunApplication *app);

uint32_t completed_test_count();
uint32_t started_test_count();
}

inline void TestApplication::before_sending() {
  test_application_before_sending(this);
}
inline void TestApplication::after_sending() {
  test_application_after_sending(this);
}

inline void TypedTestApplication::before_sending() {
  typed_test_application_before_sending(this);
}
inline void TypedTestApplication::after_sending() {
  typed_test_application_after_sending(this);
}

inline std::unique_ptr<TestApplication, void (*)(TestApplication *)>
make_test_application(aos::EventLoop *event_loop) {
  return {make_test_application_raw(event_loop), test_application_destroy};
}

inline std::unique_ptr<TypedTestApplication, void (*)(TypedTestApplication *)>
make_typed_test_application(aos::EventLoop *event_loop) {
  return {make_typed_test_application_raw(event_loop),
          typed_test_application_destroy};
}

inline std::unique_ptr<PanicApplication, void (*)(PanicApplication *)>
make_panic_application(aos::EventLoop *event_loop) {
  return {make_panic_application_raw(event_loop), panic_application_destroy};
}

inline std::unique_ptr<PanicOnRunApplication, void (*)(PanicOnRunApplication *)>
make_panic_on_run_application(aos::EventLoop *event_loop) {
  return {make_panic_on_run_application_raw(event_loop),
          panic_on_run_application_destroy};
}

}  // namespace aos::events::testing
