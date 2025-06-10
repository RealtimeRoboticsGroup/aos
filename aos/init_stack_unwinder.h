#ifndef AOS_INIT_STACK_UNWINDER_H_
#define AOS_INIT_STACK_UNWINDER_H_

namespace aos {

// Initializes the code to generate stack traces. If not called, absl will use
// its default stack unwinder which has trouble getting out of signal handlers.
void InitStackUnwinder();

}  // namespace aos

#endif  // AOS_INIT_STACK_UNWINDER_H_
