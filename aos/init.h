#ifndef AOS_INIT_H_
#define AOS_INIT_H_

namespace aos {

// Options for InitGoogle.
struct InitOptions {
  // Parses argc and argv with absl::ParseCommandLine.  If false, argc and argv
  // may be null, and the symbolizer then gets no argv[0].
  bool parse_command_line = true;
  // Installs abseil's failure signal handler when --backtrace is set.
  bool install_failure_signal_handler = true;
  // Calls absl::InitializeLog(), which can only happen once per process.
  bool initialize_log = true;
  // Installs the hook that catches allocation on a realtime thread (see
  // RegisterMallocHook() in realtime.h).  Off, nothing catches one, so under
  // --die_on_malloc going realtime dies; turn that flag off before
  // initializing, which is when it is read.  On Linux the hook is linked in
  // rather than installed, so a program built with it still has it, but this
  // cannot tell that from a library loaded with dlopen(), which never does.
  bool register_malloc_hook = true;
};

// Initializes AOS.  Only once per process: a second InitGoogle(), or one after
// InitEmbedded(), is a CHECK failure.
void InitGoogle(int *argc, char ***argv);
void InitGoogle(int *argc, char ***argv, const InitOptions &options);

// What InitEmbedded() does by default.  The command line and the crash signal
// handlers belong to whoever owns main().
inline constexpr InitOptions kEmbeddedInitOptions{
    .parse_command_line = false,
    .install_failure_signal_handler = false,
};

// Initializes AOS from a library in somebody else's process.  Does nothing if
// AOS is already initialized.  Thread-safe against other InitEmbedded() calls,
// and waits for an InitGoogle() or aos_init_from_rust() in progress on another
// thread.  If either starts after this, it dies, so a library thread has to
// wait until main() has started whichever one the program calls.
// options.parse_command_line must be false.
//
// It still changes the process in the ways InitGoogle() does apart from those
// two.  On Windows it replaces the CRT's invalid parameter handler and turns
// off its assertion dialogs.  It sets absl's stderr threshold to INFO.  With
// initialize_log, absl::InitializeLog() aborts the process if the host already
// called it, and a host that calls it afterwards aborts the same way: the host
// owns log initialization, before or after, or AOS does.  With
// register_malloc_hook, Windows Detours the CRT's allocator process-wide, and
// macOS installs a malloc zone and pthread_atfork handlers, aborting under
// --die_on_malloc if the host's default zone is its own.  Turn those two off
// when the host owns them.  It also installs aos_sync's pthread_atfork handler.
void InitEmbedded(const InitOptions &options = kEmbeddedInitOptions);

// Returns true if we have been initialized.  This is mostly here so
// ShmEventLoop can confirm the world was initialized before running.
bool IsInitialized();

// Marks the system as initialized. This is only meant to be used from
// init_for_rust. DO NOT call this from anywhere else, use InitGoogle or
// InitEmbedded instead.
void MarkInitialized();

}  // namespace aos

#endif  // AOS_INIT_H_
