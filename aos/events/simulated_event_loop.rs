use std::{ffi::CString, marker::PhantomData, time::Duration};

pub use aos_configuration::{Channel, ChannelExt, Configuration, ConfigurationExt, Node};
use aos_configuration_fbs::aos::Configuration as RustConfiguration;
pub use aos_events_event_loop_runtime::{
    aos_error_t, aos_event_loop_t, aos_exit_handle_t, CppEventLoop, ExitHandle,
};
use aos_events_event_loop_runtime::{
    check_error, EventLoopHolder, EventLoopRuntime, EventLoopRuntimeHolder,
};
use aos_flatbuffers::{transmute_table_to, Flatbuffer};

pub use aos_events_event_loop_c::aos_simulated_event_loop_factory_t;
use aos_events_event_loop_c::{
    aos_event_loop_destroy, aos_simulated_event_loop_factory_create,
    aos_simulated_event_loop_factory_destroy, aos_simulated_event_loop_factory_make_event_loop,
    aos_simulated_event_loop_factory_make_exit_handle,
    aos_simulated_event_loop_factory_non_fatal_run,
    aos_simulated_event_loop_factory_non_fatal_run_for,
};

/// A Rust-owned C++ `SimulatedEventLoopFactory` object.
///
/// Owning one of these via a heap allocation makes things a lot simpler, and all the functions
/// that are called repeatedly are heavyweight enough this is not a performance concern.
///
/// We don't want to own the `SimulatedEventLoopFactory` directly because C++ maintains multiple
/// pointers to it, so we can't create Rust mutable references to it.
pub struct SimulatedEventLoopFactory<'config> {
    factory: *mut aos_simulated_event_loop_factory_t,
    _marker: PhantomData<&'config Configuration>,
}

impl Drop for SimulatedEventLoopFactory<'_> {
    fn drop(&mut self) {
        unsafe {
            aos_simulated_event_loop_factory_destroy(self.factory);
        }
    }
}

impl<'config> SimulatedEventLoopFactory<'config> {
    pub fn new(config: &'config impl Flatbuffer<RustConfiguration<'static>>) -> Self {
        unsafe {
            let factory =
                aos_simulated_event_loop_factory_create(transmute_table_to::<Configuration>(
                    &config.message()._tab,
                ));
            Self {
                factory,
                _marker: PhantomData,
            }
        }
    }

    /// Creates a Rust-owned EventLoop on the node named `node_name`, or on the only node if
    /// `node_name` is `""`.
    ///
    /// You probably don't want to call this directly if you're creating a Rust application. This
    /// is intended for creating C++ applications. Use [`Self::make_runtime`] instead when creating Rust
    /// applications.
    pub fn make_event_loop(&mut self, name: &str, node_name: &str) -> *mut aos_event_loop_t {
        let name_c = CString::new(name).unwrap();
        let node_c = CString::new(node_name).unwrap();
        unsafe {
            aos_simulated_event_loop_factory_make_event_loop(
                self.factory,
                name_c.as_ptr(),
                node_c.as_ptr(),
            )
        }
    }

    /// Creates an [`EventLoopRuntime`] wrapper which also owns its underlying [`CppEventLoop`].
    ///
    /// All setup must be performed with `fun`, which is called before this function returns. `fun`
    /// may create further objects to use in async functions via [`EventLoop.spawn`] etc, but it is
    /// the only place to set things up before the EventLoop is run.
    ///
    /// `node_name` names the node to run on, or is `""` for a single node configuration.
    ///
    /// Note that dropping the return value will drop this EventLoop.
    pub fn make_runtime<F>(
        &mut self,
        name: &str,
        node_name: &str,
        fun: F,
    ) -> EventLoopRuntimeHolder<SimulatedEventLoopHolder>
    where
        F: for<'event_loop> FnOnce(EventLoopRuntime<'event_loop>),
    {
        let event_loop = self.make_event_loop(name, node_name);
        let holder = unsafe { SimulatedEventLoopHolder::new(event_loop) };
        EventLoopRuntimeHolder::new(holder, fun)
    }

    pub fn make_exit_handle(&mut self) -> ExitHandle {
        unsafe {
            ExitHandle::new(aos_simulated_event_loop_factory_make_exit_handle(
                self.factory,
            ))
        }
    }

    /// Runs until all the event loops have exited.
    ///
    /// # Panics
    ///
    /// Panics if any of the applications returned an error.
    pub fn run(&mut self) {
        // SAFETY: `self.factory` is valid, and we own the error which comes back.
        unsafe { check_error(aos_simulated_event_loop_factory_non_fatal_run(self.factory)) };
    }

    /// Runs for `duration` of simulated time.
    ///
    /// # Panics
    ///
    /// Panics if any of the applications returned an error.
    pub fn run_for(&mut self, duration: Duration) {
        // SAFETY: `self.factory` is valid, and we own the error which comes back.
        unsafe {
            check_error(aos_simulated_event_loop_factory_non_fatal_run_for(
                self.factory,
                duration.as_nanos() as i64,
            ))
        };
    }
}

pub struct SimulatedEventLoopHolder(*mut aos_event_loop_t);

impl Drop for SimulatedEventLoopHolder {
    fn drop(&mut self) {
        unsafe {
            aos_event_loop_destroy(self.0);
        }
    }
}

impl SimulatedEventLoopHolder {
    /// SAFETY: `event_loop` must be the exclusive owner of the underlying EventLoop.
    pub unsafe fn new(event_loop: *mut aos_event_loop_t) -> Self {
        Self(event_loop)
    }
}

// SAFETY: The simulated event loop is kept alive.
unsafe impl EventLoopHolder for SimulatedEventLoopHolder {
    fn as_raw(&self) -> *const CppEventLoop {
        self.0 as *const CppEventLoop
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use aos_events_event_loop_runtime::node_has_name;

    use std::cell::RefCell;

    use futures::future::pending;
    use std::path::Path;

    use aos_configuration::read_config_from;
    use aos_test_init::test_init;
    use aos_testing_path::artifact_path;
    use ping_rust_fbs::aos::examples::PingBuilder;

    // A really basic test of the functionality here.
    #[test]
    fn smoke_test() {
        #[derive(Debug, Default)]
        struct GlobalState {
            watcher_count: u32,
            startup_count: u32,
        }

        thread_local!(static GLOBAL_STATE: RefCell<GlobalState> = Default::default());

        test_init();
        let config = read_config_from(&artifact_path(Path::new(
            "aos/testing/ping_pong/multinode_pingpong_test_combined_config.json",
        )))
        .unwrap();
        let mut event_loop_factory = SimulatedEventLoopFactory::new(&config);
        {
            let pi1 = config.message().get_node("pi1").unwrap();

            let exit_handle = event_loop_factory.make_exit_handle();
            let _runtime1 = {
                event_loop_factory.make_runtime("runtime1", "pi1", move |runtime1| {
                    assert!(node_has_name(pi1));
                    let channel = runtime1
                        .get_raw_channel("/test", "aos.examples.Ping")
                        .unwrap();
                    let mut watcher = runtime1.make_raw_watcher(channel);
                    runtime1.spawn(async move {
                        watcher.next().await;
                        GLOBAL_STATE.with(|g| {
                            let g = &mut *g.borrow_mut();
                            g.watcher_count = g.watcher_count + 1;
                        });
                        exit_handle.exit().await
                    });
                })
            };

            let _runtime2 = {
                event_loop_factory.make_runtime("runtime2", "pi1", |runtime2| {
                    assert!(node_has_name(pi1));
                    let channel = runtime2
                        .get_raw_channel("/test", "aos.examples.Ping")
                        .unwrap();
                    let mut sender = runtime2.make_raw_sender(channel);
                    runtime2.spawn(async move {
                        GLOBAL_STATE.with(|g| {
                            let g = &mut *g.borrow_mut();
                            g.startup_count = g.startup_count + 1;
                        });

                        let mut builder = sender.make_builder();
                        let ping = PingBuilder::new(builder.fbb()).finish();
                        // SAFETY: We're using the correct message type.
                        unsafe { builder.send(ping) }.expect("send should succeed");
                        pending().await
                    });
                })
            };

            GLOBAL_STATE.with(|g| {
                let g = g.borrow();
                assert_eq!(0, g.watcher_count);
            });
            event_loop_factory.run();
            GLOBAL_STATE.with(|g| {
                let g = g.borrow();
                assert_eq!(1, g.watcher_count);
                assert_eq!(1, g.startup_count);
            });
        }
    }

    // Spawning twice used to overwrite the first task, dropping that future and
    // everything it had registered without a word.  The contract is one task
    // per runtime, so the second spawn has to be loud about it.
    #[test]
    #[should_panic(expected = "Only one task may be spawned per EventLoopRuntime")]
    fn double_spawn_panics() {
        test_init();
        let config = read_config_from(&artifact_path(Path::new(
            "aos/testing/ping_pong/multinode_pingpong_test_combined_config.json",
        )))
        .unwrap();
        let mut event_loop_factory = SimulatedEventLoopFactory::new(&config);

        let _runtime = event_loop_factory.make_runtime("runtime", "pi1", |runtime| {
            runtime.spawn(async move { pending().await });
            runtime.spawn(async move { pending().await });
        });
    }
}
