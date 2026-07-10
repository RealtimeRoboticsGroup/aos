pub use aos_configuration::{Configuration, ConfigurationExt};
pub use aos_events_event_loop_runtime::{
    aos_event_loop_t, aos_exit_handle_t, CppEventLoop, EventLoopRuntime, ExecutorState, ExitHandle,
};

use aos_configuration_fbs::aos::Configuration as RustConfiguration;
use aos_flatbuffers::{transmute_table_to, Flatbuffer};
use core::marker::PhantomData;
use std::cell::{Cell, RefCell};
use std::ops::{Deref, DerefMut};
use std::rc::Rc;

#[repr(C)]
struct aos_error_t {
    _unused: [u8; 0],
}

extern "C" {
    fn aos_shm_event_loop_create(config: *const Configuration) -> *mut aos_event_loop_t;
    fn aos_shm_event_loop_run(self_: *mut aos_event_loop_t) -> *mut aos_error_t;
    fn aos_shm_event_loop_make_exit_handle(self_: *mut aos_event_loop_t) -> *mut aos_exit_handle_t;
    fn aos_event_loop_destroy(self_: *mut aos_event_loop_t);
    fn aos_error_destroy(self_: *mut aos_error_t);
}

/// A Rust-owned C++ `ShmEventLoop` object.
pub struct ShmEventLoop<'config> {
    event_loop: *mut aos_event_loop_t,
    state: Option<Rc<ExecutorState>>,
    _config: PhantomData<&'config Configuration>,
}

impl Drop for ShmEventLoop<'_> {
    fn drop(&mut self) {
        self.state.take();
        unsafe {
            aos_event_loop_destroy(self.event_loop);
        }
    }
}

impl<'config> ShmEventLoop<'config> {
    /// Creates a Rust-owned ShmEventLoop.
    pub fn new(config: &'config impl Flatbuffer<RustConfiguration<'static>>) -> Self {
        unsafe {
            let event_loop = aos_shm_event_loop_create(transmute_table_to::<Configuration>(
                &config.message()._tab,
            ));
            Self {
                event_loop,
                state: Some(Rc::new(ExecutorState {
                    task: RefCell::new(None),
                    is_running: Cell::new(false),
                })),
                _config: PhantomData,
            }
        }
    }

    /// Provides a runtime to construct the application and runs the event loop.
    ///
    /// The runtime is the only way to interact with the event loop. It provides the functionality
    /// to spawn a task, construct timers, watchers, fetchers, and so on.
    ///
    /// Making an [`EventLoopRuntime`] is tricky since the lifetime of the runtime is invariant
    /// w.r.t the event loop. In other words, the runtime and the event loop must have the same
    /// lifetime. By providing access to the runtime through an [`FnOnce`], we can guarantee
    /// that the runtime and the event loop both have the same lifetime.
    ///
    /// # Examples
    ///
    /// A ping application might do something like the following
    ///
    /// ```no_run
    /// # use aos_events_shm_event_loop::*;
    /// use ping_rust_fbs::aos::examples as ping;
    /// use pong_rust_fbs::aos::examples as pong;
    /// use std::borrow::Borrow;
    /// use std::cell::Cell;
    /// use std::path::Path;
    /// use aos_configuration::read_config_from;
    /// use aos_events_event_loop_runtime::{Sender, Watcher};
    ///
    /// let config = read_config_from(Path::new("path/to/aos_config.json")).unwrap();
    /// let event_loop = ShmEventLoop::new(&config);
    /// event_loop.run_with(|runtime| {
    ///   // One task will send a ping, the other will listen to pong messages.
    ///   let mut sender: Sender<ping::Ping> = runtime
    ///       .make_sender("/test")
    ///       .expect("Can't create `Ping` sender");
    ///
    ///   let on_run = runtime.on_run();
    ///   // Sends a single ping message.
    ///   let send_task = async move {
    ///     on_run.borrow().await;
    ///     let mut builder = sender.make_builder();
    ///     let mut ping = ping::PingBuilder::new(builder.fbb());
    ///     ping.add_value(10);
    ///     let ping = ping.finish();
    ///     builder.send(ping).expect("Can't send ping");
    ///   };
    ///
    ///   let mut watcher: Watcher<pong::Pong> = runtime
    ///       .make_watcher("/test")
    ///       .expect("Can't create `Ping` watcher");
    ///
    ///   // Listens to pong messages and prints them.
    ///   let receive_task = async move {
    ///     loop {
    ///       let pong = dbg!(watcher.next().await);
    ///     }
    ///   };
    ///
    ///   runtime.spawn(async move {
    ///      futures::join!(send_task, receive_task);
    ///      std::future::pending().await
    ///   });
    /// }); // Event loop starts runnning...
    /// unreachable!("This can't be reached since no ExitHandle was made");
    /// ```
    ///
    /// `run_with` can also borrow data from the outer scope that can be used in the async task.
    ///
    /// ```no_run
    /// # use aos_events_shm_event_loop::*;
    /// # use std::cell::Cell;
    /// # use std::path::Path;
    /// # use aos_configuration::read_config_from;
    /// let config = read_config_from(Path::new("path/to/aos_config.json")).unwrap();
    /// let shared_data = Cell::new(3132);
    /// let shared_data = &shared_data;
    /// let event_loop = ShmEventLoop::new(&config);
    /// event_loop.run_with(|runtime| {
    ///   // Note how `Cell` is enough since the event loop is single threaded.
    ///   let t1 = async move {
    ///     shared_data.set(shared_data.get() + 1);
    ///   };
    ///   let t2 = async move {
    ///     shared_data.set(shared_data.get() + 1);
    ///   };
    ///
    ///   runtime.spawn(async move {
    ///      futures::join!(t1, t2);
    ///      std::future::pending().await
    ///   });
    /// });
    /// unreachable!("This can't be reached since no ExitHandle was made");
    /// ```
    ///
    /// However, the spawned future must outlive `run_with`.
    ///
    /// ```compile_fail
    /// # use aos_events_shm_event_loop::*;
    /// # use std::cell::Cell;
    /// # use std::path::Path;
    /// # use aos_configuration::read_config_from;
    /// let config = read_config_from(Path::new("path/to/aos_config.json")).unwrap();
    /// let event_loop = ShmEventLoop::new(&config);
    /// event_loop.run_with(|runtime| {
    ///   // ERROR: `shared_data` doesn't live long enough.
    ///   let shared_data = Cell::new(3132);
    ///   let t1 = async {
    ///     shared_data.set(shared_data.get() + 1);
    ///   };
    ///   let t2 = async {
    ///     shared_data.set(shared_data.get() + 1);
    ///   };
    ///
    ///   runtime.spawn(async move {
    ///      futures::join!(t1, t2);
    ///      std::future::pending().await
    ///   });
    /// });
    /// ```
    pub fn run_with<'env, F>(mut self, fun: F)
    where
        F: for<'event_loop> FnOnce(Scoped<'event_loop, 'env, EventLoopRuntime<'event_loop>>),
    {
        // SAFETY: The runtime and the event loop (i.e. self) both get destroyed at the end of this
        // scope: first the runtime followed by the event loop. The runtime gets exclusive access
        // during initialization in `fun` while the event loop remains unused.
        let runtime = unsafe {
            EventLoopRuntime::new(self.event_loop, Rc::as_ptr(self.state.as_ref().unwrap()))
        };
        let runtime = Scoped::new(runtime);
        fun(runtime);
        self.run();
    }

    /// Makes an exit handle.
    pub fn make_exit_handle(&mut self) -> ExitHandle {
        unsafe { aos_shm_event_loop_make_exit_handle(self.event_loop).into() }
    }

    /// Runs the spawned task to completion.
    fn run(&mut self) {
        unsafe {
            let err = aos_shm_event_loop_run(self.event_loop);
            if !err.is_null() {
                aos_error_destroy(err);
            }
        }
    }
}

/// A wrapper over some data that lives for the duration of a scope.
#[derive(Clone, Copy)]
pub struct Scoped<'scope, 'env: 'scope, T: 'scope> {
    data: T,
    _env: PhantomData<fn(&'env ()) -> &'env ()>,
    _scope: PhantomData<fn(&'scope ()) -> &'scope ()>,
}

impl<'scope, 'env: 'scope, T: 'scope> Scoped<'scope, 'env, T> {
    /// Makes the [`Scoped`].
    pub fn new(data: T) -> Self {
        Self {
            data,
            _env: PhantomData,
            _scope: PhantomData,
        }
    }
}

impl<'scope, 'env: 'scope, T: 'scope> Deref for Scoped<'scope, 'env, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<'scope, 'env: 'scope, T: 'scope> DerefMut for Scoped<'scope, 'env, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    use aos_configuration::read_config_from;
    use aos_events_event_loop_runtime::{Sender, Watcher};
    use aos_test_init::test_init;
    use aos_testing_path::artifact_path;
    use ping_rust_fbs::aos::examples as ping;
    use std::borrow::Borrow;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Barrier;

    /// Tests basic functionality with 2 threads operating their own event loops.
    #[test]
    fn smoke_test() {
        test_init();

        let config = read_config_from(&artifact_path(Path::new(
            "aos/testing/ping_pong/pingpong_config.json",
        )))
        .unwrap();

        const VALUE: i32 = 3132;
        let barrier = Barrier::new(2);
        let count = AtomicUsize::new(0);

        std::thread::scope(|s| {
            let config = &config;
            let barrier = &barrier;
            let count = &count;
            s.spawn(move || {
                let mut event_loop = ShmEventLoop::new(config);
                let exit_handle = event_loop.make_exit_handle();
                event_loop.run_with(|runtime| {
                    runtime.spawn(async move {
                        let mut watcher: Watcher<ping::Ping> = runtime
                            .make_watcher("/test")
                            .expect("Can't create `Ping` watcher");
                        runtime.on_run().borrow().await;
                        barrier.wait();
                        let ping = watcher.next().await;
                        assert_eq!(ping.message().unwrap().value(), VALUE);
                        count.fetch_add(1, Ordering::Relaxed);
                        exit_handle.exit().await
                    });
                });
            });
            s.spawn(move || {
                let mut event_loop = ShmEventLoop::new(config);
                let exit_handle = event_loop.make_exit_handle();
                event_loop.run_with(|runtime| {
                    runtime.spawn(async move {
                        let mut sender: Sender<ping::Ping> = runtime
                            .make_sender("/test")
                            .expect("Can't create `Ping` sender");
                        runtime.on_run().borrow().await;
                        // Give the waiting thread a chance to start.
                        barrier.wait();
                        let mut sender = sender.make_builder();
                        let mut ping = ping::PingBuilder::new(sender.fbb());
                        ping.add_value(VALUE);
                        let ping = ping.finish();
                        sender.send(ping).expect("send should succeed");
                        count.fetch_add(1, Ordering::Relaxed);
                        exit_handle.exit().await
                    });
                });
            });
        });

        assert_eq!(count.into_inner(), 2, "Not all event loops ran.");
    }
}
