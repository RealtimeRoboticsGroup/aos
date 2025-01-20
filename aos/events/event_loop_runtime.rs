#![warn(unsafe_op_in_unsafe_fn)]

//! This module provides a Rust async runtime on top of the C++ `aos::EventLoop` interface.
//!
//! # Rust async with `aos::EventLoop`
//!
//! The async runtimes we create are not general-purpose. They may only await the objects provided
//! by this module. Awaiting anything else will hang, until it is woken which will panic. Also,
//! doing any long-running task (besides await) will block the C++ EventLoop thread, which is
//! usually bad.
//!
//! ## Multiple tasks
//!
//! This runtime only supports a single task (aka a single [`Future`]) at a time. For many use
//! cases, this is sufficient. If you want more than that, one of these may be appropriate:
//!
//! 1. If you have a small number of tasks determined at compile time, [`futures::join`] can await
//!    them all simultaneously.
//! 2. [`futures::stream::FuturesUnordered`] can wait on a variable number of futures. It also
//!    supports adding them at runtime. Consider something like
//!    `FuturesUnordered<Pin<Box<dyn Future<Output = ()>>>` if you want a generic "container of any
//!    future".
//! 3. Multiple applications are better suited to multiple `EventLoopRuntime`s, on separate
//!    `aos::EventLoop`s. Otherwise they can't send messages to each other, among other
//!    restrictions. <https://github.com/frc971/971-Robot-Code/issues/12> covers creating an adapter
//!    that provides multiple `EventLoop`s on top of a single underlying implementation.
//!
//! ## Design
//!
//! The design of this is tricky. This is a complicated API interface between C++ and Rust. The big
//! considerations in arriving at this design include:
//!   * `EventLoop` implementations alias the objects they're returning from C++, which means
//!     creating Rust unique references to them is unsound. See
//!     <https://github.com/google/autocxx/issues/1146> for details.
//!   * For various reasons autocxx can't directly wrap APIs using types ergonomic for C++. This and
//!     the previous point mean we wrap all of the C++ objects specifically for this class.
//!   * Rust's lifetimes are only flexible enough to track everything with a single big lifetime.
//!     All the callbacks can store references to things tied to the event loop's lifetime, but no
//!     other lifetimes.
//!   * We can't use [`futures::stream::Stream`] and all of its nice [`futures::stream::StreamExt`]
//!     helpers for watchers because we need lifetime-generic `Item` types. Effectively we're making
//!     a lending stream. This is very close to lending iterators, which is one of the motivating
//!     examples for generic associated types (<https://github.com/rust-lang/rust/issues/44265>).

use std::{
    fmt,
    future::Future,
    marker::PhantomData,
    ops::{Add, Deref, DerefMut},
    panic::{catch_unwind, AssertUnwindSafe},
    pin::Pin,
    slice,
    task::Poll,
    time::Duration,
};

use autocxx::{
    subclass::{subclass, CppSubclass},
    WithinBox,
};
use cxx::UniquePtr;
use flatbuffers::{
    root_unchecked, Allocator, FlatBufferBuilder, Follow, FollowWith, FullyQualifiedName,
};
use futures::{future::pending, future::FusedFuture, never::Never};
use thiserror::Error;
use uuid::Uuid;

pub use aos_configuration::{Channel, Configuration, Node};
use aos_configuration::{ChannelLookupError, ConfigurationExt};

pub use aos_uuid::UUID;
pub use ffi::aos::EventLoop as CppEventLoop;
pub use ffi::aos::EventLoopRuntime as CppEventLoopRuntime;
pub use ffi::aos::ExitHandle as CppExitHandle;

autocxx::include_cpp! (
#include "aos/events/event_loop_runtime.h"

safety!(unsafe)

generate_pod!("aos::Context")
generate!("aos::WatcherForRust")
generate!("aos::RawSender_Error")
generate!("aos::SenderForRust")
generate!("aos::FetcherForRust")
generate!("aos::OnRunForRust")
generate!("aos::EventLoopRuntime")
generate!("aos::ExitHandle")
generate!("aos::TimerForRust")

subclass!("aos::ApplicationFuture", RustApplicationFuture)

extern_cpp_type!("aos::Configuration", crate::Configuration)
extern_cpp_type!("aos::Channel", crate::Channel)
extern_cpp_type!("aos::Node", crate::Node)
extern_cpp_type!("aos::UUID", crate::UUID)
);

/// A marker type which is invariant with respect to the given lifetime.
///
/// When interacting with functions that take and return things with a given lifetime, the lifetime
/// becomes invariant. Because we don't store these functions as Rust types, we need a type like
/// this to tell the Rust compiler that it can't substitute a shorter _or_ longer lifetime.
pub type InvariantLifetime<'a> = PhantomData<fn(&'a ()) -> &'a ()>;

/// # Safety
///
/// This should have a `'event_loop` lifetime and `future` should include that in its type, but
/// autocxx's subclass doesn't support that. Even if it did, it wouldn't be enforced. C++ is
/// enforcing the lifetime: it destroys this object along with the C++ `EventLoopRuntime`, which
/// must be outlived by the EventLoop.
#[doc(hidden)]
#[subclass]
pub struct RustApplicationFuture {
    /// This logically has a `'event_loop` bound, see the class comment for details.
    future: Pin<Box<dyn Future<Output = Never>>>,
}

impl ffi::aos::ApplicationFuture_methods for RustApplicationFuture {
    fn Poll(&mut self) -> bool {
        catch_unwind(AssertUnwindSafe(|| {
            // This is always allowed because it can never create a value of type `Ready<Never>` to
            // return, so it must always return `Pending`. That also means the value it returns doesn't
            // mean anything, so we ignore it.
            let _ = Pin::new(&mut self.future)
                .poll(&mut std::task::Context::from_waker(&panic_waker()));
        }))
        .is_ok()
    }
}

impl RustApplicationFuture {
    pub fn new<'event_loop>(
        future: impl Future<Output = Never> + 'event_loop,
    ) -> UniquePtr<ffi::aos::ApplicationFuture> {
        /// # Safety
        ///
        /// This completely removes the `'event_loop` lifetime, the caller must ensure that is
        /// sound.
        unsafe fn remove_lifetime<'event_loop>(
            future: Pin<Box<dyn Future<Output = Never> + 'event_loop>>,
        ) -> Pin<Box<dyn Future<Output = Never>>> {
            // SAFETY: Caller is responsible.
            unsafe { std::mem::transmute(future) }
        }

        Self::as_ApplicationFuture_unique_ptr(Self::new_cpp_owned(Self {
            // SAFETY: C++ manages observing the lifetime, see [`RustApplicationFuture`] for
            // details.
            future: unsafe { remove_lifetime(Box::pin(future)) },
            cpp_peer: Default::default(),
        }))
    }
}

/// An abstraction for objects which hold an `aos::EventLoop` from Rust code.
///
/// If you have an `aos::EventLoop` provided from C++ code, don't use this, just call
/// [`EventLoopRuntime.new`] directly.
///
/// # Safety
///
/// Objects implementing this trait must guarantee that the underlying event loop (as returned
/// from [`EventLoopHolder::as_raw`]), must be valid for as long as this object is. One way to do
/// this may be by managing ownership of the event loop with Rust's ownership semantics. However,
/// this is not strictly necessary.
///
/// This also implies semantics similar to `Pin<&mut CppEventLoop>` for the underlying object.
/// Implementations of this trait must guarantee that the underlying object must not be moved while
/// this object exists.
pub unsafe trait EventLoopHolder {
    /// Returns the raw C++ pointer of the underlying event loop.
    ///
    /// Caller can only assume this pointer is valid while `self` is still alive.
    fn as_raw(&self) -> *const CppEventLoop;
}

/// Owns an [`EventLoopRuntime`] and its underlying `aos::EventLoop`, with safe management of the
/// associated Rust lifetimes.
pub struct EventLoopRuntimeHolder<T: EventLoopHolder> {
    // NOTE: `runtime` must get dropped first, so we declare it before the event_loop:
    // https://doc.rust-lang.org/reference/destructors.html
    _runtime: Pin<Box<CppEventLoopRuntime>>,
    _event_loop: T,
}

impl<T: EventLoopHolder> EventLoopRuntimeHolder<T> {
    /// Creates a new [`EventLoopRuntime`] and runs an initialization function on it. This is a
    /// safe wrapper around [`EventLoopRuntime.new`] (although see [`EventLoopHolder`]'s safety
    /// requirements, part of them are just delegated there).
    ///
    /// If you have an `aos::EventLoop` provided from C++ code, don't use this, just call
    /// [`EventLoopRuntime.new`] directly.
    ///
    /// All setup of the runtime must be performed with `fun`, which is called before this function
    /// returns. `fun` may create further objects to use in async functions via [`EventLoop.spawn`]
    /// etc, but it is the only place to set things up before the EventLoop is run.
    ///
    /// `fun` cannot capture things outside of the event loop, because the event loop might outlive
    /// them:
    /// ```compile_fail
    /// # use aos_events_event_loop_runtime::*;
    /// # fn bad(event_loop: impl EventLoopHolder) {
    /// let mut x = 0;
    /// EventLoopRuntimeHolder::new(event_loop, |runtime| {
    ///     runtime.spawn(async {
    ///         x = 1;
    ///         loop {}
    ///     });
    /// });
    /// # }
    /// ```
    ///
    /// But it can capture `'event_loop` references:
    /// ```
    /// # use aos_events_event_loop_runtime::*;
    /// # use aos_configuration::ChannelExt;
    /// # fn good(event_loop: impl EventLoopHolder) {
    /// EventLoopRuntimeHolder::new(event_loop, |runtime| {
    ///     let channel = runtime.get_raw_channel("/test", "aos.examples.Ping").unwrap();
    ///     runtime.spawn(async {
    ///         loop {
    ///             eprintln!("{:?}", channel.type_());
    ///         }
    ///     });
    /// });
    /// # }
    /// ```
    pub fn new<F>(event_loop: T, fun: F) -> Self
    where
        F: for<'event_loop> FnOnce(EventLoopRuntime<'event_loop>),
    {
        // SAFETY: The event loop pointer produced by as_raw must be valid and it will get dropped
        // first (see https://doc.rust-lang.org/reference/destructors.html)
        let runtime = unsafe { CppEventLoopRuntime::new(event_loop.as_raw()).within_box() };
        EventLoopRuntime::with(&runtime, fun);
        Self {
            _runtime: runtime,
            _event_loop: event_loop,
        }
    }
}

/// Manages the Rust interface to a *single* `aos::EventLoop`.
///
/// This is intended to be used by a single application.
#[derive(Copy, Clone)]
pub struct EventLoopRuntime<'event_loop>(
    &'event_loop CppEventLoopRuntime,
    // See documentation of [`new`] for details.
    InvariantLifetime<'event_loop>,
);

impl<'event_loop> EventLoopRuntime<'event_loop> {
    /// Creates a new runtime for the underlying event loop.
    ///
    /// Consider using [`EventLoopRuntimeHolder.new`] instead, if you're working with an
    /// `aos::EventLoop` owned (indirectly) by Rust code or using [`EventLoopRuntime::with`] as a safe
    /// alternative.
    ///
    /// One common pattern is wrapping the lifetime behind a higher-rank trait bound (such as
    /// [`FnOnce`]). This would constraint the lifetime to `'static` and objects with `'event_loop`
    /// returned by this runtime.
    ///
    /// Call [`EventLoopRuntime::spawn`] to respond to events. The non-event-driven APIs may be used without calling
    /// this.
    ///
    /// This is an async runtime, but it's a somewhat unusual one. See the module-level
    /// documentation for details.
    ///
    /// # Safety
    ///
    /// This function is where all the tricky lifetime guarantees to ensure soundness come
    /// together. It all boils down to choosing `'event_loop` correctly, which is very complicated.
    /// Here are the rules:
    ///
    /// 1. `'event_loop` extends until after the last time the underlying `aos::EventLoop` is run.
    ///    **This is often beyond the lifetime of this Rust `EventLoopRuntime` object**.
    /// 2. `'event_loop` must outlive this object, because this object stores references to the
    ///    underlying `aos::EventLoop`.
    /// 3. Any other references stored in the underlying `aos::EventLoop` must be valid for
    ///    `'event_loop`. The easiest way to ensure this is by not using the `aos::EventLoop` before
    ///    passing it to this object.
    ///
    /// Here are some corollaries:
    ///
    /// 1. The underlying `aos::EventLoop` must be dropped after this object.
    /// 2. This object will store various references valid for `'event_loop` with a duration of
    ///   `'event_loop`, which is safe as long as
    ///
    /// * `'event_loop` outlives the underlying event loop, and
    /// * `'event_loop` references are not used once the event loop is destroyed
    ///
    /// Note that this requires this type to be invariant with respect to `'event_loop`. This can
    /// be achieved by using [`EventLoopRuntime::with`] since `'event_loop` referenes can't leave
    /// `fun` and the runtime holding `'event_loop` references will be destroyed before the event
    /// loop.
    ///
    /// `aos::EventLoop`'s public API is exclusively for consumers of the event loop. Some
    /// subclasses extend this API. Additionally, all useful implementations of `aos::EventLoop`
    /// must have some way to process events. Sometimes this is additional API surface (such as
    /// `aos::ShmEventLoop`), in other cases comes via other objects holding references to the
    /// `aos::EventLoop` (such as `aos::SimulatedEventLoopFactory`). This access to run the event
    /// loop functions independently of the consuming functions in every way except lifetime of the
    /// `aos::EventLoop`, and may be used independently of `'event_loop`.
    ///
    /// ## Alternatives and why they don't work
    ///
    /// Making the argument `Pin<&'event_loop mut EventLoop>` would express some (but not all) of
    /// these restrictions within the Rust type system. However, having an actual Rust mutable
    /// reference like that prevents anything else from creating one via other pointers to the
    /// same object from C++, which is a common operation. See the module-level documentation for
    /// details.
    ///
    /// spawned tasks need to hold `&'event_loop` references to things like channels. Using a
    /// separate `'config` lifetime wouldn't change much; the tasks still need to do things which
    /// require them to not outlive something they don't control. This is fundamental to
    /// self-referential objects, which `aos::EventLoop` is based around, but Rust requires unsafe
    /// code to manage manually.
    ///
    /// ## Final cautions
    ///
    /// Following these rules is very tricky. Be very cautious calling this function. The
    /// exposed lifetime doesn't actually convey all the rules to the compiler. To the compiler,
    /// `'event_loop` ends when this object is dropped which is not the case!
    pub unsafe fn new(event_loop: &'event_loop CppEventLoopRuntime) -> Self {
        Self(event_loop, InvariantLifetime::default())
    }

    /// Safely builds a "constrained" EventLoopRuntime with `fun`.
    ///
    /// We constrain the scope of the `[EventLoopRuntime]` by tying it to **any** `'a` lifetime. The
    /// idea is that the only things that satisfy this lifetime are either ``static` or produced by
    /// the event loop itself with a '`event_loop` runtime.
    pub fn with<F>(event_loop: &'event_loop CppEventLoopRuntime, fun: F)
    where
        F: for<'a> FnOnce(EventLoopRuntime<'a>),
    {
        // SAFETY: We satisfy the event loop lifetime constraint by scoping it inside of a higher-
        // rank lifetime in FnOnce. This is similar to what is done in std::thread::scope, and the
        // point is that `fun` can only assume that `'static` and types produced by this type with a
        // 'event_loop lifetime are the only lifetimes that will satisfy `'a`. This is possible due
        // to this type's invariance over its lifetime, otherwise, one could easily make a Subtype
        // that, due to its shorter lifetime, would include things from its outer scope.
        unsafe {
            fun(Self::new(event_loop));
        }
    }

    /// Returns the pointer passed into the constructor.
    ///
    /// The returned value should only be used for destroying it (_after_ `self` is dropped) or
    /// calling other C++ APIs.
    pub fn raw_event_loop(&self) -> *mut CppEventLoop {
        self.0.event_loop()
    }

    /// Returns a reference to the name of this EventLoop.
    ///
    /// TODO(Brian): Come up with a nice way to expose this safely, without memory allocations, for
    /// logging etc.
    ///
    /// # Safety
    ///
    /// The result must not be used after C++ could change it. Unfortunately C++ can change this
    /// name from most places, so you should be really careful what you do with the result.
    pub unsafe fn raw_name(&self) -> &str {
        self.0.name()
    }

    pub fn get_raw_channel(
        &self,
        name: &str,
        typename: &str,
    ) -> Result<&'event_loop Channel, ChannelLookupError> {
        self.configuration().get_channel(
            name,
            typename,
            // SAFETY: We're not calling any EventLoop methods while C++ is using this for the
            // channel lookup.
            unsafe { self.raw_name() },
            self.node(),
        )
    }

    pub fn get_channel<T: FullyQualifiedName>(
        &self,
        name: &str,
    ) -> Result<&'event_loop Channel, ChannelLookupError> {
        self.get_raw_channel(name, T::get_fully_qualified_name())
    }

    /// Starts running the given `task`, which may not return (as specified by its type). If you
    /// want your task to stop, return the result of awaiting [`futures::future::pending`], which
    /// will never complete. `task` will not be polled after the underlying `aos::EventLoop` exits.
    ///
    /// Note that task will be polled immediately, to give it a chance to initialize. If you want to
    /// defer work until the event loop starts running, await [`EventLoopRuntime::on_run`] in the task.
    ///
    /// # Panics
    ///
    /// Panics if called more than once. See the module-level documentation for alternatives if you
    /// want to do this.
    ///
    /// # Examples with interesting return types
    ///
    /// These are all valid futures which never return:
    /// ```
    /// # fn compile_check(mut runtime: aos_events_event_loop_runtime::EventLoopRuntime) {
    /// # use futures::{never::Never, future::pending};
    /// async fn pending_wrapper() -> Never {
    ///     pending().await
    /// }
    /// async fn loop_forever() -> Never {
    ///     loop {}
    /// }
    ///
    /// runtime.spawn(pending());
    /// runtime.spawn(async { pending().await });
    /// runtime.spawn(pending_wrapper());
    /// runtime.spawn(async { loop {} });
    /// runtime.spawn(loop_forever());
    /// runtime.spawn(async { println!("all done"); pending().await });
    /// # }
    /// ```
    /// but this is not:
    /// ```compile_fail
    /// # fn compile_check(mut runtime: aos_events_event_loop_runtime::EventLoopRuntime) {
    /// # use futures::ready;
    /// runtime.spawn(ready());
    /// # }
    /// ```
    /// and neither is this:
    /// ```compile_fail
    /// # fn compile_check(mut runtime: aos_events_event_loop_runtime::EventLoopRuntime) {
    /// # use futures::ready;
    /// runtime.spawn(async { println!("all done") });
    /// # }
    /// ```
    ///
    /// # Examples with capturing
    ///
    /// The future can capture things. This is important to access other objects created from the
    /// runtime, either before calling this function:
    /// ```
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// #     channel1: &'event_loop aos_events_event_loop_runtime::Channel,
    /// #     channel2: &'event_loop aos_events_event_loop_runtime::Channel,
    /// # ) {
    /// let mut watcher1 = runtime.make_raw_watcher(channel1);
    /// let mut watcher2 = runtime.make_raw_watcher(channel2);
    /// runtime.spawn(async move { loop {
    ///     watcher1.next().await;
    ///     watcher2.next().await;
    /// }});
    /// # }
    /// ```
    /// or after:
    /// ```
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// #     channel1: &'event_loop aos_events_event_loop_runtime::Channel,
    /// #     channel2: &'event_loop aos_events_event_loop_runtime::Channel,
    /// # ) {
    /// # use std::{cell::RefCell, rc::Rc};
    /// let runtime = Rc::new(RefCell::new(runtime));
    /// runtime.borrow_mut().spawn({
    ///     let mut runtime = runtime.clone();
    ///     async move {
    ///         let mut runtime = runtime.borrow_mut();
    ///         let mut watcher1 = runtime.make_raw_watcher(channel1);
    ///         let mut watcher2 = runtime.make_raw_watcher(channel2);
    ///         loop {
    ///             watcher1.next().await;
    ///             watcher2.next().await;
    ///         }
    ///     }
    /// });
    /// # }
    /// ```
    /// or both:
    /// ```
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// #     channel1: &'event_loop aos_events_event_loop_runtime::Channel,
    /// #     channel2: &'event_loop aos_events_event_loop_runtime::Channel,
    /// # ) {
    /// # use std::{cell::RefCell, rc::Rc};
    /// let mut watcher1 = runtime.make_raw_watcher(channel1);
    /// let runtime = Rc::new(RefCell::new(runtime));
    /// runtime.borrow_mut().spawn({
    ///     let mut runtime = runtime.clone();
    ///     async move {
    ///         let mut runtime = runtime.borrow_mut();
    ///         let mut watcher2 = runtime.make_raw_watcher(channel2);
    ///         loop {
    ///             watcher1.next().await;
    ///             watcher2.next().await;
    ///         }
    ///     }
    /// });
    /// # }
    /// ```
    ///
    /// But you cannot capture local variables:
    /// ```compile_fail
    /// # fn compile_check<'event_loop>(
    /// #     mut runtime: aos_events_event_loop_runtime::EventLoopRuntime<'event_loop>,
    /// # ) {
    /// let mut local: i32 = 1323;
    /// let local = &mut local;
    /// runtime.spawn(async move { loop {
    ///     println!("have: {}", local);
    /// }});
    /// # }
    /// ```
    pub fn spawn(&self, task: impl Future<Output = Never> + 'event_loop) {
        self.0.Spawn(RustApplicationFuture::new(task));
    }

    pub fn configuration(&self) -> &'event_loop Configuration {
        // SAFETY: It's always a pointer valid for longer than the underlying EventLoop.
        unsafe { &*self.0.configuration() }
    }

    pub fn node(&self) -> Option<&'event_loop Node> {
        // SAFETY: It's always a pointer valid for longer than the underlying EventLoop, or null.
        unsafe { self.0.node().as_ref() }
    }

    pub fn monotonic_now(&self) -> MonotonicInstant {
        MonotonicInstant(self.0.monotonic_now())
    }

    pub fn realtime_now(&self) -> RealtimeInstant {
        RealtimeInstant(self.0.realtime_now())
    }
    /// Note that the `'event_loop` input lifetime is intentional. The C++ API requires that it is
    /// part of `self.configuration()`, which will always have this lifetime.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_raw_watcher(&self, channel: &'event_loop Channel) -> RawWatcher {
        // SAFETY: `channel` is valid for the necessary lifetime, all other requirements fall under
        // the usual autocxx heuristics.
        RawWatcher(unsafe { self.0.MakeWatcher(channel) }.within_box())
    }

    /// Provides type-safe async blocking access to messages on a channel. `T` should be a
    /// generated flatbuffers table type, the lifetime parameter does not matter, using `'static`
    /// is easiest.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_watcher<T>(&self, channel_name: &str) -> Result<Watcher<T>, ChannelLookupError>
    where
        for<'a> T: FollowWith<'a>,
        for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
        T: FullyQualifiedName,
    {
        let channel = self.get_channel::<T>(channel_name)?;
        Ok(Watcher(self.make_raw_watcher(channel), PhantomData))
    }

    /// Note that the `'event_loop` input lifetime is intentional. The C++ API requires that it is
    /// part of `self.configuration()`, which will always have this lifetime.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_raw_sender(&self, channel: &'event_loop Channel) -> RawSender {
        // SAFETY: `channel` is valid for the necessary lifetime, all other requirements fall under
        // the usual autocxx heuristics.
        RawSender(unsafe { self.0.MakeSender(channel) }.within_box())
    }

    /// Allows sending messages on a channel with a type-safe API.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_sender<T>(&self, channel_name: &str) -> Result<Sender<T>, ChannelLookupError>
    where
        for<'a> T: FollowWith<'a>,
        for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
        T: FullyQualifiedName,
    {
        let channel = self.get_channel::<T>(channel_name)?;
        Ok(Sender(self.make_raw_sender(channel), PhantomData))
    }

    /// Note that the `'event_loop` input lifetime is intentional. The C++ API requires that it is
    /// part of `self.configuration()`, which will always have this lifetime.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_raw_fetcher(&self, channel: &'event_loop Channel) -> RawFetcher {
        // SAFETY: `channel` is valid for the necessary lifetime, all other requirements fall under
        // the usual autocxx heuristics.
        RawFetcher(unsafe { self.0.MakeFetcher(channel) }.within_box())
    }

    /// Provides type-safe access to messages on a channel, without the ability to wait for a new
    /// one. This provides APIs to get the latest message, and to follow along and retrieve each
    /// message in order.
    ///
    /// # Panics
    ///
    /// Dropping `self` before the returned object is dropped will panic.
    pub fn make_fetcher<T>(&self, channel_name: &str) -> Result<Fetcher<T>, ChannelLookupError>
    where
        for<'a> T: FollowWith<'a>,
        for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
        T: FullyQualifiedName,
    {
        let channel = self.get_channel::<T>(channel_name)?;
        Ok(Fetcher(self.make_raw_fetcher(channel), PhantomData))
    }

    // TODO(Brian): Expose timers and phased loops. Should we have `sleep`-style methods for those,
    // instead of / in addition to mirroring C++ with separate setup and wait?

    /// Returns a Future to wait until the underlying EventLoop is running. Once this resolves, all
    /// subsequent code will have any realtime scheduling applied. This means it can rely on
    /// consistent timing, but it can no longer create any EventLoop child objects or do anything
    /// else non-realtime.
    pub fn on_run(&self) -> OnRun {
        OnRun(self.0.MakeOnRun().within_box())
    }

    pub fn is_running(&self) -> bool {
        self.0.is_running()
    }

    /// Returns an unarmed timer.
    pub fn add_timer(&self) -> Timer {
        Timer(self.0.AddTimer())
    }

    /// Returns a timer that goes off every `duration`-long ticks.
    pub fn add_interval(&self, duration: Duration) -> Timer {
        let mut timer = self.add_timer();
        timer.setup(self.monotonic_now(), Some(duration));
        timer
    }

    /// Sets the scheduler priority to run the event loop at.
    pub fn set_realtime_priority(&self, priority: i32) {
        self.0.SetRuntimeRealtimePriority(priority.into());
    }
}

/// An event loop primitive that allows sleeping asynchronously.
///
/// # Examples
///
/// ```no_run
/// # use aos_events_event_loop_runtime::EventLoopRuntime;
/// # use std::time::Duration;
/// # fn compile_check(runtime: &mut EventLoopRuntime<'_>) {
/// # let mut timer = runtime.add_timer();
/// // Goes as soon as awaited.
/// timer.setup(runtime.monotonic_now(), None);
/// // Goes off once in 2 seconds.
/// timer.setup(runtime.monotonic_now() + Duration::from_secs(2), None);
/// // Goes off as soon as awaited and every 2 seconds afterwards.
/// timer.setup(runtime.monotonic_now(), Some(Duration::from_secs(1)));
/// async {
///   for i in 0..10 {
///     timer.tick().await;
///   }
///   // Timer won't off anymore. Next `tick` will never return.
///   timer.disable();
///   timer.tick().await;
/// };
/// # }
/// ```
pub struct Timer(UniquePtr<ffi::aos::TimerForRust>);

/// A "tick" for a [`Timer`].
///
/// This is the raw future generated by the [`Timer::tick`] function.
pub struct TimerTick<'a>(&'a mut Timer);

impl Timer {
    /// Arms the timer.
    ///
    /// The timer should sleep until `base`, `base + repeat`, `base + repeat * 2`, ...
    /// If `repeat` is `None`, then the timer only expires once at `base`.
    pub fn setup(&mut self, base: MonotonicInstant, repeat: Option<Duration>) {
        self.0.pin_mut().Schedule(
            base.0,
            repeat
                .unwrap_or(Duration::from_nanos(0))
                .as_nanos()
                .try_into()
                .expect("Out of range: Internal clock uses 64 bits"),
        );
    }

    /// Disarms the timer.
    ///
    /// Can be re-enabled by calling `setup` again.
    pub fn disable(&mut self) {
        self.0.pin_mut().Disable();
    }

    /// Returns `true` if the timer is enabled.
    pub fn is_enabled(&self) -> bool {
        !self.0.IsDisabled()
    }

    /// Sets the name of the timer.
    ///
    /// This can be useful to get a descriptive name in the timing reports.
    pub fn set_name(&mut self, name: &str) {
        self.0.pin_mut().set_name(name);
    }

    /// Gets the name of the timer.
    pub fn name(&self) -> &str {
        self.0.name()
    }

    /// Returns a tick which can be `.await`ed.
    ///
    /// This tick will resolve on the next timer expired.
    pub fn tick(&mut self) -> TimerTick {
        TimerTick(self)
    }

    /// Polls the timer, returning `[Poll::Ready]` only once the timer expired.
    fn poll(&mut self) -> Poll<()> {
        if self.0.pin_mut().Poll() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Future for TimerTick<'_> {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _: &mut std::task::Context) -> Poll<()> {
        self.0.poll()
    }
}

/// Provides async blocking access to messages on a channel. This will return every message on the
/// channel, in order.
///
/// Use [`EventLoopRuntime::make_raw_watcher`] to create one of these.
///
/// This is the non-typed API, which is mainly useful for reflection and does not provide safe APIs
/// for actually interpreting messages. You probably want a [`Watcher`] instead.
///
/// This is the same concept as [`futures::stream::Stream`], but can't follow that API for technical
/// reasons.
///
/// # Design
///
/// We can't use [`futures::stream::Stream`] because our `Item` type is `Context<'_>`, which means
/// it's different for each `self` lifetime so we can't write a single type alias for it. We could
/// write an intermediate type with a generic lifetime that implements `Stream` and is returned
/// from a `make_stream` method, but that's what `Stream` is doing in the first place so adding
/// another level doesn't help anything.
///
/// We also drop the extraneous `cx` argument that isn't used by this implementation anyways.
///
/// We also run into some limitations in the borrow checker trying to implement `poll`, I think it's
/// the same one mentioned here:
/// <https://blog.rust-lang.org/2022/08/05/nll-by-default.html#looking-forward-what-can-we-expect-for-the-borrow-checker-of-the-future>
/// We get around that one by moving the unbounded lifetime from the pointer dereference into the
/// function with the if statement.
// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct RawWatcher(Pin<Box<ffi::aos::WatcherForRust>>);

impl RawWatcher {
    /// Returns a Future to await the next value. This can be canceled (ie dropped) at will,
    /// without skipping any messages.
    ///
    /// Remember not to call `poll` after it returns `Poll::Ready`, just like any other future. You
    /// will need to call this function again to get the succeeding message.
    ///
    /// # Examples
    ///
    /// The common use case is immediately awaiting the next message:
    /// ```
    /// # async fn await_message(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// println!("received: {:?}", watcher.next().await);
    /// # }
    /// ```
    ///
    /// You can also await the first message from any of a set of channels:
    /// ```
    /// # async fn select(
    /// #     mut watcher1: aos_events_event_loop_runtime::RawWatcher,
    /// #     mut watcher2: aos_events_event_loop_runtime::RawWatcher,
    /// # ) {
    /// futures::select! {
    ///     message1 = watcher1.next() => println!("channel 1: {:?}", message1),
    ///     message2 = watcher2.next() => println!("channel 2: {:?}", message2),
    /// }
    /// # }
    /// ```
    ///
    /// Note that due to the returned object borrowing the `self` reference, the borrow checker will
    /// enforce only having a single of these returned objects at a time. Drop the previous message
    /// before asking for the next one. That means this will not compile:
    /// ```compile_fail
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// let first = watcher.next();
    /// let second = watcher.next();
    /// first.await;
    /// # }
    /// ```
    /// and nor will this:
    /// ```compile_fail
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// let first = watcher.next().await;
    /// watcher.next();
    /// println!("still have: {:?}", first);
    /// # }
    /// ```
    /// but this is fine:
    /// ```
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::RawWatcher) {
    /// let first = watcher.next().await;
    /// println!("have: {:?}", first);
    /// watcher.next();
    /// # }
    /// ```
    pub fn next(&mut self) -> RawWatcherNext {
        RawWatcherNext(Some(self))
    }
}

/// The type returned from [`RawWatcher::next`], see there for details.
pub struct RawWatcherNext<'a>(Option<&'a mut RawWatcher>);

impl<'a> Future for RawWatcherNext<'a> {
    type Output = Context<'a>;
    fn poll(mut self: Pin<&mut Self>, _: &mut std::task::Context) -> Poll<Context<'a>> {
        let inner = self
            .0
            .take()
            .expect("May not call poll after it returns Ready");
        let maybe_context = inner.0.as_mut().PollNext();
        if maybe_context.is_null() {
            // We're not returning a reference into it, so we can safely replace the reference to
            // use again in the future.
            self.0.replace(inner);
            Poll::Pending
        } else {
            // SAFETY: We just checked if it's null. If not, it will be a valid pointer. It will
            // remain a valid pointer for the borrow of the underlying `RawWatcher` (ie `'a`)
            // because we're dropping `inner` (which is that reference), so it will need to be
            // borrowed again which cannot happen before the end of `'a`.
            Poll::Ready(Context(unsafe { &*maybe_context }))
        }
    }
}

impl FusedFuture for RawWatcherNext<'_> {
    fn is_terminated(&self) -> bool {
        self.0.is_none()
    }
}

/// Provides async blocking access to messages on a channel. This will return every message on the
/// channel, in order.
///
/// Use [`EventLoopRuntime::make_watcher`] to create one of these.
///
/// This is the same concept as [`futures::stream::Stream`], but can't follow that API for technical
/// reasons. See [`RawWatcher`]'s documentation for details.
pub struct Watcher<T>(RawWatcher, PhantomData<*mut T>)
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>;

impl<T> Watcher<T>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    /// Returns a Future to await the next value. This can be canceled (ie dropped) at will,
    /// without skipping any messages.
    ///
    /// Remember not to call `poll` after it returns `Poll::Ready`, just like any other future. You
    /// will need to call this function again to get the succeeding message.
    ///
    /// # Examples
    ///
    /// The common use case is immediately awaiting the next message:
    /// ```
    /// # use pong_rust_fbs::aos::examples::Pong;
    /// # async fn await_message(mut watcher: aos_events_event_loop_runtime::Watcher<Pong<'static>>) {
    /// println!("received: {:?}", watcher.next().await);
    /// # }
    /// ```
    ///
    /// You can also await the first message from any of a set of channels:
    /// ```
    /// # use pong_rust_fbs::aos::examples::Pong;
    /// # async fn select(
    /// #     mut watcher1: aos_events_event_loop_runtime::Watcher<Pong<'static>>,
    /// #     mut watcher2: aos_events_event_loop_runtime::Watcher<Pong<'static>>,
    /// # ) {
    /// futures::select! {
    ///     message1 = watcher1.next() => println!("channel 1: {:?}", message1),
    ///     message2 = watcher2.next() => println!("channel 2: {:?}", message2),
    /// }
    /// # }
    /// ```
    ///
    /// Note that due to the returned object borrowing the `self` reference, the borrow checker will
    /// enforce only having a single of these returned objects at a time. Drop the previous message
    /// before asking for the next one. That means this will not compile:
    /// ```compile_fail
    /// # use pong_rust_fbs::aos::examples::Pong;
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::Watcher<Pong<'static>>) {
    /// let first = watcher.next();
    /// let second = watcher.next();
    /// first.await;
    /// # }
    /// ```
    /// and nor will this:
    /// ```compile_fail
    /// # use pong_rust_fbs::aos::examples::Pong;
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::Watcher<Pong<'static>>) {
    /// let first = watcher.next().await;
    /// watcher.next();
    /// println!("still have: {:?}", first);
    /// # }
    /// ```
    /// but this is fine:
    /// ```
    /// # use pong_rust_fbs::aos::examples::Pong;
    /// # async fn compile_check(mut watcher: aos_events_event_loop_runtime::Watcher<Pong<'static>>) {
    /// let first = watcher.next().await;
    /// println!("have: {:?}", first);
    /// watcher.next();
    /// # }
    /// ```
    pub fn next(&mut self) -> WatcherNext<'_, <T as FollowWith<'_>>::Inner> {
        WatcherNext(self.0.next(), PhantomData)
    }
}

/// The type returned from [`Watcher::next`], see there for details.
pub struct WatcherNext<'watcher, T>(RawWatcherNext<'watcher>, PhantomData<*mut T>)
where
    T: Follow<'watcher> + 'watcher;

impl<'watcher, T> Future for WatcherNext<'watcher, T>
where
    T: Follow<'watcher> + 'watcher,
{
    type Output = TypedContext<'watcher, T>;

    fn poll(self: Pin<&mut Self>, cx: &mut std::task::Context) -> Poll<Self::Output> {
        Pin::new(&mut self.get_mut().0).poll(cx).map(|context|
                 // SAFETY: The Watcher this was created from verified that the channel is the
                 // right type, and the C++ guarantees that the buffer's type matches.
                 TypedContext(context, PhantomData))
    }
}

impl<'watcher, T> FusedFuture for WatcherNext<'watcher, T>
where
    T: Follow<'watcher> + 'watcher,
{
    fn is_terminated(&self) -> bool {
        self.0.is_terminated()
    }
}

/// A wrapper around [`Context`] which exposes the flatbuffer message with the appropriate type.
pub struct TypedContext<'a, T>(
    // SAFETY: This must have a message, and it must be a valid `T` flatbuffer.
    Context<'a>,
    PhantomData<*mut T>,
)
where
    T: Follow<'a> + 'a;

impl<'a, T> TypedContext<'a, T>
where
    T: Follow<'a> + 'a,
{
    pub fn message(&self) -> Option<T::Inner> {
        self.0.data().map(|data| {
            // SAFETY: C++ guarantees that this is a valid flatbuffer. We guarantee it's the right
            // type based on invariants for our type.
            unsafe { root_unchecked::<T>(data) }
        })
    }

    pub fn monotonic_event_time(&self) -> MonotonicInstant {
        self.0.monotonic_event_time()
    }
    pub fn monotonic_remote_time(&self) -> MonotonicInstant {
        self.0.monotonic_remote_time()
    }
    pub fn realtime_event_time(&self) -> RealtimeInstant {
        self.0.realtime_event_time()
    }
    pub fn realtime_remote_time(&self) -> RealtimeInstant {
        self.0.realtime_remote_time()
    }
    pub fn queue_index(&self) -> u32 {
        self.0.queue_index()
    }
    pub fn remote_queue_index(&self) -> u32 {
        self.0.remote_queue_index()
    }
    pub fn buffer_index(&self) -> i32 {
        self.0.buffer_index()
    }
    pub fn source_boot_uuid(&self) -> &Uuid {
        self.0.source_boot_uuid()
    }
}

impl<'a, T> fmt::Debug for TypedContext<'a, T>
where
    T: Follow<'a> + 'a,
    T::Inner: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("TypedContext")
            .field("monotonic_event_time", &self.monotonic_event_time())
            .field("monotonic_remote_time", &self.monotonic_remote_time())
            .field("realtime_event_time", &self.realtime_event_time())
            .field("realtime_remote_time", &self.realtime_remote_time())
            .field("queue_index", &self.queue_index())
            .field("remote_queue_index", &self.remote_queue_index())
            .field("message", &self.message())
            .field("buffer_index", &self.buffer_index())
            .field("source_boot_uuid", &self.source_boot_uuid())
            .finish()
    }
}

/// Provides access to messages on a channel, without the ability to wait for a new one. This
/// provides APIs to get the latest message, and to follow along and retrieve each message in order.
///
/// Use [`EventLoopRuntime::make_raw_fetcher`] to create one of these.
///
/// This is the non-typed API, which is mainly useful for reflection and does not provide safe APIs
/// for actually interpreting messages. You probably want a [`Fetcher`] instead.
// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct RawFetcher(Pin<Box<ffi::aos::FetcherForRust>>);

impl RawFetcher {
    pub fn fetch_next(&mut self) -> bool {
        self.0.as_mut().FetchNext()
    }

    pub fn fetch(&mut self) -> bool {
        self.0.as_mut().Fetch()
    }

    pub fn context(&self) -> Context {
        Context(self.0.context())
    }
}

/// Provides access to messages on a channel, without the ability to wait for a new one. This
/// provides APIs to get the latest message, and to follow along and retrieve each message in order.
///
/// Use [`EventLoopRuntime::make_fetcher`] to create one of these.
pub struct Fetcher<T>(
    // SAFETY: This must produce messages of type `T`.
    RawFetcher,
    PhantomData<*mut T>,
)
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>;

impl<T> Fetcher<T>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    pub fn fetch_next(&mut self) -> bool {
        self.0.fetch_next()
    }
    pub fn fetch(&mut self) -> bool {
        self.0.fetch()
    }

    pub fn context(&self) -> TypedContext<'_, <T as FollowWith<'_>>::Inner> {
        // SAFETY: We verified that this is the correct type, and C++ guarantees that the buffer's
        // type matches.
        TypedContext(self.0.context(), PhantomData)
    }
}

/// Allows sending messages on a channel.
///
/// This is the non-typed API, which is mainly useful for reflection and does not provide safe APIs
/// for actually creating messages to send. You probably want a [`Sender`] instead.
///
/// Use [`EventLoopRuntime::make_raw_sender`] to create one of these.
// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct RawSender(Pin<Box<ffi::aos::SenderForRust>>);

impl RawSender {
    /// Returns an object which can be used to build a message.
    ///
    /// # Examples
    ///
    /// ```
    /// # use pong_rust_fbs::aos::examples::PongBuilder;
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::RawSender) {
    /// # unsafe {
    /// let mut builder = sender.make_builder();
    /// let pong = PongBuilder::new(builder.fbb()).finish();
    /// builder.send(pong);
    /// # }
    /// # }
    /// ```
    ///
    /// You can bail out of building a message and build another one:
    /// ```
    /// # use pong_rust_fbs::aos::examples::PongBuilder;
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::RawSender) {
    /// # unsafe {
    /// let mut builder1 = sender.make_builder();
    /// builder1.fbb();
    /// drop(builder1);
    /// let mut builder2 = sender.make_builder();
    /// let pong = PongBuilder::new(builder2.fbb()).finish();
    /// builder2.send(pong);
    /// # }
    /// # }
    /// ```
    /// but you cannot build two messages at the same time with a single builder:
    /// ```compile_fail
    /// # use pong_rust_fbs::aos::examples::PongBuilder;
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::RawSender) {
    /// # unsafe {
    /// let mut builder1 = sender.make_builder();
    /// let mut builder2 = sender.make_builder();
    /// PongBuilder::new(builder2.fbb()).finish();
    /// PongBuilder::new(builder1.fbb()).finish();
    /// # }
    /// # }
    /// ```
    pub fn make_builder(&mut self) -> RawBuilder {
        // SAFETY: This is a valid slice, and `u8` doesn't have any alignment
        // requirements. Additionally, the lifetime of the builder is tied to
        // the lifetime of self so the buffer won't be accessible again until
        // the builder is destroyed.
        let allocator = ChannelPreallocatedAllocator::new(unsafe {
            slice::from_raw_parts_mut(self.0.as_mut().data(), self.0.as_mut().size())
        });
        let fbb = FlatBufferBuilder::new_in(allocator);
        RawBuilder {
            raw_sender: self,
            fbb,
        }
    }
}

/// Used for building a message. See [`RawSender::make_builder`] for details.
pub struct RawBuilder<'sender> {
    raw_sender: &'sender mut RawSender,
    fbb: FlatBufferBuilder<'sender, ChannelPreallocatedAllocator<'sender>>,
}

impl<'sender> RawBuilder<'sender> {
    pub fn fbb(
        &mut self,
    ) -> &mut FlatBufferBuilder<'sender, ChannelPreallocatedAllocator<'sender>> {
        &mut self.fbb
    }

    /// # Safety
    ///
    /// `T` must match the type of the channel of the sender this builder was created from.
    pub unsafe fn send<T>(mut self, root: flatbuffers::WIPOffset<T>) -> Result<(), SendError> {
        self.fbb.finish_minimal(root);
        let data = self.fbb.finished_data();

        use ffi::aos::RawSender_Error as FfiError;
        // SAFETY: This is a valid buffer we're passing.
        match self.raw_sender.0.as_mut().SendBuffer(data.len()) {
            FfiError::kOk => Ok(()),
            FfiError::kMessagesSentTooFast => Err(SendError::MessagesSentTooFast),
            FfiError::kInvalidRedzone => Err(SendError::InvalidRedzone),
        }
    }
}

/// Allows sending messages on a channel with a type-safe API.
///
/// Use [`EventLoopRuntime::make_raw_sender`] to create one of these.
pub struct Sender<T>(
    // SAFETY: This must accept messages of type `T`.
    RawSender,
    PhantomData<*mut T>,
)
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>;

impl<T> Sender<T>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    /// Returns an object which can be used to build a message.
    ///
    /// # Examples
    ///
    /// ```
    /// # use pong_rust_fbs::aos::examples::{Pong, PongBuilder};
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::Sender<Pong<'static>>) {
    /// let mut builder = sender.make_builder();
    /// let pong = PongBuilder::new(builder.fbb()).finish();
    /// builder.send(pong);
    /// # }
    /// ```
    ///
    /// You can bail out of building a message and build another one:
    /// ```
    /// # use pong_rust_fbs::aos::examples::{Pong, PongBuilder};
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::Sender<Pong<'static>>) {
    /// let mut builder1 = sender.make_builder();
    /// builder1.fbb();
    /// drop(builder1);
    /// let mut builder2 = sender.make_builder();
    /// let pong = PongBuilder::new(builder2.fbb()).finish();
    /// builder2.send(pong);
    /// # }
    /// ```
    /// but you cannot build two messages at the same time with a single builder:
    /// ```compile_fail
    /// # use pong_rust_fbs::aos::examples::{Pong, PongBuilder};
    /// # fn compile_check(mut sender: aos_events_event_loop_runtime::Sender<Pong<'static>>) {
    /// let mut builder1 = sender.make_builder();
    /// let mut builder2 = sender.make_builder();
    /// PongBuilder::new(builder2.fbb()).finish();
    /// PongBuilder::new(builder1.fbb()).finish();
    /// # }
    /// ```
    pub fn make_builder(&mut self) -> Builder<T> {
        Builder(self.0.make_builder(), PhantomData)
    }
}

/// Used for building a message. See [`Sender::make_builder`] for details.
pub struct Builder<'sender, T>(
    // SAFETY: This must accept messages of type `T`.
    RawBuilder<'sender>,
    PhantomData<*mut T>,
)
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>;

impl<'sender, T> Builder<'sender, T>
where
    for<'a> T: FollowWith<'a>,
    for<'a> <T as FollowWith<'a>>::Inner: Follow<'a>,
{
    pub fn fbb(
        &mut self,
    ) -> &mut FlatBufferBuilder<'sender, ChannelPreallocatedAllocator<'sender>> {
        self.0.fbb()
    }

    pub fn send<'a>(
        self,
        root: flatbuffers::WIPOffset<<T as FollowWith<'a>>::Inner>,
    ) -> Result<(), SendError> {
        // SAFETY: We guarantee this is the right type based on invariants for our type.
        unsafe { self.0.send(root) }
    }
}

#[derive(Clone, Copy, Eq, PartialEq, Debug, Error)]
pub enum SendError {
    #[error("messages have been sent too fast on this channel")]
    MessagesSentTooFast,
    #[error("invalid redzone data, shared memory corruption detected")]
    InvalidRedzone,
}

#[repr(transparent)]
#[derive(Clone, Copy)]
pub struct Context<'context>(&'context ffi::aos::Context);

impl fmt::Debug for Context<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("Context")
            .field("monotonic_event_time", &self.monotonic_event_time())
            .field("monotonic_remote_time", &self.monotonic_remote_time())
            .field("realtime_event_time", &self.realtime_event_time())
            .field("realtime_remote_time", &self.realtime_remote_time())
            .field("queue_index", &self.queue_index())
            .field("remote_queue_index", &self.remote_queue_index())
            .field("size", &self.data().map(|data| data.len()))
            .field("buffer_index", &self.buffer_index())
            .field("source_boot_uuid", &self.source_boot_uuid())
            .finish()
    }
}

impl<'context> Context<'context> {
    pub fn monotonic_event_time(self) -> MonotonicInstant {
        MonotonicInstant(self.0.monotonic_event_time)
    }

    pub fn monotonic_remote_time(self) -> MonotonicInstant {
        MonotonicInstant(self.0.monotonic_remote_time)
    }

    pub fn realtime_event_time(self) -> RealtimeInstant {
        RealtimeInstant(self.0.realtime_event_time)
    }

    pub fn realtime_remote_time(self) -> RealtimeInstant {
        RealtimeInstant(self.0.realtime_remote_time)
    }

    pub fn queue_index(self) -> u32 {
        self.0.queue_index
    }
    pub fn remote_queue_index(self) -> u32 {
        self.0.remote_queue_index
    }

    pub fn data(self) -> Option<&'context [u8]> {
        if self.0.data.is_null() {
            None
        } else {
            // SAFETY:
            //  * `u8` has no alignment requirements
            //  * It must be a single initialized flatbuffers buffer
            //  * The borrow in `self.0` guarantees it won't be modified for `'context`
            Some(unsafe { slice::from_raw_parts(self.0.data as *const u8, self.0.size) })
        }
    }

    pub fn buffer_index(self) -> i32 {
        self.0.buffer_index
    }

    pub fn source_boot_uuid(self) -> &'context Uuid {
        // SAFETY: `self` has a valid C++ object. C++ guarantees that the return value will be
        // valid until something changes the context, which is `'context`.
        Uuid::from_bytes_ref(&self.0.source_boot_uuid)
    }
}

/// The type returned from [`EventLoopRuntime::on_run`], see there for details.
// SAFETY: If this outlives the parent EventLoop, the C++ code will LOG(FATAL).
#[repr(transparent)]
pub struct OnRun(Pin<Box<ffi::aos::OnRunForRust>>);

impl Future for &'_ OnRun {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _: &mut std::task::Context) -> Poll<()> {
        if self.0.is_running() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// Represents a `aos::monotonic_clock::time_point` in a natural Rust way. This
/// is intended to have the same API as [`std::time::Instant`], any missing
/// functionality can be added if useful.
#[repr(transparent)]
#[derive(Clone, Copy, Eq, PartialEq)]
pub struct MonotonicInstant(i64);

impl MonotonicInstant {
    /// `aos::monotonic_clock::min_time`, commonly used as a sentinel value.
    pub const MIN_TIME: Self = Self(i64::MIN);

    pub fn is_min_time(self) -> bool {
        self == Self::MIN_TIME
    }

    pub fn duration_since_epoch(self) -> Option<Duration> {
        if self.is_min_time() {
            None
        } else {
            Some(Duration::from_nanos(self.0.try_into().expect(
                "monotonic_clock::time_point should always be after the epoch",
            )))
        }
    }
}

impl Add<Duration> for MonotonicInstant {
    type Output = MonotonicInstant;

    fn add(self, rhs: Duration) -> Self::Output {
        Self(self.0 + i64::try_from(rhs.as_nanos()).unwrap())
    }
}

impl From<MonotonicInstant> for i64 {
    fn from(value: MonotonicInstant) -> Self {
        value.0
    }
}

impl fmt::Debug for MonotonicInstant {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.duration_since_epoch().fmt(f)
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, Eq, PartialEq)]
pub struct RealtimeInstant(i64);

impl RealtimeInstant {
    pub const MIN_TIME: Self = Self(i64::MIN);

    pub fn is_min_time(self) -> bool {
        self == Self::MIN_TIME
    }

    pub fn duration_since_epoch(self) -> Option<Duration> {
        if self.is_min_time() {
            None
        } else {
            Some(Duration::from_nanos(self.0.try_into().expect(
                "monotonic_clock::time_point should always be after the epoch",
            )))
        }
    }
}

impl From<RealtimeInstant> for i64 {
    fn from(value: RealtimeInstant) -> Self {
        value.0
    }
}

impl fmt::Debug for RealtimeInstant {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.duration_since_epoch().fmt(f)
    }
}

mod panic_waker {
    use std::task::{RawWaker, RawWakerVTable, Waker};

    unsafe fn clone_panic_waker(_data: *const ()) -> RawWaker {
        raw_panic_waker()
    }

    unsafe fn noop(_data: *const ()) {}

    unsafe fn wake_panic(_data: *const ()) {
        panic!("Nothing should wake EventLoopRuntime's waker");
    }

    const PANIC_WAKER_VTABLE: RawWakerVTable =
        RawWakerVTable::new(clone_panic_waker, wake_panic, wake_panic, noop);

    fn raw_panic_waker() -> RawWaker {
        RawWaker::new(std::ptr::null(), &PANIC_WAKER_VTABLE)
    }

    pub fn panic_waker() -> Waker {
        // SAFETY: The implementations of the RawWakerVTable functions do what is required of them.
        unsafe { Waker::from_raw(raw_panic_waker()) }
    }
}

use panic_waker::panic_waker;

pub struct ExitHandle(UniquePtr<CppExitHandle>);

impl ExitHandle {
    /// Exits the EventLoops represented by this handle. You probably want to immediately return
    /// from the context this is called in. Awaiting [`ExitHandle::exit`] instead of using this
    /// function is an easy way to do that.
    pub fn exit_sync(mut self) {
        self.0.as_mut().unwrap().Exit();
    }

    /// Exits the EventLoops represented by this handle, and never returns. Immediately awaiting
    /// this from a [`EventLoopRuntime::spawn`]ed task is usually what you want, it will ensure
    /// that no more code from that task runs.
    pub async fn exit(self) -> Never {
        self.exit_sync();
        pending().await
    }
}

impl From<UniquePtr<CppExitHandle>> for ExitHandle {
    fn from(inner: UniquePtr<ffi::aos::ExitHandle>) -> Self {
        Self(inner)
    }
}

pub struct ChannelPreallocatedAllocator<'a> {
    buffer: &'a mut [u8],
}

impl<'a> ChannelPreallocatedAllocator<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer }
    }
}

#[derive(Debug, Error)]
#[error("Can't allocate more memory with a fixed size allocator")]
pub struct OutOfMemory;

// SAFETY: Allocator follows the required behavior.
unsafe impl Allocator for ChannelPreallocatedAllocator<'_> {
    type Error = OutOfMemory;
    fn grow_downwards(&mut self) -> Result<(), Self::Error> {
        // Fixed size allocator can't grow.
        Err(OutOfMemory)
    }

    fn len(&self) -> usize {
        self.buffer.len()
    }
}

impl Deref for ChannelPreallocatedAllocator<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.buffer
    }
}

impl DerefMut for ChannelPreallocatedAllocator<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.buffer
    }
}
