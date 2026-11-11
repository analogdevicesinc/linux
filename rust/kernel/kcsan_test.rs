// SPDX-License-Identifier: GPL-2.0

//! KUnit test checking that KCSAN detects data races in Rust code.

#[macros::kunit_tests(rust_kcsan)]
mod tests {
    use crate::{
        bindings,
        error::to_result,
        ffi::{
            c_char,
            c_void, //
        },
        prelude::*,
        sync::{
            atomic::{
                Atomic,
                Relaxed, //
            },
            Arc,
            Completion, //
        },
        time::{
            Delta,
            Instant,
            Monotonic, //
        },
        types::ScopeGuard,
        workqueue, //
    };
    use core::{
        cell::UnsafeCell,
        ptr,
        slice, //
    };

    struct Racy(UnsafeCell<u64>);

    // SAFETY: Not actually safe: unsynchronized concurrent access is the point of this test.
    unsafe impl Sync for Racy {}

    static RACY: Racy = Racy(UnsafeCell::new(0));

    /// Set by [`probe_console`] once KCSAN reported a data race in [`racy_inc`].
    static OBSERVED: Atomic<bool> = Atomic::new(false);

    #[inline(never)]
    fn racy_inc() {
        let p = RACY.0.get();
        // SAFETY: `p` is valid; the data race is intentional. `wrapping_add()` keeps the load and
        // store in one basic block, so that the access is instrumented as a compound access (plain
        // writes may be assumed atomic with `CONFIG_KCSAN_ASSUME_PLAIN_WRITES_ATOMIC=y`).
        unsafe {
            *p = (*p).wrapping_add(1);
        }
    }

    fn contains(haystack: &[u8], needle: &[u8]) -> bool {
        haystack.windows(needle.len()).any(|w| w == needle)
    }

    /// Probe for the `console` tracepoint.
    ///
    /// # Safety
    ///
    /// `buf` must be non-null and valid for reads of `len` bytes.
    unsafe extern "C" fn probe_console(_data: *mut c_void, buf: *const c_char, len: usize) {
        // SAFETY: By the safety requirements of this function.
        let line = unsafe { slice::from_raw_parts(buf.cast::<u8>(), len) };

        if contains(line, b"BUG: KCSAN: data-race in ") && contains(line, b"racy_inc") {
            OBSERVED.store(true, Relaxed);
        }
    }

    #[pin_data]
    struct Ctx {
        stop: Atomic<bool>,
        #[pin]
        done: Completion,
    }

    #[test]
    fn test_data_race() -> Result {
        // Do not pass on a report from a previous run of this suite.
        OBSERVED.store(false, Relaxed);

        let ctx = Arc::pin_init(
            pin_init!(Ctx {
                stop: Atomic::new(false),
                done <- Completion::new(),
            }),
            GFP_KERNEL,
        )?;

        // SAFETY: `probe_console` has the signature expected by the `console` tracepoint. Being
        // part of the kernel image, it and `OBSERVED` outlive the registration, so there is no
        // need to synchronize with in-flight probes after unregistering.
        to_result(unsafe {
            bindings::tracepoint_probe_register(
                &raw mut bindings::__tracepoint_console,
                probe_console as *mut c_void,
                ptr::null_mut(),
            )
        })?;
        let _unregister = ScopeGuard::new(|| {
            // SAFETY: The probe was registered above.
            unsafe {
                bindings::tracepoint_probe_unregister(
                    &raw mut bindings::__tracepoint_console,
                    probe_console as *mut c_void,
                    ptr::null_mut(),
                );
            }
        });

        let worker = ctx.clone();
        workqueue::system_dfl().try_spawn(GFP_KERNEL, move || {
            while !worker.stop.load(Relaxed) {
                racy_inc();
            }
            worker.done.complete_all();
        })?;

        let start = Instant::<Monotonic>::now();
        while !OBSERVED.load(Relaxed) && start.elapsed() < Delta::from_secs(10) {
            racy_inc();
        }

        ctx.stop.store(true, Relaxed);
        ctx.done.wait_for_completion();
        drop(_unregister);

        assert!(OBSERVED.load(Relaxed));
        Ok(())
    }
}
