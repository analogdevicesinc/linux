// SPDX-License-Identifier: GPL-2.0

//! Nova-core auxbus data. Contains all the methods used by the auxbus drivers
//! to interact with nova-core.

use core::pin::Pin;

use kernel::{
    auxiliary,
    device::Bound,
    prelude::*,
    types::ForLt, //
};

pub use crate::gpu::Spec;

use crate::gpu::Gpu;

/// API handle for the auxiliary bus child drivers to interact with nova-core.
pub struct NovaCoreApi<'bound> {
    pub(crate) gpu: Pin<&'bound Gpu<'bound>>,
}

impl NovaCoreApi<'_> {
    /// Obtain a [`NovaCoreApi`] handle from an auxiliary device registered
    /// by nova-core.
    pub fn of(adev: &auxiliary::Device<Bound>) -> Result<NovaCoreApiHandle<'_>> {
        NovaCoreApiHandle::of(adev)
    }

    /// Returns the GPU [`Spec`].
    pub fn spec(&self) -> &Spec {
        &self.gpu.spec
    }
}

/// Expose a handle to nova-core API
pub struct NovaCoreApiHandle<'a> {
    adev: &'a auxiliary::Device<Bound>,
}

impl<'a> NovaCoreApiHandle<'a> {
    fn of(adev: &'a auxiliary::Device<Bound>) -> Result<Self> {
        adev.registration_data_with::<ForLt!(NovaCoreApi<'_>), ()>(|_| ())?;
        Ok(Self { adev })
    }

    /// Access the [`NovaCoreApi`] through a closure.
    pub fn with<R>(&self, f: impl for<'b> FnOnce(Pin<&'a NovaCoreApi<'b>>) -> R) -> R {
        self.adev
            .registration_data_with::<ForLt!(NovaCoreApi<'_>), R>(f)
            .expect("TypeId was validated in NovaCoreApiHandle::of()")
    }
}
