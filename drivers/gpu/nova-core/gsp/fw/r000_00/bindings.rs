// SPDX-License-Identifier: GPL-2.0

#[repr(C)]
#[derive(Default)]
pub struct __IncompleteArrayField<T>(::core::marker::PhantomData<T>, [T; 0]);
impl<T> __IncompleteArrayField<T> {
    #[inline]
    pub const fn new() -> Self {
        __IncompleteArrayField(::core::marker::PhantomData, [])
    }
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        self as *const _ as *const T
    }
    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self as *mut _ as *mut T
    }
    #[inline]
    pub unsafe fn as_slice(&self, len: usize) -> &[T] {
        ::core::slice::from_raw_parts(self.as_ptr(), len)
    }
    #[inline]
    pub unsafe fn as_mut_slice(&mut self, len: usize) -> &mut [T] {
        ::core::slice::from_raw_parts_mut(self.as_mut_ptr(), len)
    }
}
impl<T> ::core::fmt::Debug for __IncompleteArrayField<T> {
    fn fmt(&self, fmt: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
        fmt.write_str("__IncompleteArrayField")
    }
}
pub type __u8 = ffi::c_uchar;
pub type __u16 = ffi::c_ushort;
pub type __u32 = ffi::c_uint;
pub type __u64 = ffi::c_ulonglong;
pub type u8_ = __u8;
pub type u16_ = __u16;
pub type u32_ = __u32;
pub type u64_ = __u64;
#[repr(C)]
#[derive(MaybeZeroable)]
pub struct GMCAPI_HEADER {
    pub command: u32_,
    pub size: u32_,
    pub sequence: u64_,
    pub __bindgen_anon_1: GMCAPI_HEADER__bindgen_ty_1,
    pub reserved: [u32_; 5usize],
    pub data: __IncompleteArrayField<u8_>,
}
#[repr(C)]
#[derive(Copy, Clone, MaybeZeroable)]
pub union GMCAPI_HEADER__bindgen_ty_1 {
    pub request: GMCAPI_HEADER__bindgen_ty_1__bindgen_ty_1,
    pub response: GMCAPI_HEADER__bindgen_ty_1__bindgen_ty_2,
}
#[repr(C)]
#[derive(Debug, Default, Copy, Clone, MaybeZeroable)]
pub struct GMCAPI_HEADER__bindgen_ty_1__bindgen_ty_1 {
    pub max_response_size: u32_,
}
#[repr(C)]
#[derive(Debug, Default, Copy, Clone, MaybeZeroable)]
pub struct GMCAPI_HEADER__bindgen_ty_1__bindgen_ty_2 {
    pub status: u32_,
}
impl Default for GMCAPI_HEADER__bindgen_ty_1 {
    fn default() -> Self {
        let mut s = ::core::mem::MaybeUninit::<Self>::uninit();
        unsafe {
            ::core::ptr::write_bytes(s.as_mut_ptr(), 0, 1);
            s.assume_init()
        }
    }
}
impl Default for GMCAPI_HEADER {
    fn default() -> Self {
        let mut s = ::core::mem::MaybeUninit::<Self>::uninit();
        unsafe {
            ::core::ptr::write_bytes(s.as_mut_ptr(), 0, 1);
            s.assume_init()
        }
    }
}
pub const GMCAPI_COMMANDS_GMCAPI_CMD_RESERVED_LAST: GMCAPI_COMMANDS = 65535;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_GSP_INIT: GMCAPI_COMMANDS = 65537;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_EXEC_GENERIC_BOOTLOADER: GMCAPI_COMMANDS = 65538;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_EXEC_HS_BINARY: GMCAPI_COMMANDS = 65539;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_GSP_SUSPEND: GMCAPI_COMMANDS = 65540;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_GSP_RESUME_DONE: GMCAPI_COMMANDS = 65541;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_ADD_VGPU_TYPE: GMCAPI_COMMANDS = 131073;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_QUERY_SUPPORTED_VGPU_TYPES: GMCAPI_COMMANDS = 131074;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_QUERY_CREATABLE_VGPU_TYPES: GMCAPI_COMMANDS = 131075;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_ASSIGN_VGPU_TYPE: GMCAPI_COMMANDS = 131076;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_DEASSIGN_VGPU_TYPE: GMCAPI_COMMANDS = 131077;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_QUERY_VGPU_PROPERTIES: GMCAPI_COMMANDS = 131078;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_QUERY_ASSIGNED_VF_VGPU_TYPE: GMCAPI_COMMANDS = 131079;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_BOOTLOAD_GSP_VGPU_PLUGIN_TASK: GMCAPI_COMMANDS = 131104;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_SHUTDOWN_GSP_VGPU_PLUGIN_TASK: GMCAPI_COMMANDS = 131105;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_SHUTDOWN_GSP_VGPU_PLUGIN_TASK_COMPLETE: GMCAPI_COMMANDS =
    131106;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_CLEANUP_GSP_VGPU_PLUGIN_RESOURCES: GMCAPI_COMMANDS = 131107;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_VGPU_PLUGIN_TRIGGERED_EVENT: GMCAPI_COMMANDS = 131108;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_VGPU_MGR_SCRUB_GUEST_FB: GMCAPI_COMMANDS = 131109;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_VGPU_MGR_ALLOC_GSP_CEUTILS: GMCAPI_COMMANDS = 131110;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_VGPU_MGR_FREE_GSP_CEUTILS: GMCAPI_COMMANDS = 131111;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_SCHED_CONTROL: GMCAPI_COMMANDS = 196609;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_MMU_FAULT_QUEUED: GMCAPI_COMMANDS = 262145;
pub const GMCAPI_COMMANDS_GMCAPI_CMD_INVALID: GMCAPI_COMMANDS = 4294967295;
pub type GMCAPI_COMMANDS = ffi::c_uint;
