//! CspArch implementation for ESP32-P4 / ESP-IDF FreeRTOS.
//!
//! All primitives are single-threaded: queues are non-blocking VecDeques,
//! mutexes and semaphores are AtomicBool flags.  route_work() is called
//! immediately after every iface.rx() so the QFIFO is never empty when
//! we need it, and accept(0)/read(0) are non-blocking, matching our
//! single-threaded dispatch loop.

use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, Ordering};
use std::collections::VecDeque;

struct EspQueue {
    buf: VecDeque<Vec<u8>>,
    item_size: usize,
}

pub struct EspArch;

unsafe impl Send for EspArch {}
unsafe impl Sync for EspArch {}

unsafe impl libcsp::CspArch for EspArch {
    fn get_ms(&self) -> u32 {
        (unsafe { esp_idf_sys::esp_timer_get_time() } / 1_000) as u32
    }

    fn get_s(&self) -> u32 {
        (unsafe { esp_idf_sys::esp_timer_get_time() } / 1_000_000) as u32
    }

    fn sleep_ms(&self, ms: u32) {
        esp_idf_hal::delay::FreeRtos::delay_ms(ms);
    }

    fn bin_sem_create(&self) -> *mut c_void {
        Box::into_raw(Box::new(AtomicBool::new(false))) as *mut c_void
    }

    fn bin_sem_remove(&self, sem: *mut c_void) {
        unsafe { drop(Box::from_raw(sem as *mut AtomicBool)) };
    }

    fn bin_sem_wait(&self, sem: *mut c_void, _timeout: u32) -> bool {
        let f = unsafe { &*(sem as *const AtomicBool) };
        f.compare_exchange(true, false, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    }

    fn bin_sem_post(&self, sem: *mut c_void) -> bool {
        unsafe { &*(sem as *const AtomicBool) }.store(true, Ordering::Release);
        true
    }

    fn mutex_create(&self) -> *mut c_void {
        Box::into_raw(Box::new(AtomicBool::new(true))) as *mut c_void
    }

    fn mutex_remove(&self, mutex: *mut c_void) {
        unsafe { drop(Box::from_raw(mutex as *mut AtomicBool)) };
    }

    fn mutex_lock(&self, _mutex: *mut c_void, _timeout: u32) -> bool {
        true
    }

    fn mutex_unlock(&self, _mutex: *mut c_void) -> bool {
        true
    }

    fn queue_create(&self, length: usize, item_size: usize) -> *mut c_void {
        Box::into_raw(Box::new(EspQueue {
            buf: VecDeque::with_capacity(length),
            item_size,
        })) as *mut c_void
    }

    fn queue_remove(&self, queue: *mut c_void) {
        unsafe { drop(Box::from_raw(queue as *mut EspQueue)) };
    }

    fn queue_enqueue(&self, queue: *mut c_void, item: *const c_void, _timeout: u32) -> bool {
        let q = unsafe { &mut *(queue as *mut EspQueue) };
        let mut v = vec![0u8; q.item_size];
        unsafe { core::ptr::copy_nonoverlapping(item as *const u8, v.as_mut_ptr(), q.item_size) };
        q.buf.push_back(v);
        true
    }

    fn queue_dequeue(&self, queue: *mut c_void, item: *mut c_void, _timeout: u32) -> bool {
        let q = unsafe { &mut *(queue as *mut EspQueue) };
        match q.buf.pop_front() {
            Some(v) => {
                unsafe { core::ptr::copy_nonoverlapping(v.as_ptr(), item as *mut u8, q.item_size) };
                true
            }
            None => false,
        }
    }

    fn queue_size(&self, queue: *mut c_void) -> usize {
        unsafe { &*(queue as *const EspQueue) }.buf.len()
    }
}

static ARCH: EspArch = EspArch;
libcsp::export_arch!(EspArch, ARCH);
