// Copyright 2016 The android_sensor_sys Developers
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.
#![allow(non_camel_case_types)]

extern crate android_looper_sys;
extern crate libc;

use android_looper_sys::*;
use libc::{c_int, c_float, c_void, c_char};
use std::mem;

/// Earth's gravity in m/s^2
pub const ASENSOR_STANDARD_GRAVITY: f32 = 9.80665;
/// Maximum magnetic field on Earth's surface in uT
pub const ASENSOR_MAGNETIC_FIELD_EARTH_MAX: f32 = 60.0;
/// Minimum magnetic field on Earth's surface in uT
pub const ASENSOR_MAGNETIC_FIELD_EARTH_MIN: f32 = 30.0;

#[repr(C)]
#[derive(Copy)]
pub struct AHeartRateEvent {
    pub bpm: c_float,
    pub status: i8,
}
impl Clone for AHeartRateEvent {
    fn clone(&self) -> Self {
        *self
    }
}
impl Default for AHeartRateEvent {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Copy)]
pub struct AMetaDataEvent {
    pub what: i32,
    pub sensor: i32,
}
impl Clone for AMetaDataEvent {
    fn clone(&self) -> Self {
        *self
    }
}
impl Default for AMetaDataEvent {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

/// ASensor is an opaque type that provides information about an hardware sensors.
///
/// A ASensor pointer can be obtained using `ASensorManager_getDefaultSensor()`, `ASensorManager_getDefaultSensorEx()` or from a `ASensorList`.
pub enum ASensor { }

#[repr(C)]
#[derive(Copy)]
pub struct ASensorEvent {
    pub version: i32,
    pub sensor: i32,
    pub event_type: i32,
    reserved0: i32,
    pub timestamp: i64,
    _bindgen_data_1_: [u64; 8usize],
    pub flags: i32,
    reserved1: [i32; 3usize],
}
impl ASensorEvent {
    unsafe fn _data_f32(&mut self) -> *mut [c_float; 16usize] {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn vector(&mut self) -> *mut ASensorVector {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn acceleration(&mut self) -> *mut ASensorVector {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn magnetic(&mut self) -> *mut ASensorVector {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn temperature(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn distance(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn light(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn pressure(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn relative_humidity(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn uncalibrated_gyro(&mut self) -> *mut AUncalibratedEvent {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn uncalibrated_magnetic(&mut self) -> *mut AUncalibratedEvent {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn meta_data(&mut self) -> *mut AMetaDataEvent {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn heart_rate(&mut self) -> *mut AHeartRateEvent {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }

    unsafe fn data_u64(&mut self) -> *mut ASensorEvent_u64 {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn step_counter(&mut self) -> *mut u64 {
        (*self.data_u64()).step_counter()
    }
}
impl Clone for ASensorEvent {
    fn clone(&self) -> Self {
        *self
    }
}
impl Default for ASensorEvent {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Copy)]
struct ASensorEvent_u64 {
    pub _bindgen_data_: [u64; 8usize],
}
impl ASensorEvent_u64 {
    unsafe fn _data(&mut self) -> *mut [u64; 8usize] {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn step_counter(&mut self) -> *mut u64 {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_);
        mem::transmute(raw.offset(0))
    }
}
impl Clone for ASensorEvent_u64 {
    fn clone(&self) -> Self {
        *self
    }
}
impl Default for ASensorEvent_u64 {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

/// ASensorEventQueue is an opaque type that provides access to `ASensorEvent` from hardware sensors.
///
/// A new ASensorEventQueue can be obtained using `ASensorManager_createEventQueue()`.
pub enum ASensorEventQueue { }

/// ASensorList is an array of reference to `ASensor`.
///
/// A ASensorList can be initialized using `ASensorManager_getSensorList()`.
pub type ASensorList = *const ASensorRef;

/// ASensorRef is a type for constant pointers to `ASensor`.
///
/// This is used to define entry in `ASensorList` arrays.
pub type ASensorRef = *const ASensor;

/// ASensorManager is an opaque type to manage sensors and events queues.
///
/// ASensorManager is a singleton that can be obtained using `ASensorManager_getInstance()`.
pub enum ASensorManager { }

/// A sensor event.
#[repr(C)]
#[derive(Copy)]
pub struct ASensorVector {
    pub _bindgen_data_1_: [u32; 3usize],
    pub status: u8,
    pub reserved: [u8; 3usize],
}

impl ASensorVector {
    pub unsafe fn v(&mut self) -> *mut [c_float; 3usize] {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn x(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn y(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(4))
    }
    pub unsafe fn z(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(8))
    }
    pub unsafe fn azimuth(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn pitch(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(4))
    }
    pub unsafe fn roll(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(8))
    }
}
impl Clone for ASensorVector {
    fn clone(&self) -> Self {
        *self
    }
}
impl Default for ASensorVector {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Copy)]
pub struct AUncalibratedEvent {
    pub _bindgen_data_1_: [u32; 3usize],
    pub _bindgen_data_2_: [u32; 3usize],
}
impl AUncalibratedEvent {
    pub unsafe fn uncalib(&mut self) -> *mut [c_float; 3usize] {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn x_uncalib(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn y_uncalib(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(4))
    }
    pub unsafe fn z_uncalib(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_1_);
        mem::transmute(raw.offset(8))
    }
    pub unsafe fn bias(&mut self) -> *mut [c_float; 3usize] {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_2_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn x_bias(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_2_);
        mem::transmute(raw.offset(0))
    }
    pub unsafe fn y_bias(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_2_);
        mem::transmute(raw.offset(4))
    }
    pub unsafe fn z_bias(&mut self) -> *mut c_float {
        let raw: *mut u8 = mem::transmute(&self._bindgen_data_2_);
        mem::transmute(raw.offset(8))
    }
}
impl Clone for AUncalibratedEvent {
    fn clone(&self) -> Self {
        *self
    }
}
impl Default for AUncalibratedEvent {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(u32)]
pub enum SensorType {
    /// `ASENSOR_TYPE_ACCELEROMETER` reporting-mode: continuous
    ///
    /// All values are in SI units (m/s^2) and measure the acceleration of the device minus the force of gravity.
    Accelerometer = 1,
    /// `ASENSOR_TYPE_MAGNETIC_FIELD` reporting-mode: continuous
    ///
    /// All values are in micro-Tesla (uT) and measure the geomagnetic field in the X, Y and Z axis.
    MagneticField = 2,
    /// `ASENSOR_TYPE_GYROSCOPE` reporting-mode: continuous
    ///
    /// All values are in radians/second and measure the rate of rotation around the X, Y and Z axis.
    Gyroscope = 4,
    /// `ASENSOR_TYPE_LIGHT` reporting-mode: on-change
    ///
    /// The light sensor value is returned in SI lux units.
    Light = 5,
    /// `ASENSOR_TYPE_PROXIMITY` reporting-mode: on-change
    ///
    /// The proximity sensor which turns the screen off and back on during calls is the wake-up proximity sensor. Implement wake-up proximity sensor before implementing a non wake-up proximity sensor. For the wake-up proximity sensor set the flag SENSOR_FLAG_WAKE_UP. The value corresponds to the distance to the nearest object in centimeters.
    Proximity = 8,
}
#[derive(Clone, Copy, Debug)]
#[repr(i32)]
pub enum SensorStatus {
    /// `ASENSOR_STATUS_NO_CONTACT` no contact
    NoContact = -1,
    /// `ASENSOR_STATUS_UNRELIABLE` unreliable
    Unreliable = 0,
    /// `ASENSOR_STATUS_ACCURACY_LOW` low accuracy
    AccuracyLow = 1,
    /// `ASENSOR_STATUS_ACCURACY_MEDIUM` medium accuracy
    AccuracyMedium = 2,
    /// `ASENSOR_STATUS_ACCURACY_HIGH` high accuracy
    AccuracyHigh = 3,
}
#[derive(Clone, Copy, Debug)]
#[repr(u32)]
pub enum ReportingMode {
    /// `AREPORTING_MODE_CONTINUOUS` continuous reporting
    Continuous = 0,
    /// `AREPORTING_MODE_ON_CHANGE` reporting on change
    OnChange = 1,
    /// `AREPORTING_MODE_ONE_SHOT` one shot reporting
    OneShot = 2,
    /// `AREPORTING_MODE_SPECIAL_TRIGGER` special trigger reporting
    SpecialTrigger = 3,
}

extern "C" {
    pub fn ASensorManager_getInstance() -> *mut ASensorManager;
    pub fn ASensorManager_getSensorList(manager: *mut ASensorManager,
                                        list: *mut ASensorList)
                                        -> c_int;
    pub fn ASensorManager_getDefaultSensor(manager: *mut ASensorManager,
                                           seonsor_type: SensorType)
                                           -> *const ASensor;
    pub fn ASensorManager_getDefaultSensorEx(manager: *mut ASensorManager,
                                             _type: c_int,
                                             wakeUp: u8)
                                             -> *const ASensor;
    pub fn ASensorManager_createEventQueue(manager: *mut ASensorManager,
                                           looper: *mut ALooper,
                                           ident: i32,
                                           callback: ALooper_callbackFunc,
                                           data: *mut c_void)
                                           -> *mut ASensorEventQueue;
    pub fn ASensorManager_destroyEventQueue(manager: *mut ASensorManager,
                                            queue: *mut ASensorEventQueue)
                                            -> c_int;
    pub fn ASensorEventQueue_enableSensor(queue: *mut ASensorEventQueue,
                                          sensor: *const ASensor)
                                          -> c_int;
    pub fn ASensorEventQueue_disableSensor(queue: *mut ASensorEventQueue,
                                           sensor: *const ASensor)
                                           -> c_int;
    pub fn ASensorEventQueue_setEventRate(queue: *mut ASensorEventQueue,
                                          sensor: *const ASensor,
                                          usec: i32)
                                          -> c_int;
    pub fn ASensorEventQueue_hasEvents(queue: *mut ASensorEventQueue) -> c_int;
    pub fn ASensorEventQueue_getEvents(queue: *mut ASensorEventQueue,
                                       events: *mut ASensorEvent,
                                       count: isize)
                                       -> c_int;
    pub fn ASensor_getName(sensor: *const ASensor) -> *const c_char;
    pub fn ASensor_getVendor(sensor: *const ASensor) -> *const c_char;
    pub fn ASensor_getType(sensor: *const ASensor) -> c_int;
    pub fn ASensor_getResolution(sensor: *const ASensor) -> c_float;
    pub fn ASensor_getMinDelay(sensor: *const ASensor) -> c_int;
    pub fn ASensor_getFifoMaxEventCount(sensor: *const ASensor) -> c_int;
    pub fn ASensor_getFifoReservedEventCount(sensor: *const ASensor) -> c_int;
    pub fn ASensor_getStringType(sensor: *const ASensor) -> *const c_char;
    pub fn ASensor_getReportingMode(sensor: *const ASensor) -> c_int;
    pub fn ASensor_isWakeUpSensor(sensor: *const ASensor) -> u8;
}
