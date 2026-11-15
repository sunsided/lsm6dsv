//! Wokwi custom chip wrapping the `lsm6dsv-sim` behavioral model.
//!
//! Pins: `VCC`, `GND`, `SDA`, `SCL` and `INT1` (data-ready, active high).
//!
//! Controls (also settable as attributes and via `set-control` in scenarios): `accelX`,
//! `accelY`, `accelZ` in mg and `gyroX`, `gyroY`, `gyroZ` in dps. The chip converts them
//! with the configured full scales and produces a sample every [`SAMPLE_PERIOD_US`].

use lsm6dsv_registers::{Ctrl6, Ctrl8, Int1Ctrl, RegisterAddress, StatusReg};
use lsm6dsv_sim::Lsm6dsvModel;
use std::cell::RefCell;
use std::ffi::{CString, c_void};
use wokwi_chip_ll::*;

/// How often the controls are sampled into the outputs, in microseconds.
const SAMPLE_PERIOD_US: u32 = 10_000;

struct Chip {
    model: Lsm6dsvModel,
    int1: PinId,
    accel: [AttrId; 3],
    gyro: [AttrId; 3],
}

thread_local! {
    static CHIP: RefCell<Option<Chip>> = const { RefCell::new(None) };
}

fn with_chip<R>(f: impl FnOnce(&mut Chip) -> R) -> R {
    CHIP.with(|chip| f(chip.borrow_mut().as_mut().expect("chipInit not called")))
}

fn cstr(s: &str) -> CString {
    CString::new(s).expect("no interior NUL")
}

fn debug(message: &str) {
    let message = cstr(message);
    // SAFETY: `message` is a valid NUL-terminated string for the duration of the call.
    unsafe { debugPrint(message.as_ptr()) };
}

/// Converts a physical value to a raw output, saturating at the `i16` range.
fn to_raw(value: f32, per_lsb: f32) -> i16 {
    (value / per_lsb)
        .round()
        .clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
}

impl Chip {
    fn sample(&mut self) {
        // SAFETY: Attribute IDs were obtained from the simulator in `chipInit`.
        let (accel, gyro) = unsafe {
            (
                self.accel.map(|id| attrRead(id) as i32 as f32),
                self.gyro.map(|id| attrRead(id) as i32 as f32),
            )
        };

        let ctrl8 = Ctrl8::from(self.model.main_register(RegisterAddress::CTRL8.addr()));
        let ctrl6 = Ctrl6::from(self.model.main_register(RegisterAddress::CTRL6.addr()));
        let mg_per_lsb = ctrl8.fs_xl().mg_per_lsb();
        let dps_per_lsb = ctrl6.fs_g().mdps_per_lsb() / 1000.0;

        self.model.set_accel(accel.map(|v| to_raw(v, mg_per_lsb)));
        self.model.set_gyro(gyro.map(|v| to_raw(v, dps_per_lsb)));
        self.model.tick_ns(u64::from(SAMPLE_PERIOD_US) * 1000);
        self.update_int1();
    }

    /// Drives INT1 from the data-ready flags routed to it.
    fn update_int1(&mut self) {
        let routing = Int1Ctrl::from(self.model.main_register(RegisterAddress::INT1_CTRL.addr()));
        let status = StatusReg::from(self.model.main_register(RegisterAddress::STATUS_REG.addr()));
        let active =
            (routing.int1_drdy_xl() && status.xlda()) || (routing.int1_drdy_g() && status.gda());

        // SAFETY: The pin ID was obtained from the simulator in `chipInit`.
        unsafe { pinWrite(self.int1, if active { HIGH } else { LOW }) };
    }
}

extern "C" fn on_connect(_: *mut c_void, _address: u32, is_read: bool) -> bool {
    with_chip(|chip| chip.model.i2c_start(is_read))
}

extern "C" fn on_read(_: *mut c_void) -> u8 {
    with_chip(|chip| chip.model.i2c_read())
}

extern "C" fn on_write(_: *mut c_void, data: u8) -> bool {
    with_chip(|chip| chip.model.i2c_write(data))
}

extern "C" fn on_disconnect(_: *mut c_void) {
    // Reading the outputs clears data-ready, so INT1 may drop after a transfer.
    with_chip(Chip::update_int1);
}

extern "C" fn on_sample(_: *mut c_void) {
    with_chip(Chip::sample);
}

/// Entry point called by the simulator.
#[unsafe(no_mangle)]
pub extern "C" fn chipInit() {
    // SAFETY: All pointers passed to the simulator are valid for the duration of each
    // call; configs are copied by the simulator. Init functions are only called here.
    unsafe {
        let int1 = pinInit(cstr("INT1").as_ptr(), OUTPUT_LOW);
        let sda = pinInit(cstr("SDA").as_ptr(), INPUT);
        let scl = pinInit(cstr("SCL").as_ptr(), INPUT);

        let model = Lsm6dsvModel::new();
        i2cInit(&I2CConfig {
            user_data: std::ptr::null(),
            address: u32::from(model.address()),
            scl,
            sda,
            connect: on_connect,
            read: on_read,
            write: on_write,
            disconnect: on_disconnect,
        });

        let sample_timer = timerInit(&TimerConfig {
            user_data: std::ptr::null(),
            callback: on_sample,
        });

        let chip = Chip {
            model,
            int1,
            accel: ["accelX", "accelY", "accelZ"].map(|name| attrInit(cstr(name).as_ptr(), 0.0)),
            gyro: ["gyroX", "gyroY", "gyroZ"].map(|name| attrInit(cstr(name).as_ptr(), 0.0)),
        };

        CHIP.with(|c| *c.borrow_mut() = Some(chip));

        timerStart(sample_timer, SAMPLE_PERIOD_US, true);
    }

    debug("lsm6dsv: chip initialized");
}
