//! Reads acceleration and angular rate from an LSM6DSV and prints them over the serial console.
//!
//! Wiring: SDA = GPIO4, SCL = GPIO5, INT1 = GPIO7 (accelerometer data-ready).

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::Rate;
use esp_println::println;
use lsm6dsv::registers::Int1Ctrl;
use lsm6dsv::{AccelFullScale, AccelMode, DataRate, GyroFullScale, GyroMode, Lsm6dsv};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("valid I2C config")
    .with_sda(peripherals.GPIO4)
    .with_scl(peripherals.GPIO5);
    let int1 = Input::new(
        peripherals.GPIO7,
        InputConfig::default().with_pull(Pull::Down),
    );

    let mut imu = Lsm6dsv::new(i2c);
    let who_am_i = imu.who_am_i().expect("device responds");

    if who_am_i != lsm6dsv::LSM6DSV_WHO_AM_I {
        panic!("unexpected WHO_AM_I 0x{who_am_i:02X}");
    }

    imu.reset(&mut delay).unwrap();
    imu.set_accel_full_scale(AccelFullScale::G4).unwrap();
    imu.set_gyro_full_scale(GyroFullScale::Dps500).unwrap();
    imu.set_accel_mode(DataRate::Hz120, AccelMode::HighPerformance)
        .unwrap();
    imu.set_gyro_mode(DataRate::Hz120, GyroMode::HighPerformance)
        .unwrap();
    imu.set_int1_routing(Int1Ctrl::new().with_int1_drdy_xl(true))
        .unwrap();
    println!("LSM6DSV ok who_am_i=0x{who_am_i:02X}");

    loop {
        while int1.is_low() {}

        let [ax, ay, az] = imu.read_acceleration().unwrap().milli_g();
        let [gx, gy, gz] = imu.read_angular_rate().unwrap().degrees_per_second();
        println!("accel x={ax:.0} y={ay:.0} z={az:.0} mg gyro x={gx:.0} y={gy:.0} z={gz:.0} dps");

        delay.delay_millis(200);
    }
}
