//! Driver tests against the behavioral model.

use core::cell::RefCell;
use lsm6dsv::registers::{
    AdvancedRegisterAddress, EmbFuncEnA, EmbFuncInt1, Int1Ctrl, Md1Cfg, PedoCmdReg, SflpOdr,
};
use lsm6dsv::*;
use lsm6dsv_sim::{FakeDelay, FakeI2c, FakeSpi, Lsm6dsvModel};

fn model() -> RefCell<Lsm6dsvModel> {
    RefCell::new(Lsm6dsvModel::new())
}

#[test]
fn identify_on_both_addresses() {
    let m = model();
    assert!(Lsm6dsv::new(FakeI2c::new(&m)).identify().unwrap());
    let alt = RefCell::new(Lsm6dsvModel::with_address(ALTERNATIVE_DEVICE_ADDRESS));
    assert!(
        Lsm6dsv::with_address(FakeI2c::new(&alt), ALTERNATIVE_DEVICE_ADDRESS)
            .identify()
            .unwrap()
    );
    assert!(Lsm6dsv::new(FakeI2c::new(&alt)).identify().is_err());
}

#[test]
fn reset_and_configure() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_accel_mode(DataRate::Hz960, AccelMode::Normal)
        .unwrap();
    imu.reset(&mut FakeDelay::new(&m)).unwrap();
    assert_eq!(m.borrow().main_register(0x10), 0);

    imu.set_accel_mode(DataRate::Hz120, AccelMode::HighPerformance)
        .unwrap();
    imu.set_gyro_mode(DataRate::Hz240, GyroMode::LowPower)
        .unwrap();
    assert_eq!(m.borrow().main_register(0x10), 0x06);
    assert_eq!(m.borrow().main_register(0x11), 0x57);
}

#[test]
fn scaled_outputs() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_accel_full_scale(AccelFullScale::G8).unwrap();
    imu.set_gyro_full_scale(GyroFullScale::Dps1000).unwrap();
    m.borrow_mut().set_accel([0, -4098, 4098]);
    m.borrow_mut().set_gyro([100, 0, -100]);
    m.borrow_mut().set_temperature(-512);

    let status = imu.status().unwrap();
    assert!(status.xlda() && status.gda() && status.tda());
    let a = imu.read_acceleration().unwrap();
    assert_eq!(a.scale, AccelFullScale::G8);
    assert!((a.g()[2] - 1.0).abs() < 0.001);
    assert_eq!(
        imu.read_angular_rate().unwrap().degrees_per_second(),
        [3.5, 0.0, -3.5]
    );
    assert_eq!(imu.read_temperature().unwrap().celsius(), 23.0);
    assert!(!imu.status().unwrap().xlda());
}

#[test]
fn full_scale_is_read_back_once_when_unknown() {
    let m = model();
    // Configured by an earlier program run.
    Lsm6dsv::new(FakeI2c::new(&m))
        .set_accel_full_scale(AccelFullScale::G16)
        .unwrap();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    assert_eq!(imu.accel_full_scale().unwrap(), AccelFullScale::G16);
}

#[test]
fn embedded_bank_access_returns_to_main() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_pedometer(true).unwrap();
    assert!(EmbFuncEnA::from(m.borrow().embedded_register(0x04)).pedo_en());
    assert_eq!(m.borrow().main_register(0x01), 0);

    m.borrow_mut().set_steps(1234);
    assert_eq!(imu.step_count().unwrap(), 1234);
    imu.reset_step_count().unwrap();
    assert_eq!(imu.step_count().unwrap(), 0);

    imu.set_sflp(true, SflpDataRate::Hz480).unwrap();
    assert_eq!(
        SflpOdr::from(m.borrow().embedded_register(0x5E)).sflp_game_odr(),
        SflpDataRate::Hz480
    );
    assert!(imu.identify().unwrap());
}

#[test]
fn embedded_events_route_to_int1() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_int1_embedded_events(EmbFuncInt1::new().with_int1_step_detector(true))
        .unwrap();
    assert_eq!(m.borrow().embedded_register(0x0A), 0x08);
    assert!(Md1Cfg::from(m.borrow().main_register(0x5E)).int1_emb_func());
    imu.set_int1_routing(Int1Ctrl::new().with_int1_fifo_th(true))
        .unwrap();
    assert_eq!(m.borrow().main_register(0x0D), 0x08);
}

#[test]
fn advanced_pages() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.write_advanced(PedoCmdReg::new().with_carry_count_en(true))
        .unwrap();
    assert_eq!(m.borrow().page_register(1, 0x83), 0x08);
    assert_eq!(
        imu.read_advanced::<PedoCmdReg>().unwrap(),
        PedoCmdReg::new().with_carry_count_en(true)
    );
    assert_eq!(AdvancedRegisterAddress::PEDO_CMD_REG.page(), 1);

    // A transfer across the end of page 0 continues on page 1.
    imu.write_pages(0, 0xFF, &[0xAB, 0xCD]).unwrap();
    assert_eq!(m.borrow().page_register(0, 0xFF), 0xAB);
    assert_eq!(m.borrow().page_register(1, 0x00), 0xCD);
    let mut back = [0; 2];
    imu.read_pages(0, 0xFF, &mut back).unwrap();
    assert_eq!(back, [0xAB, 0xCD]);

    // PAGE_SEL and PAGE_RW are restored, and the main bank is selected.
    assert_eq!(m.borrow().embedded_register(0x02), 0x01);
    assert_eq!(m.borrow().embedded_register(0x17) & 0x60, 0);
    assert_eq!(m.borrow().main_register(0x01), 0);
}

#[test]
fn fifo() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_fifo_watermark(2).unwrap();
    imu.set_fifo_batching(DataRate::Hz120, DataRate::Off)
        .unwrap();
    imu.set_fifo_mode(FifoMode::Continuous).unwrap();
    assert_eq!(m.borrow().main_register(0x09), 0x06);
    assert_eq!(m.borrow().main_register(0x0A), 0x06);

    m.borrow_mut()
        .push_fifo(FifoTag::AccelNc.into_bits(), 0, [0x00, 0x10, 0, 0, 0, 0]);
    m.borrow_mut()
        .push_fifo(FifoTag::Timestamp.into_bits(), 1, [1, 0, 0, 0, 0, 0]);
    m.borrow_mut().push_fifo(
        FifoTag::SflpGameRotationVector.into_bits(),
        2,
        [0, 0, 0, 0, 0, 0],
    );

    let status = imu.fifo_status().unwrap();
    assert_eq!(status.level, 3);
    assert!(status.watermark);

    let mut words = [FifoWord::from_bytes(&[0; 7]); 2];
    assert_eq!(imu.read_fifo(&mut words).unwrap(), 2);
    assert_eq!(
        words[0].decode(AccelFullScale::G2, GyroFullScale::Dps125),
        FifoSample::Acceleration(Acceleration {
            raw: [4096, 0, 0],
            scale: AccelFullScale::G2
        })
    );
    assert_eq!(
        words[1].decode(AccelFullScale::G2, GyroFullScale::Dps125),
        FifoSample::Timestamp(Timestamp { raw: 1 })
    );
    let FifoSample::GameRotation(q) = imu
        .read_fifo_word()
        .unwrap()
        .decode(AccelFullScale::G2, GyroFullScale::Dps125)
    else {
        panic!("expected a quaternion");
    };
    assert_eq!((q.x, q.w), (0.0, 1.0));
    assert_eq!(imu.read_fifo_word().unwrap().tag, FifoTag::Empty);
}

#[test]
fn sensor_hub() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_sensor_hub_slave(1, 0x1E, 0x68, 6).unwrap();
    assert_eq!(m.borrow().sensor_hub_register(0x18), 0x1E << 1 | 1);
    assert_eq!(m.borrow().sensor_hub_register(0x19), 0x68);
    assert_eq!(m.borrow().sensor_hub_register(0x1A) & 0b111, 6);
    imu.set_sensor_hub(true, 2).unwrap();
    assert_eq!(m.borrow().sensor_hub_register(0x14), 0b101);

    m.borrow_mut().set_sensor_hub_data(0, &[1, 2, 3]);
    let mut data = [0; 3];
    imu.read_sensor_hub(&mut data).unwrap();
    assert_eq!(data, [1, 2, 3]);
    assert_eq!(m.borrow().main_register(0x01), 0);
}

#[test]
fn configuration_sequence_ends_in_main_bank() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    // Select the embedded bank, enable tilt, and stay there.
    imu.write_configuration(&[(0x01, 0x80), (0x04, 0x10)])
        .unwrap();
    assert_eq!(m.borrow().embedded_register(0x04), 0x10);
    assert_eq!(m.borrow().main_register(0x01), 0);
}

#[test]
fn timestamp_and_offsets() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_timestamp(true).unwrap();
    m.borrow_mut().tick_ns(21_750 * 10);
    assert_eq!(imu.read_timestamp().unwrap().raw, 10);
    imu.set_accel_offset([1, -1, 2], true, true).unwrap();
    assert_eq!(
        [0x73, 0x74, 0x75].map(|a| m.borrow().main_register(a)),
        [1, 0xFF, 2]
    );
    assert_eq!(m.borrow().main_register(0x18) & 0b11, 0b11);
}

#[test]
fn spi() {
    let m = model();
    let mut imu = Lsm6dsv::new_spi(FakeSpi::new(&m));
    assert!(imu.identify().unwrap());
    m.borrow_mut().set_steps(7);
    assert_eq!(imu.step_count().unwrap(), 7);
    let mut ois = ois::Lsm6dsvOis::new(FakeSpi::new(&m));
    assert_eq!(ois.who_am_i().unwrap(), 0x70);
}

#[cfg(feature = "lsm6dsv16x")]
#[test]
fn qvar() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_qvar(true).unwrap();
    assert_eq!(m.borrow().main_register(0x16) & 0x80, 0x80);
    m.borrow_mut().set_qvar(780);
    assert_eq!(imu.read_qvar().unwrap().millivolts(), 10.0);
}

#[cfg(feature = "async")]
#[test]
fn async_driver() {
    use embassy_futures::block_on;
    let m = model();
    let mut imu = asynch::Lsm6dsv::new(FakeI2c::new(&m));
    block_on(async {
        assert!(imu.identify().await.unwrap());
        imu.reset(&mut FakeDelay::new(&m)).await.unwrap();
        imu.set_accel_full_scale(AccelFullScale::G4).await.unwrap();
        m.borrow_mut().set_accel([0, 0, 1000]);
        assert_eq!(imu.read_acceleration().await.unwrap().milli_g()[2], 122.0);
        imu.write_advanced(PedoCmdReg::new().with_carry_count_en(true))
            .await
            .unwrap();
        m.borrow_mut().set_steps(3);
        assert_eq!(imu.step_count().await.unwrap(), 3);
    });
    assert_eq!(m.borrow().page_register(1, 0x83), 0x08);
}

#[test]
fn self_test_passes_and_restores_configuration() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_accel_mode(DataRate::Hz960, AccelMode::Normal)
        .unwrap();
    imu.set_gyro_full_scale(GyroFullScale::Dps250).unwrap();
    m.borrow_mut().set_accel([10, 20, 8000]);
    m.borrow_mut().set_gyro([1, 2, 3]);

    let mut delay = FakeDelay::new(&m);
    let accel = imu.self_test_accel(&mut delay).unwrap();
    assert!(accel.passed(), "{accel:?}");
    let gyro = imu.self_test_gyro(&mut delay).unwrap();
    assert!(gyro.passed(), "{gyro:?}");

    // CTRL1 (960 Hz normal mode), CTRL6 (±250 dps) and CTRL10 are back.
    assert_eq!(m.borrow().main_register(0x10), 0x79);
    assert_eq!(imu.gyro_full_scale().unwrap(), GyroFullScale::Dps250);
    assert_eq!(m.borrow().main_register(0x19), 0);
    assert_eq!(imu.read_acceleration().unwrap().raw, [10, 20, 8000]);
}

#[test]
fn self_test_times_out_without_samples() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    // A delay that does not advance the model: no samples arrive.
    struct NoDelay;
    impl embedded_hal::delay::DelayNs for NoDelay {
        fn delay_ns(&mut self, _: u32) {}
    }

    assert_eq!(
        imu.self_test_accel(&mut NoDelay),
        Err(SelfTestError::Timeout)
    );
}

#[test]
fn filters() {
    let m = model();
    let mut imu = Lsm6dsv::new(FakeI2c::new(&m));
    imu.set_gyro_low_pass(Some(0b101)).unwrap();
    assert_eq!(m.borrow().main_register(0x15) & 0x70, 0x50);
    assert_eq!(m.borrow().main_register(0x16) & 0x01, 0x01);
    imu.set_gyro_low_pass(None).unwrap();
    assert_eq!(m.borrow().main_register(0x16) & 0x01, 0);

    imu.set_accel_filter(AccelFilter::HighPass(AccelBandwidth::Div100))
        .unwrap();
    assert_eq!(m.borrow().main_register(0x17) >> 5, 0b100);
    assert_eq!(m.borrow().main_register(0x18) & 0b11000, 0b10000);
    imu.set_accel_filter(AccelFilter::LowPass(AccelBandwidth::Div10))
        .unwrap();
    assert_eq!(m.borrow().main_register(0x18) & 0b11000, 0b01000);
}
