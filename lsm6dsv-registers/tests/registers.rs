//! Spot checks of the register map against datasheet DS13510.

use lsm6dsv_registers::prelude::*;
use lsm6dsv_registers::*;

fn i2c_addr<R: I2CRegister<DeviceAddress7, RegisterAddress8, R1>>() -> u8 {
    R::REGISTER_ADDRESS.into_inner()
}

fn spi_addr<R: SPIRegister<RegisterAddress8, R1>>() -> u8 {
    R::REGISTER_ADDRESS.into_inner()
}

#[test]
fn main_bank_addresses() {
    assert_eq!(i2c_addr::<FuncCfgAccess>(), 0x01);
    assert_eq!(i2c_addr::<WhoAmI>(), 0x0F);
    assert_eq!(i2c_addr::<Ctrl1>(), 0x10);
    assert_eq!(i2c_addr::<Ctrl10>(), 0x19);
    assert_eq!(i2c_addr::<OutxLG>(), 0x22);
    assert_eq!(i2c_addr::<OutxLA>(), 0x28);
    assert_eq!(i2c_addr::<Timestamp0>(), 0x40);
    assert_eq!(i2c_addr::<FifoDataOutTag>(), 0x78);
    assert_eq!(i2c_addr::<FifoDataOutZH>(), 0x7E);
    assert_eq!(spi_addr::<Ctrl1>(), 0x10);
    assert_eq!(Ctrl1::BANK, Bank::Main);
}

#[test]
fn other_bank_addresses() {
    assert_eq!(i2c_addr::<PageSel>(), 0x02);
    assert_eq!(PageSel::BANK, Bank::EmbeddedFunctions);
    assert_eq!(i2c_addr::<StepCounterL>(), 0x62);
    assert_eq!(i2c_addr::<SensorHub1>(), 0x02);
    assert_eq!(SensorHub1::BANK, Bank::SensorHub);
    assert_eq!(spi_addr::<Spi2WhoAmI>(), 0x0F);
    assert_eq!(<SflpGameGbiasxL as AdvancedRegister>::PAGE, 0);
    assert_eq!(<SflpGameGbiasxL as AdvancedRegister>::ADDRESS, 0x6E);
    assert_eq!(<PedoCmdReg as AdvancedRegister>::PAGE, 1);
    assert_eq!(<PedoCmdReg as AdvancedRegister>::ADDRESS, 0x83);
    assert_eq!(<ExtFormat as AdvancedRegister>::PAGE, 2);
}

#[test]
fn ctrl1_layout() {
    // Bit 7 reserved, OP_MODE_XL[6:4], ODR_XL[3:0].
    let reg = Ctrl1::new()
        .with_odr_xl(DataRate::Hz120)
        .with_op_mode_xl(AccelMode::Normal);
    assert_eq!(u8::from(reg), 0b0111_0110);
    assert_eq!(Ctrl1::from(0b0100_1100).op_mode_xl(), AccelMode::LowPower1);
    assert_eq!(Ctrl1::from(0b0100_1100).odr_xl(), DataRate::Hz7680);
}

#[test]
fn full_scale_layout() {
    // CTRL6: LPF1_G_BW[6:4], FS_G[3:0]. CTRL8: HP_LPF2_XL_BW[7:5], XL_DualC_EN[3], FS_XL[1:0].
    assert_eq!(
        u8::from(Ctrl6::new().with_fs_g(GyroFullScale::Dps4000)),
        0b0000_1100
    );
    assert_eq!(
        u8::from(Ctrl8::new().with_fs_xl(AccelFullScale::G16)),
        0b0000_0011
    );
    assert_eq!(u8::from(Ctrl8::new().with_xl_dualc_en(true)), 0b0000_1000);
}

#[test]
fn bank_selection_bits() {
    // EMB_FUNC_REG_ACCESS is bit 7, SHUB_REG_ACCESS is bit 6.
    assert_eq!(
        u8::from(FuncCfgAccess::new().with_emb_func_reg_access(true)),
        0x80
    );
    assert_eq!(
        u8::from(FuncCfgAccess::new().with_shub_reg_access(true)),
        0x40
    );
    assert_eq!(Bank::EmbeddedFunctions.selection(), (true, false));
}

#[test]
fn fifo_layout() {
    // FIFO_CTRL3: BDR_GY[7:4], BDR_XL[3:0]. FIFO_CTRL4: DEC_TS_BATCH[7:6], ODR_T_BATCH[5:4], FIFO_MODE[2:0].
    let ctrl3 = FifoCtrl3::new()
        .with_bdr_gy(DataRate::Hz960)
        .with_bdr_xl(DataRate::Hz30);
    assert_eq!(u8::from(ctrl3), 0x94);
    let ctrl4 = FifoCtrl4::new()
        .with_fifo_mode(FifoMode::Continuous)
        .with_odr_t_batch(TemperatureBatchRate::Hz60)
        .with_dec_ts_batch(TimestampDecimation::X8);
    assert_eq!(u8::from(ctrl4), 0b1011_0110);
    // FIFO_DATA_OUT_TAG: TAG_SENSOR[7:3], TAG_CNT[2:1].
    let tag = FifoDataOutTag::from(0x13 << 3 | 0b10 << 1);
    assert_eq!(tag.tag_sensor(), FifoTag::SflpGameRotationVector);
    assert_eq!(tag.tag_cnt(), 0b10);
}

#[test]
fn page_registers_layout() {
    // PAGE_SEL: PAGE_SEL[7:4]; PAGE_RW: EMB_FUNC_LIR[7], PAGE_WRITE[6], PAGE_READ[5].
    assert_eq!(u8::from(PageSel::new().with_page_sel(2)), 0x20);
    assert_eq!(u8::from(PageRw::new().with_page_write(true)), 0x40);
    assert_eq!(u8::from(PageRw::new().with_page_read(true)), 0x20);
}

#[test]
fn ext_format_follows_the_datasheet() {
    // DS13510 table 427: EXT_FORMAT_SEL is bit 2 (ST's C headers put it at bit 1).
    assert_eq!(u8::from(ExtFormat::new().with_ext_format_sel(true)), 0b100);
}

#[test]
fn raw_values_round_trip() {
    for value in 0..=u8::MAX {
        assert_eq!(u8::from(Ctrl3::from(value)), value);
        assert_eq!(u8::from(StatusReg::from(value)), value);
    }
}

#[cfg(feature = "lsm6dsv16x")]
#[test]
fn lsm6dsv16x_only() {
    // CTRL7: AH_QVAR_EN[7], INT2_DRDY_AH_QVAR[6], AH_QVAR_C_ZIN[5:4], LPF1_G_EN[0].
    assert_eq!(u8::from(Ctrl7::new().with_ah_qvar_en(true)), 0x80);
    assert_eq!(i2c_addr::<AhQvarOutL>(), 0x3A);
    assert_eq!(i2c_addr::<MlcStatusMainpage>(), 0x4B);
    assert!(StatusReg::from(0b1000).ah_qvarda());
    assert_eq!(<MlcExtSensitivityL as AdvancedRegister>::ADDRESS, 0xE8);
}
