// LSM6DSV IMU demo for the Wokwi web editor (Arduino, ESP32-C3).
//
// Talks to the LSM6DSV Wokwi chip over raw I2C and prints acceleration and angular rate in
// the same format as the Rust example in ../esp32c3. Wiring: SDA = GPIO4, SCL = GPIO5,
// INT1 = GPIO7 (accelerometer data-ready).

#include <Wire.h>

constexpr uint8_t LSM6DSV_ADDRESS = 0x6A;
constexpr uint8_t LSM6DSV_WHO_AM_I = 0x70;

constexpr uint8_t REG_INT1_CTRL = 0x0D;
constexpr uint8_t REG_WHO_AM_I = 0x0F;
constexpr uint8_t REG_CTRL1 = 0x10;
constexpr uint8_t REG_CTRL2 = 0x11;
constexpr uint8_t REG_CTRL3 = 0x12;
constexpr uint8_t REG_CTRL6 = 0x15;
constexpr uint8_t REG_CTRL8 = 0x17;
constexpr uint8_t REG_OUTX_L_G = 0x22;
constexpr uint8_t REG_OUTX_L_A = 0x28;

// CTRL1/CTRL2: ODR in bits 3:0, operating mode in bits 6:4 (high-performance = 000).
constexpr uint8_t CTRL12_ODR_MASK = 0x0F;
constexpr uint8_t CTRL12_MODE_MASK = 0x70;
constexpr uint8_t ODR_120HZ = 0x06;

// CTRL3: SW_RESET.
constexpr uint8_t CTRL3_SW_RESET = 0x01;

// CTRL6: FS_G in bits 3:0, ±500 dps. CTRL8: FS_XL in bits 1:0, ±4 g.
constexpr uint8_t CTRL6_FS_G_MASK = 0x0F;
constexpr uint8_t FS_G_500DPS = 0x02;
constexpr uint8_t CTRL8_FS_XL_MASK = 0x03;
constexpr uint8_t FS_XL_4G = 0x01;

// INT1_CTRL: INT1_DRDY_XL.
constexpr uint8_t INT1_DRDY_XL = 0x01;

constexpr float ACCEL_MG_PER_LSB = 0.122f;
constexpr float GYRO_MDPS_PER_LSB = 17.5f;

constexpr int PIN_SDA = 4;
constexpr int PIN_SCL = 5;
constexpr int PIN_INT1 = 7;

bool readRegisters(uint8_t reg, uint8_t *buffer, size_t len) {
  Wire.beginTransmission(LSM6DSV_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(int(LSM6DSV_ADDRESS), int(len)) != int(len)) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

uint8_t readRegister(uint8_t reg) {
  uint8_t value = 0;
  readRegisters(reg, &value, 1);
  return value;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DSV_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void modifyRegister(uint8_t reg, uint8_t mask, uint8_t value) {
  writeRegister(reg, (readRegister(reg) & ~mask) | value);
}

// X, Y, Z as little-endian 16-bit raw counts (CTRL3.IF_INC is set by default).
bool readAxes(uint8_t reg, int16_t raw[3]) {
  uint8_t data[6];
  if (!readRegisters(reg, data, sizeof(data))) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    raw[i] = int16_t(uint16_t(data[2 * i + 1]) << 8 | data[2 * i]);
  }

  return true;
}

void halt() {
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_INT1, INPUT_PULLDOWN);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  uint8_t whoAmI = 0;
  if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1) || whoAmI != LSM6DSV_WHO_AM_I) {
    Serial.printf("LSM6DSV not found (WHO_AM_I 0x%02X)\n", whoAmI);
    halt();
  }

  // Software reset with both sensors powered down; SW_RESET clears itself when done.
  modifyRegister(REG_CTRL1, CTRL12_ODR_MASK, 0);
  modifyRegister(REG_CTRL2, CTRL12_ODR_MASK, 0);
  writeRegister(REG_CTRL3, CTRL3_SW_RESET);
  bool reset = false;
  for (int i = 0; i < 50 && !reset; i++) {
    delay(1);
    reset = !(readRegister(REG_CTRL3) & CTRL3_SW_RESET);
  }

  if (!reset) {
    Serial.println("LSM6DSV reset timed out");
    halt();
  }

  modifyRegister(REG_CTRL8, CTRL8_FS_XL_MASK, FS_XL_4G);
  modifyRegister(REG_CTRL6, CTRL6_FS_G_MASK, FS_G_500DPS);
  // 120 Hz, high-performance mode.
  modifyRegister(REG_CTRL1, CTRL12_ODR_MASK | CTRL12_MODE_MASK, ODR_120HZ);
  modifyRegister(REG_CTRL2, CTRL12_ODR_MASK | CTRL12_MODE_MASK, ODR_120HZ);
  writeRegister(REG_INT1_CTRL, INT1_DRDY_XL);
  Serial.printf("LSM6DSV ok who_am_i=0x%02X\n", whoAmI);
}

void loop() {
  while (digitalRead(PIN_INT1) == LOW) {
  }

  int16_t accel[3], gyro[3];
  if (readAxes(REG_OUTX_L_A, accel) && readAxes(REG_OUTX_L_G, gyro)) {
    Serial.printf("accel x=%.0f y=%.0f z=%.0f mg gyro x=%.0f y=%.0f z=%.0f dps\n",
                  accel[0] * ACCEL_MG_PER_LSB, accel[1] * ACCEL_MG_PER_LSB,
                  accel[2] * ACCEL_MG_PER_LSB, gyro[0] * GYRO_MDPS_PER_LSB / 1000.0f,
                  gyro[1] * GYRO_MDPS_PER_LSB / 1000.0f, gyro[2] * GYRO_MDPS_PER_LSB / 1000.0f);
  } else {
    Serial.println("read failed");
  }

  delay(200);
}
