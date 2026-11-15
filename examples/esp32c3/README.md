# ESP32-C3 example

Reads acceleration and angular rate from an LSM6DSV over I²C and prints them over the serial console.

Wiring: SDA = GPIO4, SCL = GPIO5, INT1 = GPIO7.

```sh
cargo run --release                      # flash and monitor real hardware (espflash)
../../sim/wokwi-chip/build.sh            # build the simulated LSM6DSV chip
cargo build --release
wokwi-cli --scenario scenarios/motion.yaml   # needs WOKWI_CLI_TOKEN
```
