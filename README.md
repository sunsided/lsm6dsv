# LSM6DSV / LSM6DSV16X

[![Rust](https://github.com/sunsided/lsm6dsv/actions/workflows/rust.yml/badge.svg)](https://github.com/sunsided/lsm6dsv/actions/workflows/rust.yml)
[![codecov](https://codecov.io/gh/sunsided/lsm6dsv/graph/badge.svg)](https://codecov.io/gh/sunsided/lsm6dsv)

`no_std` Rust support for the STMicroelectronics LSM6DSV and LSM6DSV16X 6-axis IMUs.

- [`lsm6dsv`](lsm6dsv): the driver, blocking and async, over I²C and SPI.
- [`lsm6dsv-registers`](lsm6dsv-registers): a typed register map, addressable over I²C and SPI.
- [`lsm6dsv-sim`](lsm6dsv-sim): a behavioral model of the device for host tests (not published).
- [`sim/wokwi-chip`](sim/wokwi-chip): the model as a Wokwi custom chip, and
  [`examples/esp32c3`](examples/esp32c3): ESP32-C3 firmware with a Wokwi scenario that CI runs.

Enable the `lsm6dsv16x` feature for registers and fields that exist only on the LSM6DSV16X.

The register map follows datasheets DS13510 (LSM6DSV16X) and DS13476 (LSM6DSV); see [`tools/register-gen`](tools/register-gen) for how it is generated. ST's [`lsm6dsv16x-rs`](https://github.com/STMicroelectronics/lsm6dsv16x-rs)
crate and the [`lsm6dsv-pid`](https://github.com/STMicroelectronics/lsm6dsv-pid) C driver served only as a cross-check;
no code was copied from either. If you want ST's own API, use `lsm6dsv16x-rs`.

## Emulation

The behavioral model in `lsm6dsv-sim` is used both as a fake bus in host tests and as a
[Wokwi](https://wokwi.com) custom chip (`sim/wokwi-chip`). Example firmware in
`examples/esp32c3` runs against the emulated chip using `wokwi-cli`.

### Use the chip in your own Wokwi project

Every [GitHub Release](https://github.com/sunsided/lsm6dsv/releases) carries the chip as
`chip.zip`. Add a `chip-lsm6dsv` part and this dependency to your `diagram.json`, with the
latest release version written without the tag's `v` (`@v0.3.1` fails to load):

```json
"dependencies": { "chip-lsm6dsv": "github:sunsided/lsm6dsv@0.3.1" }
```

This works on [wokwi.com](https://wokwi.com), in the Wokwi VS Code extension and with
`wokwi-cli`; nothing needs to be built.

### Run the example locally

Needs [`wokwi-cli`](https://docs.wokwi.com/wokwi-ci/cli-installation) and a
`WOKWI_CLI_TOKEN` from the [Wokwi CI dashboard](https://wokwi.com/dashboard/ci):

```sh
sim/wokwi-chip/build.sh
cd examples/esp32c3
cargo build --release
wokwi-cli . --scenario scenarios/motion.yaml --timeout 30000
```

[`sim/wokwi-chip`](sim/wokwi-chip/README.md) describes building the chip and its pins;
[`examples/esp32c3`](examples/esp32c3/README.md) the wiring and the scenario.

## License

Licensed under any of the [EUPL-1.2](LICENSE-EUPL), [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) licenses, at your option.
