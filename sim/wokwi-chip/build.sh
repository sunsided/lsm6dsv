#!/usr/bin/env sh
# Builds the chip and places it next to lsm6dsv.chip.json, as Wokwi expects.
set -eu
cd "$(dirname "$0")"
cargo build --release
cp target/wasm32-unknown-unknown/release/lsm6dsv_wokwi_chip.wasm lsm6dsv.chip.wasm
