# Register map generator

Generates the bank modules of `lsm6dsv-registers` (`main_bank.rs`, `embedded.rs`, `sensor_hub.rs`,
`advanced.rs`, `spi2.rs`). `types.rs` and `lib.rs` are written by hand.

Sources:

- The address maps, access types and one-line register descriptions come from datasheet DS13510 Rev 4 (LSM6DSV16X),
  converted with `pdftotext -layout`. Datasheet DS13476 Rev 5 (LSM6DSV) is used to check the plain LSM6DSV.
  `parse.py` finds the address map tables by their titles.
- Bit positions and widths come from ST's `lsm6dsv-pid` and `lsm6dsv16x-pid` C headers, used only as a source of
  register layout facts. Registers or bits that only the LSM6DSV16X header lists are emitted behind the `lsm6dsv16x` feature.
- `parse.py` checks that every header register exists in the matching datasheet at the same address, and applies
  the per-chip `LAYOUT_OVERRIDES` where a header disagrees with its datasheet.
- `check_bits.py` compares the reserved bits and field boundaries of every register against the datasheet bit
  tables (228 for the LSM6DSV16X, 214 for the LSM6DSV). It exits non-zero on a difference; `KNOWN` lists datasheet naming typos.

```sh
git clone --depth 1 https://github.com/STMicroelectronics/lsm6dsv-pid ref/lsm6dsv-pid
git clone --depth 1 https://github.com/STMicroelectronics/lsm6dsv16x-pid ref/lsm6dsv16x-pid
pdftotext -layout DS_lsm6dsv16x.pdf ds16x.txt
pdftotext -layout DS_lsm6dsv.pdf dsv.txt
python3 -I parse.py ref ds16x.txt model.json dsv.txt
python3 -I check_bits.py model.json ds16x.txt
python3 -I check_bits.py model.json dsv.txt dsv
python3 -I gen.py model.json ds16x.txt ../../lsm6dsv-registers/src
cargo fmt --all
```

Typed field enums are mapped in `FIELD_TYPES` in `gen.py`.
