"""Generate the lsm6dsv-registers bank modules from model.json + datasheet text.

Usage: python3 -I gen.py model.json ds16x.txt out_dir
"""
import json
import re
import sys

model = json.load(open(sys.argv[1]))
ds_lines = open(sys.argv[2]).read().split("\n")
out_dir = sys.argv[3]

# --- datasheet one-line descriptions -------------------------------------------------------------
SECTION_GROUP = {"9": "main", "11": "if2", "13": "embedded", "15": "adv", "17": "sensor_hub"}
descriptions = {}  # (group, NAME) -> text
for i, line in enumerate(ds_lines):
    m = re.match(r"\s*(\d+)\.\d+(?:\.\d+)?\s+([A-Z].*\([0-9A-F]{2,3}h.*)$", line)
    if not m or m.group(1) not in SECTION_GROUP:
        continue
    group = SECTION_GROUP[m.group(1)]
    heading = m.group(2)
    # headings can wrap onto the next line
    j = i + 1
    while j < len(ds_lines) and ds_lines[j].strip() and "(" not in ds_lines[j] and len(ds_lines[j].strip()) < 30:
        heading += " " + ds_lines[j].strip()
        j += 1
    names = re.findall(r"([A-Z][A-Z0-9_a-z]+)\s*\(", heading)
    desc = ""
    while j < len(ds_lines) and j < i + 6:
        t = ds_lines[j].strip()
        if t and not t.startswith("Table") and "DS13510" not in t:
            desc = t
            break
        j += 1
    desc = re.sub(r"\s+", " ", desc)
    for n in names:
        descriptions.setdefault((group, n.upper()), desc)


def describe(group, name):
    g = "adv" if group.endswith("_emb_adv") else group
    d = descriptions.get((g, name.upper()))
    if d and not re.match(r"^[A-Z0-9_]+$", d):
        return d.rstrip(".")
    return None


# --- datasheet access types ----------------------------------------------------------------------
ALIASES = {"INTERNAL_FREQ": "INTERNAL_FREQ_FINE"}
READ_ONLY_EXTRA = {"STATUS_MASTER_MAINPAGE", "EMB_FUNC_STATUS_MAINPAGE"}


def access(group, name):
    if name in READ_ONLY_EXTRA:
        return "R"
    table = {k.upper(): v for k, v in model["ds16x"][group].items()}
    entry = table[ALIASES.get(name, name).upper()]
    return {"RW": "R/W"}.get(entry["access"], entry["access"])


# --- field enums (hand-written in types.rs) ------------------------------------------------------
FIELD_TYPES = {
    ("CTRL1", "odr_xl"): "DataRate",
    ("CTRL1", "op_mode_xl"): "AccelMode",
    ("CTRL2", "odr_g"): "DataRate",
    ("CTRL2", "op_mode_g"): "GyroMode",
    ("CTRL6", "fs_g"): "GyroFullScale",
    ("CTRL8", "fs_xl"): "AccelFullScale",
    ("FIFO_CTRL3", "bdr_xl"): "DataRate",
    ("FIFO_CTRL3", "bdr_gy"): "DataRate",
    ("FIFO_CTRL4", "fifo_mode"): "FifoMode",
    ("FIFO_CTRL4", "odr_t_batch"): "TemperatureBatchRate",
    ("FIFO_CTRL4", "dec_ts_batch"): "TimestampDecimation",
    ("FIFO_DATA_OUT_TAG", "tag_sensor"): "FifoTag",
    ("SFLP_ODR", "sflp_game_odr"): "SflpDataRate",
    ("CTRL8", "hp_lpf2_xl_bw"): "AccelBandwidth",
    ("CTRL10", "st_xl"): "SelfTestMode",
    ("CTRL10", "st_g"): "SelfTestMode",
}

RUST_KEYWORDS = {"type", "mod", "move", "ref", "self", "loop", "match", "use", "fn", "in", "if", "else"}


def camel(name):
    return "".join(p[:1].upper() + p[1:].lower() for p in name.split("_") if p)


def field_ident(name):
    n = name.lower()
    return f"r#{n}" if n in RUST_KEYWORDS else n


def is_padding(name):
    return name.startswith("not_used")


GROUPS = {
    "main": dict(module="main_bank", enum="RegisterAddress", bank="Bank::Main", kind="bank",
                 title="Main (user interface) bank registers."),
    "embedded": dict(module="embedded", enum="EmbeddedRegisterAddress", bank="Bank::EmbeddedFunctions",
                     kind="bank", title="Embedded functions bank registers (`FUNC_CFG_ACCESS.EMB_FUNC_REG_ACCESS = 1`)."),
    "sensor_hub": dict(module="sensor_hub", enum="SensorHubRegisterAddress", bank="Bank::SensorHub", kind="bank",
                       title="Sensor hub bank registers (`FUNC_CFG_ACCESS.SHUB_REG_ACCESS = 1`)."),
    "if2": dict(module="spi2", enum="Spi2RegisterAddress", bank=None, kind="spi2",
                title="Registers of the auxiliary SPI2 (OIS) interface.\n//!\n//! These registers are only reachable over the auxiliary SPI bus, never from the primary interface."),
}
ADV_PAGES = ["pg0_emb_adv", "pg1_emb_adv", "pg2_emb_adv"]

FEATURE = 'feature = "lsm6dsv16x"'


def variants(group):
    """Yields (name, addr, fields, cfg) where cfg is None, 'only16x' or 'not16x'."""
    dsv = {r["name"]: r for r in model["dsv"][group]}
    for r in model["16x"][group]:
        d = dsv.get(r["name"])
        if d is None:
            yield r["name"], r["addr"], r["fields"], "only16x"
        elif d["fields"] == r["fields"]:
            yield r["name"], r["addr"], r["fields"], None
        else:
            yield r["name"], r["addr"], r["fields"], "only16x"
            yield d["name"], d["addr"], d["fields"], "not16x"


def cfg_attr(cfg):
    if cfg == "only16x":
        return f"#[cfg({FEATURE})]\n#[cfg_attr(docsrs, doc(cfg({FEATURE})))]\n"
    if cfg == "not16x":
        return f"#[cfg(not({FEATURE}))]\n"
    return ""


def emit_struct(out, rust_name, reg_name, addr_doc, acc, fields, cfg, doc_extra):
    ro = acc == "R"
    out.append(cfg_attr(cfg))
    out.append(f"/// {addr_doc}\n")
    if doc_extra:
        out.append(f"///\n/// {doc_extra}.\n")
    out.append("#[bitfield(u8)]\n#[derive(PartialEq, Eq)]\n#[cfg_attr(feature = \"defmt\", derive(defmt::Format))]\n")
    out.append(f"pub struct {rust_name} {{\n")
    bit = 0
    for fname, width in fields:
        lo, hi = bit, bit + width - 1
        bit += width
        if is_padding(fname):
            out.append(f"    #[bits({width})]\n    __{fname}: u8,\n")
            continue
        bits = f"bit {lo}" if width == 1 else f"bits {hi}:{lo}"
        enum = FIELD_TYPES.get((reg_name, fname))
        ty = enum or ("bool" if width == 1 else "u8")
        acc_attr = ", access = RO" if ro else ""
        if ro and enum:
            acc_attr += ", default = 0"  # bitfield-struct needs a raw default without a setter
        out.append(f"    /// `{fname.upper()}` ({bits}).\n")
        if enum:
            out.append(f"    ///\n    /// See [`{enum}`](crate::{enum}).\n")
        out.append(f"    #[bits({width}{acc_attr})]\n    pub {field_ident(fname)}: {ty},\n")
    out.append("}\n\n")
    if fields == [] or bit != 8:
        raise SystemExit(f"bad width {reg_name}")


def gen_bank(group):
    g = GROUPS[group]
    out = [f"//! {g['title']}\n\n"]
    uses_types = set()
    rows = list(variants(group))
    for name, _, fields, _ in rows:
        for f, _ in fields:
            if (name, f) in FIELD_TYPES:
                uses_types.add(FIELD_TYPES[(name, f)])
    if g["bank"]:
        out.append("use crate::Bank;\n")
    for t in sorted(uses_types):
        out.append(f"use crate::types::{t};\n")
    out.append("use bitfield_struct::bitfield;\n\n")

    # address enum (one variant per register, deduplicated across cfg variants)
    out.append(f"/// Register addresses of this bank.\n#[allow(non_camel_case_types)]\n"
               "#[derive(Debug, Copy, Clone, PartialEq, Eq)]\n"
               "#[cfg_attr(feature = \"defmt\", derive(defmt::Format))]\n#[repr(u8)]\n"
               f"pub enum {g['enum']} {{\n")
    seen = set()
    for name, addr, _, cfg in rows:
        if name in seen:
            continue
        seen.add(name)
        acc = access(group, name)
        if cfg == "only16x" and not any(n == name and c == "not16x" for n, _, _, c in rows):
            out.append(f"    #[cfg({FEATURE})]\n")
        kind = {"R": "Read-only", "R/W": "Read-write", "W": "Write-only"}[acc]
        out.append(f"    /// See [`{camel(name)}`]. {kind}.\n    {name} = 0x{addr:02X},\n")
    out.append("}\n\n")
    out.append(f"impl {g['enum']} {{\n    /// Returns the address of a register.\n"
               "    pub const fn addr(&self) -> u8 {\n        *self as u8\n    }\n}\n\n"
               f"impl From<{g['enum']}> for u8 {{\n    fn from(value: {g['enum']}) -> Self {{\n"
               "        value.addr()\n    }\n}\n\n")

    for name, addr, fields, cfg in rows:
        acc = access(group, name)
        rust = camel(name)
        addr_doc = f"[`{name}`]({g['enum']}::{name}) ({addr:02X}h), {acc}"
        emit_struct(out, rust, name, addr_doc, acc, fields, cfg, describe(group, name))
        macro = "readable" if acc == "R" else "writable"
        if g["kind"] == "bank":
            out.append(cfg_attr(cfg) + f"{macro}_register!({rust}, {g['enum']}::{name}, {g['bank']});\n\n")
        else:
            out.append(cfg_attr(cfg) + f"{macro}_spi2_register!({rust}, {g['enum']}::{name});\n\n")
    return "".join(out), [r[0] for r in rows]


def gen_advanced():
    out = ["//! Embedded advanced features registers.\n//!\n"
           "//! These registers are not directly addressable. They are accessed indirectly via\n"
           "//! [`PageSel`](crate::PageSel), [`PageAddress`](crate::PageAddress) and\n"
           "//! [`PageValue`](crate::PageValue) in the embedded functions bank.\n\n"
           "use bitfield_struct::bitfield;\n\n"]
    out.append("/// Advanced feature register addresses, encoded as `page << 8 | address`.\n"
               "#[allow(non_camel_case_types)]\n#[derive(Debug, Copy, Clone, PartialEq, Eq)]\n"
               "#[cfg_attr(feature = \"defmt\", derive(defmt::Format))]\n#[repr(u16)]\n"
               "pub enum AdvancedRegisterAddress {\n")
    rows = []
    for page, group in enumerate(ADV_PAGES):
        for name, addr, fields, cfg in variants(group):
            rows.append((page, group, name, addr, fields, cfg))
    seen = set()
    for page, group, name, addr, _, cfg in rows:
        if name in seen:
            continue
        seen.add(name)
        if cfg == "only16x" and not any(n == name and c == "not16x" for _, _, n, _, _, c in rows):
            out.append(f"    #[cfg({FEATURE})]\n")
        out.append(f"    /// See [`{camel(name)}`]. Page {page}.\n    {name} = 0x{page:X}{addr:02X},\n")
    out.append("}\n\nimpl AdvancedRegisterAddress {\n"
               "    /// Returns the page number.\n    pub const fn page(&self) -> u8 {\n"
               "        (*self as u16 >> 8) as u8\n    }\n\n"
               "    /// Returns the address within the page.\n    pub const fn addr(&self) -> u8 {\n"
               "        *self as u16 as u8\n    }\n}\n\n")
    for page, group, name, addr, fields, cfg in rows:
        acc = access(group, name)
        rust = camel(name)
        addr_doc = f"[`{name}`](AdvancedRegisterAddress::{name}) (page {page}, {addr:02X}h), {acc}"
        emit_struct(out, rust, name, addr_doc, acc, fields, cfg, describe(group, name))
        macro = "readable" if acc == "R" else "writable"
        out.append(cfg_attr(cfg) + f"{macro}_advanced_register!({rust}, AdvancedRegisterAddress::{name});\n\n")
    return "".join(out), [r[2] for r in rows]


HEADER = "// Generated by tools/register-gen/gen.py. Do not edit by hand.\n\n"

all_names = {}
for group in GROUPS:
    src, names = gen_bank(group)
    open(f"{out_dir}/{GROUPS[group]['module']}.rs", "w").write(HEADER + src)
    for n in set(names):
        all_names.setdefault(camel(n), []).append(group)
src, names = gen_advanced()
open(f"{out_dir}/advanced.rs", "w").write(HEADER + src)
for n in set(names):
    all_names.setdefault(camel(n), []).append("adv")
dups = {k: v for k, v in all_names.items() if len(v) > 1}
if dups:
    raise SystemExit(f"duplicate type names: {dups}")
missing = [(g, n) for g in list(GROUPS) for n, *_ in variants(g) if describe(g, n) is None]
print("generated; registers without datasheet description:", len(missing), missing[:20])
