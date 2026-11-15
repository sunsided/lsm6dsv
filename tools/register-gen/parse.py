"""Parse ST C register headers and the datasheet address maps into a JSON model."""
import json, re, sys

GROUPS = ["main", "if2", "embedded", "pg0_emb_adv", "pg1_emb_adv", "pg2_emb_adv", "sensor_hub"]

def parse_header(path, prefix):
    text = open(path).read().splitlines()
    group = None
    regs = {}  # group -> ordered list of dict
    pending = []  # defines awaiting a struct
    i = 0
    while i < len(text):
        line = text[i]
        m = re.search(r"@defgroup\s+bitfields_page_(\w+)", line)
        if m:
            group = m.group(1); regs.setdefault(group, []); pending = []; i += 1; continue
        if re.search(r"@defgroup\s+" + prefix + r"\s", line) or "@defgroup  " + prefix + " " in line:
            group = None
        if group:
            m = re.match(r"#define\s+" + prefix + r"_(\w+)\s+(0x[0-9A-Fa-f]+)U?\b", line)
            if m and not re.search(r"_PG_\d$", m.group(1)):
                r = {"name": m.group(1), "addr": int(m.group(2), 16), "fields": None}
                regs[group].append(r); pending.append(r)
            if line.startswith("typedef struct"):
                fields = []
                j = i + 1
                while "DRV_LITTLE_ENDIAN" not in text[j]: j += 1
                j += 1
                while not text[j].startswith("#elif"):
                    fm = re.match(r"\s*uint8_t\s+(\w+)\s*:\s*(\d+)\s*;", text[j])
                    if fm: fields.append([fm.group(1), int(fm.group(2))])
                    j += 1
                # struct applies to the last pending define(s); typically one
                assert sum(w for _, w in fields) == 8, (path, pending[-1]["name"], fields)
                if pending:
                    pending[-1]["fields"] = fields
                pending = []
                i = j
        i += 1
    return regs

# Layouts where ST's C headers disagree with the datasheets; the datasheets win.
LAYOUT_OVERRIDES = {
    "16x": {
        # DS13510 table 427: EXT_FORMAT_SEL is bit 2 (ST's lsm6dsv16x-rs agrees); the header has bit 1.
        "EXT_FORMAT": [["not_used0", 2], ["ext_format_sel", 1], ["not_used1", 5]],
    },
    "dsv": {
        # DS13476 table 397, as for the LSM6DSV16X.
        "EXT_FORMAT": [["not_used0", 2], ["ext_format_sel", 1], ["not_used1", 5]],
        # DS13476: PEDO_ADV_EN is bit 4 (MLC_EN on the LSM6DSV16X); the header leaves it out.
        "EMB_FUNC_EN_B": [["fsm_en", 1], ["not_used0", 2], ["fifo_compr_en", 1], ["pedo_adv_en", 1], ["not_used1", 3]],
        # DS13476: FP_REJECTION_EN is bit 2, as on the LSM6DSV16X; the header leaves it out.
        "PEDO_CMD_REG": [["not_used0", 2], ["fp_rejection_en", 1], ["carry_count_en", 1], ["not_used1", 4]],
    },
}


def apply_overrides(chip, regs):
    for group in regs.values():
        for r in group:
            if r["name"] in LAYOUT_OVERRIDES[chip]:
                r["fields"] = LAYOUT_OVERRIDES[chip][r["name"]]
    return regs


TABLE_TITLES = {
    "main": r"Table \d+\. Registers address map\s*$",
    "if2": r"Table \d+\. SPI2 register address map\s*$",
    "embedded": r"Table \d+\. Register address map - embedded functions\s*$",
    "pg0_emb_adv": r"Table \d+\. Register address map - embedded advanced features page 0\s*$",
    "pg1_emb_adv": r"Table \d+\. Register address map - embedded advanced features page 1\s*$",
    "pg2_emb_adv": r"Table \d+\. Register address map - embedded advanced features page 2\s*$",
    "sensor_hub": r"Table \d+\. Register address map - sensor hub registers\s*$",
}

# A table ends at the next address map or at the first register section heading.
TABLE_END = re.compile(r"Table \d+\. .*address map|^\s*\d+(\.\d+)*\s{3,}\S.*(\([0-9A-F]{2,3}h\)|register description)")


def parse_datasheet(path):
    """Returns {group: {NAME: {"access", "addr"}}} from the address map tables."""
    lines = open(path).read().split("\n")
    out = {}

    for group, title in TABLE_TITLES.items():
        start = next(i for i, l in enumerate(lines) if re.search(title, l))
        out[group] = {}

        for line in lines[start + 1:]:
            if TABLE_END.search(line):
                break

            m = re.match(r"\s*([A-Z][A-Z0-9_a-z]*)\s+(R/W|RW|R|W|-)\s+([0-9A-F]{2,3})\b", line)
            if m and m.group(1) not in ("RESERVED", "Name"):
                out[group][m.group(1)] = {"access": m.group(2), "addr": int(m.group(3), 16)}
                continue

            # Rows whose type column is split over several lines (UI_*_OIS).
            m = re.match(r"\s*([A-Z][A-Z0-9_a-z]*)\s+([0-9A-F]{2})\s+[01 ]{8,9}", line)
            if m:
                out[group][m.group(1)] = {"access": "R/W", "addr": int(m.group(2), 16)}

    return out


# Header names that the datasheet spells differently, or wraps over two lines of the table.
DATASHEET_NAMES = {
    "INTERNAL_FREQ": "INTERNAL_FREQ_FINE",
    "STATUS_MASTER_MAINPAGE": None,
    "EMB_FUNC_STATUS_MAINPAGE": None,
}


def check_addresses(chip, header, datasheet):
    """Prints header registers missing from the datasheet or at another address."""
    for group, regs in header.items():
        table = {k.upper(): v["addr"] for k, v in datasheet[group].items()}
        for r in regs:
            name = r["name"].upper()
            if name in DATASHEET_NAMES:
                if DATASHEET_NAMES[name] is None:
                    continue
                name = DATASHEET_NAMES[name]
            if name not in table:
                print(f"{chip} {group}: {r['name']} not in the datasheet address map")
            elif table[name] != r["addr"]:
                print(f"{chip} {group}: {r['name']} at {r['addr']:#x}, datasheet {table[name]:#x}")


if __name__ == "__main__":
    ref, ds16x, outp = sys.argv[1:4]
    dsv = sys.argv[4] if len(sys.argv) > 4 else None
    model = {
        "dsv": apply_overrides("dsv", parse_header(ref + "/lsm6dsv-pid/lsm6dsv_reg.h", "LSM6DSV")),
        "16x": apply_overrides("16x", parse_header(ref + "/lsm6dsv16x-pid/lsm6dsv16x_reg.h", "LSM6DSV16X")),
        "ds16x": parse_datasheet(ds16x),
    }
    if dsv:
        model["dsdsv"] = parse_datasheet(dsv)

    json.dump(model, open(outp, "w"), indent=1)
    for chip in ("dsv", "16x"):
        print(chip, {g: len(v) for g, v in model[chip].items()})
    print("ds16x", {g: len(v) for g, v in model["ds16x"].items()})
    check_addresses("16x", model["16x"], model["ds16x"])
    if dsv:
        print("dsdsv", {g: len(v) for g, v in model["dsdsv"].items()})
        check_addresses("dsv", model["dsv"], model["dsdsv"])
