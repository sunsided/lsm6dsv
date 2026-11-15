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

# Layouts where ST's C headers disagree with datasheet DS13510; the datasheet wins.
LAYOUT_OVERRIDES = {
    # Table 427: EXT_FORMAT_SEL is bit 2 (ST's lsm6dsv16x-rs agrees); the headers put it at bit 1.
    "EXT_FORMAT": [["not_used0", 2], ["ext_format_sel", 1], ["not_used1", 5]],
}


def apply_overrides(regs):
    for group in regs.values():
        for r in group:
            if r["name"] in LAYOUT_OVERRIDES:
                r["fields"] = LAYOUT_OVERRIDES[r["name"]]
    return regs


def parse_datasheet(path):
    lines = open(path).read().split("\n")
    sections = [(2780, "main"), (6078, "if2"), (6562, "embedded"), (7814, "pg0_emb_adv"),
                (7862, "pg1_emb_adv"), (7885, "pg2_emb_adv"), (8778, "sensor_hub")]
    ends = {"main": 2990, "if2": 6128, "embedded": 6646, "pg0_emb_adv": 7862, "pg1_emb_adv": 7885,
            "pg2_emb_adv": 7951, "sensor_hub": 8864}
    out = {}
    for start, g in sections:
        out[g] = {}
        prev = None
        for k in range(start, ends[g]):
            l = lines[k]
            m = re.match(r"\s*([A-Z][A-Z0-9_a-z]*)\s+(R/W|RW|R|W|-)\s+([0-9A-F]{2,3})\b", l)
            if m and m.group(1) not in ("RESERVED", "Name"):
                out[g][m.group(1)] = {"access": m.group(2), "addr": int(m.group(3), 16)}
                prev = m.group(1)
                continue
            m = re.match(r"\s*([A-Z][A-Z0-9_a-z]*)\s+([0-9A-F]{2})\s+[01 ]{8,9}", l)  # split type column
            if m:
                out[g][m.group(1)] = {"access": "R/W", "addr": int(m.group(2), 16)}
    return out

if __name__ == "__main__":
    ref, ds, outp = sys.argv[1:4]
    model = {
        "dsv": apply_overrides(parse_header(ref + "/lsm6dsv-pid/lsm6dsv_reg.h", "LSM6DSV")),
        "16x": apply_overrides(parse_header(ref + "/lsm6dsv16x-pid/lsm6dsv16x_reg.h", "LSM6DSV16X")),
        "ds16x": parse_datasheet(ds),
    }
    json.dump(model, open(outp, "w"), indent=1)
    for chip in ("dsv", "16x"):
        print(chip, {g: len(v) for g, v in model[chip].items()})
    print("ds", {g: len(v) for g, v in model["ds16x"].items()})
